"""
src/motion_control/search.py
搜索模块：支持单机全域螺旋 和 多机精细扇区螺旋（阶段二优化）

【官方代码借鉴说明】
- 多机扇区划分思路 → 参考 competition/baselines/coop_distributed.py 的 _uid_partition()
- 扇区内扩张螺旋算法 → 参考 competition/baselines/coop_distributed.py 的 _spiral()
- 多机螺旋参数（半径步进300m，角度步进15°）→ 在官方基础上优化，提高覆盖率
"""
from typing import Tuple
from .geo import (
    clamp_to_safebox, point_on_circle,
    DEFAULT_ALTITUDE, LAT_MIN, LAT_MAX, LON_MIN, LON_MAX
)
try:
    from sdk.core.commands import fly_to, Command
except ImportError:
    try:
        from competition.sdk.core.commands import fly_to, Command
    except ImportError:
        from dataclasses import dataclass

        @dataclass(frozen=True)
        class Command:
            verb: str
            params: dict

        def fly_to(lat, lon, alt=None, speed=None, loiter_radius=200.0):
            params = {"latitude": float(lat), "longitude": float(lon), "loiter_radius": float(loiter_radius)}
            if alt is not None: params["altitude"] = float(alt)
            if speed is not None: params["speed"] = float(speed)
            return Command("set_destination", params)

# 地图几何中心（用于螺旋搜索原点）
# 借鉴：coop_distributed.py 中 _BBOX 定义了地图边界，这里计算中心点
CENTER_LAT = (LAT_MIN + LAT_MAX) / 2.0   # ≈ 27.0034
CENTER_LON = (LON_MIN + LON_MAX) / 2.0   # ≈ 125.00015

# 多机模式：3 架 UAV 的扇区中心方位角（度）
# 借鉴：coop_distributed.py 中 _uid_partition() 使用 uid 哈希分区，
# 这里改用固定扇区中心（0°/120°/240°），确保三机均匀覆盖
UID_TO_SECTOR_CENTER = {
    "20001": 0.0,    # uav_alpha  -> 西北区域
    "20002": 120.0,  # uav_bravo  -> 东南区域
    "20003": 240.0,  # uav_charlie -> 西南区域
}


class SectorSearch:
    """
    搜索器：根据 multi_drone 标志自动切换单机/多机搜索策略。

    【官方代码借鉴说明】
    - 整体状态设计 → 参考 coop_distributed.py 的 CoopDistributedAgent 类结构
    - 单机全域螺旋 → 参考 coop_distributed.py 的 _spiral() 展开方式
    - 多机扇区螺旋 → 参考 coop_distributed.py 的 _spiral() + _uid_partition()
    """

    def __init__(
        self,
        uav_name: str,
        multi_drone: bool = False,
        altitude: float = DEFAULT_ALTITUDE,
        # 单机螺旋参数
        init_radius: float = 300.0,
        radius_step: float = 400.0,
        angle_step: float = 25.0,
        # 多机螺旋参数（阶段二优化）
        # 借鉴：coop_distributed.py 中 _spiral() 使用 growth=50, ang_speed=30
        # 这里将半径步进从 500m 优化为 300m，角度步进从 30° 优化为 15°
        multi_init_radius: float = 800.0,   # 初始半径（从近处开始扫描）
        multi_radius_step: float = 300.0,   # 每圈半径增量（官方约500m，优化为300m提升覆盖率）
        multi_angle_step: float = 15.0,     # 角度步进（官方约30°，优化为15°提升覆盖率）
        multi_max_radius: float = 8000.0,   # 最大半径（地图对角线约7900m）
    ):
        self.uav_name = uav_name
        self.multi_drone = multi_drone
        self.altitude = altitude

        # 单机模式属性
        self.current_radius = init_radius
        self.current_angle = 0.0
        self.radius_step = radius_step
        self.angle_step = angle_step
        self.first_run = True

        # 多机模式属性
        if multi_drone:
            # 【借鉴】coop_distributed.py 的 _uid_partition() 根据 uid 分配区域
            # 这里兼容 uav_alpha / 20001 两种命名方式
            uid = uav_name if uav_name in UID_TO_SECTOR_CENTER else None
            if uid is None:
                name_to_uid = {
                    "uav_alpha": "20001",
                    "uav_bravo": "20002",
                    "uav_charlie": "20003"
                }
                uid = name_to_uid.get(uav_name)
            self.sector_center_deg = UID_TO_SECTOR_CENTER.get(uid, 0.0)
            self.multi_radius = multi_init_radius
            self.multi_angle_offset = 0.0
            self.multi_radius_step = multi_radius_step
            self.multi_angle_step = multi_angle_step
            self.multi_max_radius = multi_max_radius
        else:
            self.sector_center_deg = 0.0

    def reset(self):
        """重置搜索状态（每局开始时调用）"""
        self.first_run = True
        self.current_radius = 300.0
        self.current_angle = 0.0
        if self.multi_drone:
            self.multi_radius = 800.0
            self.multi_angle_offset = 0.0

    def _get_single_waypoint(self) -> Tuple[float, float]:
        """
        单机模式：全域扩张螺旋

        【官方代码借鉴】参考 coop_distributed.py 的 _spiral() 方法：
        - 螺旋半径随时间增长 (growth * revs)
        - 角度随时间线性增加 (ang_speed * t)
        - 这里简化为以角度步进和半径步进驱动
        """
        if self.first_run:
            self.first_run = False
            return clamp_to_safebox(CENTER_LAT + 0.0005, CENTER_LON)

        # 【借鉴】_spiral() 中的螺旋坐标计算：radius * cos(bearing) / 111320
        target_lat, target_lon = point_on_circle(
            CENTER_LAT,
            CENTER_LON,
            self.current_radius,
            self.current_angle
        )
        # 更新螺旋状态
        self.current_angle += self.angle_step
        if self.current_angle >= 360.0:
            self.current_angle -= 360.0
            self.current_radius += self.radius_step
        return clamp_to_safebox(target_lat, target_lon)

    def _get_multi_waypoint(self) -> Tuple[float, float]:
        """
        多机模式：扇区内精细螺旋扫描

        【官方代码借鉴】参考 coop_distributed.py 的 _spiral() + _uid_partition()：
        1. 扇区中心由 _uid_partition() 决定（这里用 UID_TO_SECTOR_CENTER）
        2. _spiral() 中的 bearing = ang_speed * t，这里改为在扇区内左右摆动
        3. 半径增长逻辑与单机一致，但步进更精细（300m/圈）
        4. 超出最大半径后重置，实现多层覆盖（官方无此机制，为阶段二新增优化）
        """
        # 扇区内摆动：中心线左右交替偏移（30° 为最大摆动幅度）
        # 例如：中心+15°, 中心-15°, 中心+30°, 中心-30° ...
        # 【借鉴】_spiral() 的相位摆动思想，但将扫描方式改为扇区内摆动
        swing = (int(self.multi_angle_offset / self.multi_angle_step) % 2) * 30.0
        sign = 1.0 if (int(self.multi_angle_offset / self.multi_angle_step) % 2 == 0) else -1.0
        actual_angle = self.sector_center_deg + sign * (30.0 + swing)
        actual_angle %= 360.0

        next_point = point_on_circle(
            CENTER_LAT,
            CENTER_LON,
            self.multi_radius,
            actual_angle
        )

        # 更新状态
        self.multi_angle_offset += self.multi_angle_step
        if self.multi_angle_offset >= 360.0:
            self.multi_angle_offset = 0.0
            self.multi_radius += self.multi_radius_step
            # 【阶段二新增优化】超出最大半径后重置，实现多层覆盖
            # 官方 coop_distributed.py 无此逻辑，螺旋半径会无限增长
            if self.multi_radius > self.multi_max_radius:
                self.multi_radius = 800.0
                # 微调扇区中心，避免完全重复路径
                self.sector_center_deg = (self.sector_center_deg + 15.0) % 360.0

        return clamp_to_safebox(next_point[0], next_point[1])

    def get_next_waypoint(self) -> Tuple[float, float]:
        """获取下一个搜索航点（自动根据 multi_drone 切换）"""
        if self.multi_drone:
            return self._get_multi_waypoint()
        else:
            return self._get_single_waypoint()

    def generate_commands(
        self,
        uav_name: str,
        current_lat: float,
        current_lon: float
    ) -> list[Command]:
        """
        生成当前周期的控制命令。

        【官方代码借鉴】参考 coop_distributed.py 的 decide() 中：
        - 搜索态调用 fly_to() 飞向螺旋点 + point_gimbal() 扫描
        - 本方法仅生成 fly_to 命令，云台控制由上层或 tracker 模块负责
        """
        _ = uav_name  # 占位，预留多机扩展
        target_lat, target_lon = self.get_next_waypoint()
        # 【借鉴】coop_distributed.py 中搜索速度 = 22.0 m/s，这里取 24.0（调优表推荐）
        return [fly_to(target_lat, target_lon, alt=self.altitude, speed=24.0)]