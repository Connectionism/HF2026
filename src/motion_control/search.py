"""
src/motion_control/search.py
搜索模块：支持单机全域螺旋 和 多机扇区分区，通过 multi_drone 切换。
所有公共接口携带 uav_name，便于后续扩展。
"""
from typing import Tuple
from .geo import (
    clamp_to_safebox, point_on_circle,
    DEFAULT_ALTITUDE, LAT_MIN, LAT_MAX, LON_MIN, LON_MAX
)
from competition.sdk.core.commands import fly_to, Command

# 地图几何中心（用于螺旋搜索原点）
CENTER_LAT = (LAT_MIN + LAT_MAX) / 2.0   # ≈ 27.0034
CENTER_LON = (LON_MIN + LON_MAX) / 2.0   # ≈ 125.00015

# 多机模式：3 架 UAV 的扇区中心方位角（度）
# 根据 scenario.json 初始位置设计
UID_TO_SECTOR_CENTER = {
    "20001": 0.0,    # uav_alpha  -> 西北
    "20002": 120.0,  # uav_bravo  -> 东南
    "20003": 240.0,  # uav_charlie -> 西南
}


class SectorSearch:
    """
    搜索器：根据 multi_drone 标志自动切换单机/多机搜索策略。
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
        # 多机扇区参数（通常无需调整）
        sector_center_deg: float = None,  # 若为 None，自动从 UID_TO_SECTOR_CENTER 取
    ):
        self.uav_name = uav_name
        self.multi_drone = multi_drone
        self.altitude = altitude

        # 单机模式专用属性
        self.current_radius = init_radius
        self.current_angle = 0.0
        self.radius_step = radius_step
        self.angle_step = angle_step
        self.first_run = True

        # 多机模式专用属性
        if multi_drone:
            # 根据 uav_name 获取扇区中心角（若传入的 sector_center_deg 不为 None 则优先）
            if sector_center_deg is not None:
                self.sector_center_deg = sector_center_deg
            else:
                # 注意：uav_name 可能是 "uav_alpha" 或 "20001"，需要兼容
                # 尝试从 UID_TO_SECTOR_CENTER 中查找
                uid = uav_name if uav_name in UID_TO_SECTOR_CENTER else None
                if uid is None:
                    # 若 uav_name 是 "uav_alpha" 等，映射到对应的 uid
                    name_to_uid = {
                        "uav_alpha": "20001",
                        "uav_bravo": "20002",
                        "uav_charlie": "20003"
                    }
                    uid = name_to_uid.get(uav_name)
                self.sector_center_deg = UID_TO_SECTOR_CENTER.get(uid, 0.0)
            # 多机螺旋状态
            self.multi_radius = 1500.0
            self.multi_angle_offset = 0.0
        else:
            self.sector_center_deg = 0.0  # 占位

    def reset(self):
        """重置搜索状态（每局开始时调用）"""
        self.first_run = True
        self.current_radius = 300.0
        self.current_angle = 0.0
        if self.multi_drone:
            self.multi_radius = 1500.0
            self.multi_angle_offset = 0.0

    def _get_single_waypoint(self) -> Tuple[float, float]:
        """单机模式：全域扩张螺旋"""
        if self.first_run:
            self.first_run = False
            return clamp_to_safebox(CENTER_LAT + 0.0005, CENTER_LON)

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
        """多机模式：扇区内扩张螺旋（复用原始三机逻辑）"""
        # 类似之前的逻辑：在扇区内左右摆动，逐步扩大半径
        # 简化实现：使用与之前一致的算法
        swing_amplitude = (int(self.multi_angle_offset / 30.0) % 2) * 30.0
        if int(self.multi_angle_offset / 30.0) % 2 == 0:
            sign = 1.0
        else:
            sign = -1.0

        actual_angle = self.sector_center_deg + sign * (30.0 + swing_amplitude)
        actual_angle = (actual_angle + 360.0) % 360.0

        next_point = point_on_circle(
            CENTER_LAT,
            CENTER_LON,
            self.multi_radius,
            actual_angle
        )

        # 更新状态
        self.multi_angle_offset += 30.0
        if self.multi_angle_offset >= 360.0:
            self.multi_angle_offset = 0.0
            self.multi_radius += 500.0

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
        uav_name 用于标识，在单机模式下仅占位，多机模式下用于日志或验证。
        """
        _ = uav_name  # 占位，防止 lint 警告
        target_lat, target_lon = self.get_next_waypoint()
        return [fly_to(target_lat, target_lon, alt=self.altitude, speed=24.0)]