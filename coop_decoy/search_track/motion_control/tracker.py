"""
src/motion_control/tracker.py
跟踪模块：支持单机动态绕圈 和 多机固定站位（阶段二优化）

【官方代码借鉴说明】
- 单机绕圈逻辑 → 参考 coop_distributed.py 的 _tracking_gimbal() + fly_to 组合
- 多机站位策略 → 参考 swarm_coordinated.py 的 _team_aim_point() 方法
- 云台瞄准（pan/tilt计算）→ 参考 coop_distributed.py 的 _tracking_gimbal() 方法
- 云台角度平滑 → 阶段二新增，官方无此机制（参考 EMA 滤波思想）
"""
from typing import Optional, Tuple
from .geo import (
    haversine_m, bearing_deg, los_angles, point_on_circle,
    DEFAULT_ALTITUDE, clamp_to_safebox,
    LAT_MIN, LAT_MAX, LON_MIN, LON_MAX  # 用于场景中心计算
)
try:
    from sdk.core.commands import fly_to, point_gimbal, Command
except ImportError:
    try:
        from competition.sdk.core.commands import fly_to, point_gimbal, Command
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

        def point_gimbal(pan, tilt):
            return Command("component.gimbal_tracking.set_orientation", {"pan": float(pan), "tilt": float(tilt)})


class LoiterTracker:
    """
    跟踪器：根据 multi_drone 切换单目标 / 双槽位跟踪。
   【官方代码借鉴说明】
    - 整体结构 → 参考 coop_distributed.py 中 TRACK 状态的处理逻辑
    - 多机站位 → 参考 swarm_coordinated.py 的 _team_aim_point() 方法（核心借鉴）
    """

    def __init__(
        self,
        uav_name: str,
        multi_drone: bool = False,
        radius_m: float = 350.0,          # 单机盘旋半径（米）
        altitude: float = DEFAULT_ALTITUDE,
        speed: float = 24.0,
        turn_direction: str = "right",    # 单机盘旋方向 "right"/"left"
        multi_radius: float = 330.0,      # 多机跟踪环半径（确保 2*radius > 200m）
        filter_alpha: float = 0.3,        # 云台角度平滑系数
    ):
        self.uav_name = uav_name
        self.multi_drone = multi_drone
        self.altitude = altitude
        self.speed = speed

        # 单机模式属性
        self.radius_m = radius_m
        self.turn_direction = turn_direction

        # 多机模式属性
        # 【借鉴】swarm_coordinated.py 中 _TRACK_LOITER = 330.0
        self.multi_radius = multi_radius
        self.filter_alpha = filter_alpha

        # 当前跟踪目标
        self.current_target: Optional[Tuple[float, float]] = None
        self.slot = 0

        # 云台平滑状态
        self._last_pan = 0.0
        self._last_tilt = 0.0
        self._initialized = False

    def reset(self):
        """重置跟踪状态"""
        self.current_target = None
        self.slot = 0
        self._initialized = False
        self._last_pan = 0.0
        self._last_tilt = 0.0

    def set_target(self, target_lat: float, target_lon: float, slot: int = 0):
        """设置要跟踪的目标及槽位（多机模式下 slot 有效）"""
        self.current_target = (target_lat, target_lon)
        if self.multi_drone:
            self.slot = slot
        else:
            self.slot = 0

    def clear_target(self):
        """释放当前目标"""
        self.current_target = None

    def is_active(self) -> bool:
        return self.current_target is not None

    def _get_single_loiter_waypoint(
        self,
        uav_lat: float,
        uav_lon: float
    ) -> Optional[Tuple[float, float]]:
        """单机模式：动态顺时针/逆时针绕圈

        【官方代码借鉴】参考 coop_distributed.py 的 TRACK 状态：
        - fly_to(tgt, loiter_radius=100) 实现在目标附近绕圈
        - 这里用 point_on_circle 计算盘旋点，实现更精确的圆形轨迹
        """
        if self.current_target is None:
            return None
        tgt_lat, tgt_lon = self.current_target
        brg_from_target = bearing_deg(tgt_lat, tgt_lon, uav_lat, uav_lon)
        offset = 90.0 if self.turn_direction == "right" else -90.0
        loiter_angle = (brg_from_target + offset) % 360.0
        wp_lat, wp_lon = point_on_circle(tgt_lat, tgt_lon, self.radius_m, loiter_angle)
        return clamp_to_safebox(wp_lat, wp_lon)

    def _get_multi_loiter_waypoint(
        self,
        uav_lat: float,
        uav_lon: float
    ) -> Optional[Tuple[float, float]]:
        """
        多机模式：基于目标到场景中心的方位角分配槽位。
        K=2 时 slot0 偏移 0°，slot1 偏移 180°，确保两机间距为 2*radius。

        【核心借鉴】swarm_coordinated.py 的 _team_aim_point() 方法
        """
        if self.current_target is None:
            return None
        tgt_lat, tgt_lon = self.current_target

        # 场景几何中心
        #【借鉴】swarm_coordinated.py 的 _team_aim_point() 
        scene_center_lat = (LAT_MIN + LAT_MAX) / 2.0
        scene_center_lon = (LON_MIN + LON_MAX) / 2.0

        # 目标到场景中心的方位角（作为基准方向）
        sector_base = bearing_deg(tgt_lat, tgt_lon, scene_center_lat, scene_center_lon)

        # K=2: slot0 -> 0°, slot1 -> 180°
        offset = 0.0 if self.slot == 0 else 180.0
        loiter_angle = (sector_base + offset) % 360.0

        wp_lat, wp_lon = point_on_circle(tgt_lat, tgt_lon, self.multi_radius, loiter_angle)
        return clamp_to_safebox(wp_lat, wp_lon)

    def get_loiter_waypoint(
        self,
        uav_lat: float,
        uav_lon: float
    ) -> Optional[Tuple[float, float]]:
        """根据 multi_drone 选择盘旋点计算方式"""
        if self.multi_drone:
            return self._get_multi_loiter_waypoint(uav_lat, uav_lon)
        else:
            return self._get_single_loiter_waypoint(uav_lat, uav_lon)

    def generate_commands(
        self,
        uav_name: str,
        uav_lat: float,
        uav_lon: float,
        uav_alt: float,
        uav_yaw: float
    ) -> list[Command]:
        """
        生成控制命令：导航到盘旋点 + 云台瞄准目标（带平滑滤波）。
        uav_name 用于标识（当前仅占位）。

        【官方代码借鉴】参考 coop_distributed.py 的 TRACK 状态：
        1. fly_to() 飞向盘旋点（loiter_radius 参数在 fly_to 中设置）
        2. point_gimbal() 瞄准目标
        3. 持续 report_target() 上报（本模块仅生成云台命令，上报由上层处理）
        """
        _ = uav_name  # 占位
        if self.current_target is None:
            return []

        commands = []

        # 1. 导航命令
        # 【借鉴】coop_distributed.py TRACK 状态中的 fly_to(tgt, loiter_radius=...)
        wp = self.get_loiter_waypoint(uav_lat, uav_lon)
        if wp is not None:
            commands.append(fly_to(wp[0], wp[1], alt=self.altitude, speed=self.speed))

        # 2. 云台瞄准命令（带一阶低通滤波）
        tgt_lat, tgt_lon = self.current_target
        pan_raw, tilt_raw = los_angles(
            uav_lat, uav_lon, uav_alt, uav_yaw,
            tgt_lat, tgt_lon, tgt_alt=0.0
        )

        if not self._initialized:
            pan_smooth = pan_raw
            tilt_smooth = tilt_raw
            self._initialized = True
        else:
            alpha = self.filter_alpha
            pan_smooth = alpha * pan_raw + (1 - alpha) * self._last_pan
            tilt_smooth = alpha * tilt_raw + (1 - alpha) * self._last_tilt

        self._last_pan = pan_smooth
        self._last_tilt = tilt_smooth
        commands.append(point_gimbal(pan_smooth, tilt_smooth))

        return commands