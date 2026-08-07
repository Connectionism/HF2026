"""
src/motion_control/tracker.py
跟踪模块：支持单机单目标环绕 和 多机双槽位协同，通过 multi_drone 切换。
所有公共接口携带 uav_name。
"""
from typing import Optional, Tuple
from .geo import (
    haversine_m, bearing_deg, los_angles, point_on_circle,
    DEFAULT_ALTITUDE, clamp_to_safebox
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
    """

    def __init__(
        self,
        uav_name: str,
        multi_drone: bool = False,
        radius_m: float = 350.0,         # 单机盘旋半径（米）
        altitude: float = DEFAULT_ALTITUDE,
        speed: float = 24.0,
        turn_direction: str = "right",   # 单机盘旋方向 "right"/"left"
        # 多机模式参数
        multi_radius: float = 330.0,     # 多机跟踪环半径（确保 2*radius > 200m）
    ):
        self.uav_name = uav_name
        self.multi_drone = multi_drone
        self.altitude = altitude
        self.speed = speed

        # 单机模式属性
        self.radius_m = radius_m
        self.turn_direction = turn_direction

        # 多机模式属性
        self.multi_radius = multi_radius
        self.slot = 0  # 0 或 1，由调度模块设置

        # 当前跟踪的目标（None 表示未激活）
        self.current_target: Optional[Tuple[float, float]] = None

    def reset(self):
        """重置跟踪状态"""
        self.current_target = None
        self.slot = 0

    def set_target(self, target_lat: float, target_lon: float, slot: int = 0):
        """
        设置要跟踪的目标坐标及槽位（多机模式下 slot 有效）。
        单机模式下 slot 参数被忽略。
        """
        self.current_target = (target_lat, target_lon)
        if self.multi_drone:
            self.slot = slot
        else:
            self.slot = 0  # 单机下固定

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
        """单机模式：动态顺时针/逆时针绕圈"""
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
        """多机模式：根据 slot 计算固定方位盘旋点（保持两机夹角180°）"""
        if self.current_target is None:
            return None
        tgt_lat, tgt_lon = self.current_target
        # 计算当前 UAV 相对于目标的方向角
        brg = bearing_deg(tgt_lat, tgt_lon, uav_lat, uav_lon)
        if self.slot == 0:
            loiter_angle = (brg + 90.0) % 360.0
        else:
            loiter_angle = (brg - 90.0) % 360.0
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
        生成控制命令：导航到盘旋点 + 云台瞄准目标。
        uav_name 用于标识（当前仅占位）。
        """
        _ = uav_name  # 占位
        if self.current_target is None:
            return []

        commands = []
        wp = self.get_loiter_waypoint(uav_lat, uav_lon)
        if wp is not None:
            commands.append(fly_to(wp[0], wp[1], alt=self.altitude, speed=self.speed))

        tgt_lat, tgt_lon = self.current_target
        pan, tilt = los_angles(
            uav_lat, uav_lon, uav_alt, uav_yaw,
            tgt_lat, tgt_lon, tgt_alt=0.0
        )
        commands.append(point_gimbal(pan, tilt))
        return commands