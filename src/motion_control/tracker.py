"""
src/motion_control/tracker.py
跟踪盘旋（Loiter）与云台瞄准控制
支持 K=2 协同中的双机站位（SLOT_0 / SLOT_1）
"""
from typing import Optional, Tuple
from .geo import (
    haversine_m, bearing_deg, los_angles, point_on_circle,
    DEFAULT_ALTITUDE, clamp_to_safebox
)
from competition.sdk.core.commands import fly_to, point_gimbal, Command


class LoiterTracker:
    def __init__(
        self,
        radius_m: float = 330.0,          # 跟踪环半径（确保 2*radius > 200m 避免扣分）
        altitude: float = DEFAULT_ALTITUDE,
        speed: float = 24.0
    ):
        self.radius_m = radius_m
        self.altitude = altitude
        self.speed = speed
        # 当前跟踪的目标坐标
        self.current_target: Optional[Tuple[float, float]] = None
        # 分配的槽位 (0 或 1)，由调度模块设置
        self.slot: int = 0

    def reset(self):
        """重置跟踪状态"""
        self.current_target = None
        self.slot = 0

    def set_target(self, target_lat: float, target_lon: float, slot: int = 0):
        """设置要跟踪的目标和槽位（由调度模块调用）"""
        self.current_target = (target_lat, target_lon)
        self.slot = slot

    def clear_target(self):
        """释放当前目标（摧毁或放弃）"""
        self.current_target = None

    def _get_loiter_angle(self, uav_lat: float, uav_lon: float) -> float:
        """
        计算 UAV 在目标周围的盘旋方位角。
        SLOT_0: 目标方位角 + 90°（顺时针偏移）
        SLOT_1: 目标方位角 - 90°（逆时针偏移）
        确保两机夹角 180°，间距 = 2 * radius = 660m > 200m
        """
        if self.current_target is None:
            return 0.0
        tgt_lat, tgt_lon = self.current_target
        brg = bearing_deg(tgt_lat, tgt_lon, uav_lat, uav_lon)
        if self.slot == 0:
            return (brg + 90.0) % 360.0
        else:
            return (brg - 90.0) % 360.0

    def get_loiter_waypoint(self, uav_lat: float, uav_lon: float) -> Optional[Tuple[float, float]]:
        """获取当前周期的盘旋目标航点"""
        if self.current_target is None:
            return None
        tgt_lat, tgt_lon = self.current_target
        angle = self._get_loiter_angle(uav_lat, uav_lon)
        wp_lat, wp_lon = point_on_circle(tgt_lat, tgt_lon, self.radius_m, angle)
        return clamp_to_safebox(wp_lat, wp_lon)

    def generate_commands(
        self,
        uav_lat: float,
        uav_lon: float,
        uav_alt: float,
        uav_yaw: float
    ) -> list[Command]:
        """
        生成当前周期的控制命令：
        1. fly_to 盘旋点
        2. point_gimbal 锁定目标
        """
        if self.current_target is None:
            return []

        # 1. 导航命令
        wp = self.get_loiter_waypoint(uav_lat, uav_lon)
        if wp is None:
            return []
        commands = [fly_to(wp[0], wp[1], alt=self.altitude, speed=self.speed)]

        # 2. 云台瞄准命令
        tgt_lat, tgt_lon = self.current_target
        pan, tilt = los_angles(uav_lat, uav_lon, uav_alt, uav_yaw, tgt_lat, tgt_lon)
        commands.append(point_gimbal(pan, tilt))

        return commands