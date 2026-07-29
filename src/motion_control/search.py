"""
src/motion_control/search.py
扇区搜索：3 架 UAV 均分 360°，各自在扇区内做扩张螺旋扫描
依赖：geo.py, commands.py
"""
from typing import Optional
from .geo import (
    haversine_m, bearing_deg, clamp_to_safebox,
    DEFAULT_ALTITUDE, point_on_circle
)
from competition.sdk.core.commands import fly_to, Command

# 3 架无人机的 UID 与扇区中心方位角映射（基于 scenario.json 初始位置设计）
# 20001 在左侧 -> 扇区中心角 0°（正北偏西一点，负责西北区域）
# 20002 在右侧 -> 扇区中心角 120°
# 20003 在下方 -> 扇区中心角 240°
UID_TO_SECTOR_CENTER = {
    "20001": 0.0,
    "20002": 120.0,
    "20003": 240.0,
}


class SectorSearch:
    def __init__(self, my_uid: str, altitude: float = DEFAULT_ALTITUDE):
        self.my_uid = my_uid
        self.altitude = altitude
        self.sector_center_deg = UID_TO_SECTOR_CENTER.get(my_uid, 0.0)

        # 螺旋扩张参数
        self.arm_length_m = 1500.0      # 初始臂长（米）
        self.arm_increment = 500.0      # 每转一圈增加长度
        self.angle_step = 30.0          # 每次偏转角度（度）
        self.current_angle_offset = 0.0 # 当前在扇区内的偏移角度
        self.current_radius = self.arm_length_m
        self.search_center_lat = 27.0   # 搜索中心（地图中心附近）
        self.search_center_lon = 125.0

        # 是否已完成第一次起飞就位
        self.initialized = False

    def reset(self):
        """重置搜索状态（每局开始调用）"""
        self.current_angle_offset = 0.0
        self.current_radius = self.arm_length_m
        self.initialized = False

    def get_next_waypoint(self, current_lat: float, current_lon: float) -> tuple[float, float]:
        """
        计算下一个搜索目标点。
        实现扇区内的扩张螺旋：在扇区中心线左右摆动，逐步扩大半径。
        """
        if not self.initialized:
            # 首次：飞向扇区中心线附近的起点（避免扎堆）
            self.initialized = True
            start_angle = self.sector_center_deg - 30.0  # 从扇区左侧开始
            start_point = point_on_circle(
                self.search_center_lat,
                self.search_center_lon,
                self.arm_length_m,
                start_angle
            )
            return clamp_to_safebox(start_point[0], start_point[1])

        # 螺旋逻辑：在当前半径上，沿扇区中心线交替左右偏移
        # 例如：中心线 + 15°, 中心线 - 15°, 中心线 + 30°, 中心线 - 30° ...
        # 我们用 current_angle_offset 控制左右摆动幅度
        swing_amplitude = (int(self.current_angle_offset / 30.0) % 2) * 30.0
        if int(self.current_angle_offset / 30.0) % 2 == 0:
            sign = 1.0
        else:
            sign = -1.0

        # 实际方位角 = 扇区中心 + 摆动
        actual_angle = self.sector_center_deg + sign * (30.0 + swing_amplitude)
        actual_angle = (actual_angle + 360.0) % 360.0

        next_point = point_on_circle(
            self.search_center_lat,
            self.search_center_lon,
            self.current_radius,
            actual_angle
        )

        # 更新螺旋状态（步进）
        self.current_angle_offset += self.angle_step
        if self.current_angle_offset >= 360.0:
            self.current_angle_offset = 0.0
            self.current_radius += self.arm_increment

        return clamp_to_safebox(next_point[0], next_point[1])

    def generate_commands(self, current_lat: float, current_lon: float) -> list[Command]:
        """生成当前周期的 fly_to 命令（供 Agent 直接调用）"""
        target_lat, target_lon = self.get_next_waypoint(current_lat, current_lon)
        # 固定翼巡航速度 24 m/s（参数调优表建议）
        return [fly_to(target_lat, target_lon, alt=self.altitude, speed=24.0)]