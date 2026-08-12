"""
盘旋跟踪模块

来源: new drone_agent.py _get_single_loiter_waypoint/_get_multi_loiter_waypoint/_los_angles
功能: 单机/多机盘旋跟踪航点 + 云台瞄准角度计算
"""
from __future__ import annotations

import hashlib
import logging
import math
from typing import List, Optional, Tuple

from .geo import (
    clamp_to_safebox, haversine_m, bearing_deg, point_on_circle, los_angles,
)

logger = logging.getLogger(__name__)

try:
    from competition.sdk.core.commands import Command, fly_to, point_gimbal, set_gimbal_fov
except ImportError:
    logger.warning(
        "⚠️ SDK运动指令导入失败，启用本地模拟运动指令！"
        "仅用于本地调试，竞赛环境不应触发此分支"
    )
    from dataclasses import dataclass

    @dataclass(frozen=True)
    class Command:
        verb: str
        params: dict

    def fly_to(lat: float, lon: float, alt: float = None, speed: float = None,
               loiter_radius: float = 200.0, turn_direction: str = "right") -> Command:
        """模拟 SDK fly_to 命令（仅本地调试）。"""
        params = {
            "latitude": float(lat),
            "longitude": float(lon),
            "loiter_radius": float(loiter_radius),
        }
        if alt is not None:
            params["alt"] = float(alt)
        if speed is not None:
            params["speed"] = float(speed)
        return Command("set_destination", params)

    def point_gimbal(pan: float, tilt: float) -> Command:
        """模拟 SDK point_gimbal 命令（仅本地调试）。"""
        return Command(
            "component.gimbal_tracking.set_orientation",
            {"pan": float(pan), "tilt": float(tilt)},
        )

    def set_gimbal_fov(fov: float) -> Command:
        """模拟 SDK set_gimbal_fov 命令（仅本地调试）。"""
        return Command("set_fov", {"angle": float(fov)})


class TrackController:
    """
    盘旋跟踪控制器

    支持两种模式：
    - 单机盘旋：动态顺时针/逆时针绕目标
    - 多机盘旋：基于目标 hash 的固定方位角，两机对向分布
    """

    def __init__(
        self,
        my_uid: str,
        loiter_radius: float = 450.0,
        multi_loiter_radius: float = 450.0,
        loiter_close: float = 250.0,
        turn_direction: str = "right",
        track_fov: float = 30.0,
    ):
        self._my_uid = my_uid
        self._loiter_radius = loiter_radius
        self._multi_loiter_radius = multi_loiter_radius
        self._loiter_close = loiter_close
        self._turn_direction = turn_direction
        self._track_fov = track_fov

    def get_single_loiter_waypoint(
        self, uav_lat: float, uav_lon: float,
        tgt_lat: float, tgt_lon: float
    ) -> Tuple[float, float]:
        """
        单机模式：动态顺时针/逆时针绕目标盘旋。
        """
        brg_from_target = bearing_deg(tgt_lat, tgt_lon, uav_lat, uav_lon)
        offset = 90.0 if self._turn_direction == "right" else -90.0
        loiter_angle = (brg_from_target + offset) % 360.0
        wp_lat, wp_lon = point_on_circle(
            tgt_lat, tgt_lon, self._loiter_radius, loiter_angle)
        return clamp_to_safebox(wp_lat, wp_lon)

    def get_multi_loiter_waypoint(
        self, uav_lat: float, uav_lon: float,
        tgt_lat: float, tgt_lon: float, slot: int
    ) -> Tuple[float, float]:
        """
        多机模式：用目标绝对坐标 hash 计算固定方位盘旋点。
        两机基于同一目标算出 0° 和 180° 的固定方位，始终保持对向。
        """
        tgt_key = f"{tgt_lat:.5f}_{tgt_lon:.5f}"
        h = int(hashlib.md5(tgt_key.encode()).hexdigest(), 16)
        base_angle = float(h % 360)

        loiter_angle = (base_angle + slot * 180.0) % 360.0

        wp_lat, wp_lon = point_on_circle(
            tgt_lat, tgt_lon, self._multi_loiter_radius, loiter_angle)
        return clamp_to_safebox(wp_lat, wp_lon)

    def get_track_commands(
        self,
        uav_lat: float, uav_lon: float, uav_alt: float, uav_yaw: float,
        tgt_lat: float, tgt_lon: float,
        slot: int = 0,
        multi_search: bool = True,
        tracking: bool = True,
    ) -> List[Command]:
        """
        生成跟踪命令：飞向盘旋点 + 云台瞄准目标。

        Args:
            uav_lat/uav_lon/uav_alt/uav_yaw: 本机位姿
            tgt_lat/tgt_lon: 目标位置
            slot: 多机槽位 (0 或 1)
            multi_search: 是否多机模式
            tracking: 是否正在跟踪（影响逼近速度）
        """
        if multi_search:
            aim_lat, aim_lon = self.get_multi_loiter_waypoint(
                uav_lat, uav_lon, tgt_lat, tgt_lon, slot)
        else:
            aim_lat, aim_lon = self.get_single_loiter_waypoint(
                uav_lat, uav_lon, tgt_lat, tgt_lon)
        aim_lat, aim_lon = clamp_to_safebox(aim_lat, aim_lon)

        approach_speed = 30.0 if not tracking else 22.0

        cmds: List[Command] = []
        cmds.append(fly_to(aim_lat, aim_lon, speed=approach_speed,
                           loiter_radius=self._loiter_close))

        pan, tilt = los_angles(
            uav_lat, uav_lon, uav_alt, uav_yaw, tgt_lat, tgt_lon)
        cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(self._track_fov))

        return cmds

    def get_verify_commands(
        self,
        uav_lat: float, uav_lon: float, uav_alt: float, uav_yaw: float,
        tgt_lat: float, tgt_lon: float,
        search_fov: float = 50.0,
    ) -> List[Command]:
        """生成验证状态命令：飞向目标 + 云台对准（大FOV）。"""
        tlat, tlon = clamp_to_safebox(tgt_lat, tgt_lon)
        pan, tilt = los_angles(
            uav_lat, uav_lon, uav_alt, uav_yaw, tgt_lat, tgt_lon)
        return [
            fly_to(tlat, tlon, speed=22.0, loiter_radius=self._loiter_close),
            point_gimbal(pan, tilt),
            set_gimbal_fov(search_fov),
        ]

    def reset(self) -> None:
        pass
