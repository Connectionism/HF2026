"""
搜索航点生成模块

来源: new drone_agent.py _spiral/_make_search_cmds/_get_multi_search_waypoint/_get_single_search_waypoint
功能: 螺旋搜索 + 网格蛇形扫描 + 扇区分区搜索
"""
from __future__ import annotations

import hashlib
import logging
import math
from typing import List, Tuple

from .geo import (
    clamp_to_safebox, haversine_m, bearing_deg, point_on_circle,
    partition_centers, uid_phase, uid_partition_idx, bbox_inset,
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


_BBOX: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (26.982, 124.980), (27.025, 125.020))
_SAFEBOX_MARGIN_M = 600.0
_SAFEBOX = bbox_inset(_BBOX, _SAFEBOX_MARGIN_M)
_PARTITION_CENTERS = partition_centers(_BBOX, 3)


class SearchController:
    """
    搜索航点生成控制器

    支持三种搜索模式：
    - 螺旋搜索 (_spiral)：从分区中心出发，螺旋扩张搜索
    - 多机网格扫描 (_get_multi_search_waypoint)：三机按经度条带蛇形扫描
    - 单机网格扫描 (_get_single_search_waypoint)：全域蛇形扫描
    """

    def __init__(
        self,
        my_uid: str,
        search_radius: float = 300.0,
        growth: float = 15.0,
        ang_speed: float = 30.0,
        sweep_period: float = 4.0,
        pitch_min: float = -60.0,
        pitch_max: float = -30.0,
        search_fov: float = 50.0,
        grid_scan_spacing_m: float = 400.0,
    ):
        self._my_uid = my_uid
        self._search_radius = search_radius
        self._growth = growth
        self._ang_speed = ang_speed
        self._sweep_period = sweep_period
        self._pitch_min = pitch_min
        self._pitch_max = pitch_max
        self._search_fov = search_fov
        self._grid_scan_spacing_m = grid_scan_spacing_m

        self._region = _PARTITION_CENTERS[uid_partition_idx(my_uid)]
        self._phase = uid_phase(my_uid)

        # 网格扫描运行时状态
        self._grid_scan_sector: Tuple[float, float, float, float] = (0, 0, 0, 0)
        self._grid_scan_phase: float = 0.0
        self._grid_scan_inited: bool = False

    def _spiral(self, t: float) -> Tuple[float, float, float, float]:
        """
        三方向螺旋搜索：从分区中心出发，以自转角度线性增长半径，
        到达最大搜索半径后不再向外扩散，向内循环往复搜索。

        返回 (lat, lon, pan, tilt)
        """
        home_lat, home_lon = self._region
        t = t + self._phase * 12.0

        bearing = (self._ang_speed * t) % 360.0
        revs = (self._ang_speed * t) / 360.0

        raw_radius = max(1.0, self._growth * revs)
        if raw_radius > self._search_radius:
            fold_count = int(raw_radius / self._search_radius)
            rem = raw_radius - fold_count * self._search_radius
            if fold_count % 2 == 0:
                radius = 1.0 + rem
            else:
                radius = self._search_radius - rem
            radius = max(1.0, min(self._search_radius, radius))
        else:
            radius = raw_radius

        dlat = (radius * math.cos(math.radians(bearing))) / 111320.0
        dlon = (radius * math.sin(math.radians(bearing))) / \
               (111320.0 * math.cos(math.radians(home_lat)))

        phase = (t % self._sweep_period) / self._sweep_period
        tilt = self._pitch_min + (self._pitch_max - self._pitch_min) * 0.5 * \
               (1 - math.cos(2 * math.pi * phase))

        pan_phase = (t % (self._sweep_period * 2)) / (self._sweep_period * 2)
        pan = -90.0 + 180.0 * 0.5 * (1 - math.cos(2 * math.pi * pan_phase))

        slat = home_lat + dlat
        slon = home_lon + dlon
        slat, slon = clamp_to_safebox(slat, slon)
        return slat, slon, pan, tilt

    def get_multi_search_waypoint(self, t: float) -> Tuple[float, float]:
        """
        多机模式：网格扫描（割草机/蛇形路径）。
        三机按经度均分地图条带，条带之间重叠 30%。
        """
        (lat_min, lon_min), (lat_max, lon_max) = _SAFEBOX
        n_uavs = 3
        speed = 22.0

        _FULL_SCAN_INTERVAL = 30.0
        _FULL_SCAN_DURATION = 4.0
        _full_scan_phase = int(t / _FULL_SCAN_INTERVAL)
        _full_scan_offset = t - _full_scan_phase * _FULL_SCAN_INTERVAL
        _use_full_scan = (_full_scan_offset < _FULL_SCAN_DURATION)

        if not self._grid_scan_inited:
            sector_width = (lon_max - lon_min) / n_uavs
            sector_idx = uid_partition_idx(self._my_uid)
            base_lon_min = lon_min + sector_width * sector_idx
            base_lon_max = lon_min + sector_width * (sector_idx + 1)
            overlap_extend = sector_width * 0.15
            self._grid_scan_sector = (
                lat_min,
                max(lon_min, base_lon_min - overlap_extend),
                lat_max,
                min(lon_max, base_lon_max + overlap_extend),
            )
            phase = uid_phase(self._my_uid)
            self._grid_scan_phase = phase
            self._grid_scan_inited = True

        if _use_full_scan:
            sec_lat_min, sec_lon_min = lat_min, lon_min
            sec_lat_max, sec_lon_max = lat_max, lon_max
        else:
            sec_lat_min, sec_lon_min, sec_lat_max, sec_lon_max = self._grid_scan_sector

        lat_mid = (sec_lat_min + sec_lat_max) / 2.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat_mid))
        sec_width_m = (sec_lon_max - sec_lon_min) * meters_per_deg_lon
        sec_height_m = (sec_lat_max - sec_lat_min) * 111320.0

        n_scanlines = max(2, int(sec_height_m / self._grid_scan_spacing_m))
        margin_m = sec_width_m * 0.03
        scan_width_m = sec_width_m - 2 * margin_m
        round_trip_time = 2 * scan_width_m / speed

        t_eff = t + self._grid_scan_phase * round_trip_time
        raw_line = t_eff / round_trip_time
        line_idx = int(raw_line)
        frac_in_line = raw_line - line_idx

        line_mod = line_idx % (2 * n_scanlines)
        if line_mod < n_scanlines:
            actual_line = line_mod
            going_north = True
        else:
            actual_line = 2 * n_scanlines - 1 - line_mod
            going_north = False

        frac_lat = actual_line / max(1, n_scanlines - 1)
        current_lat = sec_lat_min + frac_lat * (sec_lat_max - sec_lat_min)

        if going_north:
            current_lon_offset = margin_m + scan_width_m * (1.0 - frac_in_line)
        else:
            current_lon_offset = margin_m + scan_width_m * frac_in_line

        current_lon_deg = sec_lon_min + current_lon_offset / meters_per_deg_lon

        lookahead_s = 2.0
        ahead_frac = frac_in_line + lookahead_s / round_trip_time
        if going_north:
            ahead_lon_offset = margin_m + scan_width_m * max(0.0, 1.0 - ahead_frac)
        else:
            ahead_lon_offset = margin_m + scan_width_m * min(1.0, ahead_frac)
        ahead_lon_deg = sec_lon_min + ahead_lon_offset / meters_per_deg_lon

        return clamp_to_safebox(current_lat, ahead_lon_deg)

    def get_single_search_waypoint(self, t: float) -> Tuple[float, float]:
        """单机模式：全域网格扫描（基于全局时间 t 连续运动）。"""
        (lat_min, lon_min), (lat_max, lon_max) = _SAFEBOX
        speed = 22.0
        lat_mid = (lat_min + lat_max) / 2.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat_mid))
        map_width_m = (lon_max - lon_min) * meters_per_deg_lon
        map_height_m = (lat_max - lat_min) * 111320.0

        n_scanlines = max(2, int(map_height_m / self._grid_scan_spacing_m))
        margin_m = map_width_m * 0.10
        scan_width_m = map_width_m - 2 * margin_m
        round_trip_time = 2 * scan_width_m / speed

        raw_line = t / round_trip_time
        line_idx = int(raw_line)
        frac_in_line = raw_line - line_idx

        line_mod = line_idx % (2 * n_scanlines)
        if line_mod < n_scanlines:
            actual_line = line_mod
            going_north = True
        else:
            actual_line = 2 * n_scanlines - 1 - line_mod
            going_north = False

        frac_lat = actual_line / max(1, n_scanlines - 1)
        current_lat = lat_min + frac_lat * (lat_max - lat_min)

        if going_north:
            current_lon_offset = margin_m + scan_width_m * (1.0 - frac_in_line)
        else:
            current_lon_offset = margin_m + scan_width_m * frac_in_line
        current_lon_deg = lon_min + current_lon_offset / meters_per_deg_lon

        lookahead_s = 2.0
        ahead_frac = frac_in_line + lookahead_s / round_trip_time
        if going_north:
            ahead_lon_offset = margin_m + scan_width_m * max(0.0, 1.0 - ahead_frac)
        else:
            ahead_lon_offset = margin_m + scan_width_m * min(1.0, ahead_frac)
        ahead_lon_deg = lon_min + ahead_lon_offset / meters_per_deg_lon

        return clamp_to_safebox(current_lat, ahead_lon_deg)

    def make_search_cmds(self, t: float) -> List[Command]:
        """生成搜索命令列表：螺旋搜索 + 云台扫描。"""
        slat, slon, pan, tilt = self._spiral(t)
        return [
            fly_to(slat, slon, speed=22.0),
            point_gimbal(pan, tilt),
            set_gimbal_fov(self._search_fov),
        ]

    def make_idle_cmds(self, center_lat: float, center_lon: float) -> List[Command]:
        """全歼后的低功耗模式：低速大半径盘旋。"""
        return [
            fly_to(center_lat, center_lon, speed=18.0, loiter_radius=500.0),
            point_gimbal(0.0, -45.0),
            set_gimbal_fov(self._search_fov),
        ]

    def reset(self) -> None:
        self._grid_scan_sector = (0, 0, 0, 0)
        self._grid_scan_phase = 0.0
        self._grid_scan_inited = False
