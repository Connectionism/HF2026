"""
motion_control 地理工具模块

来源: new drone_agent.py 第 55-169 行
功能: 距离计算、方位角、边界钳位、圆周点生成、扇区分区、UID映射
"""
from __future__ import annotations

import hashlib
import math
from typing import Tuple

# ── 地图边界常量 ──
_BBOX: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (26.982, 124.980), (27.025, 125.020))
_SAFEBOX_MARGIN_M = 600.0
_EARTH_RADIUS_M = 6_371_000.0


def bbox_inset(bbox, margin_m: float):
    """从边界框向内收缩指定距离（米），得到安全飞行区域。"""
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    lat_mid = (lat_min + lat_max) / 2
    dlat = margin_m / 111320.0
    dlon = margin_m / (111320.0 * math.cos(math.radians(lat_mid)))
    return ((lat_min + dlat, lon_min + dlon), (lat_max - dlat, lon_max - dlon))


_SAFEBOX = bbox_inset(_BBOX, _SAFEBOX_MARGIN_M)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """使用 Haversine 公式计算两点间距离（米）。"""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算从点1到点2的绝对方位角（0°=北，顺时针）。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = (math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def clamp_to_safebox(lat: float, lon: float) -> Tuple[float, float]:
    """将坐标限制在安全飞行区域内。"""
    (lat_min, lon_min), (lat_max, lon_max) = _SAFEBOX
    return (min(max(lat, lat_min), lat_max),
            min(max(lon, lon_min), lon_max))


def point_on_circle(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    angle_deg: float
) -> Tuple[float, float]:
    """
    根据中心点、半径（米）和方位角（度），计算圆周上一点的经纬度。
    用于生成盘旋航点。
    """
    angular_dist = radius_m / _EARTH_RADIUS_M
    brg_rad = math.radians(angle_deg)
    lat1_rad = math.radians(center_lat)
    lon1_rad = math.radians(center_lon)

    lat2_rad = math.asin(
        math.sin(lat1_rad) * math.cos(angular_dist)
        + math.cos(lat1_rad) * math.sin(angular_dist) * math.cos(brg_rad)
    )
    lon2_rad = lon1_rad + math.atan2(
        math.sin(brg_rad) * math.sin(angular_dist) * math.cos(lat1_rad),
        math.cos(angular_dist) - math.sin(lat1_rad) * math.sin(lat2_rad)
    )
    return math.degrees(lat2_rad), math.degrees(lon2_rad)


def partition_centers(bbox, n: int = 3):
    """将边界框按经度均分为 n 个分区，返回各分区中心坐标列表。"""
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    lat_mid = (lat_min + lat_max) / 2
    sub_w = (lon_max - lon_min) / n
    return [(lat_mid, lon_min + sub_w * (i + 0.5)) for i in range(n)]


_PARTITION_CENTERS = partition_centers(_BBOX, 3)


def uid_phase(uid: str) -> float:
    """根据 UID 生成相位偏移（0～1），用于搜索轨迹差异化。"""
    h = int(hashlib.md5(uid.encode()).hexdigest(), 16)
    return (h % 1000) / 1000.0


def uid_partition_idx(uid: str) -> int:
    """将 UID 映射到条带索引 0/1/2（纯数字索引）。"""
    n = 3
    if uid.isdigit():
        return int(uid) % n
    elif "_" in uid:
        tail = uid.rsplit("_", 1)[-1]
        return int(tail) % n if tail.isdigit() else (
            int(hashlib.md5(uid.encode()).hexdigest(), 16) % n)
    else:
        return int(hashlib.md5(uid.encode()).hexdigest(), 16) % n


def los_angles(uav_lat: float, uav_lon: float, uav_alt: float,
               uav_yaw: float, tgt_lat: float, tgt_lon: float,
               tgt_alt: float = 0.0) -> Tuple[float, float]:
    """
    计算云台瞄准目标所需的机体坐标系 (pan, tilt) 角度（度）
    - pan: 相对机头方向的偏航偏移（-180~180）
    - tilt: 俯仰角（负值=向下看）
    """
    brg = bearing_deg(uav_lat, uav_lon, tgt_lat, tgt_lon)
    d_h = haversine_m(uav_lat, uav_lon, tgt_lat, tgt_lon)
    if d_h <= 1e-6:
        elv = 0.0
    else:
        elv = math.degrees(math.atan2(tgt_alt - uav_alt, d_h))
    pan = ((brg - uav_yaw + 540.0) % 360.0) - 180.0
    return pan, elv
