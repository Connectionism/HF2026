"""
src/motion_control/geo.py
公共地理计算工具（全员依赖）
包含：距离/方位角/边界裁剪/瞄准线角度/盘旋点生成
基于 examples/uav_search_track_car/search_track/geometry.py 重构

【阶段二修复】2026-08-09：增加安全内缩边界（Safety Margin）
- 问题：原 clamp_to_safebox 直接裁剪到精确边界，固定翼因转弯惯性易飞出界
- 解决方案：参考 coop_distributed.py 的 _SAFEBOX_MARGIN_M = 600.0
- 将地图边界向内收缩 550 米，生成"安全活动区"
- 所有坐标裁剪到安全区内，为固定翼留足转弯余量
"""
import math

# 地球半径（米）
EARTH_RADIUS_M = 6_371_000.0

# 赛题二地图边界（来自参赛手册 §3 / scenario.json 综合）
LAT_MIN, LAT_MAX = 26.9818, 27.0250
LON_MIN, LON_MAX = 124.9800, 125.0203

_SAFETY_MARGIN_M = 550.0# 需要至少 500m 的安全余量。这里取 550m 略大于官方的 600m

def _bbox_inset(margin_m: float):
    """
    将地图边界向内收缩，生成安全边界
    【借鉴】coop_distributed.py 第 22-28 行 _bbox_inset() 函数
    """
    lat_mid = (LAT_MIN + LAT_MAX) / 2.0
    # 纬度 1° ≈ 111320 米（固定值）
    dlat = margin_m / 111320.0
    # 经度 1° ≈ 111320 * cos(lat) 米（随纬度变化）
    dlon = margin_m / (111320.0 * math.cos(math.radians(lat_mid)))
    return (LAT_MIN + dlat, LON_MIN + dlon), (LAT_MAX - dlat, LON_MAX - dlon)


# 计算安全边界（比原始地图小一圈，向内收缩 550 米）
(SAFE_LAT_MIN, SAFE_LON_MIN), (SAFE_LAT_MAX, SAFE_LON_MAX) = _bbox_inset(_SAFETY_MARGIN_M)

# 默认固定翼飞行高度（米）
DEFAULT_ALTITUDE = 500.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点之间的大圆距离（米）"""
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlat = rlat2 - rlat1
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算从点1到点2的方位角（度，0=正北，顺时针）"""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def clamp_to_safebox(lat: float, lon: float) -> tuple[float, float]:
    """
    裁剪经纬度到【安全活动区域】（内缩 550米），防止越界扣分
    参考：coop_distributed.py 的 _clamp_to_safebox 使用内缩后的 _SAFEBOX
    """
    return (max(SAFE_LAT_MIN, min(SAFE_LAT_MAX, lat)),
            max(SAFE_LON_MIN, min(SAFE_LON_MAX, lon)))



def los_angles(
    uav_lat: float,
    uav_lon: float,
    uav_alt: float,
    uav_yaw: float,
    tgt_lat: float,
    tgt_lon: float,
    tgt_alt: float = 0.0,
) -> tuple[float, float]:
    """
    计算云台瞄准目标所需的机体坐标系 (pan, tilt) 角度（度）
    - pan: 相对机头方向的偏航偏移（-180~180）
    - tilt: 俯仰角（负值=向下看）
    """
    brg = bearing_deg(uav_lat, uav_lon, tgt_lat, tgt_lon)
    d_h = haversine_m(uav_lat, uav_lon, tgt_lat, tgt_lon)
    # 垂直夹角（仰角）
    if d_h <= 1e-6:
        elv = 0.0
    else:
        elv = math.degrees(math.atan2(tgt_alt - uav_alt, d_h))
    pan = ((brg - uav_yaw + 540.0) % 360.0) - 180.0
    return pan, elv


def point_on_circle(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    angle_deg: float
) -> tuple[float, float]:
    """
    根据中心点、半径（米）和方位角（度），计算圆周上一点的经纬度。
    用于生成盘旋航点。
    """
    # 将距离转换为弧度（角距离）
    angular_dist = radius_m / EARTH_RADIUS_M
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