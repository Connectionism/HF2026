"""
通信协议编解码模块

七类消息格式（全部 ≤50 字节，UTF-8）：
    T: — 目标确认 (Target)
    D: — 诱饵确认 (Decoy)
    A: — 集结请求 (Assemble)
    C: — 确认应答 (Confirm)
    R: — 解除/取消 (Release)
    J: — 干扰报告 (Jamming)
    H: — 心跳存活 (Heartbeat)

编码方案（简单版）：坐标直接写小数，代码简单，29 字节左右
如需压缩，可用整数偏移版（17 字节），两套 API 都提供
"""

from dataclasses import dataclass
from typing import Optional, Union

from . import config

# ==========================================================================
# 消息类型常量
# ==========================================================================

MSG_TYPE_TARGET    = 'T'
MSG_TYPE_DECOY     = 'D'
MSG_TYPE_ASSEMBLE  = 'A'
MSG_TYPE_CONFIRM   = 'C'
MSG_TYPE_RELEASE   = 'R'
MSG_TYPE_JAMMING   = 'J'
MSG_TYPE_HEARTBEAT = 'H'

# ==========================================================================
# 消息数据类
# ==========================================================================


@dataclass(frozen=True)
class TargetMsg:
    """T: 发现真目标，请求协同"""
    tid:  str          # 目标编号 a~z
    lat:  float        # 纬度
    lon:  float        # 经度
    conf: float        # 置信度 0.0~1.0


@dataclass(frozen=True)
class DecoyMsg:
    """D: 确认诱饵，通知队友别来"""
    lat: float
    lon: float


@dataclass(frozen=True)
class AssembleMsg:
    """A: 请求队友来这个位置集合"""
    lat:    float
    lon:    float
    reason: str         # 't'=协同跟踪, 's'=扇区搜索, 'h'=待命


@dataclass(frozen=True)
class ConfirmMsg:
    """C: 收到队友消息，回复确认"""
    ref_type: str       # 'T', 'D', 'A' — 回应哪种消息
    ref_id:   str       # 目标 id 或坐标摘要


@dataclass(frozen=True)
class ReleaseMsg:
    """R: 目标已处理 / 误判，解除盯防"""
    target: str         # 目标 id 或 'A' (取消集结)


@dataclass(frozen=True)
class JammingMsg:
    """J: 通信干扰状态报告"""
    state: str          # 'on'=进入干扰区, 'off'=脱离干扰区


@dataclass(frozen=True)
class HeartbeatMsg:
    """H: 心跳存活广播"""
    status: str         # 'a'=alive, 'd'=destroyed


MsgType = Union[
    TargetMsg, DecoyMsg, AssembleMsg, ConfirmMsg, ReleaseMsg,
    JammingMsg, HeartbeatMsg,
]

# ==========================================================================
# 编码（简单版：小数坐标）
# ==========================================================================


def encode(msg: MsgType) -> str:
    """消息对象 → 字符串（简单版，直接用小数坐标）

    示例:
        encode(TargetMsg('a', 27.01234, 125.03456, 0.85))
        → "T:a,27.01234,125.03456,85"
    """
    if isinstance(msg, TargetMsg):
        return f"T:{msg.tid},{msg.lat:.{config.COORD_DECIMALS}f},{msg.lon:.{config.COORD_DECIMALS}f},{int(msg.conf * 100)}"
    elif isinstance(msg, DecoyMsg):
        return f"D:{msg.lat:.{config.COORD_DECIMALS}f},{msg.lon:.{config.COORD_DECIMALS}f}"
    elif isinstance(msg, AssembleMsg):
        return f"A:{msg.lat:.{config.COORD_DECIMALS}f},{msg.lon:.{config.COORD_DECIMALS}f},{msg.reason}"
    elif isinstance(msg, ConfirmMsg):
        return f"C:{msg.ref_type},{msg.ref_id}"
    elif isinstance(msg, ReleaseMsg):
        return f"R:{msg.target}"
    elif isinstance(msg, JammingMsg):
        return f"J:{msg.state}"
    elif isinstance(msg, HeartbeatMsg):
        return f"H:{msg.status}"
    else:
        raise ValueError(f"不支持的消息类型: {type(msg)}")


# ==========================================================================
# 编码（压缩版：整数偏移）
# ==========================================================================


def _lat_to_int(lat: float) -> int:
    """纬度 → 整数偏移"""
    return round((lat - config.BASE_LAT) * config.COORD_SCALE)


def _lon_to_int(lon: float) -> int:
    """经度 → 整数偏移"""
    return round((lon - config.BASE_LON) * config.COORD_SCALE)


def _int_to_lat(v: int) -> float:
    """整数偏移 → 纬度"""
    return config.BASE_LAT + v / config.COORD_SCALE


def _int_to_lon(v: int) -> float:
    """整数偏移 → 经度"""
    return config.BASE_LON + v / config.COORD_SCALE


def encode_compact(msg: MsgType) -> str:
    """消息对象 → 字符串（压缩版，整数偏移，约 17 字节）

    示例:
        encode_compact(TargetMsg('a', 27.01234, 125.03456, 0.85))
        → "T:a,1234,3456,85"
    """
    if isinstance(msg, TargetMsg):
        return f"T:{msg.tid},{_lat_to_int(msg.lat)},{_lon_to_int(msg.lon)},{int(msg.conf * 100)}"
    elif isinstance(msg, DecoyMsg):
        return f"D:{_lat_to_int(msg.lat)},{_lon_to_int(msg.lon)}"
    elif isinstance(msg, AssembleMsg):
        return f"A:{_lat_to_int(msg.lat)},{_lon_to_int(msg.lon)},{msg.reason}"
    elif isinstance(msg, ConfirmMsg):
        return f"C:{msg.ref_type},{msg.ref_id}"
    elif isinstance(msg, ReleaseMsg):
        return f"R:{msg.target}"
    elif isinstance(msg, JammingMsg):
        return f"J:{msg.state}"
    elif isinstance(msg, HeartbeatMsg):
        return f"H:{msg.status}"
    else:
        raise ValueError(f"不支持的消息类型: {type(msg)}")


# ==========================================================================
# 解码
# ==========================================================================


def decode(payload: str, compact: bool = False) -> Optional[MsgType]:
    """字符串 → 消息对象

    参数:
        payload: 收到的消息字符串，如 "T:a,27.01234,125.03456,85"
        compact: 是否为压缩版（整数偏移），默认 False

    返回:
        解析成功返回对应消息对象，失败返回 None
    """
    if not payload or len(payload) < 2:
        return None

    msg_type = payload[0]
    data = payload[2:]          # 跳过 "X:"

    try:
        if msg_type == 'T':
            parts = data.split(',')
            if len(parts) >= 4:
                lat = _int_to_lat(int(parts[1])) if compact else float(parts[1])
                lon = _int_to_lon(int(parts[2])) if compact else float(parts[2])
                return TargetMsg(
                    tid=parts[0],
                    lat=lat,
                    lon=lon,
                    conf=int(parts[3]) / 100.0,
                )

        elif msg_type == 'D':
            parts = data.split(',')
            if len(parts) >= 2:
                lat = _int_to_lat(int(parts[0])) if compact else float(parts[0])
                lon = _int_to_lon(int(parts[1])) if compact else float(parts[1])
                return DecoyMsg(lat=lat, lon=lon)

        elif msg_type == 'A':
            parts = data.split(',')
            if len(parts) >= 3:
                lat = _int_to_lat(int(parts[0])) if compact else float(parts[0])
                lon = _int_to_lon(int(parts[1])) if compact else float(parts[1])
                return AssembleMsg(lat=lat, lon=lon, reason=parts[2])

        elif msg_type == 'C':
            parts = data.split(',')
            if len(parts) >= 2:
                return ConfirmMsg(ref_type=parts[0], ref_id=parts[1])

        elif msg_type == 'R':
            return ReleaseMsg(target=data)

        elif msg_type == 'J':
            return JammingMsg(state=data)

        elif msg_type == 'H':
            return HeartbeatMsg(status=data)

    except (ValueError, IndexError):
        pass

    return None


def decode_to_dict(payload: str, compact: bool = False) -> Optional[dict]:
    """字符串 → 字典（比 decode 更灵活，适合业务代码直接使用）

    示例:
        decode_to_dict("T:a,27.01234,125.03456,85")
        → {'type': 'T', 'tid': 'a', 'lat': 27.01234, 'lon': 125.03456, 'conf': 0.85}
    """
    msg = decode(payload, compact=compact)
    if msg is None:
        return None

    base = {'type': payload[0]}

    if isinstance(msg, TargetMsg):
        return {**base, 'tid': msg.tid, 'lat': msg.lat, 'lon': msg.lon, 'conf': msg.conf}
    elif isinstance(msg, DecoyMsg):
        return {**base, 'lat': msg.lat, 'lon': msg.lon}
    elif isinstance(msg, AssembleMsg):
        return {**base, 'lat': msg.lat, 'lon': msg.lon, 'reason': msg.reason}
    elif isinstance(msg, ConfirmMsg):
        return {**base, 'ref_type': msg.ref_type, 'ref_id': msg.ref_id}
    elif isinstance(msg, ReleaseMsg):
        return {**base, 'target': msg.target}
    elif isinstance(msg, JammingMsg):
        return {**base, 'state': msg.state}
    elif isinstance(msg, HeartbeatMsg):
        return {**base, 'status': msg.status}

    return None


# ==========================================================================
# 工具函数
# ==========================================================================


def check_length(payload: str) -> bool:
    """检查消息是否 ≤ 50 字节限制"""
    return len(payload.encode('utf-8')) <= config.PAYLOAD_MAX_BYTES


def get_length(payload: str) -> int:
    """返回消息的字节长度 (UTF-8)"""
    return len(payload.encode('utf-8'))
