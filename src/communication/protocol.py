"""
通信协议编解码模块

七类消息格式（全部 ≤50 字节，UTF-8）：
    T: — 目标确认 (Target)
    D: — 诱饵确认 (Decoy)
    A: — 集结请求 (Assemble)
    C: — 确认应答 (Confirm)
    R: — 解除/取消 (Release)
    J: — 干扰报告 (Jamming)     ← V1.1.0 新增
    H: — 心跳存活 (Heartbeat)   ← V1.1.0 新增

编码方案（简单版）：坐标直接写小数，代码简单，29字节左右
如需压缩，可用整数偏移版（17字节），两套 API 都提供
"""

from dataclasses import dataclass
from typing import Union, Optional


# ========== 常量 ==========

MSG_TYPE_TARGET   = 'T'
MSG_TYPE_DECOY    = 'D'
MSG_TYPE_ASSEMBLE = 'A'
MSG_TYPE_CONFIRM  = 'C'
MSG_TYPE_RELEASE  = 'R'
MSG_TYPE_JAMMING  = 'J'
MSG_TYPE_HEARTBEAT = 'H'

MAX_PAYLOAD_BYTES = 50  # 消息最大字节数


# ========== 消息数据类 ==========

@dataclass(frozen=True)
class TargetMsg:
    """T: 发现真目标，请求协同"""
    tid: str          # 目标编号 a~z
    lat: float        # 纬度
    lon: float        # 经度
    conf: float       # 置信度 0.0~1.0


@dataclass(frozen=True)
class DecoyMsg:
    """D: 确认诱饵，通知队友别来"""
    lat: float
    lon: float


@dataclass(frozen=True)
class AssembleMsg:
    """A: 请求队友来这个位置集合"""
    lat: float
    lon: float
    reason: str       # 't'=协同跟踪, 's'=扇区搜索, 'h'=待命


@dataclass(frozen=True)
class ConfirmMsg:
    """C: 收到队友消息，回复确认"""
    ref_type: str     # 'T', 'D', 'A' — 回应哪种消息
    ref_id: str       # 目标id 或 坐标摘要


@dataclass(frozen=True)
class ReleaseMsg:
    """R: 目标已处理/误判，解除盯防"""
    target: str       # 目标id 或 'A'(取消集结)


@dataclass(frozen=True)
class JammingMsg:
    """J: 通信干扰状态报告（V1.1.0 新增）"""
    state: str        # 'on'=进入干扰区, 'off'=脱离干扰区


@dataclass(frozen=True)
class HeartbeatMsg:
    """H: 心跳存活广播（V1.1.0 新增）"""
    status: str       # 'a'=alive, 'd'=destroyed


MsgType = Union[TargetMsg, DecoyMsg, AssembleMsg, ConfirmMsg, ReleaseMsg,
                JammingMsg, HeartbeatMsg]


# ========== 编码（简单版：小数坐标）==========

def encode(msg: MsgType) -> str:
    """
    把消息对象变成字符串，直接用小数坐标（代码最简单）

    示例：
        encode(TargetMsg('a', 27.01234, 125.03456, 0.85))
        → "T:a,27.01234,125.03456,85"
    """
    if isinstance(msg, TargetMsg):
        return f"T:{msg.tid},{msg.lat:.5f},{msg.lon:.5f},{int(msg.conf * 100)}"
    elif isinstance(msg, DecoyMsg):
        return f"D:{msg.lat:.5f},{msg.lon:.5f}"
    elif isinstance(msg, AssembleMsg):
        return f"A:{msg.lat:.5f},{msg.lon:.5f},{msg.reason}"
    elif isinstance(msg, ConfirmMsg):
        return f"C:{msg.ref_type},{msg.ref_id}"
    elif isinstance(msg, ReleaseMsg):
        return f"R:{msg.target}"
    elif isinstance(msg, JammingMsg):
        return f"J:{msg.state}"
    elif isinstance(msg, HeartbeatMsg):
        return f"H:{msg.status}"
    else:
        raise ValueError(f"不认识的消息类型: {type(msg)}")


# ========== 编码（压缩版：整数偏移）==========

BASE_LAT = 27.0
BASE_LON = 125.0
SCALE = 100000  # 0.00001° ≈ 1.1米


def _lat_to_int(lat: float) -> int:
    return round((lat - BASE_LAT) * SCALE)


def _lon_to_int(lon: float) -> int:
    return round((lon - BASE_LON) * SCALE)


def _int_to_lat(v: int) -> float:
    return BASE_LAT + v / SCALE


def _int_to_lon(v: int) -> float:
    return BASE_LON + v / SCALE


def encode_compact(msg: MsgType) -> str:
    """
    压缩版编码：坐标变成整数偏移（省字节，17字节左右）

    示例：
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
        raise ValueError(f"不认识的消息类型: {type(msg)}")


# ========== 解码 ==========

def decode(payload: str, compact: bool = False) -> Optional[MsgType]:
    """
    把收到的字符串解析成消息对象

    参数:
        payload: 收到的消息字符串，如 "T:a,27.01234,125.03456,85"
        compact: 是否为压缩版（整数偏移），默认 False

    返回:
        解析成功返回对应消息对象，失败返回 None
    """
    if not payload or len(payload) < 2:
        return None

    msg_type = payload[0]
    data = payload[2:]  # 跳过 "X:"

    try:
        if msg_type == 'T':
            parts = data.split(',')
            if len(parts) >= 4:
                tid = parts[0]
                if compact:
                    lat = _int_to_lat(int(parts[1]))
                    lon = _int_to_lon(int(parts[2]))
                else:
                    lat = float(parts[1])
                    lon = float(parts[2])
                conf = int(parts[3]) / 100.0
                return TargetMsg(tid=tid, lat=lat, lon=lon, conf=conf)

        elif msg_type == 'D':
            parts = data.split(',')
            if len(parts) >= 2:
                if compact:
                    lat = _int_to_lat(int(parts[0]))
                    lon = _int_to_lon(int(parts[1]))
                else:
                    lat = float(parts[0])
                    lon = float(parts[1])
                return DecoyMsg(lat=lat, lon=lon)

        elif msg_type == 'A':
            parts = data.split(',')
            if len(parts) >= 3:
                if compact:
                    lat = _int_to_lat(int(parts[0]))
                    lon = _int_to_lon(int(parts[1]))
                else:
                    lat = float(parts[0])
                    lon = float(parts[1])
                reason = parts[2]
                return AssembleMsg(lat=lat, lon=lon, reason=reason)

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
    """
    把消息解析成字典（更灵活，适合直接用在业务代码里）

    示例:
        decode_to_dict("T:a,27.01234,125.03456,85")
        → {'type': 'T', 'tid': 'a', 'lat': 27.01234, 'lon': 125.03456, 'conf': 0.85}
    """
    msg = decode(payload, compact=compact)
    if msg is None:
        return None

    if isinstance(msg, TargetMsg):
        return {'type': 'T', 'tid': msg.tid, 'lat': msg.lat, 'lon': msg.lon, 'conf': msg.conf}
    elif isinstance(msg, DecoyMsg):
        return {'type': 'D', 'lat': msg.lat, 'lon': msg.lon}
    elif isinstance(msg, AssembleMsg):
        return {'type': 'A', 'lat': msg.lat, 'lon': msg.lon, 'reason': msg.reason}
    elif isinstance(msg, ConfirmMsg):
        return {'type': 'C', 'ref_type': msg.ref_type, 'ref_id': msg.ref_id}
    elif isinstance(msg, ReleaseMsg):
        return {'type': 'R', 'target': msg.target}
    elif isinstance(msg, JammingMsg):
        return {'type': 'J', 'state': msg.state}
    elif isinstance(msg, HeartbeatMsg):
        return {'type': 'H', 'status': msg.status}

    return None


# ========== 工具函数 ==========

def check_length(payload: str) -> bool:
    """检查消息是否超过 50 字节限制，超限返回 False"""
    return len(payload.encode('utf-8')) <= MAX_PAYLOAD_BYTES


def get_length(payload: str) -> int:
    """返回消息的字节长度"""
    return len(payload.encode('utf-8'))
