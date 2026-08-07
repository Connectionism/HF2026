"""
通信模块

提供：
    - 七类消息编解码   (protocol)
    - 通信常量配置     (config)
    - 原生通信客户端   (client)
    - 对外统一入口     (main)
"""

# ---- 配置常量 ----
from .config import (
    PAYLOAD_MAX_BYTES,
    SEND_RATE_HZ,
    COMM_RANGE_M,
    INBOX_MAX_SIZE,
    BASE_LAT,
    BASE_LON,
    COORD_SCALE,
    COORD_DECIMALS,
    SPEED_MIN_MPS,
    SPEED_MAX_MPS,
    FOV_MIN_DEG,
    FOV_MAX_DEG,
    SIM_DURATION_S,
    DECIDE_RATE_HZ,
    TICK_PERIOD_S,
)

# ---- 消息数据类 ----
from .protocol import (
    TargetMsg,
    DecoyMsg,
    AssembleMsg,
    ConfirmMsg,
    ReleaseMsg,
    JammingMsg,
    HeartbeatMsg,
)

# ---- 消息类型常量 ----
from .protocol import (
    MSG_TYPE_TARGET,
    MSG_TYPE_DECOY,
    MSG_TYPE_ASSEMBLE,
    MSG_TYPE_CONFIRM,
    MSG_TYPE_RELEASE,
    MSG_TYPE_JAMMING,
    MSG_TYPE_HEARTBEAT,
)

# ---- 编解码函数 ----
from .protocol import (
    encode,
    encode_compact,
    decode,
    decode_to_dict,
    check_length,
    get_length,
)

# ---- 通信客户端 ----
from .client import CommClient

# ---- 对外统一入口 ----
from .main import CommHandle


__all__ = [
    # --- 配置常量 ---
    "PAYLOAD_MAX_BYTES",
    "SEND_RATE_HZ",
    "COMM_RANGE_M",
    "INBOX_MAX_SIZE",
    "BASE_LAT",
    "BASE_LON",
    "COORD_SCALE",
    "COORD_DECIMALS",
    "SPEED_MIN_MPS",
    "SPEED_MAX_MPS",
    "FOV_MIN_DEG",
    "FOV_MAX_DEG",
    "SIM_DURATION_S",
    "DECIDE_RATE_HZ",
    "TICK_PERIOD_S",
    # --- 消息数据类 ---
    "TargetMsg",
    "DecoyMsg",
    "AssembleMsg",
    "ConfirmMsg",
    "ReleaseMsg",
    "JammingMsg",
    "HeartbeatMsg",
    # --- 消息类型常量 ---
    "MSG_TYPE_TARGET",
    "MSG_TYPE_DECOY",
    "MSG_TYPE_ASSEMBLE",
    "MSG_TYPE_CONFIRM",
    "MSG_TYPE_RELEASE",
    "MSG_TYPE_JAMMING",
    "MSG_TYPE_HEARTBEAT",
    # --- 编解码 ---
    "encode",
    "encode_compact",
    "decode",
    "decode_to_dict",
    "check_length",
    "get_length",
    # --- 客户端 ---
    "CommClient",
    # --- 对外入口 ---
    "CommHandle",
]
