"""
通信模块
提供：五类消息编解码 + Redis 客户端封装
"""

from .protocol import (
    TargetMsg, DecoyMsg, AssembleMsg, ConfirmMsg, ReleaseMsg,
    JammingMsg, HeartbeatMsg,
    encode, decode, decode_to_dict, check_length,
    MSG_TYPE_TARGET, MSG_TYPE_DECOY, MSG_TYPE_ASSEMBLE,
    MSG_TYPE_CONFIRM, MSG_TYPE_RELEASE,
    MSG_TYPE_JAMMING, MSG_TYPE_HEARTBEAT,
)

from .client import CommClient

__all__ = [
    # 消息类型
    "TargetMsg", "DecoyMsg", "AssembleMsg", "ConfirmMsg", "ReleaseMsg",
    "JammingMsg", "HeartbeatMsg",
    # 常量
    "MSG_TYPE_TARGET", "MSG_TYPE_DECOY", "MSG_TYPE_ASSEMBLE",
    "MSG_TYPE_CONFIRM", "MSG_TYPE_RELEASE",
    "MSG_TYPE_JAMMING", "MSG_TYPE_HEARTBEAT",
    # 编解码函数
    "encode", "decode", "decode_to_dict", "check_length",
    # 客户端
    "CommClient",
]
