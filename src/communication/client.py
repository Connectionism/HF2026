"""
原生通信客户端 — 对接赛事平台原生通信 API。

彻底移除 Redis 依赖，纯在 decide() 周期中工作：

    发送：build_broadcast / build_unicast 校验 payload，
         通过 decide() 返回 broadcast() / send_to() 下发
    接收：parse_inbox 解析 obs.comm_inbox（Message 序列）

阶段一 (单无人机)：
    - 仅 uav_alpha 运行任务，其余两架悬停
    - 未开发跨机广播、目标共享、召唤队友等逻辑
"""

from typing import List, Optional, Sequence, Tuple

from . import config
from .protocol import MsgType, decode, decode_to_dict


class CommClient:
    """原生通信客户端（de-Redis 重构版）。

    用法（在 decide() 中）:

        client = CommClient(uav_name="uav_alpha")

        # 发送 — 校验通过后返回字符串，直接交给引擎指令
        payload = client.build_broadcast("T:a,27.01234,125.03456,85")
        return [broadcast(payload)]

        # 接收 — 解析 comp_inbox 得到结构化消息
        for info in client.parse_inbox(obs.comm_inbox):
            print(info["sender"], info["msg"])   # info["msg"] 是 dataclass
    """

    # ------------------------------------------------------------------
    # 构造
    # ------------------------------------------------------------------

    def __init__(self, uav_name: str):
        """
        参数:
            uav_name: 无人机标识，如 "uav_alpha"（不写死）
        """
        if not uav_name or not isinstance(uav_name, str):
            raise ValueError("uav_name 必须是非空字符串")

        self._uav_name = uav_name

        # 流速控制 (SEND_RATE_HZ Hz)
        self._send_timestamps: List[float] = []

        # 统计
        self._sent_total:   int = 0
        self._recv_total:   int = 0
        self._dropped_rate: int = 0
        self._dropped_size: int = 0

    # ------------------------------------------------------------------
    # 只读属性
    # ------------------------------------------------------------------

    @property
    def uav_name(self) -> str:
        """当前无人机标识"""
        return self._uav_name

    @property
    def stats(self) -> dict:
        """通信统计快照（只读）"""
        return {
            "sent_total":   self._sent_total,
            "recv_total":   self._recv_total,
            "dropped_rate": self._dropped_rate,
            "dropped_size": self._dropped_size,
        }

    # ------------------------------------------------------------------
    # 发送（构建 payload）
    # ------------------------------------------------------------------

    def build_broadcast(self, payload: str) -> str:
        """构建广播 payload。

        校验长度 → 更新统计 → 返回原字符串。
        调用方拿到字符串后通过 decide() 返回 broadcast(payload) 下发。

        返回:
            原 payload 字符串

        异常:
            ValueError: payload 无效或超长
        """
        self._validate(payload)
        self._sent_total += 1
        return payload

    def build_unicast(self, target_uid: str, payload: str) -> Tuple[str, str]:
        """构建单播 payload。

        校验长度 → 更新统计 → 返回 (target_uid, payload)。
        调用方拿到后通过 decide() 返回 send_to(target_uid, payload) 下发。

        返回:
            (target_uid, payload) 二元组

        异常:
            ValueError: target_uid 无效或 payload 超长
        """
        if not target_uid or not isinstance(target_uid, str):
            raise ValueError("target_uid 必须是非空字符串")

        self._validate(payload)
        self._sent_total += 1
        return (target_uid, payload)

    # ------------------------------------------------------------------
    # 接收（解析收件箱）
    # ------------------------------------------------------------------

    def parse_inbox(
        self,
        inbox: Sequence,
        compact: bool = False,
    ) -> List[dict]:
        """解析 obs.comm_inbox，返回结构化消息列表。

        参数:
            inbox:   obs.comm_inbox（Message 序列），每条含 .sender_uid / .payload / .recv_time
            compact: 是否用压缩版解码，默认 False

        返回:
            [{
                "sender":   str,       # 发送者 uid
                "payload":  str,       # 原始字符串
                "recv_time": float,    # 接收时间戳
                "msg":      MsgType | None,   # 解码后的消息对象
                "dict":     dict | None,      # 解码后的字典
            }, ...]
        """
        results: List[dict] = []
        for m in inbox:
            self._recv_total += 1
            msg_obj = decode(m.payload, compact=compact)
            results.append({
                "sender":    m.sender_uid,
                "payload":   m.payload,
                "recv_time": m.recv_time,
                "msg":       msg_obj,
                "dict":      decode_to_dict(m.payload, compact=compact),
            })
        return results

    # ------------------------------------------------------------------
    # 流速控制
    # ------------------------------------------------------------------

    def can_send(self, now: Optional[float] = None) -> bool:
        """检查是否允许发送（SEND_RATE_HZ Hz 限流）。

        参数:
            now: 当前时间戳（monotonic），未传则自动取

        返回:
            True  允许发送
            False 超频，本周期应跳过发送
        """
        if now is None:
            now = __import__("time").monotonic()

        window_s = 1.0
        # 清除 1 秒前的记录
        self._send_timestamps = [
            t for t in self._send_timestamps
            if now - t < window_s
        ]

        if len(self._send_timestamps) < config.SEND_RATE_HZ:
            self._send_timestamps.append(now)
            return True

        self._dropped_rate += 1
        return False

    # ------------------------------------------------------------------
    # 内部校验
    # ------------------------------------------------------------------

    def _validate(self, payload: str) -> None:
        """校验 payload 合法性。"""
        if not payload or not isinstance(payload, str):
            raise ValueError("payload 必须是非空字符串")

        byte_len = len(payload.encode("utf-8"))
        if byte_len > config.PAYLOAD_MAX_BYTES:
            self._dropped_size += 1
            raise ValueError(
                f"payload 超长: {byte_len} > {config.PAYLOAD_MAX_BYTES} 字节"
            )

    # ------------------------------------------------------------------
    # 重置
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """清空流速窗口和统计，用于新一局开始时调用。"""
        self._send_timestamps.clear()
        self._sent_total   = 0
        self._recv_total   = 0
        self._dropped_rate = 0
        self._dropped_size = 0
