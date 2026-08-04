"""
原生通信客户端 — 对接赛事平台通信 API。

彻底移除 Redis 依赖，改为在 Runner 的 decide() 周期中工作：
    发送：调用 build_broadcast / build_unicast 校验 payload，
         再通过 decide() 返回 broadcast() / send_to() 下发
    接收：调用 parse_inbox 解析 obs.comm_inbox

阶段一 (单无人机)：
    - 仅 uav_alpha 运行任务，其余两架悬停
    - 未开发跨机广播、目标共享、召唤队友等逻辑
"""

import time
from typing import Optional

from . import config
from .protocol import decode


class CommClient:
    """
    原生通信客户端 (de-Redis 重构版)。

    用法 (在 decide() 中):
        client = CommClient(uav_name="uav_alpha")

        # 发消息
        payload = client.build_broadcast("T:a,27.0,125.0,85")
        return [broadcast(payload)]

        # 收消息
        for msg in client.parse_inbox(obs.comm_inbox):
            print(msg)
    """

    # ------------------------------------------------------------------
    # 构造 / 元信息
    # ------------------------------------------------------------------

    def __init__(self, uav_name: str):
        """
        参数:
            uav_name: 无人机标识，如 "uav_alpha" (不写死)
        """
        if not uav_name or not isinstance(uav_name, str):
            raise ValueError("uav_name 必须是非空字符串")

        self._uav_name = uav_name

        # 流速控制 (4 Hz)
        self._send_window: list[float] = []

        # 统计
        self._sent_total   = 0
        self._recv_total   = 0
        self._dropped_rate = 0
        self._dropped_size = 0

    # ------------------------------------------------------------------
    # 属性
    # ------------------------------------------------------------------

    @property
    def uav_name(self) -> str:
        return self._uav_name

    @property
    def stats(self) -> dict:
        """返回通信统计快照 (只读)。"""
        return {
            "sent_total":   self._sent_total,
            "recv_total":   self._recv_total,
            "dropped_rate": self._dropped_rate,
            "dropped_size": self._dropped_size,
        }

    # ------------------------------------------------------------------
    # 发送
    # ------------------------------------------------------------------

    def build_broadcast(self, payload: str) -> str:
        """
        构建广播 payload。
        校验 → 通过返回原样字符串，失败抛 ValueError。
        """
        self._validate(payload)
        self._sent_total += 1
        return payload

    def build_unicast(self, target_uid: str, payload: str) -> tuple[str, str]:
        """
        构建单播 payload。
        返回 (target_uid, payload)，供 send_to() 使用。
        """
        if not target_uid or not isinstance(target_uid, str):
            raise ValueError("target_uid 必须是非空字符串")

        self._validate(payload)
        self._sent_total += 1
        return (target_uid, payload)

    # ------------------------------------------------------------------
    # 接收
    # ------------------------------------------------------------------

    def parse_inbox(self, inbox) -> list:
        """
        解析 obs.comm_inbox，返回已解码的消息对象列表。

        inbox 是 CommMsg 序列，每条包含:
            .sender_uid : str
            .payload    : str

        返回 list[dataclass | None]，未解码项为 None。
        """
        results = []
        for msg in inbox:
            self._recv_total += 1
            obj = decode(msg.payload)
            results.append(obj)
        return results

    # ------------------------------------------------------------------
    # 流速控制
    # ------------------------------------------------------------------

    def can_send(self, now: Optional[float] = None) -> bool:
        """
        是否允许发送 (4 Hz 限流)。

        每周期调用一次即可；若不调用，build_broadcast/build_unicast
        仍会执行但不做限流。
        """
        if now is None:
            now = time.monotonic()

        window_s = 1.0
        # 驱逐 1 秒前的记录
        self._send_window = [t for t in self._send_window if now - t < window_s]

        if len(self._send_window) < config.SEND_RATE_HZ:
            self._send_window.append(now)
            return True

        self._dropped_rate += 1
        return False

    # ------------------------------------------------------------------
    # 内部校验
    # ------------------------------------------------------------------

    def _validate(self, payload: str) -> None:
        """校验 payload 合法性，不通过抛 ValueError。"""
        if not payload or not isinstance(payload, str):
            raise ValueError("payload 必须是非空字符串")

        byte_len = len(payload.encode("utf-8"))
        if byte_len > config.PAYLOAD_MAX_BYTES:
            self._dropped_size += 1
            raise ValueError(
                f"payload 超长: {byte_len} > {config.PAYLOAD_MAX_BYTES} 字节"
            )

    # ------------------------------------------------------------------
    # 重置 / 清理
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """清空流速窗口和统计，用于新一局开始时调用。"""
        self._send_window.clear()
        self._sent_total   = 0
        self._recv_total   = 0
        self._dropped_rate = 0
        self._dropped_size = 0
