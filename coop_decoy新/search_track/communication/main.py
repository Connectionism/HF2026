"""
communication 模块对外统一入口

对外暴露:
    CommHandler  — 通信处理器（解析 inbox + 构建 broadcast/report_target 指令）

内部实现:
    client.py    — CommClient（原生通信客户端）
    protocol.py  — 编解码（七类消息格式 T:/D:/A:/C:/R:/J:/H:）
    config.py    — 配置常量

数据流:
    obs.comm_inbox → CommHandler.parse_inbox() → 结构化消息列表
    drone_agent 内部逻辑 → CommHandler.build_broadcast() / build_report() → Command 对象
"""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .client import CommClient
from . import config as comm_config

logger = logging.getLogger(__name__)

try:
    from competition.sdk.core.commands import Command, broadcast, report_target
except ImportError:
    logger.warning(
        "⚠️ SDK通信命令导入失败，启用本地模拟通信实现！"
        "仅用于本地调试，竞赛环境不应触发此分支"
    )
    from dataclasses import dataclass

    @dataclass(frozen=True)
    class Command:
        verb: str
        params: dict

    def broadcast(payload: str) -> Command:
        """模拟 SDK broadcast 命令（仅本地调试）。"""
        return Command("broadcast", {"payload": payload})

    def report_target(lat: float, lon: float, target_id: Optional[str] = None) -> Command:
        """模拟 SDK report_target 命令（仅本地调试）。"""
        params = {"latitude": float(lat), "longitude": float(lon)}
        if target_id is not None:
            params["target_id"] = target_id
        return Command("report_target", params)


class CommHandler:
    """
    通信处理器

    封装通信发送/接收/解析的完整流程。
    drone_agent.py 通过本类完成所有通信操作，不直接访问 client/protocol/config。
    """

    def __init__(self, uav_name: str):
        self._client = CommClient(uav_name)
        self._uav_name = uav_name

    @property
    def uav_name(self) -> str:
        return self._uav_name

    @property
    def stats(self) -> dict:
        return self._client.stats

    # ── 接收 ──
    def parse_inbox(self, inbox: Sequence) -> List[dict]:
        """解析 obs.comm_inbox，返回结构化消息列表。"""
        return self._client.parse_inbox(inbox)

    # ── 发送 ──
    def build_broadcast(self, payload: str) -> Command:
        """构建广播命令。校验 payload → 返回 broadcast() Command 对象。"""
        validated = self._client.build_broadcast(payload)
        return broadcast(validated)

    def build_unicast(self, target_uid: str, payload: str) -> Command:
        """构建单播命令（如果引擎支持 send_to）。"""
        validated = self._client.build_unicast(target_uid, payload)
        return broadcast(validated[1])  # fallback to broadcast

    def build_report(self, lat: float, lon: float, target_id: Optional[str] = None) -> Command:
        """构建上报目标坐标命令。"""
        return report_target(lat, lon, target_id=target_id)

    def build_r_msg(self, lat: float, lon: float, sender_uid: str) -> Command:
        """构建 R: 发现真目标广播 → 格式: R:lat,lon,sender_uid（5位小数精度，与 ingest_comms 解析一致）。"""
        return self.build_broadcast(f"R:{lat:.5f},{lon:.5f},{sender_uid}")

    def build_t_msg(self, lat: float, lon: float, dwell: float) -> Command:
        """构建 T: 本机 dwell 状态广播 → 格式: T:lat,lon,dwell（5位小数精度）。"""
        return self.build_broadcast(f"T:{lat:.5f},{lon:.5f},{dwell:.2f}")

    def build_c_msg(self, lat: float, lon: float) -> Command:
        """构建 C: 目标已摧毁广播 → 格式: C:lat,lon（5位小数精度）。"""
        return self.build_broadcast(f"C:{lat:.5f},{lon:.5f}")

    def can_send(self, now: Optional[float] = None) -> bool:
        """流速控制检查。"""
        return self._client.can_send(now)

    def reset(self) -> None:
        self._client.reset()


__all__ = ["CommHandler", "comm_config"]
