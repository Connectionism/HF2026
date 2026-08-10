"""Multi-entity SimClient for the cooperative challenge.

Wraps redis-py and parses each sim:state frame into a MultiSimState
(multi-UAV + multi-vehicle view) via parse_multi_sim_state.

The client is UAV-agnostic: it does not assume a fixed uav_id. Instead it
discovers UAVs from each state frame by scanning entities of kind=="uav".
"""
# 可以用来连接仿真引擎、拿状态、发命令
from __future__ import annotations

import json
import time
from typing import Any

import redis

from .multi_state import MultiSimState, parse_multi_sim_state


CMD_CHANNEL = "sim:commands"# 给引擎发指令的
STATE_CHANNEL = "sim:state"# 引擎给队伍推数据流的群
EVENTS_CHANNEL = "sim:events"# 队伍给裁判系统报日志


class MultiSimClient:
    """Redis wrapper that yields MultiSimState per tick.

    Unlike the single-uav SimClient, this client parses ALL entities in
    each sim:state frame, supporting 3 UAVs + multiple vehicles out of
    the box.
    """
# 初始化
    def __init__(self, *, host: str = "127.0.0.1", port: int = 6379) -> None:
        self.host = host
        self.port = port
        self._redis: redis.Redis | None = None
        self._pubsub: redis.client.PubSub | None = None
        self._latest_state: MultiSimState | None = None
# 连 Redis，订阅 sim:state频道 
    def connect(self) -> None:
        self._redis = redis.Redis(
            host=self.host, port=self.port, decode_responses=True,
        )
        self._redis.ping()
        self._pubsub = self._redis.pubsub(ignore_subscribe_messages=True)
        self._pubsub.subscribe(STATE_CHANNEL)
# 断开链接
    def close(self) -> None:
        if self._pubsub is not None:
            try:
                self._pubsub.close()
            except Exception:
                pass
            self._pubsub = None
        if self._redis is not None:
            try:
                self._redis.close()
            except Exception:
                pass
            self._redis = None
# 仿真引擎启动时要加载地图、地形，加载完才发第一帧，这个是等待第一帧
    def wait_first_state(self, timeout: float = 120.0) -> MultiSimState:
        """Block until the first sim:state frame arrives.

        Default 120 s: opensim-sim loads terrain data before publishing
        the first frame.
        """
        if self._pubsub is None:
            raise RuntimeError(
                "MultiSimClient not connected; call connect() first"
            )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._pubsub.get_message(timeout=0.5)
            if msg and msg.get("type") == "message":
                try:
                    raw = json.loads(msg["data"])
                    state = parse_multi_sim_state(raw)
                    self._latest_state = state
                    return state
                except (json.JSONDecodeError, KeyError, TypeError):
                    continue
        raise TimeoutError(
            f"no sim:state received within {timeout}s — "
            f"is opensim-sim running?"
        )
# 快速接收最新帧，最多等 50ms
    def poll_latest(self, timeout: float = 0.05) -> MultiSimState | None:
        """Drain PubSub queue and return the latest state (non-blocking)."""
        if self._pubsub is None:
            raise RuntimeError("MultiSimClient not connected")
        latest: MultiSimState | None = self._latest_state
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._pubsub.get_message(timeout=0.01)
            if not (msg and msg.get("type") == "message"):
                break
            try:
                raw = json.loads(msg["data"])
                latest = parse_multi_sim_state(raw)
            except (json.JSONDecodeError, KeyError, TypeError):
                continue
        self._latest_state = latest
        return latest
# 发简单命令
    def publish_dict(self, d: dict[str, Any]) -> int:
        if self._redis is None:
            raise RuntimeError("MultiSimClient not connected")
        return self._redis.publish(CMD_CHANNEL, json.dumps(d))

    def send_engine(self, verb: str) -> int:
        return self.publish_dict({"cmd": verb, "params": {}})
# 将记录结果发给裁判看的
    def publish_event(
        self,
        *,
        event_type: str,
        entity_uid: str,
        sim_time: float,
        payload: dict[str, Any] | None = None,
        team: str | None = None,
    ) -> int:
        """Publish a SimEvent to the ``sim:events`` channel."""
        if self._redis is None:
            raise RuntimeError("MultiSimClient not connected")
        source: dict[str, Any] = {
            "kind": "external",
            "producer": "multi-uav-coop-decoy",
        }
        if team is not None:
            source["team"] = team
        message: dict[str, Any] = {
            "event_type": event_type,
            "source": source,
            "entity_uid": entity_uid,
            "sim_time": sim_time,
            "payload": payload or {},
        }
        return self._redis.publish(EVENTS_CHANNEL, json.dumps(message))
# 支持 with 语法
    def __enter__(self) -> "MultiSimClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
