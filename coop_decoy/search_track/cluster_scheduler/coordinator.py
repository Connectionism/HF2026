"""
协同调度器核心模块

来源: new drone_agent.py CooperativeCoordinator (第 540-801 行)
功能: K=2 双机协同目标分配 —— 基于 R:/T:/C: 通信协议的任务分配、目标融合、配对确认
"""
from __future__ import annotations

import hashlib
import math
import time as _time_module
from typing import Dict, List, Optional, Set, Tuple

from ..motion_control.geo import haversine_m as _haversine_m


class CooperativeCoordinator:
    """
    用于 K=2 目标协同攻击的分布式调度器。

    通信协议消息格式：
      R:<lat>,<lon>            — 发现真目标，召唤队友（简化：直接传坐标）
      T:<lat>,<lon>,<dwell>   — 定期广播本机 dwell 状态
      C:<lat>,<lon>            — 目标已摧毁通知

    内部使用 (lat:.2f, lon:.2f) 作为粗粒度目标 ID，300m 内视为同一目标。
    """

    _MERGE_DIST_M = 400.0

    def __init__(self, my_uid: str, k: int = 2):
        self.my_uid = my_uid
        self.k = k
        self._targets: Dict[str, dict] = {}
        self._peer_dwell: Dict[str, float] = {}
        self._last_r_sent: Dict[str, float] = {}
        self._last_c_sent: Dict[str, float] = {}
        # T 广播限流：基于 sim_t 的时间间隔控制（替代 tick%30）
        self._last_t_broadcast_t: float = -1e9
        self._T_BROADCAST_INTERVAL: float = 3.0
        # 任务分配：记录每个目标有哪些 UID 在跟踪（用于 K=2 配对）
        self._target_discoverers: Dict[str, set] = {}

    # ── 内部工具 ──

    @staticmethod
    def _stable_id(lat: float, lon: float) -> str:
        """稳定 ID：2 位小数（~1.1km 精度），减少检测噪声导致的 ID 漂移。"""
        return f"{lat:.2f}_{lon:.2f}"

    def _find_nearby(self, lat: float, lon: float, skip_destroyed: bool = True) -> Optional[str]:
        """在已有目标中查找最近的未摧毁目标（300m 内）。"""
        best_id = None
        best_dist = float("inf")
        for tgt_id, info in self._targets.items():
            if skip_destroyed and info.get("destroyed", False):
                continue
            d = _haversine_m(lat, lon, info["pos"][0], info["pos"][1])
            if d < self._MERGE_DIST_M and d < best_dist:
                best_dist = d
                best_id = tgt_id
        return best_id

    # ── 公开 API ──

    def ingest_comms(self, comm_inbox: List, sim_t: float = 0.0) -> None:
        """解析通信收件箱。sim_t 用于 peer_dwell 过期清理。"""
        # 定期清理过期 peer_dwell（超过 30s 未更新的队友 dwell 视为过期）
        if sim_t > 0 and hasattr(self, '_last_peer_cleanup_t'):
            if sim_t - self._last_peer_cleanup_t >= 5.0:
                self._cleanup_peer_dwell(sim_t, max_age=30.0)
                self._last_peer_cleanup_t = sim_t
        elif sim_t > 0:
            self._last_peer_cleanup_t = sim_t

        for msg in comm_inbox:
            payload = msg.payload
            if payload.startswith("R:"):
                # R:<lat>,<lon>[,<sender_uid>]  — sender_uid 为可选字段，向后兼容
                try:
                    parts = payload[2:].split(",")
                    if len(parts) >= 2:
                        lat, lon = float(parts[0]), float(parts[1])
                        sender = parts[2].strip() if len(parts) >= 3 else "?"
                        tgt_id = self._add_or_update_target(lat, lon)
                        # 注册发送者为该目标的发现者
                        if sender != "?" and sender != self.my_uid:
                            self.register_discoverer(tgt_id, sender)
                except Exception:
                    pass
            elif payload.startswith("T:"):
                # T:<lat>,<lon>,<dwell>
                try:
                    parts = payload[2:].split(",")
                    if len(parts) >= 3:
                        lat, lon = float(parts[0]), float(parts[1])
                        dwell = float(parts[2])
                        cid = self._stable_id(lat, lon)
                        merged = self._find_nearby(lat, lon, skip_destroyed=False)
                        if merged is not None:
                            cid = merged
                        self._peer_dwell[cid] = dwell
                        # 记录更新时间戳用于过期清理
                        if not hasattr(self, '_peer_dwell_ts'):
                            self._peer_dwell_ts: Dict[str, float] = {}
                        self._peer_dwell_ts[cid] = sim_t if sim_t > 0 else _time_module.monotonic()
                except Exception:
                    pass
            elif payload.startswith("C:"):
                # C:<lat>,<lon>
                try:
                    parts = payload[2:].split(",")
                    if len(parts) >= 2:
                        lat, lon = float(parts[0]), float(parts[1])
                        self._mark_nearby_destroyed_by_pos(lat, lon)
                except Exception:
                    pass

    def _cleanup_peer_dwell(self, sim_t: float, max_age: float = 30.0) -> None:
        """清理过期的 peer_dwell 条目，防止内存泄漏。"""
        if not hasattr(self, '_peer_dwell_ts'):
            return
        expired = [cid for cid, ts in self._peer_dwell_ts.items()
                   if sim_t - ts > max_age]
        for cid in expired:
            self._peer_dwell.pop(cid, None)
            self._peer_dwell_ts.pop(cid, None)

    def _add_or_update_target(self, lat: float, lon: float) -> str:
        """添加或更新目标，返回稳定 ID（不改变已有目标的 ID）。"""
        cid = self._stable_id(lat, lon)
        # 只在不存在时创建新记录；已存在则只更新位置
        if cid not in self._targets:
            merged = self._find_nearby(lat, lon)
            if merged is not None:
                cid = merged
        self._targets[cid] = {
            "pos": (lat, lon), "confirmed": True, "destroyed": False}
        return cid

    def _mark_nearby_destroyed_by_pos(self, lat: float, lon: float) -> None:
        """根据位置标记附近目标为摧毁。"""
        for tid, info in list(self._targets.items()):
            d = _haversine_m(lat, lon, info["pos"][0], info["pos"][1])
            if d < self._MERGE_DIST_M:
                info["destroyed"] = True

    def confirm_target(self, lat: float, lon: float) -> str:
        """确认真目标，返回稳定 ID（同一目标始终返回相同 ID）。"""
        return self._add_or_update_target(lat, lon)

    def confirm_decoy(self, lat: float, lon: float) -> None:
        """标记为假目标。"""
        cid = self._stable_id(lat, lon)
        merged = self._find_nearby(lat, lon, skip_destroyed=False)
        if merged is not None:
            cid = merged
        self._targets[cid] = {
            "pos": (lat, lon), "confirmed": False, "destroyed": True}

    def compute_assignment(self, self_lat: float, self_lon: float,
                           current_target: Optional[str] = None) -> Optional[str]:
        """
        任务分配制 K=2 配对：基于通信协商选择本机应跟踪的目标。

        策略（修正版，避免三机追同一目标）：
        1. 优先加入 peer_count=1 的目标（正好缺一人完成 K=2 配对）
        2. 其次选择 peer_count=0 的目标（无人跟踪的新目标）
        3. 避免 peer_count>=2 的目标（已满员）
        4. 同优先级按距离升序

        current_target: 当前已跟踪的目标 ID，如果分配结果不变则跳过重复 add。
        """
        candidates: List[Tuple[str, float, int]] = []
        # (tgt_id, distance, peer_count)
        for tgt_id, info in self._targets.items():
            if info.get("destroyed", False) or not info.get("confirmed", False):
                continue
            d = _haversine_m(self_lat, self_lon, info["pos"][0], info["pos"][1])
            peer_count = len(self._target_discoverers.get(tgt_id, set()))
            candidates.append((tgt_id, d, peer_count))

        if not candidates:
            return None

        # 排序优先级：
        # - peer_count=1 最高（正好缺一人）
        # - peer_count=0 次之（新目标）
        # - peer_count>=2 最低（已满员，尽量不选）
        # 同优先级按 distance 升序
        def sort_key(x):
            tgt_id, d, pc = x
            if pc == 1:
                priority = 0
            elif pc == 0:
                priority = 1
            else:
                priority = 2 + pc  # pc>=2 按人数进一步排序
            return (priority, d)
        candidates.sort(key=sort_key)
        best = candidates[0]
        chosen_id = best[0]

        # 记录本机加入该目标（仅目标变更时执行 add）
        if chosen_id != current_target:
            if chosen_id not in self._target_discoverers:
                self._target_discoverers[chosen_id] = set()
            self._target_discoverers[chosen_id].add(self.my_uid)

        return chosen_id

    def register_discoverer(self, tgt_id: str, uid: str) -> None:
        """注册某个 UID 为某目标的发现者/跟踪者。"""
        if tgt_id not in self._target_discoverers:
            self._target_discoverers[tgt_id] = set()
        self._target_discoverers[tgt_id].add(uid)

    def unregister_discoverer(self, tgt_id: str, uid: str) -> None:
        """移除某个 UID 对某目标的跟踪。"""
        if tgt_id in self._target_discoverers:
            self._target_discoverers[tgt_id].discard(uid)

    def target_pos(self, tgt_id: str) -> Optional[Tuple[float, float]]:
        return self._targets.get(tgt_id, {}).get("pos")

    def my_slot(self, tgt_id: str) -> int:
        slot = int(hashlib.md5((tgt_id + self.my_uid).encode()).hexdigest(), 16) % 2
        return slot

    def need_r_broadcast(self, tgt_id: str, now: float, cooldown: float = 3.0) -> bool:
        return (tgt_id not in self._last_r_sent
                or (now - self._last_r_sent[tgt_id]) > cooldown)

    def need_c_broadcast(self, tgt_id: str, now: float, cooldown: float = 3.0) -> bool:
        return (tgt_id not in self._last_c_sent
                or (now - self._last_c_sent[tgt_id]) > cooldown)

    def mark_r_sent(self, tgt_id: str, now: float) -> None:
        self._last_r_sent[tgt_id] = now

    def mark_c_sent(self, tgt_id: str, now: float) -> None:
        self._last_c_sent[tgt_id] = now

    def peer_dwell(self, tgt_id: str) -> float:
        """获取队友报告的该目标最大 dwell（含附近目标）。"""
        pos = self._targets.get(tgt_id, {}).get("pos")
        best = self._peer_dwell.get(tgt_id, 0.0)
        if pos is not None:
            for tid, dwell in self._peer_dwell.items():
                if tid == tgt_id:
                    continue
                tpos = self._targets.get(tid, {}).get("pos")
                if tpos is None:
                    continue
                d = _haversine_m(pos[0], pos[1], tpos[0], tpos[1])
                if d < self._MERGE_DIST_M and dwell > best:
                    best = dwell
        return best

    def mark_destroyed(self, tgt_id: str) -> None:
        pos = self._targets.get(tgt_id, {}).get("pos")
        if pos is not None:
            self._mark_nearby_destroyed_by_pos(pos[0], pos[1])

    # ── 公开访问器（封装私有属性，供 drone_agent 使用） ──

    def is_target_destroyed(self, tgt_id: str) -> bool:
        """查询目标是否已被摧毁。"""
        return self._targets.get(tgt_id, {}).get("destroyed", False)

    def find_nearby(self, lat: float, lon: float, skip_destroyed: bool = True) -> Optional[str]:
        """在已有目标中查找最近的未摧毁目标（公开封装）。"""
        return self._find_nearby(lat, lon, skip_destroyed=skip_destroyed)

    def target_discoverers_count(self, tgt_id: str) -> int:
        """返回指定目标当前的跟踪者数量。"""
        return len(self._target_discoverers.get(tgt_id, set()))

    def stable_id(self, lat: float, lon: float) -> str:
        """返回位置对应的稳定 ID（公开封装）。"""
        return self._stable_id(lat, lon)

    def need_t_broadcast(self, sim_t: float) -> bool:
        """检查是否达到 T 广播间隔。"""
        return sim_t - self._last_t_broadcast_t >= self._T_BROADCAST_INTERVAL

    def mark_t_broadcast(self, sim_t: float) -> None:
        """记录 T 广播时间。"""
        self._last_t_broadcast_t = sim_t

    def update_target_pos(self, tgt_id: str, lat: float, lon: float) -> None:
        """直接更新目标位置（TRACK 状态自身检测更新）。"""
        self._targets[tgt_id] = {
            "pos": (lat, lon),
            "confirmed": True, "destroyed": False}

    def reset(self) -> None:
        self._targets.clear()
        self._peer_dwell.clear()
        self._last_r_sent.clear()
        self._last_c_sent.clear()
        self._target_discoverers.clear()
        self._last_t_broadcast_t = -1e9
        if hasattr(self, '_peer_dwell_ts'):
            self._peer_dwell_ts.clear()
        if hasattr(self, '_last_peer_cleanup_t'):
            delattr(self, '_last_peer_cleanup_t')
