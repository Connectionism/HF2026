
"""
协同调度模块
============================================================
K=2协同核心逻辑 (赛题二硬约束: 2架UAV同时盯防20s才能摧毁):

  1. UAV-A发现并确认真目标 -> 广播 R:lat,lon 召唤队友
  2. UAV-B收到R:消息 -> 飞向目标位置(SLOT_1方位)
  3. UAV-A在目标SLOT_0方位盘旋 -> 两机间距>200m
  4. 2架同时检测到目标 -> dwell同时累计 -> 满20s
  5. 广播 C:tgtidx 目标摧毁通知 -> 释放 -> 转下一目标

贪婪自选目标分配:
  - 每架UAV独立选择最优目标(距离最近/优先级最高)
  - 通过A:消息声明认领，避免冲突
  - 目标摧毁后自动转向下一个未完成目标

可复用代码:
  competition/baselines/swarm_coordinated.py
  (_select_target / _slot_for_target / _team_aim_point)
  需将K=3改为K=2

接口定义:
  class CooperativeCoordinator:
      def __init__(self, my_uid, k=2): ...
      def ingest_comms(self, comm_inbox) -> None: ...
      def confirm_target(self, lat, lon) -> None: ...
      def confirm_decoy(self, lat, lon) -> None: ...
      def select_target(self, self_lat, self_lon) -> tuple|None: ...
      def my_slot(self, tgt, fleet_size) -> int: ...
      def aim_point(self, tgt, slot) -> tuple[float, float]: ...
      def is_destroyed(self, tgt) -> bool: ...

负责人: 成员3
"""

# ── 任务几何参数 ─────────────────────────────────────────────────────────────
from __future__ import annotations

import hashlib
import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

# ── SDK 导入兼容层 ────────────────────────────────────────────────────────────
try:
    from sdk.core.commands import (Command, broadcast, fly_to,
                                   point_gimbal, report_target,
                                   set_gimbal_fov)
    from sdk.scenarios.coop_decoy import CoopAgent
    from sdk.scenarios.coop_decoy.observation import CoopObs
except ImportError:
    try:
        from competition.sdk.core.commands import (Command, broadcast, fly_to,
                                                   point_gimbal, report_target,
                                                   set_gimbal_fov)
        from competition.sdk.scenarios.coop_decoy import CoopAgent
        from competition.sdk.scenarios.coop_decoy.observation import CoopObs
    except ImportError:
        from dataclasses import dataclass

        @dataclass(frozen=True)
        class Command:
            verb: str
            params: dict

        def fly_to(lat, lon, alt=None, speed=None, loiter_radius=200.0):
            params = {"latitude": float(lat), "longitude": float(lon),
                      "loiter_radius": float(loiter_radius)}
            if alt is not None:
                params["altitude"] = float(alt)
            if speed is not None:
                params["speed"] = float(speed)
            return Command("set_destination", params)

        def point_gimbal(pan, tilt):
            return Command("component.gimbal_tracking.set_orientation",
                           {"pan": float(pan), "tilt": float(tilt)})

        def set_gimbal_fov(fov):
            return Command("set_fov", {"angle": float(fov)})

        def broadcast(payload):
            return Command("comm.broadcast", {"payload": str(payload)})

        def report_target(lat, lon):
            return Command("agent.report", {"lat": float(lat), "lon": float(lon)})

        class CoopAgent:
            pass

        @dataclass
        class CoopObs:
            pass


# ── 全局几何参数 ──────────────────────────────────────────────────────────────
_BBOX: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (26.982, 124.980), (27.025, 125.020))
_SAFEBOX_MARGIN_M = 600.0


def _bbox_inset(bbox, margin_m: float):
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    lat_mid = (lat_min + lat_max) / 2
    dlat = margin_m / 111320.0
    dlon = margin_m / (111320.0 * math.cos(math.radians(lat_mid)))
    return ((lat_min + dlat, lon_min + dlon), (lat_max - dlat, lon_max - dlon))


_SAFEBOX = _bbox_inset(_BBOX, _SAFEBOX_MARGIN_M)

# 地图中心：三机螺旋搜索的公共起点 & 归中目标点
_MAP_CENTER: Tuple[float, float] = (
    (_BBOX[0][0] + _BBOX[1][0]) / 2,
    (_BBOX[0][1] + _BBOX[1][1]) / 2,
)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = (math.cos(p1) * math.sin(p2)
         - math.sin(p1) * math.cos(p2) * math.cos(dl))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _clamp_to_safebox(lat: float, lon: float) -> Tuple[float, float]:
    (lat_min, lon_min), (lat_max, lon_max) = _SAFEBOX
    return (min(max(lat, lat_min), lat_max),
            min(max(lon, lon_min), lon_max))


def _uid_index(uid: str) -> int:
    """将 UID 确定性映射为 0/1/2，用于螺旋初始相位偏移。"""
    if uid.isdigit():
        return int(uid) % 3
    if "_" in uid:
        tail = uid.rsplit("_", 1)[-1]
        if tail.isdigit():
            return int(tail) % 3
    return int(hashlib.md5(uid.encode()).hexdigest(), 16) % 3


def _make_target_id(lat: float, lon: float) -> str:
    """确定性目标 ID，保留 4 位小数（≈11m 精度）。"""
    return f"{lat:.4f}_{lon:.4f}"


# ── EMA 跟踪器 ────────────────────────────────────────────────────────────────
class _EMATracker:
    # ... __init__, append, value, count, reset 保持不变 ...

    def speed_and_linearity(self, tick_hz: float = 10.0) -> Tuple[float, float]:
        """
        返回 (估算速度 m/s, 轨迹线性度 R²)。
        R² ∈ [0, 1]：1.0 表示完美直线（真目标），接近 0 表示随机游走（诱饵）。
        """
        n = len(self._raw)
        if n < 12:  # 样本太少无法可靠计算
            return 0.0, 0.0

        ts = list(range(n))
        lats = [p[0] for p in self._raw]
        lons = [p[1] for p in self._raw]
        ns = float(n)
        sx = sum(ts)
        sxx = sum(t * t for t in ts)
        denom = ns * sxx - sx * sx
        if abs(denom) < 1e-20:
            return 0.0, 0.0

        # ── 纬度方向拟合 & SS_res / SS_tot ──
        sy_lat = sum(lats)
        sxy_lat = sum(t * la for t, la in zip(ts, lats))
        slope_lat = (ns * sxy_lat - sx * sy_lat) / denom
        mean_lat = sy_lat / ns
        ss_tot_lat = sum((la - mean_lat) ** 2 for la in lats)
        ss_res_lat = sum((la - (slope_lat * t + (sy_lat - slope_lat * sx) / ns)) ** 2
                         for t, la in zip(ts, lats))

        # ── 经度方向拟合 & SS_res / SS_tot ──
        sy_lon = sum(lons)
        sxy_lon = sum(t * lo for t, lo in zip(ts, lons))
        slope_lon = (ns * sxy_lon - sx * sy_lon) / denom
        mean_lon = sy_lon / ns
        ss_tot_lon = sum((lo - mean_lon) ** 2 for lo in lons)
        ss_res_lon = sum((lo - (slope_lon * t + (sy_lon - slope_lon * sx) / ns)) ** 2
                         for t, lo in zip(ts, lons))

        # ── 合成速度与 R² ──
        lat_mid = lats[-1]
        v_lat = slope_lat * 111320.0 * tick_hz
        v_lon = slope_lon * 111320.0 * math.cos(math.radians(lat_mid)) * tick_hz
        speed = math.sqrt(v_lat ** 2 + v_lon ** 2)

        ss_tot = ss_tot_lat + ss_tot_lon
        ss_res = ss_res_lat + ss_res_lon
        r_squared = max(0.0, 1.0 - ss_res / ss_tot) if ss_tot > 1e-20 else 0.0

        return speed, r_squared


# ── 协同调度器（K=2 角色配额制）────────────────────────────────────────────────
class CooperativeCoordinator:
    """
    分布式协同调度器。
    通信协议：
      R:<tgt_id>,<lat>,<lon>           — 发现真目标，召唤队友
      J:<tgt_id>,<uid>                 — 我已加入跟踪（占槽位）
      T:<tgt_id>,<dwell>,<uid>         — 广播本机累积照射时间
      C:<tgt_id>                       — 目标已摧毁，释放槽位
    """

    def __init__(self, my_uid: str, k: int = 2):
        self.my_uid = my_uid
        self.k = k                          # 最大同时跟踪数 = 2
        self._targets: Dict[str, dict] = {}
        self._trackers: Dict[str, set] = {}  # tgt_id → {uid, ...}
        self._peer_dwell: Dict[str, Dict[str, float]] = {}
        self._last_r_sent: Dict[str, float] = {}
        self._last_c_sent: Dict[str, float] = {}
        self._last_j_sent: Dict[str, float] = {}

    def ingest_comms(self, comm_inbox: List) -> None:
        for msg in comm_inbox:
            payload = msg.payload
            if payload.startswith("R:"):
                try:
                    parts = payload[2:].split(",")
                    if len(parts) == 3:
                        tgt_id, lat, lon = parts[0], float(parts[1]), float(parts[2])
                        if tgt_id not in self._targets:
                            self._targets[tgt_id] = {
                                "pos": (lat, lon), "confirmed": True, "destroyed": False}
                        else:
                            self._targets[tgt_id]["pos"] = (lat, lon)
                            self._targets[tgt_id]["confirmed"] = True
                            self._targets[tgt_id]["destroyed"] = False
                except Exception:
                    pass
            elif payload.startswith("J:"):
                try:
                    parts = payload[2:].split(",")
                    if len(parts) == 2:
                        tgt_id, uid = parts[0], parts[1]
                        if tgt_id not in self._trackers:
                            self._trackers[tgt_id] = set()
                        self._trackers[tgt_id].add(uid)
                except Exception:
                    pass
            elif payload.startswith("T:"):
                try:
                    parts = payload[2:].split(",")
                    if len(parts) >= 3:
                        tgt_id, dwell, uid = parts[0], float(parts[1]), parts[2]
                        if tgt_id not in self._peer_dwell:
                            self._peer_dwell[tgt_id] = {}
                        if dwell > self._peer_dwell[tgt_id].get(uid, 0.0):
                            self._peer_dwell[tgt_id][uid] = dwell
                        if tgt_id not in self._trackers:
                            self._trackers[tgt_id] = set()
                        self._trackers[tgt_id].add(uid)
                except Exception:
                    pass
            elif payload.startswith("C:"):
                tgt_id = payload[2:]
                if tgt_id in self._targets:
                    self._targets[tgt_id]["destroyed"] = True
                if tgt_id in self._trackers:
                    self._trackers[tgt_id].clear()

    # ── 槽位管理 ──

    def total_tracker_count(self, tgt_id: str) -> int:
        return len(self._trackers.get(tgt_id, set()))

    def slot_available(self, tgt_id: str) -> bool:
        return self.total_tracker_count(tgt_id) < self.k

    def claim_slot(self, tgt_id: str) -> None:
        if tgt_id not in self._trackers:
            self._trackers[tgt_id] = set()
        self._trackers[tgt_id].add(self.my_uid)

    def release_slot(self, tgt_id: str) -> None:
        if tgt_id in self._trackers:
            self._trackers[tgt_id].discard(self.my_uid)

    def need_j_broadcast(self, tgt_id: str, now: float, cooldown: float = 3.0) -> bool:
        return (tgt_id not in self._last_j_sent
                or (now - self._last_j_sent[tgt_id]) > cooldown)

    def mark_j_sent(self, tgt_id: str, now: float) -> None:
        self._last_j_sent[tgt_id] = now

    # ── 目标管理 ──

    def confirm_target(self, lat: float, lon: float) -> str:
        tgt_id = _make_target_id(lat, lon)
        if tgt_id not in self._targets:
            self._targets[tgt_id] = {
                "pos": (lat, lon), "confirmed": True, "destroyed": False}
        else:
            self._targets[tgt_id]["pos"] = (lat, lon)
            self._targets[tgt_id]["confirmed"] = True
            self._targets[tgt_id]["destroyed"] = False
        return tgt_id

    def confirm_decoy(self, lat: float, lon: float) -> None:
        tgt_id = _make_target_id(lat, lon)
        self._targets[tgt_id] = {
            "pos": (lat, lon), "confirmed": False, "destroyed": True}

    def select_target(self, self_lat: float, self_lon: float) -> Optional[str]:
        """选择最近的、有空槽位的、已确认未摧毁目标。"""
        best_id = None
        best_dist = float("inf")
        for tgt_id, info in self._targets.items():
            if info.get("destroyed", False) or not info.get("confirmed", False):
                continue
            if not self.slot_available(tgt_id):
                continue
            d = _haversine_m(self_lat, self_lon, info["pos"][0], info["pos"][1])
            if d < best_dist:
                best_dist = d
                best_id = tgt_id
        return best_id

    def is_destroyed(self, tgt_id: str) -> bool:
        return self._targets.get(tgt_id, {}).get("destroyed", False)

    def target_pos(self, tgt_id: str) -> Optional[Tuple[float, float]]:
        return self._targets.get(tgt_id, {}).get("pos")

    def slot(self, tgt_id: str) -> int:
        """槽位 0 或 1，基于 UID 字典序排序。"""
        trackers = sorted(self._trackers.get(tgt_id, set()))
        if self.my_uid in trackers:
            return trackers.index(self.my_uid) % 2
        return 0 if self.my_uid < "20003" else 1

    def aim_point(self, tgt_id: str, slot: int,
                  standoff: float = 250.0) -> Tuple[float, float]:
        pos = self.target_pos(tgt_id)
        if pos is None:
            return _MAP_CENTER
        lat, lon = pos
        heading = 0.0 if slot == 0 else 180.0
        dlat = standoff * math.cos(math.radians(heading)) / 111320.0
        dlon = standoff * math.sin(math.radians(heading)) / \
               (111320.0 * math.cos(math.radians(lat)))
        return (lat + dlat, lon + dlon)

    def need_r_broadcast(self, tgt_id: str, now: float, cooldown: float = 2.0) -> bool:
        return (tgt_id not in self._last_r_sent
                or (now - self._last_r_sent[tgt_id]) > cooldown)

    def need_c_broadcast(self, tgt_id: str, now: float, cooldown: float = 2.0) -> bool:
        return (tgt_id not in self._last_c_sent
                or (now - self._last_c_sent[tgt_id]) > cooldown)

    def mark_r_sent(self, tgt_id: str, now: float) -> None:
        self._last_r_sent[tgt_id] = now

    def mark_c_sent(self, tgt_id: str, now: float) -> None:
        self._last_c_sent[tgt_id] = now

    def peer_total_dwell(self, tgt_id: str) -> float:
        peers = self._peer_dwell.get(tgt_id, {})
        return sum(v for uid, v in peers.items() if uid != self.my_uid)

    def mark_destroyed(self, tgt_id: str) -> None:
        if tgt_id in self._targets:
            self._targets[tgt_id]["destroyed"] = True
        if tgt_id in self._trackers:
            self._trackers[tgt_id].clear()


# ── 智能体主体 ────────────────────────────────────────────────────────────────
class CoopDistributedAgent(CoopAgent):
    """
    三机协同智能体（2 锁定 + 1 搜索 + 2 分钟归中）。
    状态机：SEARCH → VERIFY → TRACK → SEARCH
                      ↘ RETURN → SEARCH
    """
    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    RETURN = "RETURN"

    def configure(self, config) -> None:
        # ── 螺旋搜索参数 ──
        self._search_alt: float = 200.0
        self._spiral_max_radius: float = 2500.0
        self._spiral_growth: float = 80.0
        self._spiral_ang_speed: float = 18.0
        self._search_speed: float = 25.0

        # ── 云台扫描参数 ──
        self._sweep_period: float = 5.0
        self._pitch_min: float = -65.0
        self._pitch_max: float = -25.0
        self._search_fov: float = 60.0
        self._track_fov: float = 45.0

        # ── 验证参数 ──
        self._verify_timeout: float = 6.0
        self._verify_warmup: float = 2.0
        self._verify_speed_confirm: float = 3.5
        self._verify_r2_confirm: float = 0.6
        self._verify_speed_reject: float = 2.0
        self._verify_reject_min_t: float = 3.0
        self._verify_speed_reject: float = 2.0
        self._verify_r2_reject: float = 0.3
        self._ema_alpha: float = 0.3

        # ── 跟踪/摧毁参数 ──
        self._dwell_target: float = 20.0      # 20 秒锁定摧毁
        self._dwell_grace: float = 3.0
        self._track_timeout: float = 50.0
        self._loiter_radius: float = 200.0

        # ── 2 分钟归中参数 ──
        self._idle_timeout: float = 120.0
        self._return_speed: float = 30.0
        self._return_arrive_threshold: float = 100.0

        # ── 通信参数 ──
        self._status_period: int = 5
        self._r_cooldown: float = 2.0

        # ── 运行时状态 ──
        self._t: float = 0.0
        self._tick: int = 0
        self._uav_idx: int = 0
        self._phase_offset: float = 0.0
        self._state = self.SEARCH
        self._candidate: Optional[Tuple[float, float]] = None
        self._ema = _EMATracker(self._ema_alpha)
        self._verify_t: float = 0.0
        self._dwell_time: float = 0.0
        self._last_det_tick: float = -1e9
        self._track_t: float = 0.0
        self._last_report_t: float = -1e9
        self._known_decoys: List[Tuple[float, float]] = []
        self._coord = CooperativeCoordinator(self.my_uid, k=2)
        self._track_target_id: Optional[str] = None
        self._tracking_active: bool = False
        self._last_any_det_t: float = 0.0      # 最后一次检测到任何目标的时间

    def reset(self) -> None:
        self._t = 0.0
        self._tick = 0
        self._uav_idx = _uid_index(self.my_uid)
        self._phase_offset = self._uav_idx * 120.0
        self._state = self.SEARCH
        self._candidate = None
        self._ema = _EMATracker(self._ema_alpha)
        self._verify_t = 0.0
        self._dwell_time = 0.0
        self._last_det_tick = -1e9
        self._track_t = 0.0
        self._last_report_t = -1e9
        self._known_decoys = []
        self._coord = CooperativeCoordinator(self.my_uid, k=2)
        self._track_target_id = None
        self._tracking_active = False
        self._last_any_det_t = 0.0

    # ── 辅助方法 ──

    def _tracking_gimbal(self, self_lat, self_lon, self_heading,
                         tgt_lat, tgt_lon) -> Tuple[float, float]:
        brg = _bearing_deg(self_lat, self_lon, tgt_lat, tgt_lon)
        diff_rad = math.atan2(math.sin(math.radians(brg - self_heading)),
                              math.cos(math.radians(brg - self_heading)))
        pan = math.degrees(diff_rad)
        ground = max(1.0, _haversine_m(self_lat, self_lon, tgt_lat, tgt_lon))
        tilt = -math.degrees(math.atan2(self._search_alt, ground))
        return pan, tilt

    def _spiral_waypoint(self) -> Tuple[float, float, float, float]:
        """
        阿基米德螺旋搜索航点。
        三机从中心出发，120° 间隔，螺旋向外推进。
        到达最大半径后保持圆周运动（由 2 分钟归中逻辑接管返回）。
        """
        center_lat, center_lon = _MAP_CENTER
        t = self._t
        bearing = (self._spiral_ang_speed * t + self._phase_offset) % 360.0
        total_angle = self._spiral_ang_speed * t
        revs = total_angle / 360.0
        # 半径增长到最大值后保持（钳位），不再自动重置
        radius = min(self._spiral_max_radius, max(10.0, self._spiral_growth * revs))

        dlat = (radius * math.cos(math.radians(bearing))) / 111320.0
        dlon = (radius * math.sin(math.radians(bearing))) / \
               (111320.0 * math.cos(math.radians(center_lat)))

        phase = (t % self._sweep_period) / self._sweep_period
        tilt = self._pitch_min + (self._pitch_max - self._pitch_min) * 0.5 * \
               (1 - math.cos(2 * math.pi * phase))
        pan_phase = (t % (self._sweep_period * 2)) / (self._sweep_period * 2)
        pan = -90.0 + 180.0 * 0.5 * (1 - math.cos(2 * math.pi * pan_phase))

        return center_lat + dlat, center_lon + dlon, pan, tilt

    def _enter_track(self, tgt_id: str, tgt_pos: Tuple[float, float]) -> None:
        self._track_target_id = tgt_id
        self._state = self.TRACK
        self._dwell_time = 0.0
        self._track_t = 0.0
        self._last_det_tick = self._t
        self._tracking_active = False
        self._ema = _EMATracker(self._ema_alpha)
        self._ema.append(tgt_pos[0], tgt_pos[1])

    def _do_search(self, obs, cmds) -> List[Command]:
        slat, slon, pan, tilt = self._spiral_waypoint()
        slat, slon = _clamp_to_safebox(slat, slon)
        cmds.append(fly_to(slat, slon, alt=self._search_alt, speed=self._search_speed))
        cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(self._search_fov))
        return cmds

    def _do_return_center(self, obs, cmds) -> List[Command]:
        clat, clon = _clamp_to_safebox(*_MAP_CENTER)
        cmds.append(fly_to(clat, clon, alt=self._search_alt, speed=self._return_speed))
        phase = (self._t % self._sweep_period) / self._sweep_period
        tilt = self._pitch_min + (self._pitch_max - self._pitch_min) * 0.5 * \
               (1 - math.cos(2 * math.pi * phase))
        pan_phase = (self._t % (self._sweep_period * 2)) / (self._sweep_period * 2)
        pan = -90.0 + 180.0 * 0.5 * (1 - math.cos(2 * math.pi * pan_phase))
        cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(self._search_fov))
        return cmds

    # ── 主决策函数 ──
    def decide(self, obs: CoopObs, dt: float) -> List[Command]:
        self._tick += 1
        self._t += dt
        sim_t = self._t
        det = obs.self.detection
        cmds: List[Command] = []
        self._coord.ingest_comms(obs.comm_inbox)

        # ★ 全局：更新"最后一次看到任何东西"的计时器
        if det.detected and det.target_lat is not None:
            self._last_any_det_t = sim_t

        # ══════════════════════════════════════════════════════
        #  SEARCH — 螺旋搜索
        # ══════════════════════════════════════════════════════
        if self._state == self.SEARCH:
            # ★ 2 分钟无任何检测 → 归中
            idle_duration = sim_t - self._last_any_det_t
            if idle_duration >= self._idle_timeout:
                self._state = self.RETURN
                return self._do_return_center(obs, cmds)

            # 检查队友召唤（有空槽位才去）
            active_tgt = self._coord.select_target(obs.self.lat, obs.self.lon)
            if active_tgt is not None:
                tgt_pos = self._coord.target_pos(active_tgt)
                if tgt_pos is not None:
                    # ★ 二次检查：防止多架飞机同帧竞争导致超额
                    if self._coord.slot_available(active_tgt):
                        self._coord.claim_slot(active_tgt)
                        if self._coord.need_j_broadcast(active_tgt, sim_t):
                            cmds.append(broadcast(f"J:{active_tgt},{self.my_uid}"))
                            self._coord.mark_j_sent(active_tgt, sim_t)
                        self._enter_track(active_tgt, tgt_pos)
                    # 如果槽位刚被占满，继续搜索

            if self._state == self.SEARCH:
                if det.detected and det.target_lat is not None:
                    near_decoy = any(
                        _haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                        for d in self._known_decoys
                    ) if self._known_decoys else False
                    if not near_decoy:
                        self._state = self.VERIFY
                        self._candidate = (det.target_lat, det.target_lon)
                        self._ema = _EMATracker(self._ema_alpha)
                        self._ema.append(det.target_lat, det.target_lon)
                        self._verify_t = 0.0

                if self._state == self.SEARCH:
                    return self._do_search(obs, cmds)

        # ══════════════════════════════════════════════════════
        #  RETURN — 返回中心
        # ══════════════════════════════════════════════════════
        if self._state == self.RETURN:
            if det.detected and det.target_lat is not None:
                self._last_any_det_t = sim_t
                near_decoy = any(
                    _haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                    for d in self._known_decoys
                ) if self._known_decoys else False
                if not near_decoy:
                    self._state = self.VERIFY
                    self._candidate = (det.target_lat, det.target_lon)
                    self._ema = _EMATracker(self._ema_alpha)
                    self._ema.append(det.target_lat, det.target_lon)
                    self._verify_t = 0.0

            if self._state == self.RETURN:
                dist_to_center = _haversine_m(
                    obs.self.lat, obs.self.lon, _MAP_CENTER[0], _MAP_CENTER[1])
                if dist_to_center < self._return_arrive_threshold:
                    self._state = self.SEARCH
                    self._last_any_det_t = sim_t  # ★ 到达中心后重置计时
                    return self._do_search(obs, cmds)
                return self._do_return_center(obs, cmds)

        # ══════════════════════════════════════════════════════
        #  VERIFY — 真假目标判别
        # ══════════════════════════════════════════════════════
        # ══════════════════════════════════════════════════════
        #  VERIFY — 真假目标判别（速度 + 线性度双重校验）
        # ══════════════════════════════════════════════════════
        if self._state == self.VERIFY:
            self._verify_t += dt
            tgt = self._candidate

            if det.detected and det.target_lat is not None:
                self._last_any_det_t = sim_t
                d = _haversine_m(det.target_lat, det.target_lon, tgt[0], tgt[1])
                if d < 250.0:
                    self._ema.append(det.target_lat, det.target_lon)
                    self._candidate = self._ema.value
                tgt = self._candidate

        #使用新的双重指标
        speed, r_squared = self._ema.speed_and_linearity()

        # 确认真目标：预热完成 + 速度达标 + 轨迹足够直
        confirmed = (
            self._verify_t >= self._verify_warmup
            and speed >= self._verify_speed_confirm
            and r_squared >= self._verify_r2_confirm
        )

        # 拒绝诱饵：最短时间过 + (速度太低 OR 轨迹太乱) + 样本充足
        rejected = (
            self._verify_t >= self._verify_reject_min_t
            and self._ema.count >= 15
            and (speed < self._verify_speed_reject or r_squared < self._verify_r2_reject)
        )

        if confirmed:
            lat, lon = tgt
            tgt_id = self._coord.confirm_target(lat, lon)
            if self._coord.slot_available(tgt_id):
                if self._coord.need_r_broadcast(tgt_id, sim_t, self._r_cooldown):
                    cmds.append(broadcast(f"R:{tgt_id},{lat:.5f},{lon:.5f}"))
                    self._coord.mark_r_sent(tgt_id, sim_t)
            self._coord.claim_slot(tgt_id)
            if self._coord.need_j_broadcast(tgt_id, sim_t):
                cmds.append(broadcast(f"J:{tgt_id},{self.my_uid}"))
                self._coord.mark_j_sent(tgt_id, sim_t)
            self._enter_track(tgt_id, tgt)

        elif rejected or self._verify_t >= self._verify_timeout:
            if rejected and self._ema.value:
                self._known_decoys.append(self._ema.value)
                self._coord.confirm_decoy(*self._ema.value)
            self._state = self.SEARCH
            self._candidate = None
            self._ema.reset()
            return self._do_search(obs, cmds)
        else:
            # 验证进行中：飞向候选点保持观察
            tlat, tlon = _clamp_to_safebox(*tgt)
            cmds.append(fly_to(tlat, tlon, alt=self._search_alt,
                            speed=self._search_speed,
                            loiter_radius=self._loiter_radius))
            pan, tilt = self._tracking_gimbal(
                obs.self.lat, obs.self.lon, obs.self.heading_deg, tgt[0], tgt[1])
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(self._track_fov))
            return cmds