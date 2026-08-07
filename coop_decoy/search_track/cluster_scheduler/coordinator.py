
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

from __future__ import annotations

import hashlib
import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

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
            params = {"latitude": float(lat), "longitude": float(lon), "loiter_radius": float(loiter_radius)}
            if alt is not None: params["altitude"] = float(alt)
            if speed is not None: params["speed"] = float(speed)
            return Command("set_destination", params)

        def point_gimbal(pan, tilt):
            return Command("component.gimbal_tracking.set_orientation", {"pan": float(pan), "tilt": float(tilt)})

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

# ── 任务几何参数 ─────────────────────────────────────────────────────────────
_BBOX: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (26.982, 124.980), (27.025, 125.020))
_SAFEBOX_MARGIN_M = 600.0


def _bbox_inset(bbox, margin_m: float):
    """从边界框向内收缩指定距离（米），得到安全飞行区域。"""
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    lat_mid = (lat_min + lat_max) / 2
    dlat = margin_m / 111320.0
    dlon = margin_m / (111320.0 * math.cos(math.radians(lat_mid)))
    return ((lat_min + dlat, lon_min + dlon), (lat_max - dlat, lon_max - dlon))


_SAFEBOX = _bbox_inset(_BBOX, _SAFEBOX_MARGIN_M)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """使用 Haversine 公式计算两点间距离（米）。"""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算从点1到点2的绝对方位角（0°=北，顺时针）。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = (math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _clamp_to_safebox(lat: float, lon: float) -> Tuple[float, float]:
    """将坐标限制在安全飞行区域内。"""
    (lat_min, lon_min), (lat_max, lon_max) = _SAFEBOX
    return (min(max(lat, lat_min), lat_max),
            min(max(lon, lon_min), lon_max))


def _partition_centers(bbox, n: int = 3):
    """将边界框按经度均分为 n 个分区，返回各分区中心坐标列表。"""
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    lat_mid = (lat_min + lat_max) / 2
    sub_w = (lon_max - lon_min) / n
    return [(lat_mid, lon_min + sub_w * (i + 0.5)) for i in range(n)]


_PARTITION_CENTERS = _partition_centers(_BBOX, 3)


def _uid_phase(uid: str) -> float:
    """根据 UID 生成相位偏移（0～1），用于搜索轨迹差异化。"""
    h = int(hashlib.md5(uid.encode()).hexdigest(), 16)
    return (h % 1000) / 1000.0


def _uid_partition(uid: str) -> Tuple[float, float]:
    """将 UID 映射到一个分区中心，确保多架无人机搜索不同区域。"""
    n = len(_PARTITION_CENTERS)
    if uid.isdigit():
        idx = int(uid) % n
    elif "_" in uid:
        tail = uid.rsplit("_", 1)[-1]
        idx = int(tail) % n if tail.isdigit() else (
            int(hashlib.md5(uid.encode()).hexdigest(), 16) % n)
    else:
        idx = int(hashlib.md5(uid.encode()).hexdigest(), 16) % n
    return _PARTITION_CENTERS[idx]


# ── 目标 ID 生成 ─────────────────────────────────────────────────────────────
def _make_target_id(lat: float, lon: float) -> str:
    """根据坐标生成确定性的目标 ID（多机一致）。"""
    return f"{lat:.4f}_{lon:.4f}"


# ── EMA 跟踪器（未修改） ────────────────────────────────────────────────────
class _EMATracker:
    """基于指数移动平均和线性回归速度估计的目标位置跟踪器。"""

    def __init__(self, alpha: float = 0.3, history: int = 80):
        self._alpha = alpha
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._raw: Deque[Tuple[float, float]] = deque(maxlen=history)

    def append(self, lat: float, lon: float) -> None:
        """添加新检测点，更新 EMA 位置和原始数据缓冲区。"""
        if self._lat is None:
            self._lat, self._lon = lat, lon
        else:
            a = self._alpha
            self._lat = self._lat * (1 - a) + lat * a
            self._lon = self._lon * (1 - a) + lon * a
        self._raw.append((lat, lon))

    @property
    def value(self) -> Optional[Tuple[float, float]]:
        """返回当前 EMA 平滑位置。"""
        if self._lat is None:
            return None
        return (self._lat, self._lon)

    def speed_mps(self, tick_hz: float = 10.0) -> float:
        """通过原始纬度序列的线性回归斜率估算目标速度（米/秒）。"""
        n = len(self._raw)
        if n < 10:
            return 0.0
        ts = list(range(n))
        lats = [p[0] for p in self._raw]
        ns = float(n)
        sx = sum(ts); sy = sum(lats)
        sxx = sum(t * t for t in ts)
        sxy = sum(t * la for t, la in zip(ts, lats))
        denom = ns * sxx - sx * sx
        if abs(denom) < 1e-20:
            return 0.0
        slope = (ns * sxy - sx * sy) / denom   # 度/帧
        return abs(slope) * 111320.0 * tick_hz  # 转换为米/秒

    def reset(self) -> None:
        """重置跟踪器状态。"""
        self._lat = self._lon = None
        self._raw.clear()


# ── K=2 协同调度器 ─────────────────────────────────────────────────────────
class CooperativeCoordinator:
    """
    用于 K=2 目标协同攻击的分布式调度器。

    通信协议消息格式：
      R:<tgt_id>,<lat>,<lon>   — 发现真目标，召唤队友
      T:<tgt_id>,<dwell>       — 定期广播本机 dwell 状态
      C:<tgt_id>               — 目标已摧毁通知
    """

    def __init__(self, my_uid: str, k: int = 2):
        self.my_uid = my_uid
        self.k = k  # 需要同时观测的数量（固定为2）
        # 活动目标记录：tgt_id -> {'pos': (lat,lon), 'confirmed': bool, 'destroyed': bool}
        self._targets: Dict[str, dict] = {}
        # 队友的 dwell 报告：tgt_id -> 最新 dwell 值
        self._peer_dwell: Dict[str, float] = {}
        # 记录上次发送 R / C 消息的时间，避免频繁广播
        self._last_r_sent: Dict[str, float] = {}
        self._last_c_sent: Dict[str, float] = {}

    # ── 公开 API ──────────────────────────────────────────────────────────

    def ingest_comms(self, comm_inbox: List) -> None:
        """解析通信收件箱，更新目标列表和队友状态。"""
        for msg in comm_inbox:
            payload = msg.payload
            if payload.startswith("R:"):
                # R:<tgt_id>,<lat>,<lon>
                try:
                    parts = payload[2:].split(",")
                    if len(parts) == 3:
                        tgt_id = parts[0]
                        lat, lon = float(parts[1]), float(parts[2])
                        if tgt_id not in self._targets:
                            self._targets[tgt_id] = {
                                "pos": (lat, lon), "confirmed": True, "destroyed": False}
                except Exception:
                    pass
            elif payload.startswith("T:"):
                # T:<tgt_id>,<dwell>
                try:
                    parts = payload[2:].split(",")
                    if len(parts) == 2:
                        tgt_id = parts[0]
                        dwell = float(parts[1])
                        self._peer_dwell[tgt_id] = dwell
                except Exception:
                    pass
            elif payload.startswith("C:"):
                # C:<tgt_id>
                tgt_id = payload[2:]
                if tgt_id in self._targets:
                    self._targets[tgt_id]["destroyed"] = True

    def confirm_target(self, lat: float, lon: float) -> str:
        """确认真目标，生成或更新其记录，并返回目标 ID。"""
        tgt_id = _make_target_id(lat, lon)
        if tgt_id not in self._targets:
            self._targets[tgt_id] = {
                "pos": (lat, lon), "confirmed": True, "destroyed": False}
        else:
            # 更新位置信息
            self._targets[tgt_id]["pos"] = (lat, lon)
            self._targets[tgt_id]["confirmed"] = True
            self._targets[tgt_id]["destroyed"] = False
        return tgt_id

    def confirm_decoy(self, lat: float, lon: float) -> None:
        """标记为假目标，此后不再被选中。"""
        tgt_id = _make_target_id(lat, lon)
        self._targets[tgt_id] = {
            "pos": (lat, lon), "confirmed": False, "destroyed": True}  # 视为已摧毁

    def select_target(self, self_lat: float, self_lon: float) -> Optional[str]:
        """
        选择最优的当前活动目标（未被摧毁）。
        优先选择已由队友召唤的目标（R 消息收到），其次选择自己发现的目标，
        同优先级下选择距离最近的。
        """
        best_id = None
        best_dist = float("inf")

        for tgt_id, info in self._targets.items():
            if info.get("destroyed", False) or not info.get("confirmed", False):
                continue
            d = _haversine_m(self_lat, self_lon, info["pos"][0], info["pos"][1])
            if d < best_dist:
                best_dist = d
                best_id = tgt_id
        return best_id

    def is_destroyed(self, tgt_id: str) -> bool:
        """检查目标是否已被标记为摧毁。"""
        return self._targets.get(tgt_id, {}).get("destroyed", False)

    def target_pos(self, tgt_id: str) -> Optional[Tuple[float, float]]:
        """获取目标当前位置。"""
        return self._targets.get(tgt_id, {}).get("pos")

    def my_slot(self, tgt_id: str) -> int:
        """
        分配本机在目标周围的槽位（0 或 1），保证两架无人机获得不同槽位。
        这里通过比较 UID 大小简单确定：较小者取槽位 0，较大者取槽位 1。
        """
        # 实际部署时可通过哈希 (tgt_id + my_uid) % K 实现更通用分配
        return 0 if self.my_uid < "20003" else 1

    def aim_point(self, tgt_id: str, slot: int,
                  standoff: float = 200.0) -> Tuple[float, float]:
        """
        计算指定槽位的盘旋瞄准点坐标，两槽位间距离 > 200 米。
        槽位 0：目标北侧；槽位 1：目标南侧。
        """
        pos = self.target_pos(tgt_id)
        if pos is None:
            return (0.0, 0.0)
        lat, lon = pos
        # 槽位0向北偏移，槽位1向南偏移（间距约400米）
        heading = 0.0 if slot == 0 else 180.0
        dlat = standoff * math.cos(math.radians(heading)) / 111320.0
        dlon = standoff * math.sin(math.radians(heading)) / \
               (111320.0 * math.cos(math.radians(lat)))
        return (lat + dlat, lon + dlon)

    def need_r_broadcast(self, tgt_id: str, now: float, cooldown: float = 2.0) -> bool:
        """判断是否需要再次广播 R 消息（冷却期内不会重复发送）。"""
        if tgt_id not in self._last_r_sent or (now - self._last_r_sent[tgt_id]) > cooldown:
            return True
        return False

    def need_c_broadcast(self, tgt_id: str, now: float, cooldown: float = 2.0) -> bool:
        """判断是否需要再次广播 C 消息。"""
        if tgt_id not in self._last_c_sent or (now - self._last_c_sent[tgt_id]) > cooldown:
            return True
        return False

    def mark_r_sent(self, tgt_id: str, now: float) -> None:
        """记录 R 消息发送时间。"""
        self._last_r_sent[tgt_id] = now

    def mark_c_sent(self, tgt_id: str, now: float) -> None:
        """记录 C 消息发送时间。"""
        self._last_c_sent[tgt_id] = now

    def peer_dwell(self, tgt_id: str) -> float:
        """获取队友报告的该目标 dwell 时间。"""
        return self._peer_dwell.get(tgt_id, 0.0)

    def mark_destroyed(self, tgt_id: str) -> None:
        """将目标标记为已摧毁。"""
        if tgt_id in self._targets:
            self._targets[tgt_id]["destroyed"] = True


# ── 智能体主体（适配 K=2 协同） ─────────────────────────────────────────────

class CoopDistributedAgent(CoopAgent):
    """支持 K=2 协同的分布式诱饵对抗智能体。"""

    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"

    def configure(self, config) -> None:
        # 搜索几何参数
        self._search_alt: float = 200.0
        self._search_radius: float = 700.0
        self._growth: float = 50.0
        self._ang_speed: float = 30.0
        self._sweep_period: float = 4.0
        self._pitch_min: float = -60.0
        self._pitch_max: float = -30.0
        # 视场角
        self._track_fov: float = 60.0
        self._search_fov: float = 60.0
        # 验证阶段（假目标识别）参数
        self._verify_timeout: float = 8.0
        self._verify_warmup: float = 3.0
        self._verify_speed_confirm: float = 3.0   # 米/秒，高于此值视为真目标
        self._verify_speed_reject: float = 2.5    # 米/秒，低于此值（且够时间）视为假目标
        self._verify_reject_min_t: float = 5.0    # 至少验证这么久才能拒绝
        self._ema_alpha: float = 0.3
        # 跟踪（摧毁）参数
        self._dwell_target: float = 20.0
        self._dwell_grace: float = 2.0
        self._track_timeout: float = 40.0         # 协同模式下适当放宽超时
        self._loiter_close: float = 100.0
        # 通信周期
        self._status_period: int = 5              # 每 N 帧广播一次 T: 状态
        self._r_cooldown: float = 2.0
        # 运行时状态
        self._t: float = 0.0
        self._tick: int = 0
        self._region = _uid_partition(self.my_uid)
        self._phase: float = _uid_phase(self.my_uid)
        self._state = self.SEARCH
        self._candidate: Optional[Tuple[float, float]] = None
        self._ema = _EMATracker(self._ema_alpha)
        self._verify_t: float = 0.0
        self._dwell_time: float = 0.0
        self._last_det_tick: float = -1e9
        self._track_t: float = 0.0
        self._last_report_t: float = -1e9
        self._known_decoys: List[Tuple[float, float]] = []
        # 协同调度器
        self._coord = CooperativeCoordinator(self.my_uid, k=2)
        self._track_target_id: Optional[str] = None  # 当前跟踪的目标 ID

    def reset(self) -> None:
        """重置智能体状态（每回合开始调用）。"""
        self._t = 0.0
        self._tick = 0
        self._state = self.SEARCH
        self._candidate = None
        self._ema = _EMATracker(self._ema_alpha)
        self._verify_t = 0.0
        self._dwell_time = 0.0
        self._last_det_tick = -1e9
        self._track_t = 0.0
        self._last_report_t = -1e9
        self._known_decoys = []
        self._region = _uid_partition(self.my_uid)
        self._phase = _uid_phase(self.my_uid)
        self._coord = CooperativeCoordinator(self.my_uid, k=2)
        self._track_target_id = None

    # ── 辅助函数 ──

    def _tracking_gimbal(self, self_lat, self_lon, self_heading,
                         tgt_lat, tgt_lon) -> Tuple[float, float]:
        """计算对准目标的云台角度（pan, tilt），补偿本机航向。"""
        brg = _bearing_deg(self_lat, self_lon, tgt_lat, tgt_lon)
        pan = ((brg - self_heading + 180.0) % 360.0) - 180.0
        ground = max(1.0, _haversine_m(self_lat, self_lon, tgt_lat, tgt_lon))
        tilt = -math.degrees(math.atan2(self._search_alt, ground))
        return pan, tilt

    def _spiral(self) -> Tuple[float, float, float, float]:
        """
        生成当前时刻的搜索螺旋航点及云台扫描角度。
        返回：(纬度, 经度, 云台pan, 云台tilt)
        """
        home_lat, home_lon = self._region
        t = self._t + self._phase * 12.0
        bearing = (self._ang_speed * t) % 360.0
        revs = (self._ang_speed * t) / 360.0
        radius = max(1.0, min(self._search_radius, self._growth * revs))
        dlat = (radius * math.cos(math.radians(bearing))) / 111320.0
        dlon = (radius * math.sin(math.radians(bearing))) / \
               (111320.0 * math.cos(math.radians(home_lat)))
        phase = (t % self._sweep_period) / self._sweep_period
        tilt = self._pitch_min + (self._pitch_max - self._pitch_min) * 0.5 * \
               (1 - math.cos(2 * math.pi * phase))
        pan_phase = (t % (self._sweep_period * 2)) / (self._sweep_period * 2)
        pan = -90.0 + 180.0 * 0.5 * (1 - math.cos(2 * math.pi * pan_phase))
        return home_lat + dlat, home_lon + dlon, pan, tilt

    # ── 主决策函数 ──

    def decide(self, obs: CoopObs, dt: float) -> List[Command]:
        """每帧调用，生成当前控制指令。"""
        self._tick += 1
        self._t += dt
        sim_t = self._t
        det = obs.self.detection
        cmds: List[Command] = []
        self._coord.ingest_comms(obs.comm_inbox)

        # ── 搜索状态 ────────────────────────────────────────────────────
        if self._state == self.SEARCH:
            # 如果存在活动协同目标，直接切换到跟踪状态
            active_tgt = self._coord.select_target(
                obs.self.lat, obs.self.lon)
            if active_tgt is not None:
                self._track_target_id = active_tgt
                self._state = self.TRACK
                self._dwell_time = 0.0
                self._track_t = 0.0
                self._last_det_tick = sim_t
                # 本帧直接进入跟踪逻辑
            else:
                if det.detected and det.target_lat is not None:
                    # 跳过已知假目标附近的检测
                    near_decoy = any(
                        _haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                        for d in self._known_decoys) if self._known_decoys else False
                    if not near_decoy:
                        self._state = self.VERIFY
                        self._candidate = (det.target_lat, det.target_lon)
                        self._ema = _EMATracker(self._ema_alpha)
                        self._verify_t = 0.0
                if self._state == self.SEARCH:
                    slat, slon, pan, tilt = self._spiral()
                    slat, slon = _clamp_to_safebox(slat, slon)
                    cmds.append(fly_to(slat, slon, alt=self._search_alt, speed=22.0))
                    cmds.append(point_gimbal(pan, tilt))
                    cmds.append(set_gimbal_fov(self._search_fov))
                    return cmds

        # ── 验证状态（真假目标判别） ────────────────────────────────────
        if self._state == self.VERIFY:
            self._verify_t += dt
            tgt = self._candidate
            if det.detected and det.target_lat is not None:
                d = _haversine_m(det.target_lat, det.target_lon,
                                 tgt[0], tgt[1])
                if d < 250.0:
                    self._ema.append(det.target_lat, det.target_lon)
                    self._candidate = self._ema.value
                    tgt = self._candidate
            speed = self._ema.speed_mps()
            confirmed = (self._verify_t >= self._verify_warmup
                         and speed >= self._verify_speed_confirm)
            rejected = (self._verify_t >= self._verify_reject_min_t
                        and speed < self._verify_speed_reject)
            if confirmed:
                # 确认真目标，注册并召唤队友
                lat, lon = tgt
                tgt_id = self._coord.confirm_target(lat, lon)
                if self._coord.need_r_broadcast(tgt_id, sim_t, self._r_cooldown):
                    cmds.append(broadcast(f"R:{tgt_id},{lat:.5f},{lon:.5f}"))
                    self._coord.mark_r_sent(tgt_id, sim_t)
                self._track_target_id = tgt_id
                self._state = self.TRACK
                self._dwell_time = 0.0
                self._track_t = 0.0
                self._last_det_tick = sim_t
                # 继续执行跟踪逻辑
            elif rejected or self._verify_t >= self._verify_timeout:
                # 假目标或超时，放弃并记忆
                if rejected and self._ema.value:
                    self._known_decoys.append(self._ema.value)
                    self._coord.confirm_decoy(*self._ema.value)
                self._state = self.SEARCH
                self._candidate = None
                self._ema.reset()
                slat, slon, pan, tilt = self._spiral()
                slat, slon = _clamp_to_safebox(slat, slon)
                cmds.append(fly_to(slat, slon, alt=self._search_alt, speed=22.0))
                cmds.append(point_gimbal(pan, tilt))
                cmds.append(set_gimbal_fov(self._search_fov))
                return cmds
            else:
                # 验证中：飞向目标并保持观察
                tlat, tlon = _clamp_to_safebox(*tgt)
                cmds.append(fly_to(tlat, tlon, alt=self._search_alt, speed=22.0,
                                   loiter_radius=self._loiter_close))
                pan, tilt = self._tracking_gimbal(
                    obs.self.lat, obs.self.lon, obs.self.heading_deg, tgt[0], tgt[1])
                cmds.append(point_gimbal(pan, tilt))
                cmds.append(set_gimbal_fov(self._track_fov))
                return cmds

        # ── 跟踪状态（协同摧毁） ─────────────────────────────────────────
        if self._state == self.TRACK:
            self._track_t += dt
            tgt_id = self._track_target_id
            if tgt_id is None:
                self._state = self.SEARCH
                return self.decide(obs, 0)

            # 获取目标位置（可能来自队友通信或自身观测）
            tgt_pos = self._coord.target_pos(tgt_id)
            if tgt_pos is None:
                # 目标丢失（可能已摧毁）
                self._state = self.SEARCH
                return self.decide(obs, 0)

            # 自身检测更新目标位置
            if det.detected and det.target_lat is not None:
                d = _haversine_m(det.target_lat, det.target_lon,
                                 tgt_pos[0], tgt_pos[1])
                if d < 250.0:
                    self._coord.confirm_target(det.target_lat, det.target_lon)  # 更新
                    tgt_pos = (det.target_lat, det.target_lon)

            # 积累 dwell 时间
            tracking = det.detected and det.target_lat is not None and \
                _haversine_m(det.target_lat, det.target_lon,
                             tgt_pos[0], tgt_pos[1]) < 250.0
            if tracking:
                gap = sim_t - self._last_det_tick
                if self._dwell_time > 0 and gap <= self._dwell_grace + dt:
                    self._dwell_time += dt
                elif self._dwell_time == 0:
                    self._dwell_time += dt
                else:
                    self._dwell_time = dt
                self._last_det_tick = sim_t

            # 定期广播本机状态
            if self._tick % self._status_period == 0:
                cmds.append(broadcast(f"T:{tgt_id},{self._dwell_time:.2f}"))

            # 检查协同摧毁条件
            peer_dwell = self._coord.peer_dwell(tgt_id)
            destroyed_by_env = (
                self._dwell_time >= self._dwell_target and
                peer_dwell >= self._dwell_target - 0.5  # 容忍小幅通信延迟
            )
            # 超时处理
            if destroyed_by_env or self._track_t >= self._track_timeout:
                if destroyed_by_env:
                    # 广播摧毁通知
                    if self._coord.need_c_broadcast(tgt_id, sim_t):
                        cmds.append(broadcast(f"C:{tgt_id}"))
                        self._coord.mark_c_sent(tgt_id, sim_t)
                    self._coord.mark_destroyed(tgt_id)
                else:
                    # 超时但未达到 dwell，如果目标速度低则标记为假目标
                    if self._dwell_time < self._dwell_target and self._ema.speed_mps() < 2.5:
                        self._coord.confirm_decoy(*tgt_pos)
                # 回到搜索状态
                self._state = self.SEARCH
                self._track_target_id = None
                self._dwell_time = 0.0
                slat, slon, pan, tilt = self._spiral()
                slat, slon = _clamp_to_safebox(slat, slon)
                cmds.append(fly_to(slat, slon, alt=self._search_alt, speed=22.0))
                cmds.append(point_gimbal(pan, tilt))
                cmds.append(set_gimbal_fov(self._search_fov))
                return cmds

            # 槽位分离：飞向本机瞄准点，而非目标正上方
            slot = self._coord.my_slot(tgt_id)
            aim_lat, aim_lon = self._coord.aim_point(tgt_id, slot, standoff=200.0)
            aim_lat, aim_lon = _clamp_to_safebox(aim_lat, aim_lon)
            cmds.append(fly_to(aim_lat, aim_lon, alt=self._search_alt, speed=22.0,
                               loiter_radius=self._loiter_close))

            # 云台始终对准真实目标位置
            pan, tilt = self._tracking_gimbal(
                obs.self.lat, obs.self.lon, obs.self.heading_deg,
                tgt_pos[0], tgt_pos[1])
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(self._track_fov))

            # 仅当目标速度高于阈值时上报位置（避免上报假目标导致 RMSE 恶化）
            if (sim_t - self._last_report_t >= 1.0
                    and self._ema.value is not None
                    and self._ema.speed_mps() > 3.0):
                self._last_report_t = sim_t
                cmds.append(report_target(tgt_pos[0], tgt_pos[1]))
            return cmds

        # 兜底返回搜索指令（理论上不会执行到这里）
        slat, slon, pan, tilt = self._spiral()
        cmds.append(fly_to(slat, slon, alt=self._search_alt, speed=22.0))
        cmds.append(point_gimbal(pan, tilt))
        return cmds