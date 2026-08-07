# -*- coding: utf-8 -*-
"""
集群调度模块入口 — SchedulerBrain 核心决策类
============================================================
对外固定接口:
    SchedulerBrain.decide(target_list, recv_msg) -> dict

功能:
    接收视觉识别的目标列表 + 通信模块的队友消息，
    通过状态机判断当前全局局面，协调多机分工，
    输出控制指令字典给运动模块。

状态机流程:
    SEARCH → VERIFY → TRACK → RELEASE → SEARCH
       ^                                    |
       |________ K=2 召唤 (R:消息) _________|

盯防规则 (K=2):
    >=2架UAV同时对同一真目标连续有效跟踪累计满20s 方可摧毁。
    短暂中断 ≤2s 不清零且回补；中断 >2s 清零重来。

负责人: 成员3
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

from .coordinator import CooperativeCoordinator

# ── 全局常量 ─────────────────────────────────────────────────────────────
_BBOX: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (26.982, 124.980), (27.025, 125.020))
_SAFEBOX_MARGIN_M: float = 600.0
_SEARCH_ALT: float = 200.0
_SEARCH_RADIUS: float = 700.0
_GROWTH: float = 50.0
_ANG_SPEED: float = 30.0
_SWEEP_PERIOD: float = 4.0
_PITCH_MIN: float = -60.0
_PITCH_MAX: float = -30.0
_TRACK_FOV: float = 60.0
_SEARCH_FOV: float = 60.0
_LOITER_CLOSE: float = 100.0
_SPEED: float = 22.0


# ── 工具函数 ─────────────────────────────────────────────────────────────
def _bbox_inset(bbox, margin_m: float):
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    lat_mid = (lat_min + lat_max) / 2
    dlat = margin_m / 111320.0
    dlon = margin_m / (111320.0 * math.cos(math.radians(lat_mid)))
    return ((lat_min + dlat, lon_min + dlon), (lat_max - dlat, lon_max - dlon))


_SAFEBOX = _bbox_inset(_BBOX, _SAFEBOX_MARGIN_M)


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
    x = (math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _clamp_to_safebox(lat: float, lon: float) -> Tuple[float, float]:
    (lat_min, lon_min), (lat_max, lon_max) = _SAFEBOX
    return (min(max(lat, lat_min), lat_max), min(max(lon, lon_min), lon_max))


# ── EMA 目标跟踪器 ───────────────────────────────────────────────────────
class _EMATracker:
    """基于指数移动平均 + 线性回归速度估计的目标位置跟踪器。"""

    def __init__(self, alpha: float = 0.3, history: int = 80):
        self._alpha = alpha
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._raw: Deque[Tuple[float, float]] = deque(maxlen=history)

    def append(self, lat: float, lon: float) -> None:
        if self._lat is None:
            self._lat, self._lon = lat, lon
        else:
            a = self._alpha
            self._lat = self._lat * (1 - a) + lat * a
            self._lon = self._lon * (1 - a) + lon * a
        self._raw.append((lat, lon))

    @property
    def value(self) -> Optional[Tuple[float, float]]:
        if self._lat is None:
            return None
        return (self._lat, self._lon)

    def speed_mps(self, tick_hz: float = 10.0) -> float:
        n = len(self._raw)
        if n < 10:
            return 0.0
        ts = list(range(n))
        lats = [p[0] for p in self._raw]
        ns = float(n)
        sx = sum(ts)
        sy = sum(lats)
        sxx = sum(t * t for t in ts)
        sxy = sum(t * la for t, la in zip(ts, lats))
        denom = ns * sxx - sx * sx
        if abs(denom) < 1e-20:
            return 0.0
        slope = (ns * sxy - sx * sy) / denom
        return abs(slope) * 111320.0 * tick_hz

    def reset(self) -> None:
        self._lat = self._lon = None
        self._raw.clear()


# ── 状态机 ────────────────────────────────────────────────────────────────
class StateMachine:
    """
    单机状态机：管理 UAV 的 SEARCH → VERIFY → TRACK → RELEASE 状态流转。

    状态说明:
        SEARCH:   扇区搜索巡航，发现目标后转入 VERIFY
        VERIFY:   多帧验证目标真伪 (EMA 滤波 + 诱饵判别)
        TRACK:    K=2 协同盯防，累计 dwell 时间
        RELEASE:  目标摧毁后释放，转向下一未完成目标
    """

    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    RELEASE = "RELEASE"

    def __init__(self):
        self._state: str = self.SEARCH
        self._dwell_time: float = 0.0
        self._track_target_id: Optional[str] = None
        self._candidate: Optional[Tuple[float, float]] = None
        self._ema: _EMATracker = _EMATracker()
        self._verify_t: float = 0.0
        self._track_t: float = 0.0
        self._last_det_tick: float = -1e9
        self._last_report_t: float = -1e9
        self._known_decoys: List[Tuple[float, float]] = []

        # 验证阶段参数
        self._verify_timeout: float = 8.0
        self._verify_warmup: float = 3.0
        self._verify_speed_confirm: float = 3.0
        self._verify_speed_reject: float = 2.5
        self._verify_reject_min_t: float = 5.0

        # 跟踪阶段参数
        self._dwell_target: float = 20.0
        self._dwell_grace: float = 2.0
        self._track_timeout: float = 40.0

    # ── 属性 ──────────────────────────────────────────────────────────
    @property
    def state(self) -> str:
        return self._state

    @property
    def dwell_time(self) -> float:
        return self._dwell_time

    @property
    def track_target_id(self) -> Optional[str]:
        return self._track_target_id

    @property
    def candidate(self) -> Optional[Tuple[float, float]]:
        return self._candidate

    @property
    def ema_value(self) -> Optional[Tuple[float, float]]:
        return self._ema.value

    def ema_speed_mps(self) -> float:
        return self._ema.speed_mps()

    @property
    def verify_t(self) -> float:
        return self._verify_t

    @property
    def track_t(self) -> float:
        return self._track_t

    @property
    def last_report_t(self) -> float:
        return self._last_report_t

    @property
    def known_decoys(self) -> List[Tuple[float, float]]:
        return self._known_decoys

    # ── 状态转换 ──────────────────────────────────────────────────────
    def refresh(self, target_list: List[dict], recv_msg: List,
                self_lat: float, self_lon: float,
                sim_t: float, dt: float, dt_frames: float = 0.1) -> None:
        """
        每帧调用，根据当前观测更新状态机。

        Args:
            target_list: 视觉模块检测到的目标列表
            recv_msg:    通信模块收到的队友消息
            self_lat:    本机当前纬度
            self_lon:    本机当前经度
            sim_t:       仿真时间 (秒)
            dt:          帧间隔 (秒)
        """
        # --- SEARCH 状态 ---
        if self._state == self.SEARCH:
            self._handle_search(target_list, self_lat, self_lon, sim_t)

        # --- VERIFY 状态 ---
        elif self._state == self.VERIFY:
            self._handle_verify(target_list, dt, sim_t)

        # --- TRACK 状态 ---
        elif self._state == self.TRACK:
            self._handle_track(target_list, sim_t)

        # --- RELEASE 状态 ---
        elif self._state == self.RELEASE:
            self._transition_to(self.SEARCH)

    def _handle_search(self, target_list: List[dict],
                       self_lat: float, self_lon: float,
                       sim_t: float) -> None:
        """搜索状态：检测到目标则切换到验证。"""
        if not target_list:
            return

        # 选最近且非已知假目标
        best = None
        best_dist = float("inf")
        for tgt in target_list:
            tlat = tgt.get("lat")
            tlon = tgt.get("lon")
            if tlat is None or tlon is None:
                continue
            d = _haversine_m(self_lat, self_lon, tlat, tlon)
            # 跳过已知假目标附近
            near_decoy = any(
                _haversine_m(tlat, tlon, dlat, dlon) < 150.0
                for dlat, dlon in self._known_decoys)
            if near_decoy:
                continue
            if d < best_dist:
                best_dist = d
                best = (tlat, tlon)

        if best is not None:
            self._state = self.VERIFY
            self._candidate = best
            self._ema.reset()
            self._verify_t = 0.0

    def _handle_verify(self, target_list: List[dict],
                       dt: float, sim_t: float) -> None:
        """验证状态：多帧 EMA 滤波 + 速度判别真伪。"""
        self._verify_t += dt
        tgt = self._candidate
        if tgt is None:
            self._transition_to(self.SEARCH)
            return

        # 用最新检测更新 EMA
        for det in target_list:
            dlat = det.get("lat")
            dlon = det.get("lon")
            if dlat is None or dlon is None:
                continue
            d = _haversine_m(dlat, dlon, tgt[0], tgt[1])
            if d < 250.0:
                self._ema.append(dlat, dlon)
                if self._ema.value:
                    self._candidate = self._ema.value
                    tgt = self._candidate

        speed = self._ema.speed_mps()
        confirmed = (self._verify_t >= self._verify_warmup
                     and speed >= self._verify_speed_confirm)
        rejected = (self._verify_t >= self._verify_reject_min_t
                    and speed < self._verify_speed_reject)

        if confirmed:
            self._transition_to(self.TRACK)
        elif rejected or self._verify_t >= self._verify_timeout:
            if rejected and self._ema.value:
                self._known_decoys.append(self._ema.value)
            self._transition_to(self.SEARCH)
            self._candidate = None

    def _handle_track(self, target_list: List[dict], sim_t: float) -> None:
        """跟踪状态：判断是否仍在有效跟踪目标。"""
        self._track_t += 0.1  # 近似帧间隔
        if self._track_target_id is None:
            self._transition_to(self.SEARCH)
            return

        # 判断本帧是否检测到目标
        tracking = False
        tgt_id = self._track_target_id
        for det in target_list:
            dlat = det.get("lat")
            dlon = det.get("lon")
            if dlat is None or dlon is None:
                continue
            tracking = True
            break

        # 更新 dwell 时间
        if tracking:
            gap = sim_t - self._last_det_tick
            if self._dwell_time > 0 and gap <= self._dwell_grace + 0.1:
                self._dwell_time += 0.1
            elif self._dwell_time == 0:
                self._dwell_time += 0.1
            else:
                self._dwell_time = 0.1
            self._last_det_tick = sim_t

        # 超时处理
        if self._track_t >= self._track_timeout:
            self._transition_to(self.RELEASE)

    def _transition_to(self, new_state: str) -> None:
        """执行状态切换，重置相关计时器。"""
        self._state = new_state
        if new_state == self.SEARCH:
            self._candidate = None
            self._ema.reset()
            self._verify_t = 0.0
        elif new_state == self.TRACK:
            self._dwell_time = 0.0
            self._track_t = 0.0
        elif new_state == self.RELEASE:
            self._track_target_id = None
            self._dwell_time = 0.0

    def set_track_target(self, tgt_id: str) -> None:
        """设置当前追踪目标 ID。"""
        self._track_target_id = tgt_id
        if self._state != self.TRACK:
            self._transition_to(self.TRACK)

    def mark_destroyed(self) -> None:
        """标记当前目标已被摧毁，进入释放状态。"""
        self._transition_to(self.RELEASE)

    def set_last_report_t(self, t: float) -> None:
        self._last_report_t = t

    def reset(self) -> None:
        self._state = self.SEARCH
        self._dwell_time = 0.0
        self._track_target_id = None
        self._candidate = None
        self._ema.reset()
        self._verify_t = 0.0
        self._track_t = 0.0
        self._last_det_tick = -1e9
        self._last_report_t = -1e9
        self._known_decoys.clear()


# ── SchedulerBrain 对外决策类 ────────────────────────────────────────────
class SchedulerBrain:
    """
    集群调度核心决策类。

    对外固定接口:
        decide(target_list, recv_msg) -> dict

    功能:
        综合视觉目标 + 队友消息 →
        状态机判断 → 任务分配 → 输出运动控制指令字典。
    """

    # 状态常量
    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    RELEASE = "RELEASE"

    def __init__(self, my_uid: str = "0", k: int = 2,
                 search_region: Optional[Tuple[float, float]] = None):
        """
        Args:
            my_uid:         本机 UID
            k:              协同无人机数量 (赛题固定为 2)
            search_region:  搜索区域中心 (lat, lon)，None 则自动分配
        """
        self.my_uid = my_uid
        self.k = k
        self.coord = CooperativeCoordinator(my_uid, k=k)
        self.state = StateMachine()

        # 搜索螺旋参数
        self._search_region = search_region or self._default_region()
        self._t: float = 0.0
        self._tick: int = 0
        self._phase: float = self._uid_phase()

        # 自身位置（由 decide 调用方通过 target_list 配套字段或独立设置）
        self._self_lat: float = 0.0
        self._self_lon: float = 0.0
        self._self_heading: float = 0.0

    # ── 公开 API ──────────────────────────────────────────────────────

    def decide(self, target_list: List[dict], recv_msg: List) -> dict:
        """
        对外固定决策函数。

        Args:
            target_list: 视觉模块 detect 输出的所有目标，每项为 dict:
                {"lat": float, "lon": float, "confidence": float, ...}
            recv_msg:    通信模块 receive 拿到的队友消息，
                每项应有 .payload 属性 (str)，格式如:
                    "R:<tgt_id>,<lat>,<lon>"   — 召唤队友
                    "T:<tgt_id>,<dwell>"       — 定期状态
                    "C:<tgt_id>"               — 目标摧毁通知

        Returns:
            dict: 给运动控制的 cmd 指令字典:
                {
                    "mode":           str,   # SEARCH / VERIFY / TRACK / RELEASE
                    "lat":            float, # 目标航点纬度
                    "lon":            float, # 目标航点经度
                    "alt":            float, # 飞行高度
                    "speed":          float, # 飞行速度
                    "loiter_radius":  float, # 盘旋半径
                    "gimbal_pan":     float, # 云台 pan 角
                    "gimbal_tilt":    float, # 云台 tilt 角
                    "fov":            float, # 视场角
                    "broadcast":      str | None, # 需要广播的消息
                    "report_target":  Tuple[float, float] | None, # 上报目标位置
                }
        """
        self._tick += 1
        dt: float = 0.1          # 帧间隔
        sim_t: float = self._t   # 当前仿真时间

        # 1. 更新自身位置（从 target_list 附属信息或外部注入）
        self._update_self_pos(target_list)

        # 2. 注入通信消息
        self.coord.ingest_comms(recv_msg)

        # 3. 检查是否有协同目标可接手（队友已召唤）
        active_tgt = self.coord.select_target(self._self_lat, self._self_lon)
        should_join = (
            active_tgt is not None
            and self.state.state == self.SEARCH
        )

        # 4. 状态机刷新
        self.state.refresh(
            target_list, recv_msg,
            self._self_lat, self._self_lon,
            sim_t, dt
        )

        # 5. 如果队友已召唤且有活动目标，切入跟踪
        if should_join and self.state.state == self.SEARCH:
            self.state.set_track_target(active_tgt)

        # 6. 按当前状态生成控制指令
        cmd = self._gen_cmd(target_list, recv_msg, sim_t)

        self._t += dt
        return cmd

    # ── 内部指令生成 ──────────────────────────────────────────────────

    def _gen_cmd(self, target_list: List[dict],
                 recv_msg: List, sim_t: float) -> dict:
        """根据当前状态生成运动控制指令字典。"""
        st = self.state.state

        if st == self.SEARCH:
            return self._gen_search_cmd()
        elif st == self.VERIFY:
            return self._gen_verify_cmd(target_list)
        elif st == self.TRACK:
            return self._gen_track_cmd(target_list, recv_msg, sim_t)
        elif st == self.RELEASE:
            return self._gen_release_cmd()
        else:
            return self._gen_search_cmd()

    def _gen_search_cmd(self) -> dict:
        """搜索状态：螺旋搜索航点 + 扫描云台。"""
        home_lat, home_lon = self._search_region
        t = self._t + self._phase * 12.0
        bearing = (self._ANG_SPEED * t) % 360.0
        revs = (self._ANG_SPEED * t) / 360.0
        radius = max(1.0, min(_SEARCH_RADIUS, _GROWTH * revs))
        dlat = (radius * math.cos(math.radians(bearing))) / 111320.0
        dlon = (radius * math.sin(math.radians(bearing))) / (
            111320.0 * math.cos(math.radians(home_lat)))
        slat, slon = _clamp_to_safebox(home_lat + dlat, home_lon + dlon)

        phase = (t % _SWEEP_PERIOD) / _SWEEP_PERIOD
        tilt = (_PITCH_MIN + (_PITCH_MAX - _PITCH_MIN) * 0.5
                * (1 - math.cos(2 * math.pi * phase)))
        pan_phase = (t % (_SWEEP_PERIOD * 2)) / (_SWEEP_PERIOD * 2)
        pan = -90.0 + 180.0 * 0.5 * (1 - math.cos(2 * math.pi * pan_phase))

        return {
            "mode": self.SEARCH,
            "lat": slat,
            "lon": slon,
            "alt": _SEARCH_ALT,
            "speed": _SPEED,
            "loiter_radius": 0.0,
            "gimbal_pan": pan,
            "gimbal_tilt": tilt,
            "fov": _SEARCH_FOV,
            "broadcast": None,
            "report_target": None,
        }

    def _gen_verify_cmd(self, target_list: List[dict]) -> dict:
        """验证状态：飞向候选目标，云台对准。"""
        tgt = self.state.candidate
        if tgt is None:
            return self._gen_search_cmd()

        tlat, tlon = _clamp_to_safebox(*tgt)
        pan, tilt = self._tracking_gimbal(
            self._self_lat, self._self_lon, self._self_heading, tgt[0], tgt[1])

        return {
            "mode": self.VERIFY,
            "lat": tlat,
            "lon": tlon,
            "alt": _SEARCH_ALT,
            "speed": _SPEED,
            "loiter_radius": _LOITER_CLOSE,
            "gimbal_pan": pan,
            "gimbal_tilt": tilt,
            "fov": _TRACK_FOV,
            "broadcast": None,
            "report_target": None,
        }

    def _gen_track_cmd(self, target_list: List[dict],
                       recv_msg: List, sim_t: float) -> dict:
        """跟踪状态：槽位分离盘旋 + 云台对准目标 + K=2 协同。"""
        tgt_id = self.state.track_target_id
        broadcast_msg = None
        report = None

        # 无目标则回搜索
        if tgt_id is None:
            return self._gen_search_cmd()

        # 获取目标位置（可能来自自身观测或队友通信）
        tgt_pos = self.coord.target_pos(tgt_id)
        if tgt_pos is None:
            return self._gen_search_cmd()

        # 用当前检测更新目标位置
        for det in target_list:
            dlat = det.get("lat")
            dlon = det.get("lon")
            if dlat is None or dlon is None:
                continue
            d = _haversine_m(dlat, dlon, tgt_pos[0], tgt_pos[1])
            if d < 250.0:
                self.coord.confirm_target(dlat, dlon)  # type: ignore[arg-type]
                tgt_pos = (dlat, dlon)

        # 槽位盘旋
        slot = self.coord.my_slot(tgt_id)
        aim_lat, aim_lon = self.coord.aim_point(tgt_id, slot, standoff=200.0)
        aim_lat, aim_lon = _clamp_to_safebox(aim_lat, aim_lon)

        # 云台对准真实目标
        pan, tilt = self._tracking_gimbal(
            self._self_lat, self._self_lon, self._self_heading,
            tgt_pos[0], tgt_pos[1])

        # 定期广播本机 dwell
        if self._tick % 5 == 0:
            broadcast_msg = f"T:{tgt_id},{self.state.dwell_time:.2f}"

        # 检查摧毁条件：本机 + 队友 dwell 均达标
        peer_dwell = self.coord.peer_dwell(tgt_id)
        destroyed = (
            self.state.dwell_time >= 20.0
            and peer_dwell >= 19.5
        )

        if destroyed:
            self.state.mark_destroyed()
            self.coord.mark_destroyed(tgt_id)
            if self.coord.need_c_broadcast(tgt_id, sim_t):
                broadcast_msg = f"C:{tgt_id}"
                self.coord.mark_c_sent(tgt_id, sim_t)

        # 上报目标位置 (速度 > 3m/s 且间隔 ≥ 1s)
        if (sim_t - self.state.last_report_t >= 1.0
                and self.state.ema_speed_mps() > 3.0
                and self.state.ema_value is not None):
            self.state.set_last_report_t(sim_t)
            report = (tgt_pos[0], tgt_pos[1])

        return {
            "mode": self.TRACK,
            "lat": aim_lat,
            "lon": aim_lon,
            "alt": _SEARCH_ALT,
            "speed": _SPEED,
            "loiter_radius": _LOITER_CLOSE,
            "gimbal_pan": pan,
            "gimbal_tilt": tilt,
            "fov": _TRACK_FOV,
            "broadcast": broadcast_msg,
            "report_target": report,
        }

    def _gen_release_cmd(self) -> dict:
        """释放状态：回归搜索。"""
        return self._gen_search_cmd()

    # ── 辅助方法 ──────────────────────────────────────────────────────

    def _tracking_gimbal(self, self_lat: float, self_lon: float,
                         self_heading: float,
                         tgt_lat: float, tgt_lon: float) -> Tuple[float, float]:
        """计算对准目标的云台角度 (pan, tilt)。"""
        brg = _bearing_deg(self_lat, self_lon, tgt_lat, tgt_lon)
        pan = ((brg - self_heading + 180.0) % 360.0) - 180.0
        ground = max(1.0, _haversine_m(self_lat, self_lon, tgt_lat, tgt_lon))
        tilt = -math.degrees(math.atan2(_SEARCH_ALT, ground))
        return pan, tilt

    def _default_region(self) -> Tuple[float, float]:
        """根据 UID 分配默认搜索区域中心。"""
        (lat_min, lon_min), (lat_max, lon_max) = _BBOX
        lat_mid = (lat_min + lat_max) / 2
        sub_w = (lon_max - lon_min) / 3
        import hashlib
        uid = self.my_uid
        if uid.isdigit():
            idx = int(uid) % 3
        elif "_" in uid:
            tail = uid.rsplit("_", 1)[-1]
            idx = int(tail) % 3 if tail.isdigit() else (
                int(hashlib.md5(uid.encode()).hexdigest(), 16) % 3)
        else:
            idx = int(hashlib.md5(uid.encode()).hexdigest(), 16) % 3
        return (lat_mid, lon_min + sub_w * (idx + 0.5))

    def _uid_phase(self) -> float:
        """根据 UID 生成搜索相位偏移 (0~1)。"""
        import hashlib
        h = int(hashlib.md5(self.my_uid.encode()).hexdigest(), 16)
        return (h % 1000) / 1000.0

    def _update_self_pos(self, target_list: List[dict]) -> None:
        """
        从 target_list 附属字段更新本机位置。
        若 target_list 带 self_lat / self_lon / self_heading 则更新。
        """
        # 尝试从 target_list 获取本机位姿（兼容扩展字段）
        if target_list and isinstance(target_list, list):
            pass  # 本机位姿由外部注入时可通过 set_self_pose 设置

    def set_self_pose(self, lat: float, lon: float, heading: float = 0.0) -> None:
        """外部注入本机位姿 (每帧在 decide 前调用)。"""
        self._self_lat = lat
        self._self_lon = lon
        self._self_heading = heading

    def reset(self) -> None:
        """重置调度器状态 (每回合开始调用)。"""
        self._t = 0.0
        self._tick = 0
        self.state.reset()
        self.coord = CooperativeCoordinator(self.my_uid, k=self.k)
