"""
EMA 跟踪器模块

来源: new drone_agent.py _EMATracker (第 162-342 行)
功能: 基于指数移动平均和线性回归速度估计的目标位置跟踪器，
      支持速度方差、位移、方向变化方差等高级运动特征。

修复: v2 — append() 接收仿真时间 sim_t 替代 time.monotonic()，
      speed_mps() / speed_variance() 统一使用仿真时间差，消除 tick_hz 回退路径。
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

from ..motion_control.geo import haversine_m as _haversine_m


class EMATracker:
    """
    基于指数移动平均和线性回归速度估计的目标位置跟踪器。

    融合了 coordinator.py 的 _EMATracker 和 ema_filter.py 的 EMATracker，
    支持速度方差、位移、方向变化方差等高级运动特征。
    """

    def __init__(self, alpha: float = 0.3, history: int = 80):
        self._alpha = alpha
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        # raw_history: (lat, lon, sim_t) — sim_t 为仿真时间（秒）
        self._raw: Deque[Tuple[float, float, float]] = deque(maxlen=history)

    def append(self, lat: float, lon: float, sim_t: float = 0.0) -> None:
        """添加新检测点，更新 EMA 位置和原始数据缓冲区。

        Args:
            lat, lon: 检测位置
            sim_t: 仿真时间（秒），用于速度/方差的准确计算
        """
        if self._lat is None:
            self._lat, self._lon = lat, lon
        else:
            a = self._alpha
            self._lat = self._lat * (1 - a) + lat * a
            self._lon = self._lon * (1 - a) + lon * a
        self._raw.append((lat, lon, sim_t))

    @property
    def value(self) -> Optional[Tuple[float, float]]:
        """返回当前 EMA 平滑位置。"""
        if self._lat is None:
            return None
        return (self._lat, self._lon)

    @property
    def raw_history(self) -> List[Tuple[float, float, float]]:
        """返回原始历史记录列表。"""
        return list(self._raw)

    def speed_mps(self) -> float:
        """
        估算目标速度（米/秒）。

        v3: 放弃不可靠的线性回归（sim_t 为 epoch 偏移秒，var_t 极小导致爆炸），
        改用两种稳健方法：
        1. 首尾位移 / 时间跨度（主方法，对匀速真目标最准确）
        2. 中位数瞬时速度（兜底，抗离群值）
        """
        n = len(self._raw)
        if n < 4:
            return 0.0

        samples = list(self._raw)
        # 使用最近 25 个样本
        if len(samples) > 25:
            samples = samples[-25:]
        n = len(samples)
        if n < 4:
            return 0.0

        # ── 方法1: 首尾位移 / 时间跨度 ──
        lat_first, lon_first, t_first = samples[0]
        lat_last, lon_last, t_last = samples[-1]
        dt_total = t_last - t_first

        if dt_total >= 0.5:
            dist_total = _haversine_m(lat_first, lon_first, lat_last, lon_last)
            speed_from_endpoints = dist_total / dt_total
            # 合理性检查：如果首尾速度在 1~30 m/s，直接返回
            if 1.0 <= speed_from_endpoints <= 30.0:
                return speed_from_endpoints

        # ── 方法2: 中位数瞬时速度（兜底，对离群值稳健） ──
        speeds = []
        for i in range(1, n):
            lat1, lon1, t1 = samples[i - 1]
            lat2, lon2, t2 = samples[i]
            dt = t2 - t1
            if dt < 0.01:
                continue
            dist = _haversine_m(lat1, lon1, lat2, lon2)
            speeds.append(dist / dt)

        if not speeds:
            # 最后兜底：用首尾法（即使 dt 很短也返回）
            if dt_total > 0.001:
                return _haversine_m(lat_first, lon_first, lat_last, lon_last) / dt_total
            return 0.0

        speeds.sort()
        mid = len(speeds) // 2
        if len(speeds) % 2 == 0:
            return (speeds[mid - 1] + speeds[mid]) / 2.0
        return speeds[mid]

    def speed_variance(self, window: int = 15) -> float:
        """
        计算速度方差（基于仿真时间差）

        真目标（匀速行驶）：速度方差小
        诱饵（随机游走/噪声驱动）：速度方差大
        """
        if len(self._raw) < window + 1:
            return 0.0

        samples = list(self._raw)[-window - 1:]
        speeds = []
        for i in range(1, len(samples)):
            lat1, lon1, t1 = samples[i - 1]
            lat2, lon2, t2 = samples[i]
            dt = t2 - t1  # 仿真时间差（v2: 已存储 sim_t）
            if dt < 0.001:
                continue
            dist = _haversine_m(lat1, lon1, lat2, lon2)
            speeds.append(dist / dt)

        if len(speeds) < 3:
            return 0.0

        mean_speed = sum(speeds) / len(speeds)
        variance = sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)
        return variance

    def displacement(self) -> float:
        """
        计算历史窗口内的总位移（米）
        """
        if len(self._raw) < 2:
            return 0.0
        first = self._raw[0]
        last = self._raw[-1]
        return _haversine_m(first[0], first[1], last[0], last[1])

    def direction_change_variance(self, window: int = 20) -> float:
        """
        计算方向变化方差

        真目标运动有规律，方向变化小；诱饵方向变化大
        """
        if len(self._raw) < window + 1:
            return 0.0

        samples = list(self._raw)[-window - 1:]
        directions = []

        for i in range(1, len(samples)):
            lat1, lon1, _ = samples[i - 1]
            lat2, lon2, _ = samples[i]
            dx = (lon2 - lon1) * 111320 * math.cos(math.radians((lat1 + lat2) / 2))
            dy = (lat2 - lat1) * 111320
            angle = math.atan2(dy, dx)
            directions.append(angle)

        if len(directions) < 3:
            return 0.0

        changes = []
        for i in range(1, len(directions)):
            diff = directions[i] - directions[i - 1]
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            changes.append(diff)

        if len(changes) < 2:
            return 0.0

        mean_change = sum(changes) / len(changes)
        variance = sum((c - mean_change) ** 2 for c in changes) / len(changes)
        return variance

    def reset(self) -> None:
        """重置跟踪器状态。"""
        self._lat = self._lon = None
        self._raw.clear()
