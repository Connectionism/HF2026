# src/vision_detect/report.py
"""
上报优化模块 - 阶段二适配版
支持多目标独立上报管理
"""

from typing import Optional, Tuple, Dict, List, Set
import math

from .ema_filter import EMATracker, haversine_distance


class ReportOptimizer:
    """
    上报优化器 - 阶段二适配版
    """
    
    def __init__(
        self,
        alpha: float = 0.25,
        min_confidence: float = 0.5,
        min_track_time: float = 1.5,
        max_report_frequency: float = 2.0,
    ):
        self.alpha = alpha
        self.min_confidence = min_confidence
        self.min_track_time = min_track_time
        self.max_report_frequency = max_report_frequency
        
        self._filters: Dict[int, EMATracker] = {}
        self._track_start_time: Dict[int, float] = {}
        self._last_report_time: Dict[int, float] = {}
        self._last_report_pos: Dict[int, Tuple[float, float]] = {}
        self._destroyed_targets: Set[int] = set()
        self._next_id = 0
    
    def _get_time(self) -> float:
        import time
        return time.monotonic()
    
    def register_target(self, lat: float, lon: float) -> int:
        target_id = self._next_id
        self._next_id += 1
        self._filters[target_id] = EMATracker(alpha=self.alpha)
        self._filters[target_id].append(lat, lon)
        self._track_start_time[target_id] = self._get_time()
        return target_id
    
    def update(self, target_id: int, lat: float, lon: float) -> None:
        if target_id not in self._filters:
            self._filters[target_id] = EMATracker(alpha=self.alpha)
            self._track_start_time[target_id] = self._get_time()
        self._filters[target_id].append(lat, lon)
    
    def mark_destroyed(self, target_id: int) -> None:
        self._destroyed_targets.add(target_id)
    
    def is_destroyed(self, target_id: int) -> bool:
        return target_id in self._destroyed_targets
    
    def should_report(self, target_id: int, confidence: float) -> bool:
        if self.is_destroyed(target_id):
            return False
        if target_id not in self._filters:
            return False
        if self._filters[target_id].value is None:
            return False
        if confidence < self.min_confidence:
            return False
        if target_id not in self._track_start_time:
            return False
        if self._get_time() - self._track_start_time[target_id] < self.min_track_time:
            return False
        if target_id in self._last_report_time:
            if self._get_time() - self._last_report_time[target_id] < 1.0 / self.max_report_frequency:
                return False
        return True
    
    def get_report_position(self, target_id: int) -> Optional[Tuple[float, float]]:
        if target_id not in self._filters:
            return None
        return self._filters[target_id].value
    
    def mark_reported(self, target_id: int) -> None:
        self._last_report_time[target_id] = self._get_time()
        if target_id in self._filters and self._filters[target_id].value is not None:
            self._last_report_pos[target_id] = self._filters[target_id].value
    
    def get_all_active_targets(self) -> List[int]:
        return [tid for tid in self._filters.keys() if tid not in self._destroyed_targets]
    
    def reset(self) -> None:
        self._filters.clear()
        self._track_start_time.clear()
        self._last_report_time.clear()
        self._last_report_pos.clear()
        self._destroyed_targets.clear()
        self._next_id = 0


def make_report_message(lat: float, lon: float) -> str:
    return f"REPORT:{lat:.6f},{lon:.6f}"


def parse_report_message(message: str) -> Optional[Tuple[float, float]]:
    try:
        parts = message.strip().split(':', 1)
        if len(parts) != 2 or parts[0] != 'REPORT':
            return None
        lat_str, lon_str = parts[1].split(',')
        return float(lat_str), float(lon_str)
    except (ValueError, IndexError):
        return None