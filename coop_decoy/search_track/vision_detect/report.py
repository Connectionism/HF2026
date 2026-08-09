from typing import Optional, Tuple, Dict, List, Set
import math

from .ema_filter import EMATracker, haversine_distance


class ReportOptimizer:
    """
    上报优化器 - 新规则适配版
    
    核心职责：
    1. 为每个目标独立维护EMA滤波器
    2. 判断何时应该上报（防止漏报）
    3. 追踪哪些目标已被摧毁（防止上报尸体）
    4. 上报时使用平滑位置降低RMSE
    """
    
    def __init__(
        self,
        alpha: float = 0.25,
        min_confidence: float = 0.5,  # 新规则：降低阈值，更激进
        min_track_time: float = 1.5,
        max_report_frequency: float = 1.0,
        max_targets: int = 10,
    ):
        self.alpha = alpha
        self.min_confidence = min_confidence
        self.min_track_time = min_track_time
        self.max_report_frequency = max_report_frequency
        self.max_targets = max_targets
        
        # 每个目标的滤波器
        self._filters: Dict[int, EMATracker] = {}
        self._track_start_time: Dict[int, float] = {}
        self._last_report_time: Dict[int, float] = {}
        self._last_report_pos: Dict[int, Tuple[float, float]] = {}
        
        # 新规则：追踪已摧毁目标（防止报尸体）
        self._destroyed_targets: Set[int] = set()
        
        # 目标计数
        self._next_id = 0
    
    def _get_time(self) -> float:
        import time
        return time.monotonic()
    
    def register_target(self, lat: float, lon: float) -> int:
        """注册一个新目标，返回目标ID"""
        target_id = self._next_id
        self._next_id += 1
        
        self._filters[target_id] = EMATracker(alpha=self.alpha)
        self._filters[target_id].append(lat, lon)
        self._track_start_time[target_id] = self._get_time()
        
        return target_id
    
    def update(self, target_id: int, lat: float, lon: float) -> None:
        """更新目标观测值"""
        if target_id not in self._filters:
            self._filters[target_id] = EMATracker(alpha=self.alpha)
            self._track_start_time[target_id] = self._get_time()
        
        self._filters[target_id].append(lat, lon)
    
    def mark_destroyed(self, target_id: int) -> None:
        """
        标记目标已被摧毁
        
        新规则：摧毁后上报丢弃，RMSE冻结在摧毁时刻
        """
        self._destroyed_targets.add(target_id)
    
    def is_destroyed(self, target_id: int) -> bool:
        return target_id in self._destroyed_targets
    
    def should_report(self, target_id: int, confidence: float, lat: float, lon: float) -> bool:
        """
        判断是否应该上报该目标
        
        新规则关键：
        1. 已摧毁目标不上报
        2. 所有目标必须覆盖（漏报惩罚重）
        3. 不能报尸体（离已摧毁目标更近）
        """
        # 已摧毁目标不上报
        if self.is_destroyed(target_id):
            return False
        
        if target_id not in self._filters:
            return False
        if self._filters[target_id].value is None:
            return False
        
        # 置信度检查（新规则：阈值降低）
        if confidence < self.min_confidence:
            return False
        
        # 最小跟踪时间检查
        if target_id not in self._track_start_time:
            return False
        if self._get_time() - self._track_start_time[target_id] < self.min_track_time:
            return False
        
        # 频率限制
        if target_id in self._last_report_time:
            if self._get_time() - self._last_report_time[target_id] < 1.0 / self.max_report_frequency:
                return False
        
        # 新规则：检查是否离已摧毁目标更近（报尸体检测）
        smooth_pos = self._filters[target_id].value
        for destroyed_id in self._destroyed_targets:
            if destroyed_id in self._filters and self._filters[destroyed_id].value is not None:
                d_truth = haversine_distance(lat, lon, smooth_pos[0], smooth_pos[1])
                d_destroyed = haversine_distance(
                    lat, lon,
                    self._filters[destroyed_id].value[0],
                    self._filters[destroyed_id].value[1]
                )
                if d_destroyed < d_truth * 0.8:  # 离已摧毁目标更近
                    return False  # 疑似报尸体，丢弃
        
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
        """获取所有活跃（未摧毁）目标ID"""
        return [tid for tid in self._filters.keys() if tid not in self._destroyed_targets]
    
    def get_target_count(self) -> int:
        return len(self._filters)
    
    def reset(self) -> None:
        self._filters.clear()
        self._track_start_time.clear()
        self._last_report_time.clear()
        self._last_report_pos.clear()
        self._destroyed_targets.clear()
        self._next_id = 0


def make_report_message(lat: float, lon: float) -> str:
    """构造上报消息（新规则：不再区分T/D，统一上报）"""
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