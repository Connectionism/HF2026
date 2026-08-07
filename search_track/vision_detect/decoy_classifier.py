"""
诱饵判别模块 - 适配新规则（诱饵也移动）
核心策略：诱饵运动模式（随机游走/噪声驱动）与真目标（匀速直线运动）不同
通过运动模式特征区分，而非单纯速度有无
"""

from typing import Optional, Tuple, Dict, Any
from collections import deque
import math

from .ema_filter import EMATracker, haversine_distance


class DecoyClassifier:
    """
    诱饵判别器 - 新规则适配版
    
    新规则核心变化：
    1. 诱饵也是移动的！不能靠"有没有速度"判断
    2. 诱饵误判不扣分 → 策略可以更激进（宁可误报，不可漏报）
    
    判别策略：多特征投票，区分"匀速直线运动（真目标）" vs "随机游走（诱饵）"
    
    特征1: 速度方差 - 真目标小，诱饵大（随机加减速）
    特征2: 方向变化方差 - 真目标小（直线行驶），诱饵大（随机转向）
    特征3: 位移平滑度 - 真目标位移均匀，诱饵跳跃
    特征4: 加速度方差 - 真目标小（匀速），诱饵大（随机）
    """
    
    DEFAULT_SPEED_VAR_THRESHOLD = 2.0  # 速度方差阈值
    DEFAULT_DIR_VAR_THRESHOLD = 0.15   # 方向变化方差阈值
    DEFAULT_MIN_TRACK_FRAMES = 10      # 最少跟踪帧数（1秒）
    DEFAULT_VERIFY_FRAMES = 12         # 确认帧数（1.2秒）
    DEFAULT_ALPHA = 0.25
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        cfg = config or {}
        self.speed_var_threshold = cfg.get('speed_var_threshold', self.DEFAULT_SPEED_VAR_THRESHOLD)
        self.dir_var_threshold = cfg.get('dir_var_threshold', self.DEFAULT_DIR_VAR_THRESHOLD)
        self.verify_frames = cfg.get('verify_frames', self.DEFAULT_VERIFY_FRAMES)
        self.min_track_frames = cfg.get('min_track_frames', self.DEFAULT_MIN_TRACK_FRAMES)
        self.alpha = cfg.get('alpha', self.DEFAULT_ALPHA)
        
        self._ema = EMATracker(alpha=self.alpha, max_history=60)
        self._position_history: deque = deque(maxlen=60)
        self._frame_count = 0
        self._consecutive_real = 0
        self._consecutive_fake = 0
        self._is_real = False
        self._confidence = 0.0
        self._last_features = {}
        
        # 上报管理（新规则：漏报惩罚极重）
        self._reported_ids = set()  # 已上报的目标ID（用于防重复）
        self._target_id_counter = 0
        self._current_target_id = None
    
    def update(self, lat: float, lon: float, dt: float = 0.1) -> None:
        """更新观测值"""
        self._frame_count += 1
        self._ema.append(lat, lon)
        self._position_history.append((lat, lon))
        
        if self._frame_count < self.min_track_frames:
            # 数据不足，保持观望
            self._confidence = 0.3
            return
        
        features = self._compute_features()
        self._last_features = features
        
        # 新规则：诱饵也移动，用运动模式区分
        is_real = self._voting_decision(features)
        
        if is_real:
            self._consecutive_real += 1
            self._consecutive_fake = 0
        else:
            self._consecutive_real = 0
            self._consecutive_fake += 1
        
        # 连续确认才改变状态
        if self._consecutive_real >= self.verify_frames:
            self._is_real = True
            self._confidence = min(1.0, self._confidence + 0.08)
        elif self._consecutive_fake >= self.verify_frames:
            self._is_real = False
            self._confidence = max(0.0, self._confidence - 0.05)
        
        self._confidence = max(0.0, min(1.0, self._confidence))
    
    def _compute_features(self) -> Dict[str, float]:
        """计算多维度运动模式特征"""
        features = {}
        
        # 特征1: 速度方差（核心特征）
        features['speed_variance'] = self._ema.speed_variance(window=15)
        
        # 特征2: 方向变化方差（核心特征）
        features['dir_variance'] = self._ema.direction_change_variance(window=15)
        
        # 特征3: 平均速度（辅助特征）
        features['avg_speed'] = self._ema.speed_mps()
        
        # 特征4: 位移跨度
        features['displacement'] = self._ema.displacement()
        
        return features
    
    def _voting_decision(self, features: Dict[str, float]) -> bool:
        """
        多特征投票决策
        
        新规则下：诱饵误判不扣分 → 阈值偏向"宁可误报，不可漏报"
        """
        votes = 0
        total_weight = 0
        
        # 特征1: 速度方差投票（权重最高）
        # 真目标速度方差小，诱饵速度方差大
        speed_var = features.get('speed_variance', 100.0)
        if speed_var < self.speed_var_threshold:
            votes += 4  # 真目标特征
        elif speed_var < self.speed_var_threshold * 2:
            votes += 2  # 中等
        total_weight += 4
        
        # 特征2: 方向变化方差投票
        # 真目标方向变化小（直线行驶），诱饵方向变化大
        dir_var = features.get('dir_variance', 1.0)
        if dir_var < self.dir_var_threshold:
            votes += 3
        elif dir_var < self.dir_var_threshold * 2:
            votes += 1.5
        total_weight += 3
        
        # 特征3: 位移充分性（真目标必须有足够位移）
        displacement = features.get('displacement', 0.0)
        if displacement > 5.0:  # 至少移动了5米
            votes += 2
        total_weight += 2
        
        # 特征4: 平均速度合理性（真目标速度在合理范围）
        avg_speed = features.get('avg_speed', 0.0)
        if 2.0 <= avg_speed <= 15.0:
            votes += 1
        total_weight += 1
        
        # 新规则：降低判定阈值（宁可误报，不可漏报）
        score = votes / max(1, total_weight)
        return score > 0.35  # 从0.5降到0.35，更激进
    
    @property
    def is_real_target(self) -> bool:
        return self._is_real
    
    @property
    def confidence(self) -> float:
        return self._confidence
    
    @property
    def smoothed_position(self) -> Optional[Tuple[float, float]]:
        return self._ema.value
    
    def should_report(self) -> bool:
        """
        判断是否应该上报
        
        新规则要点：
        1. 诱饵误判不扣分 → 只要置信度>0.5就报
        2. 漏报惩罚极重 → 宁可多报不可漏报
        """
        if self._frame_count < self.min_track_frames:
            return False
        if self._ema.value is None:
            return False
        # 新规则：置信度阈值从0.6降到0.5，更激进
        return self._is_real and self._confidence >= 0.5
    
    def get_report_position(self) -> Optional[Tuple[float, float]]:
        """获取用于上报的位置（平滑后的位置）"""
        return self._ema.value
    
    def reset(self) -> None:
        self._ema.reset()
        self._position_history.clear()
        self._frame_count = 0
        self._consecutive_real = 0
        self._consecutive_fake = 0
        self._is_real = False
        self._confidence = 0.0
        self._last_features.clear()
    
    def get_debug_info(self) -> Dict[str, Any]:
        return {
            'is_real': self._is_real,
            'confidence': self._confidence,
            'consecutive_real': self._consecutive_real,
            'consecutive_fake': self._consecutive_fake,
            'features': self._last_features,
            'smooth_pos': self._ema.value,
            'speed': self._ema.speed_mps(),
            'speed_var': self._last_features.get('speed_variance', 0),
            'dir_var': self._last_features.get('dir_variance', 0),
            'displacement': self._last_features.get('displacement', 0),
        }