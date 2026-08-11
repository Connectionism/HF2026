# src/vision_detect/decoy_classifier.py
"""
诱饵判别模块 - 阶段二增强版
新增：目标ID管理、摧毁状态标记、上报冻结、调试日志
"""

from typing import Optional, Tuple, Dict, Any, Set, List
from collections import deque
import math
import time

from .ema_filter import EMATracker, haversine_distance


class DecoyClassifier:
    """
    诱饵判别器 - 阶段二增强版
    
    新增功能：
    1. 每个目标独立维护分类器
    2. 支持标记目标已摧毁（冻结上报）
    3. 多特征投票区分真目标与诱饵
    4. 调试日志输出
    """
    
    DEFAULT_SPEED_VAR_THRESHOLD = 2.0
    DEFAULT_DIR_VAR_THRESHOLD = 0.15
    DEFAULT_MIN_TRACK_FRAMES = 10
    DEFAULT_VERIFY_FRAMES = 12
    DEFAULT_ALPHA = 0.25
    
    def __init__(self, target_id: Optional[int] = None, config: Optional[Dict[str, Any]] = None, debug: bool = False):
        """
        初始化分类器
        
        Args:
            target_id: 关联的目标ID（由调度模块分配）
            config: 配置参数
            debug: 是否开启调试日志
        """
        cfg = config or {}
        self.target_id = target_id
        self.debug = debug
        
        self.speed_var_threshold = cfg.get('speed_var_threshold', self.DEFAULT_SPEED_VAR_THRESHOLD)
        self.dir_var_threshold = cfg.get('dir_var_threshold', self.DEFAULT_DIR_VAR_THRESHOLD)
        self.verify_frames = cfg.get('verify_frames', self.DEFAULT_VERIFY_FRAMES)
        self.min_track_frames = cfg.get('min_track_frames', self.DEFAULT_MIN_TRACK_FRAMES)
        self.alpha = cfg.get('alpha', self.DEFAULT_ALPHA)
        
        # 核心滤波器
        self._ema = EMATracker(alpha=self.alpha, max_history=80)
        self._position_history: deque = deque(maxlen=80)
        
        # 状态追踪
        self._frame_count = 0
        self._consecutive_real = 0
        self._consecutive_fake = 0
        self._is_real = False
        self._confidence = 0.0
        self._last_features = {}
        
        # 摧毁状态管理
        self._is_destroyed = False
        self._destroyed_at_frame = None
        self._last_report_pos = None
        self._has_ever_reported = False
        
        # 上报计数器（频率限制）
        self._report_count = 0
        self._last_report_frame = -10
        
        # 得分统计
        self._real_frames = 0
        self._fake_frames = 0
        self._start_time = time.time()
        
        if self.debug:
            print(f"[DecoyClassifier] 目标 {target_id} 初始化完成")
    
    def update(self, lat: float, lon: float, dt: float = 0.1) -> None:
        """
        更新观测值
        
        Args:
            lat: 纬度
            lon: 经度
            dt: 时间步长（秒）
        """
        # 如果已摧毁，不再更新
        if self._is_destroyed:
            return
        
        self._frame_count += 1
        self._ema.append(lat, lon)
        self._position_history.append((lat, lon))
        
        if self._frame_count < self.min_track_frames:
            self._confidence = 0.3
            if self.debug and self._frame_count % 10 == 0:
                print(f"[DecoyClassifier] 目标 {self.target_id}: 预热中 {self._frame_count}/{self.min_track_frames}")
            return
        
        features = self._compute_features()
        self._last_features = features
        
        is_real = self._voting_decision(features)
        
        if is_real:
            self._consecutive_real += 1
            self._consecutive_fake = 0
            self._real_frames += 1
        else:
            self._consecutive_real = 0
            self._consecutive_fake += 1
            self._fake_frames += 1
        
        # 连续确认才改变状态
        if self._consecutive_real >= self.verify_frames:
            self._is_real = True
            self._confidence = min(1.0, self._confidence + 0.08)
        elif self._consecutive_fake >= self.verify_frames:
            self._is_real = False
            self._confidence = max(0.0, self._confidence - 0.05)
        
        self._confidence = max(0.0, min(1.0, self._confidence))
        
        # 调试日志
        if self.debug and self._frame_count % 10 == 0:
            print(f"[DecoyClassifier] 目标 {self.target_id}: 帧={self._frame_count}, "
                  f"is_real={self._is_real}, conf={self._confidence:.2f}, "
                  f"speed={features.get('avg_speed', 0):.2f}m/s")
    
    def _compute_features(self) -> Dict[str, float]:
        """计算多维度运动模式特征"""
        features = {}
        features['speed_variance'] = self._ema.speed_variance(window=15)
        features['dir_variance'] = self._ema.direction_change_variance(window=15)
        features['avg_speed'] = self._ema.speed_mps()
        features['displacement'] = self._ema.displacement()
        return features
    
    def _voting_decision(self, features: Dict[str, float]) -> bool:
        """多特征投票决策"""
        votes = 0
        total_weight = 0
        
        # 特征1: 速度方差（权重最高）
        speed_var = features.get('speed_variance', 100.0)
        if speed_var < self.speed_var_threshold:
            votes += 4
        elif speed_var < self.speed_var_threshold * 2:
            votes += 2
        total_weight += 4
        # 特征2: 方向变化方差
        dir_var = features.get('dir_variance', 1.0)
        if dir_var < self.dir_var_threshold:
            votes += 3
        elif dir_var < self.dir_var_threshold * 2:
            votes += 1.5
        total_weight += 3
        
        # 特征3: 位移
        displacement = features.get('displacement', 0.0)
        if displacement > 5.0:
            votes += 2
        total_weight += 2
        
        # 特征4: 平均速度
        avg_speed = features.get('avg_speed', 0.0)
        if 2.0 <= avg_speed <= 15.0:
            votes += 1
        total_weight += 1
        
        score = votes / max(1, total_weight)
        return score > 0.35
    
    # ========== 阶段二新增接口 ==========
    
    def mark_destroyed(self) -> None:
        """标记目标已摧毁"""
        if self._is_destroyed:
            return
        self._is_destroyed = True
        self._destroyed_at_frame = self._frame_count
        if self._ema.value is not None:
            self._last_report_pos = self._ema.value
        if self.debug:
            print(f"[DecoyClassifier] 目标 {self.target_id} 已标记为摧毁")
    
    def is_destroyed(self) -> bool:
        return self._is_destroyed
    
    def get_frozen_position(self) -> Optional[Tuple[float, float]]:
        return self._last_report_pos
    
    def get_target_id(self) -> Optional[int]:
        return self.target_id
    
    def set_target_id(self, target_id: int) -> None:
        self.target_id = target_id
    
    # ========== 原有接口 ==========
    
    @property
    def is_real_target(self) -> bool:
        if self._is_destroyed:
            return False
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
        
        条件：
        1. 未摧毁
        2. 有足够帧数
        3. 判定为真目标
        4. 置信度 >= 0.5
        5. 频率限制（每5帧上报一次）
        """
        if self._is_destroyed:
            return False
        
        if self._frame_count < self.min_track_frames:
            return False
        
        if self._ema.value is None:
            return False
        
        if not self._is_real:
            return False
        
        if self._confidence < 0.5:
            return False
        
        # 频率限制：每5帧上报一次（2Hz）
        if self._frame_count - self._last_report_frame < 5:
            return False
        
        return True
    
    def get_report_position(self) -> Optional[Tuple[float, float]]:
        if self._is_destroyed:
            return self._last_report_pos
        return self._ema.value
    
    def mark_reported(self) -> None:
        """标记已上报"""
        self._report_count += 1
        self._last_report_frame = self._frame_count
        self._has_ever_reported = True
        if self.debug:
            print(f"[DecoyClassifier] 目标 {self.target_id}: 已上报 (第{self._report_count}次)")
    
    def reset(self) -> None:
        """重置分类器状态"""
        self._ema.reset()
        self._position_history.clear()
        self._frame_count = 0
        self._consecutive_real = 0
        self._consecutive_fake = 0
        self._is_real = False
        self._confidence = 0.0
        self._last_features = {}
        self._report_count = 0
        self._last_report_frame = -10
    
    def reset_full(self) -> None:
        """完全重置"""
        self.reset()
        self._is_destroyed = False
        self._destroyed_at_frame = None
        self._last_report_pos = None
        self._has_ever_reported = False
    
    def get_debug_info(self) -> Dict[str, Any]:
        """获取调试信息"""
        elapsed = time.time() - self._start_time
        return {
            'target_id': self.target_id,
            'is_real': self._is_real,
            'is_destroyed': self._is_destroyed,
            'confidence': self._confidence,
            'consecutive_real': self._consecutive_real,
            'consecutive_fake': self._consecutive_fake,
            'frame_count': self._frame_count,
            'has_ever_reported': self._has_ever_reported,
            'report_count': self._report_count,
            'real_frames': self._real_frames,
            'fake_frames': self._fake_frames,
            'elapsed': elapsed,
            'features': self._last_features,
            'smooth_pos': self._ema.value,
            'speed': self._ema.speed_mps(),
            'speed_var': self._last_features.get('speed_variance', 0),
            'dir_var': self._last_features.get('dir_variance', 0),
            'displacement': self._last_features.get('displacement', 0),
        }