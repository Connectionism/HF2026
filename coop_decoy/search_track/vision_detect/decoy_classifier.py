"""
诱饵判别器模块

来源: new drone_agent.py _DecoyClassifier (第 349-536 行)
功能: 多特征投票诱饵判别 —— 区分"匀速直线运动（真目标）" vs "随机游走（诱饵）"

特征1: 速度方差 - 真目标小，诱饵大（随机加减速）
特征2: 方向变化方差 - 真目标小（直线行驶），诱饵大（随机转向）
特征3: 位移平滑度 - 真目标位移均匀，诱饵跳跃
特征4: 平均速度合理性 - 真目标速度在合理范围
特征5: 视觉检测融合 - 利用 detection.confidence 辅助判别
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

from .ema_filter import EMATracker


class DecoyClassifier:
    """
    诱饵判别器 - 新规则适配版

    新规则核心变化：
    1. 诱饵也是移动的！不能靠"有没有速度"判断
    2. 诱饵误判不扣分 → 策略可以更激进（宁可误报，不可漏报）

    判别策略：多特征投票，区分"匀速直线运动（真目标）" vs "随机游走（诱饵）"
    """

    SPEED_VAR_THRESHOLD = 2.5      # 真目标速度方差应更小
    DIR_VAR_THRESHOLD = 0.20      # 真目标方向变化应更小
    MIN_TRACK_FRAMES = 4          # 从 10 降到 4：VERIFY 期间检测帧数有限，尽早启动投票
    VERIFY_FRAMES = 3             # 从 5 降到 3：只需连续 3 帧投票通过即可确认

    def __init__(self, alpha: float = 0.25):
        self._alpha = alpha
        self._ema = EMATracker(alpha=alpha, history=60)
        self._frame_count = 0
        self._consecutive_real = 0
        self._consecutive_fake = 0
        self._is_real = False
        self._confidence = 0.0
        self._last_features: Dict[str, float] = {}
        # 视觉融合接口：后续接入 YOLO/视觉检测后，由外部调用 set_visual_confidence() 填充
        self._visual_confidence: Optional[float] = None
        self._visual_label: Optional[str] = None  # "target" / "decoy" / None(未启用)

    def update(self, lat: float, lon: float, dt: float = 0.1, sim_t: float = 0.0) -> None:
        """更新观测值"""
        self._frame_count += 1
        self._ema.append(lat, lon, sim_t)

        if self._frame_count < self.MIN_TRACK_FRAMES:
            self._confidence = 0.5  # 中性初始值，速度计算修复后不应预设为偏低
            return

        features = self._compute_features()
        self._last_features = features

        is_real = self._voting_decision(features)

        if is_real:
            self._consecutive_real += 1
            self._consecutive_fake = max(0, self._consecutive_fake - 1)
        else:
            self._consecutive_real = max(0, self._consecutive_real - 1)
            self._consecutive_fake += 1

        if self._consecutive_real >= self.VERIFY_FRAMES:
            self._is_real = True
            self._confidence = min(1.0, self._confidence + 0.10)
        elif self._consecutive_fake >= self.VERIFY_FRAMES:
            self._is_real = False
            self._confidence = max(0.0, self._confidence - 0.08)

        self._confidence = max(0.0, min(1.0, self._confidence))

    def _compute_features(self) -> Dict[str, float]:
        """计算多维度运动模式特征"""
        features = {}
        features['speed_variance'] = self._ema.speed_variance(window=15)
        features['dir_variance'] = self._ema.direction_change_variance(window=15)
        features['avg_speed'] = self._ema.speed_mps()
        features['displacement'] = self._ema.displacement()
        return features

    def _voting_decision(self, features: Dict[str, float]) -> bool:
        """
        多特征投票决策

        移动诱饵(5m/s)与慢速真目标(5m/s)速度重叠，需综合多维度判别。
        阈值调整：提高位移权重，使用更保守的分数阈值避免将慢速真目标误判为诱饵。
        """
        votes = 0
        total_weight = 0

        # 特征1: 速度方差投票（权重最高）
        speed_var = features.get('speed_variance', 100.0)
        if speed_var < self.SPEED_VAR_THRESHOLD:
            votes += 4
        elif speed_var < self.SPEED_VAR_THRESHOLD * 2:
            votes += 2
        total_weight += 4

        # 特征2: 方向变化方差投票
        dir_var = features.get('dir_variance', 1.0)
        if dir_var < self.DIR_VAR_THRESHOLD:
            votes += 3
        elif dir_var < self.DIR_VAR_THRESHOLD * 2:
            votes += 1.5
        total_weight += 3

        # 特征3: 位移充分性（真目标必须有足够位移，权重提高到3）
        displacement = features.get('displacement', 0.0)
        if displacement > 8.0:
            votes += 3
        elif displacement > 5.0:
            votes += 1.5
        total_weight += 3

        # 特征4: 平均速度合理性（真目标 5~15 m/s）
        avg_speed = features.get('avg_speed', 0.0)
        if 3.0 <= avg_speed <= 18.0:
            votes += 1
        total_weight += 1

        # 特征5: 视觉检测融合（仅利用 detection.confidence 做弱辅助）
        # target_type 不可靠（手册 §6.2），权重从 3 降到 1
        if self._visual_confidence is not None:
            if self._visual_label == "target" and self._visual_confidence >= 0.80:
                votes += 1  # 高置信度，弱支持真目标
            elif self._visual_label == "decoy" and self._visual_confidence < 0.30:
                votes -= 1  # 极低置信度，弱指向诱饵
            elif self._visual_label == "unknown":
                pass  # 中性，不加不减
            total_weight += 1

        score = votes / max(1, total_weight)
        return score > 0.55  # 从 0.65 降到 0.55，降低单帧投票门槛

    @property
    def is_real_target(self) -> bool:
        return self._is_real

    @property
    def confidence(self) -> float:
        return self._confidence

    @property
    def smoothed_position(self) -> Optional[Tuple[float, float]]:
        return self._ema.value

    @property
    def speed_mps(self) -> float:
        return self._ema.speed_mps()

    def should_report(self) -> bool:
        """判断是否应该上报（新规则：置信度 > 0.5 即可）"""
        if self._frame_count < self.MIN_TRACK_FRAMES:
            return False
        if self._ema.value is None:
            return False
        return self._is_real and self._confidence >= 0.5

    def get_report_position(self) -> Optional[Tuple[float, float]]:
        return self._ema.value

    def reset(self) -> None:
        self._ema.reset()
        self._frame_count = 0
        self._consecutive_real = 0
        self._consecutive_fake = 0
        self._is_real = False
        self._confidence = 0.0
        self._last_features.clear()
        self._visual_confidence = None
        self._visual_label = None

    def set_visual_confidence(self, confidence: float, label: str) -> None:
        """
        设置视觉检测结果（预留接口）。
        后续接入 YOLO/视觉模型后调用此方法填充视觉判别信息。

        Args:
            confidence: 视觉模型置信度 (0.0~1.0)
            label: "target" 或 "decoy"
        """
        self._visual_confidence = max(0.0, min(1.0, confidence))
        self._visual_label = label

    def apply_yolo_hint(self, det) -> None:
        """
        从 YOLO/默认识别器的 Detection 结果中提取 confidence 作为弱辅助信号。

        重要：手册 §6.2 和 SDK-API §3.2 明确说明 target_type 字段不可靠，
        诱饵会被伪装成 ground_vehicle。因此仅使用 confidence 做弱辅助，
        不依赖 target_type 做分类。

        应由 drone_agent.decide() 每帧调用（在 update() 之前或之后均可）。
        """
        if det is None or not getattr(det, 'detected', False):
            return
        conf = float(getattr(det, 'confidence', 0.5))

        # 高置信度 → 可能是真目标（YOLO 对真目标检出更稳定）
        # 低置信度 → 可能是诱饵或误检（YOLO 对诱饵置信度波动大）
        # 中等置信度 → 无法确定，中性
        if conf >= 0.80:
            self.set_visual_confidence(conf, "target")
        elif conf < 0.35:
            self.set_visual_confidence(conf, "decoy")
        else:
            self.set_visual_confidence(conf, "unknown")

    def get_debug_info(self) -> Dict[str, object]:
        return {
            'is_real': self._is_real,
            'confidence': self._confidence,
            'consecutive_real': self._consecutive_real,
            'consecutive_fake': self._consecutive_fake,
            'features': self._last_features,
            'smooth_pos': self._ema.value,
            'speed': self._ema.speed_mps(),
        }
