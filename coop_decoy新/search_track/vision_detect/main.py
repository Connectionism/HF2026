"""
vision_detect 模块对外统一入口

对外暴露:
    EMATracker       — EMA 目标跟踪器（含速度方差、位移、方向变化方差等运动特征）
    DecoyClassifier  — 多特征投票诱饵判别器

内部实现:
    ema_filter.py        — EMATracker 类
    decoy_classifier.py  — DecoyClassifier 类

数据流:
    raw detection (lat, lon) → EMATracker.append() → DecoyClassifier.update()
    → is_real_target / confidence / should_report / get_report_position
"""

from .ema_filter import EMATracker
from .decoy_classifier import DecoyClassifier

__all__ = ["EMATracker", "DecoyClassifier"]
