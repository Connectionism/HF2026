# src/vision_detect/__init__.py
"""
视觉识别模块 - 阶段二
提供目标滤波、诱饵判别、上报优化功能
"""

from .ema_filter import EMATracker, haversine_distance
from .decoy_classifier import DecoyClassifier
from .report import ReportOptimizer, make_report_message, parse_report_message

# 别名，兼容队长代码
VisionDetect = DecoyClassifier

__all__ = [
    'EMATracker',
    'haversine_distance',
    'DecoyClassifier',
    'VisionDetect',
    'ReportOptimizer',
    'make_report_message',
    'parse_report_message',
]