"""
vision_detect 模块对外统一入口

对外暴露:
    EMATracker          — EMA 目标跟踪器（含速度方差、位移、方向变化方差等运动特征）
    DecoyClassifier     — 多特征投票诱饵判别器
    YOLODetector        — YOLO 推理器（目标检测 + 像素→经纬度坐标转换 + 超时降级）
    get_detect_result   — 视觉检测统一入口（供 drone_agent.sensor() 调用，返回 List[Detection]）
    DetBox              — 像素检测框数据结构（类别 / 置信度 / 像素 bbox）

内部实现:
    ema_filter.py        — EMATracker 类
    decoy_classifier.py  — DecoyClassifier 类
    detect.py            — YOLODetector / get_detect_result / DetBox

数据流:
    photo(bytes) / 相机帧(numpy HxWx3)
    → detect.get_detect_result(): YOLO 推理 → 像素框 → pan_tilt_to_latlon 坐标转换
    → List[Detection]（sensor() 返回给 SDK，list[0] 注入 obs.self.detection）
    → decide() 中: raw detection(lat,lon) → EMATracker.append() → DecoyClassifier.update()
"""

from .ema_filter import EMATracker
from .decoy_classifier import DecoyClassifier
from .detect import YOLODetector, DetBox, get_detect_result

__all__ = [
    "EMATracker",
    "DecoyClassifier",
    "YOLODetector",
    "DetBox",
    "get_detect_result",
]
