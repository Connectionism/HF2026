"""视觉检测模块统一入口 —— VisionDetect 类。

对外固定接口：
    VisionDetect.detect(obs) -> List[dict]
    VisionDetect.reset()     -> None

内部封装：
    DecoyClassifier —— EMA 滤波 + 运动模式诱饵判别
    ReportOptimizer —— 上报时机/频率优化（可选使用）
"""
from __future__ import annotations

from typing import Any, Dict, List

from .decoy_classifier import DecoyClassifier
from .report import ReportOptimizer


class VisionDetect:
    """视觉检测模块：从 obs 中提取平台检测结果并做真伪判别。

    detect(obs) 返回目标列表，每项为 dict：
        {"lat": float, "lon": float, "confidence": float, "is_real": bool}
    本帧无检测时返回空列表。
    """

    def __init__(self) -> None:
        self._classifier = DecoyClassifier()
        self._report_optimizer = ReportOptimizer()
        self._tick: int = 0

    def detect(self, obs: Any) -> List[Dict[str, Any]]:
        """从 obs.self.detection 提取本帧检测，做 EMA 滤波 + 诱饵判别。

        Args:
            obs: 平台注入的观测对象（含 self.detection）。

        Returns:
            list[dict]: 标准化目标列表；无检测时为空列表。
        """
        self._tick += 1
        results: List[Dict[str, Any]] = []

        self_view = getattr(obs, "self", None)
        det = getattr(self_view, "detection", None)
        if det is None or not getattr(det, "detected", False):
            return results

        lat = getattr(det, "target_lat", None)
        lon = getattr(det, "target_lon", None)
        if lat is None or lon is None:
            return results

        # EMA 滤波 + 运动模式诱饵判别
        self._classifier.update(lat, lon, dt=0.1)
        smoothed = self._classifier.get_report_position() or (lat, lon)

        results.append({
            "lat": smoothed[0],
            "lon": smoothed[1],
            "confidence": self._classifier.confidence,
            "is_real": self._classifier.is_real_target,
        })
        return results

    def reset(self) -> None:
        """每局开始前清零全部内部状态。"""
        self._classifier.reset()
        self._report_optimizer.reset()
        self._tick = 0
