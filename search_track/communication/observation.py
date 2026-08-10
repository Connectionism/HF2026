"""桥接文件：将 SDK 的 Observation/Detection 暴露为 src.observation，供 drone_agent 导入。

在仿真环境内运行时自动走 SDK 真路径；本地调试时走 fallback 定义。
"""
try:
    from sdk.core.observation import Detection, Observation, SKIP_DETECTION
except ImportError:
    try:
        from competition.sdk.core.observation import (
            Detection,
            Observation,
            SKIP_DETECTION,
        )
    except ImportError:
        from dataclasses import dataclass
        from typing import Any, Optional, Tuple

        @dataclass(frozen=True)
        class Detection:
            detected: bool = False                      # 是否看见
            confidence: float = 0.0                     # 确定度 0 到 1
            target_lat: Optional[float] = None          # 目标纬度
            target_lon: Optional[float] = None          # 目标经度
            target_type: str = ""                       # 类型

        @dataclass(frozen=True)
        class Observation:
            self: Any = None          # 自身状态
            comm_inbox: Tuple = ()    # 接收别人发给自己的消息
            briefing: Any = None      # 任务简报

        SKIP_DETECTION: Any = object()


__all__ = ["Detection", "Observation", "SKIP_DETECTION"]
