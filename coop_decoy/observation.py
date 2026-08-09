"""桥接文件：将 SDK 的 Observation/Detection 暴露为 coop_decoy.observation，供 agent.py 导入。"""
try:
    from sdk.core.observation import Detection, Observation, SKIP_DETECTION
except ImportError:
    try:
        from competition.sdk.core.observation import Detection, Observation, SKIP_DETECTION
    except ImportError:
        from dataclasses import dataclass, field
        from typing import Any, Optional, Tuple

        @dataclass(frozen=True)
        class Detection:
            detected: bool = False
            confidence: float = 0.0
            target_lat: Optional[float] = None
            target_lon: Optional[float] = None
            target_type: str = ""

        @dataclass(frozen=True)
        class Observation:
            self: Any = None
            comm_inbox: Tuple = ()
            briefing: Any = None

        SKIP_DETECTION: Any = object()

__all__ = ["Detection", "Observation", "SKIP_DETECTION"]
