"""
上报辅助模块（保留备用）
提供上报消息编解码工具函数。
"""
from typing import Optional, Tuple


def make_report_message(lat: float, lon: float) -> str:
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
