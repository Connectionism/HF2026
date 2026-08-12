"""
motion_control 模块对外统一入口

对外暴露:
    SearchController  — 搜索航点生成（螺旋搜索 + 网格蛇形扫描）
    TrackController   — 盘旋跟踪航点 + 云台瞄准
    geo_utils         — 地理计算工具函数模块（haversine/bearing/clamp/point_on_circle/partition/los_angles）

内部实现:
    geo.py      — 地理计算工具函数
    search.py   — 搜索航点生成
    tracker.py  — 盘旋跟踪航点生成
"""

from . import geo as geo_utils
from .search import SearchController
from .tracker import TrackController

__all__ = ["SearchController", "TrackController", "geo_utils"]
