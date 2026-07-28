# -*- coding: utf-8 -*-
"""
地理工具模块
============================================================
提供公共几何计算函数，全员依赖，Day 2 最先完成。

接口定义:
  haversine_m(lat1, lon1, lat2, lon2) -> float
    两点间大圆距离(米)

  bearing_deg(lat1, lon1, lat2, lon2) -> float
    方位角(度, 0=正北, 顺时针)

  clamp_to_safebox(lat, lon) -> tuple[float, float]
    裁剪到飞行区域边界内(避免越界扣分)

飞行区域 (参赛手册 3):
  地图范围: 6.6km x 4.4km
  纬度: 26.9818 ~ 27.0250
  经度: 124.9800 ~ 125.0203
  越界阈值: 超出边界500m开始扣分

可复用代码:
  examples/uav_search_track_car/search_track/geometry.py
  (haversine / bearing / los_angles)

负责人: 成员2
"""

# TODO: 实现以下接口
#
# import math
#
# def haversine_m(lat1, lon1, lat2, lon2) -> float:
#     """两点间距离(米)"""
#     ...
#
# def bearing_deg(lat1, lon1, lat2, lon2) -> float:
#     """方位角(度, 0=正北, 顺时针)"""
#     ...
#
# def clamp_to_safebox(lat, lon) -> tuple:
#     """裁剪到飞行区域边界内"""
#     ...
