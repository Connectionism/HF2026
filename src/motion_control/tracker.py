# -*- coding: utf-8 -*-
"""
跟踪控制模块
============================================================
K=2站位跟踪策略:

  - 2架UAV在目标周围不同方位(SLOT_0/SLOT_1)盘旋
  - 保持两机间距 > 200m 避免过近扣分
  - 云台LOS瞄准保持目标在FOV中心
  - 跟踪环半径可配置，建议330m

站位几何:
  SLOT_0: 目标方位角+0度, 环半径R处盘旋
  SLOT_1: 目标方位角+180度, 环半径R处盘旋
  两者间距 = 2R, 需保证 2R > 200m

跟踪参数 (config/algorithm.yaml):
  track_radius:    330    跟踪环半径(米)
  track_speed:     20     跟踪盘旋速度(m/s)
  slot_spacing:    250    站位间距(米), 必须>200m
  gimbal_fov:      30     云台视场角(度)

可复用代码:
  competition/baselines/coop_distributed.py 的 _tracking_gimbal() 方法

负责人: 成员2
"""

# TODO: 实现跟踪控制
#
# class TrackController:
#     def __init__(self, track_radius=330, track_speed=20):
#         ...
#     def loiter_waypoint(self, target_lat, target_lon, slot, current_heading) -> tuple:
#         """返回盘旋航点(lat, lon)"""
#         ...
#     def gimbal_point(self, target_lat, target_lon) -> dict:
#         """返回云台瞄准指令"""
#         ...
#     def slot_position(self, target_lat, target_lon, slot) -> tuple:
#         """返回站位位置(lat, lon)"""
#         ...
