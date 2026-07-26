# -*- coding: utf-8 -*-
"""
目标上报模块
============================================================
report_target(lat, lon) 上报优化 (参赛手册 5.2):

上报规则:
  - Agent用专用接口 report_target(lat, lon) 上报
  - 评分器按 per-target 独立考核
  - 每个真目标分别计算上报的RMSE得出子分
  - 最终accuracy为全部真目标子分的算术平均
  - 从未上报的目标子分为0计入平均
  - 每秒每目标最多记1次
  - 上报点若离已摧毁目标更近(报"尸体")则整条丢弃

优化策略:
  - 只报告EMA平滑后的位置，降低RMSE
  - 目标摧毁后停止对该目标的上报
  - 目标RMSE < 60m 为优

评分公式:
  accuracy = 100 * max(0, 1 - RMSE/120m)

负责人: 成员4
"""

# TODO: 实现上报优化
#
# class ReportManager:
#     def __init__(self, report_interval=1.0):
#         ...
#     def should_report(self, target_id) -> bool:
#         ...
#     def get_report_position(self, ema_tracker) -> tuple | None:
#         """返回EMA平滑后的上报位置"""
#         ...
#     def mark_destroyed(self, target_id) -> None:
#         ...
