# -*- coding: utf-8 -*-
"""
状态机模块
============================================================
每架UAV的状态转换流程:

  SEARCH -> VERIFY -> TRACK -> RELEASE -> SEARCH
    ^                                    |
    |________ K=2 召唤(R:消息) _________|

状态说明:
  SEARCH:   扇区搜索巡航，发现目标后转入VERIFY
  VERIFY:   多帧验证目标真伪(EMA滤波+诱饵判别)
            确认真目标 -> 广播R:召唤队友 -> 转入TRACK
            判定诱饵   -> 广播D:共享 -> 返回SEARCH
  TRACK:    K=2协同盯防，累计dwell时间
            dwell>=20s -> 广播C:摧毁通知 -> 转入RELEASE
  RELEASE:  目标摧毁后释放，转向下一未完成目标 -> 返回SEARCH

盯防规则 (参赛手册 5.2):
  - K=2: >=2架UAV同时对同一真目标连续有效跟踪累计满20s
  - 短暂中断<=2s不清零且会回补
  - 中断>2s清零重来
  - 继续跟踪已摧毁目标不计入得分

负责人: 成员3
"""

# TODO: 实现状态机
#
# from enum import Enum
#
# class UAVState(Enum):
#     SEARCH = "SEARCH"
#     VERIFY = "VERIFY"
#     TRACK = "TRACK"
#     RELEASE = "RELEASE"
#
# class StateMachine:
#     def __init__(self):
#         self.state = UAVState.SEARCH
#         self.dwell_timer = 0.0
#         ...
#     def transition(self, obs, dt) -> UAVState:
#         """状态转换逻辑"""
#         ...
