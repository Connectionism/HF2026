# -*- coding: utf-8 -*-
"""
EMA滤波与速度估计模块
============================================================
指数移动平均(EMA)滤波 + 线性回归速度估计:

  - 平滑GPS位置噪声 (sigma=50m)
  - 估计目标运动速度(线性回归)
  - 维护历史位置队列用于运动学分析

EMA公式:
  S_t = alpha * X_t + (1 - alpha) * S_{t-1}
  alpha越大越跟踪噪声，越小越平滑

参数 (config/algorithm.yaml):
  ema_alpha:    0.3    EMA平滑系数
  ema_history:  80     历史队列长度(帧), 8s@10Hz

接口定义:
  class EMATracker:
      def __init__(self, alpha=0.3, history=80): ...
      def append(self, lat, lon) -> None: ...
      @property
      def value(self) -> tuple[float, float] | None: ...
      def speed_mps(self, tick_hz=10.0) -> float: ...
      def reset(self) -> None: ...

可复用代码:
  competition/baselines/coop_distributed.py 的 _EMATracker 类

负责人: 成员4
"""

# TODO: 实现EMA滤波器
#
# class EMATracker:
#     def __init__(self, alpha: float = 0.3, history: int = 80):
#         ...
#     def append(self, lat: float, lon: float) -> None:
#         ...
#     @property
#     def value(self) -> tuple | None:
#         ...
#     def speed_mps(self, tick_hz: float = 10.0) -> float:
#         ...
#     def reset(self) -> None:
#         ...
