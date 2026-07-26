# -*- coding: utf-8 -*-
"""
诱饵判别模块
============================================================
多特征融合诱饵判别 (参赛手册 6.2):

关键约束:
  - 诱饵视觉特征与真目标完全一致(target_type字段不可靠)
  - 诱饵会被伪装成ground_vehicle
  - 仅能依靠多帧坐标时序变化(运动学一致性)判别
  - 诱饵是移动的，真目标也是移动的
  - 需靠连续多帧位置变化模式区分

判别特征:
  1. 速度特征:    真目标移动(5/9/12 m/s)，诱饵速度模式不同
  2. 加速度方差:  真目标运动模式稳定
  3. 运动方向一致性
  4. 位移跨度

多特征投票，误识别率目标 < 30%

参数 (config/algorithm.yaml):
  speed_confirm_threshold: 3.0   速度确认阈值(m/s)
  speed_reject_threshold:  1.0   速度拒绝阈值(m/s)
  verify_timeout:          8     验证超时(秒)

接口定义:
  class DecoyClassifier:
      def update(self, lat, lon, dt) -> None: ...
      @property
      def is_real_target(self) -> bool: ...
      @property
      def confidence(self) -> float: ...
      def reset(self) -> None: ...

可复用代码:
  examples/multi_uav_coop_decoy/search_track/decoy_classifier.py
  (位移跨度 + 平滑度判别)

负责人: 成员4
"""

# TODO: 实现诱饵判别器
#
# class DecoyClassifier:
#     def __init__(self, speed_confirm=3.0, speed_reject=1.0, timeout=8.0):
#         ...
#     def update(self, lat: float, lon: float, dt: float) -> None:
#         ...
#     @property
#     def is_real_target(self) -> bool:
#         ...
#     @property
#     def confidence(self) -> float:
#         ...
#     def reset(self) -> None:
#         ...
