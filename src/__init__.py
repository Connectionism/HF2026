# -*- coding: utf-8 -*-
"""
HF2026 - 无人机集群自主协同跟踪系统
红枫2026无人集群自主协同智能算法挑战赛 - 赛题二：多机协同识别

项目模块:
  communication/      通信协议编解码 + 平台API封装
  motion_control/     航迹规划 + 云台跟踪 + 地理工具
  cluster_scheduler/  K=2协同分配 + 站位策略 + 状态机
  vision_detect/      EMA滤波 + 诱饵判别 + 上报优化
  main.py             主程序入口
"""

__version__ = "0.1.0"
__author__ = "HF2026 Team"
