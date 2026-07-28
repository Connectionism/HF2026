# -*- coding: utf-8 -*-
"""
通信协议编解码模块
============================================================
五类消息格式定义与编解码:

  消息类型    格式                 含义                发送时机
  ------------------------------------------------------------------
  T:        T:lat,lon           确认真目标位置共享    VERIFY确认真目标后
  D:        D:lat,lon           确认诱饵位置共享      VERIFY判定诱饵后
  A:        A:tgtidx,rank       目标认领声明          进入TRACK时
  C:        C:tgtidx            目标已摧毁通知        dwell>=20s后
  R:        R:lat,lon           召唤队友汇聚          K=2需要队友时

通信约束 (参赛手册 6.3):
  - 单条消息 <= 50 字节
  - 发送频率 <= 4 Hz
  - 通信极限距离 ~1000 m
  - 收件箱容量 32 条

负责人: 成员1
"""

# TODO: 实现以下接口
#
# def encode_target(lat, lon) -> str:      # "T:27.00512,125.00134"
# def decode_target(payload) -> tuple|None:
# def encode_decoy(lat, lon) -> str:       # "D:..."
# def decode_decoy(payload) -> tuple|None:
# def encode_claim(tgt_idx, rank) -> str:  # "A:0,20002"
# def decode_claim(payload) -> tuple|None:
# def encode_destroyed(tgt_idx) -> str:    # "C:0"
# def decode_destroyed(payload) -> int|None:
# def encode_summon(lat, lon) -> str:      # "R:27.00512,125.00134"
# def decode_summon(payload) -> tuple|None:
