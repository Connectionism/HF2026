"""
通信模块对外统一入口

对外固定类名 CommHandle，配套 receive() / broadcast() 两个固定方法。
上层总控只需要知道这三个名字，不用关心内部协议、适配器的实现细节。

用法（在 decide() 中）:

    from HF2026_UAV_Challenge2.src.communication.main import CommHandle

    handle = CommHandle(drone_id="uav_alpha")

    # 收消息 — 传入 obs.comm_inbox
    msgs = handle.receive(obs.comm_inbox)
    for m in msgs:
        print(m["sender"], m["msg"])

    # 发消息 — 传出 CommCommand 给引擎
    cmd = handle.broadcast(TargetMsg('a', 27.01234, 125.03456, 0.85))
    return [cmd]
"""

from .client import CommClient
from .protocol import MsgType, encode

# 外部适配层 — 对接底层仿真通信通道（comm_adapter 位于 search_track 包根）
from .. import comm_adapter


class CommHandle:
    """无人机通信句柄（对外固定类名，不可修改）。

    封装了三层：
        1. 协议层 (protocol.py)     — encode / decode 编解码
        2. 客户端层 (client.py)     — 校验 / 统计
        3. 适配层 (comm_adapter.py) — 构建引擎 Command
    """

    def __init__(self, drone_id: str):
        """初始化通信句柄。

        参数:
            drone_id: 当前无人机编号，如 "uav_alpha"
        """
        self._drone_id = drone_id

        # 内部实例化自己的客户端（校验 / 统计）
        self._client = CommClient(uav_name=drone_id)

    # ------------------------------------------------------------------
    # receive — 收消息
    # ------------------------------------------------------------------

    def receive(self, inbox):
        """从收件箱拉取并解析所有队友发来的消息。

        参数:
            inbox: obs.comm_inbox，引擎每周期注入的 Message 序列

        返回:
            list[dict]: 每条消息包含
                sender     — 发送者 uid
                payload    — 原始字符串
                recv_time  — 接收时间戳
                msg        — 解码后的消息对象 (TargetMsg/DecoyMsg/...)
                dict       — 解码后的字典
        """
        return self._client.parse_inbox(inbox)

    # ------------------------------------------------------------------
    # broadcast — 发消息
    # ------------------------------------------------------------------

    def broadcast(self, data):
        """把本机数据广播给其余无人机。

        参数:
            data: 要发送的内容，可以是:
                  - MsgType 对象 (TargetMsg / DecoyMsg / ...)，内部自动 encode
                  - str 原始字符串，直接发送

        返回:
            comm_adapter.CommCommand — 可直接交给引擎执行
        """
        if isinstance(data, str):
            payload = data
        else:
            payload = encode(data)

        # 内部校验 (长度 / 格式)
        self._client.build_broadcast(payload)

        # 通过适配层构建引擎命令
        return comm_adapter.broadcast(self._drone_id, payload)
