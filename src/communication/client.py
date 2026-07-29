"""
Redis 通信客户端封装

用法:
    client = CommClient(host="127.0.0.1", port=6379)
    client.connect()
    client.subscribe("sim:states")          # 订阅状态频道
    client.publish("sim:commands", "hello") # 发送消息
"""

import json
import time
import redis
from typing import Optional, Callable


class CommClient:
    """
    基于 Redis Pub/Sub 的通信客户端

    两个默认频道（跟比赛 SDK 一致）:
        CMD_CHANNEL   = "sim:commands"   # UAV 发命令给引擎
        STATE_CHANNEL = "sim:states"     # 引擎广播状态给 UAV
    """

    CMD_CHANNEL = "sim:commands"
    STATE_CHANNEL = "sim:state"

    def __init__(self, host: str = "127.0.0.1", port: int = 6379):
        self.host = host
        self.port = port
        self._redis: Optional[redis.Redis] = None
        self._pubsub: Optional[redis.client.PubSub] = None

    # ---------- 连接管理 ----------

    def connect(self) -> bool:
        """连接 Redis，成功返回 True"""
        try:
            self._redis = redis.Redis(
                host=self.host,
                port=self.port,
                decode_responses=True,      # 自动把 bytes 转成 str
                socket_connect_timeout=5,   # 连接超时 5 秒
                socket_timeout=5,           # 读写超时 5 秒
            )
            self._redis.ping()  # 测试连接
            return True
        except Exception as e:
            print(f"[CommClient] Redis 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self._pubsub:
            self._pubsub.close()
            self._pubsub = None
        if self._redis:
            self._redis.close()
            self._redis = None

    def is_connected(self) -> bool:
        """检查是否已连接"""
        if not self._redis:
            return False
        try:
            self._redis.ping()
            return True
        except:
            return False

    # ---------- 发布 ----------

    def publish(self, channel: str, message: str):
        """
        向指定频道发布一条消息

        示例:
            client.publish("sim:commands", "T:a,27.01234,125.03456,85")
        """
        if not self._redis:
            raise RuntimeError("Redis 未连接，先调用 connect()")
        self._redis.publish(channel, message)

    def publish_json(self, channel: str, data: dict):
        """发布 JSON 格式的消息（自动序列化）"""
        self.publish(channel, json.dumps(data, ensure_ascii=False))

    # ---------- 订阅 ----------

    def subscribe(self, channel: str):
        """订阅一个频道，开始接收消息"""
        if not self._redis:
            raise RuntimeError("Redis 未连接，先调用 connect()")
        self._pubsub = self._redis.pubsub()
        self._pubsub.subscribe(channel)

    def subscribe_multi(self, channels: list[str]):
        """同时订阅多个频道"""
        if not self._redis:
            raise RuntimeError("Redis 未连接，先调用 connect()")
        self._pubsub = self._redis.pubsub()
        self._pubsub.subscribe(*channels)

    # ---------- 接收 ----------

    def get_message(self, timeout: float = 0.1) -> Optional[dict]:
        """
        非阻塞获取一条消息

        参数:
            timeout: 等待秒数，默认 0.1 秒。0 表示立即返回

        返回:
            有消息返回 {'channel': 'xxx', 'data': 'yyy'}
            没消息返回 None
        """
        if not self._pubsub:
            return None

        msg = self._pubsub.get_message(timeout=timeout)

        if msg and msg.get("type") == "message":
            return {
                "channel": msg["channel"],
                "data": msg["data"],
            }
        return None

    def get_message_json(self, timeout: float = 0.1) -> Optional[dict]:
        """获取消息并自动解析 JSON"""
        msg = self.get_message(timeout=timeout)
        if msg:
            try:
                msg["data"] = json.loads(msg["data"])
            except json.JSONDecodeError:
                pass  # 不是 JSON 就保持原样
        return msg

    def listen(self, callback: Callable[[str], None], stop_check: Optional[Callable[[], bool]] = None):
        """
        持续监听消息（阻塞式）

        参数:
            callback: 收到消息时调用的函数，参数是消息字符串
            stop_check: 可选，返回 True 时停止监听

        示例:
            def on_msg(data):
                print(f"收到: {data}")

            client.listen(on_msg)
        """
        if not self._pubsub:
            raise RuntimeError("未订阅频道，先调用 subscribe()")

        print("[CommClient] 开始监听消息...")
        try:
            while True:
                if stop_check and stop_check():
                    break

                msg = self.get_message(timeout=0.5)
                if msg:
                    callback(msg["data"])

        except KeyboardInterrupt:
            print("[CommClient] 监听被手动停止")

    # ---------- 高级：带 UID 的命令发送（跟比赛 SDK 对接）----------

    def send_command(self, uid: str, payload: str, channel: str = CMD_CHANNEL):
        """
        发送带 UAV 身份标识的命令（跟比赛引擎格式一致）

        格式: {"uid": "uav_0", "payload": "T:a,27.0,125.0,85", "timestamp": 1234567.89}
        """
        cmd = {
            "uid": uid,
            "payload": payload,
            "timestamp": time.time(),
        }
        self.publish_json(channel, cmd)

    def recv_state(self, timeout: float = 0.1) -> Optional[dict]:
        """
        从状态频道接收一条引擎广播的状态（自动解析 JSON）
        """
        msg = self.get_message(timeout=timeout)
        if not msg:
            return None
        try:
            return json.loads(msg["data"])
        except json.JSONDecodeError:
            return {"raw": msg["data"]}
