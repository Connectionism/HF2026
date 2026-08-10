"""桥接文件：将 SDK 的 Command 暴露为 src.commands，供 drone_agent 导入。

在仿真环境内运行时自动走 SDK 真路径；本地调试时走 fallback 定义。
"""
#转接头
try:
    from sdk.core.commands import (
        Command,
        fly_to,
        set_heading,
        set_speed,
        point_gimbal,
        set_gimbal_fov,
        broadcast,
        send_to,
        report_target,
        PayloadTooLarge,
    )
except ImportError:
    try:
        from competition.sdk.core.commands import (
            Command,
            fly_to,
            set_heading,
            set_speed,
            point_gimbal,
            set_gimbal_fov,
            broadcast,
            send_to,
            report_target,
            PayloadTooLarge,
        )
    except ImportError:
        from dataclasses import dataclass

        class PayloadTooLarge(ValueError):
            """Raised when a comm payload exceeds the 50-byte cap."""
            pass

        @dataclass(frozen=True)
        class Command:
            """SDK-standard command: verb + params dict."""
            verb: str
            params: dict
#飞到坐标
        def fly_to(lat, lon, alt=None, speed=None, loiter_radius=200.0):
            params = {
                "latitude": float(lat),
                "longitude": float(lon),
                "loiter_radius": float(loiter_radius),
            }
            if alt is not None:
                params["altitude"] = float(alt)
            if speed is not None:
                params["speed"] = float(speed)
            return Command("set_destination", params)
#机头朝向
        def set_heading(heading_deg):
            return Command("set_heading", {"heading": float(heading_deg)})
#设速度
        def set_speed(speed):
            return Command("set_speed", {"speed": float(speed)})
#转云台
        def point_gimbal(pan, tilt):
            return Command(
                "component.gimbal_tracking.set_orientation",
                {"pan": float(pan), "tilt": float(tilt)},
            )
#调焦距
        def set_gimbal_fov(fov):
            return Command("set_fov", {"angle": float(fov)})
#群发消息
        def broadcast(payload):
            return Command("comm.broadcast", {"payload": str(payload)})
#跟QQ私聊差不多
        def send_to(peer_uid, payload):
            return Command(
                "comm.send",
                {"peer_target_unique_id": str(peer_uid), "payload": str(payload)},
            )
#上报裁判
        def report_target(lat, lon):
            return Command("agent.report", {"lat": float(lat), "lon": float(lon)})


__all__ = [
    "Command",
    "fly_to",
    "set_heading",
    "set_speed",
    "point_gimbal",
    "set_gimbal_fov",
    "broadcast",
    "send_to",
    "report_target",
    "PayloadTooLarge",
]
