"""桥接文件：将 SDK 的 Command 暴露为 coop_decoy.commands，供 agent.py 导入。"""
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
            pass

        @dataclass(frozen=True)
        class Command:
            verb: str
            params: dict

        def fly_to(lat, lon, alt=None, speed=None, loiter_radius=200.0):
            params = {"latitude": float(lat), "longitude": float(lon), "loiter_radius": float(loiter_radius)}
            if alt is not None: params["altitude"] = float(alt)
            if speed is not None: params["speed"] = float(speed)
            return Command("set_destination", params)

        def set_heading(heading_deg): return Command("set_heading", {"heading": float(heading_deg)})
        def set_speed(speed): return Command("set_speed", {"speed": float(speed)})
        def point_gimbal(pan, tilt): return Command("component.gimbal_tracking.set_orientation", {"pan": float(pan), "tilt": float(tilt)})
        def set_gimbal_fov(fov): return Command("set_fov", {"angle": float(fov)})
        def broadcast(payload): return Command("comm.broadcast", {"payload": str(payload)})
        def send_to(peer_uid, payload): return Command("comm.send", {"peer_target_unique_id": str(peer_uid), "payload": str(payload)})
        def report_target(lat, lon): return Command("agent.report", {"lat": float(lat), "lon": float(lon)})

__all__ = [
    "Command", "fly_to", "set_heading", "set_speed", "point_gimbal",
    "set_gimbal_fov", "broadcast", "send_to", "report_target", "PayloadTooLarge",
]
