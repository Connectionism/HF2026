"""Multi-entity state view for the cooperative challenge.

Parses one sim:state Redis frame into a MultiSimState containing all
entities (UAVs + ground vehicles + decoys) indexed by unique_id.

The state dataclasses (GeoPosition / Attitude / UavState / GimbalState /
Detection / TargetState) are defined inline to remove the external
dependency on ``examples.uav_search_track_car.search_track.state``.
"""
#把仿真引擎发来的原始 JSON 拆成好用的 Python 对象
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


# ── shared state primitives (inline — no external dependency) ────────────
# 避免每次都写 raw["uav_0"]["platform"]["position"]["latitude"]，之后直接 uav.position.latitude

@dataclass(frozen=True)
class GeoPosition:
    latitude: float
    longitude: float
    altitude: float


@dataclass(frozen=True)
class Attitude:
    yaw: float
    pitch: float
    roll: float


@dataclass(frozen=True)
class UavState:
    position: GeoPosition
    attitude: Attitude
    velocity: float
    heading: float


@dataclass(frozen=True)
class GimbalState:
    pan_angle: float
    tilt_angle: float
    track_enabled: bool
    fov_deg: float | None = None


@dataclass(frozen=True)
class Detection:
    detected: bool
    confidence: float
    target_position: GeoPosition | None
    azimuth_error_deg: float | None


@dataclass(frozen=True)
class TargetState:
    """Ground-truth target state (used by metrics, NOT by controller)."""
    position: GeoPosition
    speed: float
    heading: float


# ── 017 extended types ───────────────────────────────────────────────────


@dataclass(frozen=True)
class CommStats:    
    sent: int = 0
    delivered: int = 0
    received: int = 0
    rejected_bytes: int = 0
    rejected_rate: int = 0
    rejected_range: int = 0
        rejected_jam: int = 0
# CommStats：通信统计（发了多少、成功了多少）

@dataclass(frozen=True)
class CommInboxEntry:
    sender: str
    payload: str
    recv_time: float
# CommInboxEntry：跟QQ邮箱作用差不多

@dataclass(frozen=True)
class CommState:
    """Per-UAV communication state (only present on UAV entities)."""
    enabled: bool = False
    range_m: float = 1000.0
    max_bytes: int = 50
    max_rate_hz: float = 4.0
    inbox: tuple[CommInboxEntry, ...] = field(default_factory=tuple)
    stats: CommStats = field(default_factory=CommStats)
# CommState：一台 UAV 的完整通信状态（收件箱 + 统计 + 通信参数）

@dataclass(frozen=True)
class ExtendedDetection(Detection):
    """Detection + mis-id fields (FR-014/015).

    ``target_uid`` mirrors ``gimbal_tracking.target_entity`` if the engine
    ever publishes it; today it is empty, so the evaluator falls back to
    nearest-neighbour position matching.
    """
    target_type: str = ""
    misid_flag: bool = False
    misid_count: int = 0
    misid_track_duration: float = 0.0
    target_uid: str = ""
# ExtendedDetection：继承 Detection，额外加了诱饵识别字段

@dataclass(frozen=True)
class EntityState:
    """One entity (UAV or vehicle) in the multi-entity view."""
    uid: str
    kind: str                      # "uav" | "ground_vehicle" | "decoy_vehicle"
    name: str
    uav: UavState | None = None
    gimbal: GimbalState | None = None
    detection: ExtendedDetection | None = None
    comm: CommState | None = None
    vehicle_truth: TargetState | None = None
# EntityState：一个实体的"完整画像"（UAV 或车辆）

@dataclass(frozen=True)
class MultiSimState:
    """All entities for one tick + sim metadata."""
    sim_time: float
    timestamp: float
    status: str
    entities: dict[str, EntityState]
# MultiSimState：一帧的所有实体 + 仿真元数据

# ── parsers ──────────────────────────────────────────────────────────────
# 三个内部函数 + 一个主入口

def _parse_detection(det_raw: dict[str, Any]) -> ExtendedDetection:
    det_pos_raw = det_raw.get("target_position")
    det_pos = None
    if det_pos_raw:
        det_pos = GeoPosition(
            latitude=float(det_pos_raw.get("latitude", 0.0)),
            longitude=float(det_pos_raw.get("longitude", 0.0)),
            altitude=float(det_pos_raw.get("altitude", 0.0)),
        )
    return ExtendedDetection(
        detected=bool(det_raw.get("detected", False)),
        confidence=float(det_raw.get("confidence", 0.0)),
        target_position=det_pos,
        azimuth_error_deg=det_raw.get("azimuth_error"),
        target_type=str(det_raw.get("target_type", "")),
        misid_flag=bool(det_raw.get("misid_flag", False)),
        misid_count=int(det_raw.get("misid_count", 0)),
        misid_track_duration=float(det_raw.get("misid_track_duration", 0.0)),
        target_uid=str(det_raw.get("target_entity", "")),
    )

# _parse_uav：拆一台无人机（位置/姿态/云台/检测/通信）
def _parse_uav(uid: str, raw: dict[str, Any]) -> EntityState:
    platform = raw.get("platform", {}) or {}
    pos = platform.get("position", {}) or {}
    att = platform.get("attitude", {}) or {}
    uav = UavState(
        position=GeoPosition(
            latitude=float(pos.get("latitude", 0.0)),
            longitude=float(pos.get("longitude", 0.0)),
            altitude=float(pos.get("altitude", 0.0)),
        ),
        attitude=Attitude(
            yaw=float(att.get("yaw", 0.0)),
            pitch=float(att.get("pitch", 0.0)),
            roll=float(att.get("roll", 0.0)),
        ),
        velocity=float(raw.get("velocity", 0.0)),
        heading=float(raw.get("heading", att.get("yaw", 0.0))),
    )
    gimbal_raw = raw.get("gimbal_tracking", {}) or {}
    gimbal = GimbalState(
        pan_angle=float(gimbal_raw.get("pan_angle", 0.0)),
        tilt_angle=float(gimbal_raw.get("tilt_angle", 0.0)),
        track_enabled=bool(gimbal_raw.get("track_enabled", False)),
        fov_deg=gimbal_raw.get("fov_deg"),
    )
    det_raw = gimbal_raw.get("detection", {}) or {}
    detection = _parse_detection(det_raw)
    comm_raw = raw.get("comm", {}) or {}
    comm: CommState | None = None
    if comm_raw:
        inbox = tuple(
            CommInboxEntry(
                sender=str(e.get("sender", "")),
                payload=str(e.get("payload", "")),
                recv_time=float(e.get("recv_time", 0.0)),
            )
            for e in (comm_raw.get("inbox", []) or [])
        )
        stats_raw = comm_raw.get("stats", {}) or {}
        comm = CommState(
            enabled=bool(comm_raw.get("enabled", False)),
            range_m=float(comm_raw.get("range_m", 1000.0)),
            max_bytes=int(comm_raw.get("max_bytes", 50)),
            max_rate_hz=float(comm_raw.get("max_rate_hz", 4.0)),
            inbox=inbox,
            stats=CommStats(
                sent=int(stats_raw.get("sent", 0)),
                delivered=int(stats_raw.get("delivered", 0)),
                received=int(stats_raw.get("received", 0)),
                rejected_bytes=int(stats_raw.get("rejected_bytes", 0)),
                rejected_rate=int(stats_raw.get("rejected_rate", 0)),
                rejected_jam=int(stats_raw.get("rejected_jam", 0)),
            ),
        )
    return EntityState(
        uid=uid, kind="uav", name=str(raw.get("name", uid)),
        uav=uav, gimbal=gimbal, detection=detection, comm=comm,
    )

# _parse_vehicle：拆一台地面车/诱饵（仅位置/速度/航向）
def _parse_vehicle(uid: str, raw: dict[str, Any], kind: str) -> EntityState:
    platform = raw.get("platform", {}) or {}
    pos = platform.get("position", {}) or {}
    truth = TargetState(
        position=GeoPosition(
            latitude=float(pos.get("latitude", 0.0)),
            longitude=float(pos.get("longitude", 0.0)),
            altitude=float(pos.get("altitude", 0.0)),
        ),
        speed=float(raw.get("speed", 0.0)),
        heading=float(raw.get("heading", 0.0)),
    )
    return EntityState(
        uid=uid, kind=kind, name=str(raw.get("name", uid)),
        vehicle_truth=truth,
    )


_NON_ENTITY_KEYS = frozenset({
    "timestamp", "status", "sim_time", "sim_time_str", "step_perf",
})


def parse_multi_sim_state(raw: dict[str, Any]) -> MultiSimState:
    """Parse one sim:state frame into a MultiSimState."""
    sim_time = float(raw.get("sim_time", 0.0))
    timestamp = float(raw.get("timestamp", 0.0))
    status = str(raw.get("status", "unknown"))
    entities: dict[str, EntityState] = {}
    for key, val in raw.items():
        if key in _NON_ENTITY_KEYS or not isinstance(val, dict):
            continue
        etype = str(val.get("type", ""))
        if etype in ("fixed_wing_uav", "uav"):
            entities[key] = _parse_uav(key, val)
        elif etype == "ground_vehicle":
            entities[key] = _parse_vehicle(key, val, "ground_vehicle")
        elif etype == "decoy_vehicle":
            entities[key] = _parse_vehicle(key, val, "decoy_vehicle")
    return MultiSimState(
        sim_time=sim_time, timestamp=timestamp,
        status=status, entities=entities,
    )
