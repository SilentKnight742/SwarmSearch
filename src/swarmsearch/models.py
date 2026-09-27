from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class UAVState(str, Enum):
    DISCONNECTED = "disconnected"
    INITIALIZING = "initializing"
    READY = "ready"

    ARMING = "arming"
    TAKING_OFF = "taking_off"

    SEARCHING = "searching"
    RECOVERING = "recovering"

    RETURNING = "returning"
    LANDING = "landing"
    LANDED = "landed"

    FAILED = "failed"


class MissionState(str, Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    READY = "ready"
    TAKING_OFF = "taking_off"
    ACTIVE = "active"
    RECOVERING = "recovering"
    LANDING = "landing"

    COMPLETED = "completed"
    PARTIAL = "partial"
    ABORTED = "aborted"


@dataclass
class LocalPosition:
    east: float
    north: float
    altitude: float


@dataclass
class GeoPosition:
    latitude: float
    longitude: float
    relative_altitude: float


@dataclass
class Waypoint:
    east: float
    north: float


@dataclass
class UAVStatus:
    name: str
    system_id: int

    state: UAVState = UAVState.DISCONNECTED

    altitude: float = 0.0

    latitude: Optional[float] = None
    longitude: Optional[float] = None

    current_waypoint: Optional[int] = None
    total_waypoints: int = 0

    current_task: Optional[str] = None

    healthy: bool = True
    failure_reason: Optional[str] = None

    completed_waypoints: int = 0


@dataclass
class MissionStatus:
    state: MissionState = MissionState.IDLE

    total_waypoints: int = 0
    completed_waypoints: int = 0

    failed_uavs: list[str] = field(default_factory=list)

    recovery_count: int = 0