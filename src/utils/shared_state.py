from dataclasses import dataclass
from typing import Optional, Tuple, Any

@dataclass
class SharedState:
    # Latest telemetry snapshots
    pos_ned: Optional[Tuple[float, float, float]] = None   # (north, east, down)
    vel_ned: Optional[Tuple[float, float, float]] = None   # (vn, ve, vd)
    attitude_deg: Optional[Tuple[float, float, float]] = None  # (roll, pitch, yaw)
    flight_mode: Optional[Any] = None

    # Control flags
    running: bool = True
    emergency_stop: bool = False
    emergency_reason: str = ""
