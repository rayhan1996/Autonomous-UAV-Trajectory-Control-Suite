from dataclasses import dataclass

@dataclass
class PX4Config:
    system_address: str = "udp://:14540"   # SITL default
    takeoff_alt_m: float = 2.5
    offboard_rate_hz: float = 20.0         # 0.05s
