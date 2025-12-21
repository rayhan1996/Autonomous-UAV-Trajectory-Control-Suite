import asyncio
import math
import time
from typing import Callable, Tuple
from mavsdk import System
from .offboard_helpers import stop_offboard_and_land
from ..utils.shared_state import SharedState

def norm2(x: float, y: float) -> float:
    return math.sqrt(x*x + y*y)

async def safety_watchdog(
    drone: System,
    state: SharedState,
    reference_xy: Callable[[float], Tuple[float, float]],
    *,
    drift_max_m: float = 1.8,
    speed_max_m_s: float = 3.5,
    roll_pitch_max_deg: float = 30.0,
    timeout_factor: float = 1.5,
    nominal_duration_s: float = 30.0,
):
    start = time.time()

    while state.running and not state.emergency_stop:
        await asyncio.sleep(0.1)

        if state.pos_ned is None or state.vel_ned is None or state.attitude_deg is None:
            continue

        t = time.time() - start

        # Timeout
        if t > nominal_duration_s * timeout_factor:
            state.emergency_stop = True
            state.emergency_reason = "MISSION TIMEOUT"
            break

        # Drift check (XY)
        x_ref, y_ref = reference_xy(t)
        x, y, _ = state.pos_ned
        drift = norm2(x - x_ref, y - y_ref)
        if drift > drift_max_m:
            state.emergency_stop = True
            state.emergency_reason = f"DRIFT TOO HIGH: {drift:.2f} m"
            break

        # Speed check
        vx, vy, vz = state.vel_ned
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        if speed > speed_max_m_s:
            state.emergency_stop = True
            state.emergency_reason = f"SPEED TOO HIGH: {speed:.2f} m/s"
            break

        # Attitude check
        roll, pitch, _ = state.attitude_deg
        if abs(roll) > roll_pitch_max_deg or abs(pitch) > roll_pitch_max_deg:
            state.emergency_stop = True
            state.emergency_reason = f"UNSAFE ATTITUDE: roll={roll:.1f}, pitch={pitch:.1f}"
            break

    if state.emergency_stop:
        print(f"⚠️ SAFETY TRIGGERED → {state.emergency_reason}")
        state.running = False
        await stop_offboard_and_land(drone)

    print("Safety watchdog stopped.")
