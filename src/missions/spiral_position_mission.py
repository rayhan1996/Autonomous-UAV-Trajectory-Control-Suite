"""
Autonomous Spiral (Helix) Position Mission (Fixed)

Fixes applied:
1) Shared time base (t0) between mission and safety_watchdog
2) Nose aligned with path (yaw from trajectory velocity)
"""

import asyncio
import time
import math
from mavsdk.offboard import PositionNedYaw

from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import (
    prestream_position_setpoints,
    start_offboard,
    stop_offboard_and_land,
)
from src.core.safety_watchdog import safety_watchdog

from src.utils.shared_state import SharedState
from src.utils.telemetry_watchers import (
    watch_posvel,
    watch_attitude,
    watch_flight_mode,
)
from src.utils.telemetry_logger import log_telemetry_csv

from src.trajectories.spiral import SpiralTrajectory


LOG_FILE = "spiral_position_mission_log.csv"


# --------------------------------------------------
# Helper: yaw aligned with trajectory (finite diff)
# --------------------------------------------------

def yaw_from_spiral(trajectory, t: float, eps: float = 0.05) -> float:
    x1, y1, _ = trajectory.position_xyz(t)
    x2, y2, _ = trajectory.position_xyz(t + eps)

    vx = (x2 - x1) / eps
    vy = (y2 - y1) / eps

    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return 0.0

    return math.degrees(math.atan2(vy, vx))


# --------------------------------------------------
# Trajectory execution (uses shared t0)
# --------------------------------------------------

async def fly_spiral(drone, state, trajectory, rate_hz, t0):
    dt = 1.0 / rate_hz

    print("▶ Starting Spiral trajectory...")

    while state.running and not state.emergency_stop:
        t = time.time() - t0

        if t > trajectory.duration():
            break

        x, y, z = trajectory.position_xyz(t)
        yaw = yaw_from_spiral(trajectory, t)

        await drone.offboard.set_position_ned(
            PositionNedYaw(x, y, z, yaw)
        )

        await asyncio.sleep(dt)

    print("✔ Spiral trajectory finished.")


# --------------------------------------------------
# Main
# --------------------------------------------------

async def main():
    cfg = PX4Config(
        system_address="udpin://0.0.0.0:14540",
        offboard_rate_hz=20.0,
    )

    drone = await connect_px4(cfg.system_address)
    await wait_armable(drone)

    # ---- Arm & Takeoff ----
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # ---- Spiral trajectory ----
    trajectory = SpiralTrajectory(
        radius=3.0,
        start_z=0.0,
        end_z=-5.0,
        omega=0.3,
    )

    # ---- Offboard preparation ----
    await prestream_position_setpoints(drone, down_m=trajectory.z0)
    if not await start_offboard(drone):
        return

    # ---- Shared state & telemetry ----
    state = SharedState()
    asyncio.create_task(watch_posvel(drone, state))
    asyncio.create_task(watch_attitude(drone, state))
    asyncio.create_task(watch_flight_mode(drone, state))
    logger = asyncio.create_task(log_telemetry_csv(drone, state, LOG_FILE))

    # --------------------------------------------------
    # ✅ Shared mission start time
    # --------------------------------------------------
    t0 = time.time()

    # ---- Safety watchdog (uses SAME t0) ----
    asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xyz(t)[:2],
            nominal_duration_s=trajectory.duration(),
            t0=t0,
        )
    )

    # ---- Execute mission ----
    await fly_spiral(drone, state, trajectory, cfg.offboard_rate_hz, t0)

    # ---- Shutdown ----
    state.running = False
    await logger
    await stop_offboard_and_land(drone)

    print("🏁 Spiral mission completed.")


if __name__ == "__main__":
    asyncio.run(main())
