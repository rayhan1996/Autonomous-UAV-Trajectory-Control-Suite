"""
Autonomous Circular Position Mission

Executes a constant-radius circular trajectory using PX4 Position Offboard
control with telemetry logging and safety monitoring.
"""

import asyncio
import time
from mavsdk.offboard import PositionNedYaw

from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import (
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

from src.trajectories.circle import CircleTrajectory


ALTITUDE_M = 2.5
LOG_FILE = "circle_position_mission_log.csv"


# --------------------------------------------------
# Trajectory execution
# --------------------------------------------------

async def fly_circle(drone, state, trajectory, rate_hz):
    dt = 1.0 / rate_hz
    t0 = time.time()

    while state.running and not state.emergency_stop:
        t = time.time() - t0
        if t > trajectory.duration():
            break

        x, y = trajectory.position_xy(t)

        await drone.offboard.set_position_ned(
            PositionNedYaw(x, y, -ALTITUDE_M, 0.0)
        )
        await asyncio.sleep(dt)


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
    await drone.action.set_takeoff_altitude(ALTITUDE_M)
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # --------------------------------------------------
    # CRITICAL FIX:
    # Pre-stream position setpoints BEFORE starting Offboard
    # --------------------------------------------------
    print("Pre-streaming position setpoints...")
    for _ in range(15):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -ALTITUDE_M, 0.0)
        )
        await asyncio.sleep(0.05)

    # ---- Start Offboard ----
    if not await start_offboard(drone):
        print("Offboard start failed, aborting mission.")
        return

    # ---- Shared state & telemetry ----
    state = SharedState()
    asyncio.create_task(watch_posvel(drone, state))
    asyncio.create_task(watch_attitude(drone, state))
    asyncio.create_task(watch_flight_mode(drone, state))
    logger_task = asyncio.create_task(
        log_telemetry_csv(drone, state, LOG_FILE)
    )

    # ---- Trajectory ----
    trajectory = CircleTrajectory(radius=3.0, omega=0.3)

    # ---- Safety watchdog ----
    asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xy(t),
            nominal_duration_s=trajectory.duration(),
        )
    )

    # ---- Execute mission ----
    await fly_circle(drone, state, trajectory, cfg.offboard_rate_hz)

    # ---- Shutdown ----
    state.running = False
    await logger_task
    await stop_offboard_and_land(drone)


if __name__ == "__main__":
    asyncio.run(main())
