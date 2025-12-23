"""
Autonomous Circular Flight Mission

Executes a fully autonomous circular trajectory using PX4 Position Offboard
control with continuous safety monitoring and telemetry logging.

Features:
- Position Offboard control
- Reusable trajectory planning (CircleTrajectory)
- Safety watchdog (drift, speed, attitude, timeout)
- Telemetry logging to CSV
- Automatic landing on completion or emergency
"""

import asyncio
import time

from mavsdk.offboard import PositionNedYaw

# ---- Core infrastructure ----
from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import (
    prestream_position_setpoints,
    start_offboard,
    stop_offboard_and_land,
)
from src.core.safety_watchdog import safety_watchdog

# ---- Telemetry & state ----
from src.utils.shared_state import SharedState
from src.utils.telemetry_watchers import (
    watch_posvel,
    watch_attitude,
    watch_flight_mode,
)
from src.utils.telemetry_logger import log_telemetry_csv

# ---- Trajectory ----
from src.trajectories.circle import CircleTrajectory


# ============================================================
# Mission parameters
# ============================================================

ALTITUDE_M = 2.5
LOG_FILENAME = "circle_autonomous_flight_log.csv"


# ============================================================
# Trajectory execution
# ============================================================

async def fly_trajectory(
    drone,
    state: SharedState,
    trajectory: CircleTrajectory,
    rate_hz: float,
):
    """
    Execute the circular reference trajectory until completion
    or until a safety violation occurs.
    """
    dt = 1.0 / rate_hz
    start_time = time.time()

    print("▶ Starting autonomous circular trajectory...")

    while state.running and not state.emergency_stop:
        t = time.time() - start_time

        if t > trajectory.duration():
            break

        x, y = trajectory.position_xy(t)

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                x,               # north
                y,               # east
                -ALTITUDE_M,     # down (NED)
                0.0              # yaw
            )
        )

        await asyncio.sleep(dt)

    print("✔ Circular trajectory execution finished.")


# ============================================================
# Main mission entry point
# ============================================================

async def main():
    cfg = PX4Config(
        system_address="udpin://0.0.0.0:14540",
        takeoff_alt_m=ALTITUDE_M,
        offboard_rate_hz=20.0,
    )

    # --------------------------------------------------------
    # Connect to PX4
    # --------------------------------------------------------
    drone = await connect_px4(cfg.system_address)
    await wait_armable(drone)

    # --------------------------------------------------------
    # Arm & takeoff
    # --------------------------------------------------------
    await drone.action.set_takeoff_altitude(cfg.takeoff_alt_m)
    await drone.action.arm()
    print("✔ Armed")

    await drone.action.takeoff()
    print("▲ Taking off...")
    await asyncio.sleep(5)

    # --------------------------------------------------------
    # Prepare Offboard mode
    # --------------------------------------------------------
    await prestream_position_setpoints(
        drone,
        down_m=-ALTITUDE_M,
        n=20,
    )

    if not await start_offboard(drone):
        return

    # --------------------------------------------------------
    # Shared state & background tasks
    # --------------------------------------------------------
    state = SharedState()

    task_posvel = asyncio.create_task(watch_posvel(drone, state))
    task_att = asyncio.create_task(watch_attitude(drone, state))
    task_mode = asyncio.create_task(watch_flight_mode(drone, state))
    task_logger = asyncio.create_task(
        log_telemetry_csv(drone, state, LOG_FILENAME)
    )

    # --------------------------------------------------------
    # Trajectory & safety watchdog
    # --------------------------------------------------------
    trajectory = CircleTrajectory(
        radius=3.0,
        omega=0.3,
    )

    task_safety = asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xy(t),
            nominal_duration_s=trajectory.duration(),
        )
    )

    # --------------------------------------------------------
    # Execute flight
    # --------------------------------------------------------
    await fly_trajectory(
        drone,
        state,
        trajectory,
        cfg.offboard_rate_hz,
    )

    # --------------------------------------------------------
    # Shutdown sequence
    # --------------------------------------------------------
    state.running = False
    await task_logger

    for task in (task_posvel, task_att, task_mode, task_safety):
        task.cancel()

    if not state.emergency_stop:
        await stop_offboard_and_land(drone)

    print("🏁 Autonomous circular mission completed.")


# ============================================================
# Script entry
# ============================================================

if __name__ == "__main__":
    asyncio.run(main())
