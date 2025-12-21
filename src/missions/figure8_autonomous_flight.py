"""
Autonomous Figure-8 Flight Mission

This mission executes a fully autonomous figure-8 trajectory using
PX4 Position Offboard control, while continuously monitoring flight
safety and logging telemetry data.

Responsibilities:
- PX4 connection & arm/takeoff
- Start/stop Offboard mode
- Execute Figure-8 trajectory (via trajectory library)
- Run safety watchdog (drift, speed, attitude, timeout)
- Log telemetry to CSV
- Safe landing on completion or emergency
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
from src.trajectories.figure8 import Figure8Trajectory


# ============================================================
# Mission parameters (can later be moved to config / CLI)
# ============================================================

ALTITUDE_M = 2.5           # Flight altitude (positive up)
LOG_FILENAME = "figure8_autonomous_flight_log.csv"


# ============================================================
# Trajectory execution
# ============================================================

async def fly_trajectory(
    drone,
    state: SharedState,
    trajectory: Figure8Trajectory,
    rate_hz: float,
):
    """
    Execute the reference trajectory until completion
    or until safety watchdog triggers an emergency.
    """
    dt = 1.0 / rate_hz
    start_time = time.time()

    print("▶ Starting autonomous Figure-8 trajectory...")

    while state.running and not state.emergency_stop:
        t = time.time() - start_time

        # Stop normally when trajectory duration is complete
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

    print("✔ Trajectory execution finished.")


# ============================================================
# Main mission entry point
# ============================================================

async def main():
    # --------------------------------------------------------
    # Configuration
    # --------------------------------------------------------
    cfg = PX4Config(
        system_address="udp://:14540",   # PX4 SITL default
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
    trajectory = Figure8Trajectory(
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

    print("🏁 Autonomous Figure-8 mission completed.")


# ============================================================
# Script entry
# ============================================================

if __name__ == "__main__":
    asyncio.run(main())
