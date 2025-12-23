"""
Autonomous Spiral (Helix) Position Mission

Executes a 3D spiral trajectory using PX4 Position Offboard control.
Demonstrates altitude-aware path tracking.
"""

import asyncio
import time
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


async def fly_spiral(drone, state, trajectory, rate_hz):
    dt = 1.0 / rate_hz
    t0 = time.time()

    while state.running and not state.emergency_stop:
        t = time.time() - t0
        if t > trajectory.duration():
            break

        x, y, z = trajectory.position_xyz(t)

        await drone.offboard.set_position_ned(
            PositionNedYaw(x, y, z, 0.0)
        )
        await asyncio.sleep(dt)


async def main():
    cfg = PX4Config(system_address="udpin://0.0.0.0:14540", offboard_rate_hz=20)

    drone = await connect_px4(cfg.system_address)
    await wait_armable(drone)

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # Spiral: climb while circling
    trajectory = SpiralTrajectory(
        radius=3.0,
        start_z=0.0,
        end_z=-5.0,
        omega=0.3,
    )

    await prestream_position_setpoints(drone, down_m=trajectory.z0)
    if not await start_offboard(drone):
        return

    state = SharedState()
    asyncio.create_task(watch_posvel(drone, state))
    asyncio.create_task(watch_attitude(drone, state))
    asyncio.create_task(watch_flight_mode(drone, state))
    logger = asyncio.create_task(log_telemetry_csv(drone, state, LOG_FILE))

    asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xyz(t)[:2],
            nominal_duration_s=trajectory.duration(),
        )
    )

    await fly_spiral(drone, state, trajectory, cfg.offboard_rate_hz)

    state.running = False
    await logger
    await stop_offboard_and_land(drone)


if __name__ == "__main__":
    asyncio.run(main())
