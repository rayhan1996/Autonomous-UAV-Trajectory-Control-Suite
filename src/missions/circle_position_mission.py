"""
Autonomous Circular Position Mission

Executes a constant-radius circular trajectory using PX4 Position Offboard
control with telemetry logging and safety monitoring.

Fix:
- Align spatial phase BEFORE starting the circular reference.
- Move UAV from current position (center) to the circle start point (cx + R, cy)
  so initial drift is ~0 (no immediate DRIFT=R trigger).
- Use a shared time reference t0 for BOTH mission and watchdog.
"""

import asyncio
import time
from mavsdk.offboard import PositionNedYaw

from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import start_offboard, stop_offboard_and_land
from src.core.safety_watchdog import safety_watchdog

from src.utils.shared_state import SharedState
from src.utils.telemetry_watchers import watch_posvel, watch_attitude, watch_flight_mode
from src.utils.telemetry_logger import log_telemetry_csv

from src.trajectories.circle import CircleTrajectory


# --------------------------------------------------
# Configuration
# --------------------------------------------------

ALTITUDE_M = 2.5
LOG_FILE = "circle_position_mission_log.csv"

RADIUS_M = 3.0
OMEGA = 0.3  # rad/s (slow and safe)

# Phase/spatial alignment: move to start point before circle begins
GOTO_START_SECONDS = 3.0      # time to move to start point
GOTO_RATE_HZ = 20.0           # command rate for the goto segment
START_SETTLE_SECONDS = 0.5    # small settle time at start point


# --------------------------------------------------
# Helpers
# --------------------------------------------------

async def prestream_position(drone, altitude_m: float, n: int = 15, dt: float = 0.05):
    """PX4 requires setpoints to be streamed before starting offboard."""
    for _ in range(n):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -altitude_m, 0.0))
        await asyncio.sleep(dt)


async def goto_xy_linear(drone, state: SharedState, x_target: float, y_target: float, altitude_m: float,
                         duration_s: float, rate_hz: float):
    """
    Move linearly from current XY to target XY in given duration.
    This aligns the spatial phase so the circle reference starts where the UAV already is.
    """
    dt = 1.0 / rate_hz

    # Wait until we have a valid position from watcher (should be fast)
    while state.pos_ned is None and state.running and not state.emergency_stop:
        await asyncio.sleep(0.05)

    if state.pos_ned is None:
        return  # fail silently; watchdog will handle if needed

    x0, y0, _ = state.pos_ned

    steps = max(1, int(duration_s * rate_hz))
    for i in range(steps):
        if not state.running or state.emergency_stop:
            break

        alpha = (i + 1) / steps  # 0->1
        x_cmd = x0 + alpha * (x_target - x0)
        y_cmd = y0 + alpha * (y_target - y0)

        await drone.offboard.set_position_ned(PositionNedYaw(x_cmd, y_cmd, -altitude_m, 0.0))
        await asyncio.sleep(dt)

    # brief settle at the start point
    if state.running and not state.emergency_stop:
        t_end = time.time() + START_SETTLE_SECONDS
        while time.time() < t_end:
            await drone.offboard.set_position_ned(PositionNedYaw(x_target, y_target, -altitude_m, 0.0))
            await asyncio.sleep(dt)


# --------------------------------------------------
# Trajectory execution (uses shared t0)
# --------------------------------------------------

async def fly_circle(drone, state, trajectory, rate_hz, t0):
    dt = 1.0 / rate_hz

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
    # Start shared state & telemetry watchers EARLY
    # so goto phase has access to current pos_ned
    # --------------------------------------------------
    state = SharedState()
    asyncio.create_task(watch_posvel(drone, state))
    asyncio.create_task(watch_attitude(drone, state))
    asyncio.create_task(watch_flight_mode(drone, state))

    # --------------------------------------------------
    # Pre-stream position setpoints (PX4 requirement)
    # --------------------------------------------------
    print("Pre-streaming position setpoints...")
    await prestream_position(drone, ALTITUDE_M, n=15, dt=0.05)

    # --------------------------------------------------
    # Capture current local position as trajectory center
    # --------------------------------------------------
    async for pv in drone.telemetry.position_velocity_ned():
        cx = pv.position.north_m
        cy = pv.position.east_m
        break

    print(f"[ORIGIN] cx={cx:.2f}, cy={cy:.2f}")

    # --------------------------------------------------
    # Create trajectory (phase=0 means start at (cx+R, cy))
    # --------------------------------------------------
    trajectory = CircleTrajectory(
        radius=RADIUS_M,
        omega=OMEGA,
        center_x=cx,
        center_y=cy,
    )

    v_ref = RADIUS_M * OMEGA
    print(f"[CFG] R={RADIUS_M}, omega={OMEGA}, v_ref={v_ref:.3f} m/s")

    # --------------------------------------------------
    # Start Offboard
    # --------------------------------------------------
    if not await start_offboard(drone):
        print("Offboard start failed, aborting mission.")
        return

    # --------------------------------------------------
    # Telemetry logger
    # --------------------------------------------------
    logger_task = asyncio.create_task(log_telemetry_csv(drone, state, LOG_FILE))
    print(f"Telemetry logger started → {LOG_FILE}")

    # --------------------------------------------------
    # ✅ Spatial phase alignment BEFORE t0
    # Move UAV to the first point of the circle to avoid drift=R at t=0
    # Circle at t=0 -> (cx+R, cy)
    # --------------------------------------------------
    x_start = cx + RADIUS_M
    y_start = cy
    print(f"[ALIGN] Going to circle start point: x={x_start:.2f}, y={y_start:.2f} (duration={GOTO_START_SECONDS:.1f}s)")
    await goto_xy_linear(
        drone=drone,
        state=state,
        x_target=x_start,
        y_target=y_start,
        altitude_m=ALTITUDE_M,
        duration_s=GOTO_START_SECONDS,
        rate_hz=GOTO_RATE_HZ,
    )

    # --------------------------------------------------
    # ✅ Shared mission start time AFTER alignment
    # --------------------------------------------------
    t0 = time.time()

    # --------------------------------------------------
    # Safety watchdog (uses SAME t0)
    # --------------------------------------------------
    asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xy(t),
            nominal_duration_s=trajectory.duration(),
            t0=t0,
        )
    )

    # --------------------------------------------------
    # Execute mission
    # --------------------------------------------------
    await fly_circle(drone, state, trajectory, cfg.offboard_rate_hz, t0)

    # --------------------------------------------------
    # Shutdown
    # --------------------------------------------------
    state.running = False
    await logger_task
    await stop_offboard_and_land(drone)


if __name__ == "__main__":
    asyncio.run(main())
