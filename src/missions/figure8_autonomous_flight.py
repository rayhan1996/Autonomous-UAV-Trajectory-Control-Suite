"""
Autonomous Figure-8 Flight Mission (Fixed & Hardened)

This mission executes a fully autonomous figure-8 trajectory using
PX4 Position Offboard control, while continuously monitoring flight
safety and logging telemetry data.

Fixes applied (4 issues):
1) ✅ Unified time-base: shared t0 passed to BOTH fly_trajectory and safety_watchdog
2) ✅ Always stop offboard + land even on emergency/exception (guaranteed in finally)
3) ✅ Proper task cancellation: cancel + await with CancelledError suppression
4) ✅ Explicit yaw policy (still constant 0.0, but clean & extensible)

Also added (recommended):
- Optional spatial alignment step BEFORE starting t0 (like circle mission)
  to reduce initial drift spikes.
"""

import asyncio
import time
from contextlib import suppress

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
# Mission parameters
# ============================================================

ALTITUDE_M = 2.5
LOG_FILENAME = "figure8_autonomous_flight_log.csv"

# Trajectory config
RADIUS_M = 3.0
OMEGA = 0.3

# Offboard rate
OFFBOARD_RATE_HZ = 20.0

# Yaw policy (deg)
DEFAULT_YAW_DEG = 0.0

# Optional: spatial alignment before starting the reference clock
ENABLE_ALIGNMENT = True
ALIGN_SECONDS = 3.0
ALIGN_RATE_HZ = 20.0
ALIGN_SETTLE_SECONDS = 0.5


# ============================================================
# Helpers
# ============================================================

async def cancel_and_await(tasks):
    """Cancel tasks and await them to avoid warnings/unfinished coroutines."""
    for t in tasks:
        if t is None:
            continue
        t.cancel()
    for t in tasks:
        if t is None:
            continue
        with suppress(asyncio.CancelledError):
            await t


async def goto_xy_linear(
    drone,
    state: SharedState,
    x_target: float,
    y_target: float,
    altitude_m: float,
    duration_s: float,
    rate_hz: float,
    settle_s: float = 0.0,
):
    """
    Move linearly from current XY to target XY in given duration.
    Useful to align spatial phase before starting the reference trajectory clock.
    """
    dt = 1.0 / rate_hz

    # Wait until we have a valid position
    while state.pos_ned is None and state.running and not state.emergency_stop:
        await asyncio.sleep(0.05)

    if state.pos_ned is None:
        return

    x0, y0, _ = state.pos_ned
    steps = max(1, int(duration_s * rate_hz))

    for i in range(steps):
        if not state.running or state.emergency_stop:
            break

        alpha = (i + 1) / steps
        x_cmd = x0 + alpha * (x_target - x0)
        y_cmd = y0 + alpha * (y_target - y0)

        await drone.offboard.set_position_ned(
            PositionNedYaw(x_cmd, y_cmd, -altitude_m, DEFAULT_YAW_DEG)
        )
        await asyncio.sleep(dt)

    # Optional settle
    if settle_s > 0.0 and state.running and not state.emergency_stop:
        t_end = time.time() + settle_s
        while time.time() < t_end:
            await drone.offboard.set_position_ned(
                PositionNedYaw(x_target, y_target, -altitude_m, DEFAULT_YAW_DEG)
            )
            await asyncio.sleep(dt)


# ============================================================
# Trajectory execution (uses shared t0)
# ============================================================

async def fly_trajectory(
    drone,
    state: SharedState,
    trajectory: Figure8Trajectory,
    rate_hz: float,
    t0: float,  # ✅ shared mission start time reference
):
    dt = 1.0 / rate_hz
    print("▶ Starting autonomous Figure-8 trajectory...")

    while state.running and not state.emergency_stop:
        t = time.time() - t0  # ✅ shared time base with watchdog

        if t > trajectory.duration():
            break

        x, y = trajectory.position_xy(t)

        yaw = trajectory.yaw_deg(t)

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                x,
                y,
                -ALTITUDE_M,
                yaw
            )
        )

        await asyncio.sleep(dt)

    print("✔ Trajectory execution finished.")


# ============================================================
# Main mission entry point
# ============================================================

async def main():
    cfg = PX4Config(
        system_address="udpin://0.0.0.0:14540",
        takeoff_alt_m=ALTITUDE_M,
        offboard_rate_hz=OFFBOARD_RATE_HZ,
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
    await prestream_position_setpoints(drone, down_m=-ALTITUDE_M, n=20)

    print("Pre-streaming OFFBOARD setpoints...")

    for _ in range(20):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -ALTITUDE_M, 0.0)
        )
        await asyncio.sleep(0.05)
    
    print("Starting Offboard...")
    if not await start_offboard(drone):
        print("Offboard start failed, aborting mission.")
        return

    # --------------------------------------------------------
    # Shared state & telemetry/watchers EARLY
    # --------------------------------------------------------
    state = SharedState()
    state.running = True
    state.emergency_stop = False

    task_posvel = asyncio.create_task(watch_posvel(drone, state))
    task_att = asyncio.create_task(watch_attitude(drone, state))
    task_mode = asyncio.create_task(watch_flight_mode(drone, state))
    task_logger = asyncio.create_task(log_telemetry_csv(drone, state, LOG_FILENAME))

    background_tasks = [task_posvel, task_att, task_mode]

    # --------------------------------------------------------
    # Trajectory
    # --------------------------------------------------------
    trajectory = Figure8Trajectory(radius=RADIUS_M, omega=OMEGA)

    # --------------------------------------------------------
    # Optional: spatial alignment BEFORE t0
    # --------------------------------------------------------
    if ENABLE_ALIGNMENT:
        # Align to the trajectory start point at t=0
        # For your Figure8Trajectory, position_xy(0) is the start reference.
        x_start, y_start = trajectory.position_xy(0.0)

        print(
            f"[ALIGN] Going to Figure-8 start point: "
            f"x={x_start:.2f}, y={y_start:.2f} (duration={ALIGN_SECONDS:.1f}s)"
        )
        await goto_xy_linear(
            drone=drone,
            state=state,
            x_target=x_start,
            y_target=y_start,
            altitude_m=ALTITUDE_M,
            duration_s=ALIGN_SECONDS,
            rate_hz=ALIGN_RATE_HZ,
            settle_s=ALIGN_SETTLE_SECONDS,
        )

    # --------------------------------------------------------
    # ✅ Shared mission start time AFTER alignment
    # --------------------------------------------------------
    t0 = time.time()

    # --------------------------------------------------------
    # Safety watchdog (uses SAME t0) ✅
    # NOTE: Your watchdog already lands on emergency.
    # We still guarantee landing in finally for robustness.
    # --------------------------------------------------------
    task_safety = asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xy(t),
            nominal_duration_s=trajectory.duration(),
            t0=t0,  # ✅ shared reference
        )
    )

    background_tasks.append(task_safety)

    # --------------------------------------------------------
    # Execute mission
    # --------------------------------------------------------
    try:
        await fly_trajectory(
            drone=drone,
            state=state,
            trajectory=trajectory,
            rate_hz=cfg.offboard_rate_hz,
            t0=t0,
        )

    finally:
        # ----------------------------------------------------
        # Shutdown (always)
        # ----------------------------------------------------
        state.running = False

        # Let logger finish
        with suppress(Exception):
            await task_logger

        # Cancel watchers + safety cleanly
        await cancel_and_await(background_tasks)

        # ✅ Always stop offboard + land (even if emergency or exception)
        with suppress(Exception):
            await stop_offboard_and_land(drone)

        if getattr(state, "emergency_stop", False):
            reason = getattr(state, "emergency_reason", "UNKNOWN")
            print(f"🏁 Mission ended (EMERGENCY) → {reason}")
        else:
            print("🏁 Autonomous Figure-8 mission completed.")


if __name__ == "__main__":
    asyncio.run(main())
