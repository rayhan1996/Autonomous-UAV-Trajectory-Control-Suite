"""
Autonomous Spiral (Helix) Position Mission (Rebuilt & Hardened)

What this version fixes:
1) ✅ Shared time base (t0) between mission + safety_watchdog
2) ✅ Spatial alignment BEFORE t0 (go to spiral start point) -> prevents initial ~R drift
3) ✅ Correct NED "down" usage for altitude (start_z/end_z consistent with takeoff altitude)
4) ✅ Robust shutdown: always stop offboard + land, cancel tasks cleanly
5) ✅ Optional yaw alignment with path (finite diff on trajectory)

Notes:
- NED: (north, east, down). To fly "ALTITUDE_M meters above home", use down = -ALTITUDE_M
- Spiral starts at (R, 0) at t=0, so we align to that point before starting reference clock.
"""

import asyncio
import time
import math
from contextlib import suppress

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


# ============================================================
# Mission parameters
# ============================================================

LOG_FILE = "spiral_position_mission_log.csv"

ALTITUDE_M = 2.5          # takeoff altitude (m) => down = -ALTITUDE_M
CLIMB_M = 5.0             # additional climb along spiral (m) => down becomes more negative

RADIUS_M = 3.0
OMEGA = 0.3               # rad/s
OFFBOARD_RATE_HZ = 20.0

# Alignment to spiral start point before t0
ENABLE_ALIGNMENT = True
ALIGN_SECONDS = 3.0
ALIGN_RATE_HZ = 20.0
ALIGN_SETTLE_SECONDS = 0.5

# Yaw behavior
ENABLE_YAW_FROM_PATH = True
DEFAULT_YAW_DEG = 0.0


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


def yaw_from_spiral(trajectory: SpiralTrajectory, t: float, eps: float = 0.05) -> float:
    """
    Compute yaw (deg) aligned with trajectory velocity (finite difference).
    """
    x1, y1, _ = trajectory.position_xyz(t)
    x2, y2, _ = trajectory.position_xyz(t + eps)

    vx = (x2 - x1) / eps
    vy = (y2 - y1) / eps

    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return DEFAULT_YAW_DEG

    return math.degrees(math.atan2(vy, vx))


async def goto_xyz_linear(
    drone,
    state: SharedState,
    x_target: float,
    y_target: float,
    z_target: float,
    duration_s: float,
    rate_hz: float,
    yaw_deg: float = 0.0,
    settle_s: float = 0.0,
):
    """
    Linearly move from current NED to target NED in duration_s.
    Useful for alignment BEFORE starting the reference trajectory clock.
    """
    dt = 1.0 / rate_hz

    # Wait for valid position
    while state.pos_ned is None and state.running and not state.emergency_stop:
        await asyncio.sleep(0.05)

    if state.pos_ned is None:
        return

    x0, y0, z0 = state.pos_ned
    steps = max(1, int(duration_s * rate_hz))

    for i in range(steps):
        if not state.running or state.emergency_stop:
            break

        a = (i + 1) / steps
        x = x0 + a * (x_target - x0)
        y = y0 + a * (y_target - y0)
        z = z0 + a * (z_target - z0)

        await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw_deg))
        await asyncio.sleep(dt)

    # Optional settle hold
    if settle_s > 0.0 and state.running and not state.emergency_stop:
        t_end = time.time() + settle_s
        while time.time() < t_end:
            await drone.offboard.set_position_ned(PositionNedYaw(x_target, y_target, z_target, yaw_deg))
            await asyncio.sleep(dt)


# ============================================================
# Trajectory execution (uses shared t0)
# ============================================================

async def fly_spiral(
    drone,
    state: SharedState,
    trajectory: SpiralTrajectory,
    rate_hz: float,
    t0: float,
):
    dt = 1.0 / rate_hz
    print("▶ Starting Spiral trajectory...")

    while state.running and not state.emergency_stop:
        t = time.time() - t0  # ✅ shared time base with watchdog

        if t > trajectory.duration():
            break

        x, y, z = trajectory.position_xyz(t)

        if ENABLE_YAW_FROM_PATH:
            yaw = yaw_from_spiral(trajectory, t)
        else:
            yaw = DEFAULT_YAW_DEG

        await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))
        await asyncio.sleep(dt)

    print("✔ Spiral trajectory finished.")


# ============================================================
# Main mission entry
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
    # Build trajectory (✅ correct NED down)
    # Start at -ALTITUDE_M, end at -(ALTITUDE_M + CLIMB_M)
    # --------------------------------------------------------
    trajectory = SpiralTrajectory(
        radius=RADIUS_M,
        start_z=-ALTITUDE_M,
        end_z=-(ALTITUDE_M + CLIMB_M),
        omega=OMEGA,
    )

    # --------------------------------------------------------
    # Offboard prep (prestream)
    # --------------------------------------------------------
    await prestream_position_setpoints(drone, down_m=trajectory.z0, n=20)

    # (Optional) Extra local prestream burst (helps some setups)
    for _ in range(20):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, trajectory.z0, 0.0))
        await asyncio.sleep(0.05)

    print("Starting Offboard...")
    if not await start_offboard(drone):
        print("Offboard start failed, aborting mission.")
        return

    # --------------------------------------------------------
    # Shared state + watchers + logger (start EARLY)
    # --------------------------------------------------------
    state = SharedState()
    state.running = True
    state.emergency_stop = False

    task_posvel = asyncio.create_task(watch_posvel(drone, state))
    task_att = asyncio.create_task(watch_attitude(drone, state))
    task_mode = asyncio.create_task(watch_flight_mode(drone, state))
    task_logger = asyncio.create_task(log_telemetry_csv(drone, state, LOG_FILE))

    background_tasks = [task_posvel, task_att, task_mode]

    # --------------------------------------------------------
    # ✅ Alignment BEFORE starting reference clock (prevents drift≈R)
    # Spiral start point is at t=0 -> (cx+R, cy+0, z0)
    # --------------------------------------------------------
    if ENABLE_ALIGNMENT:
        x_start, y_start, z_start = trajectory.position_xyz(0.0)

        print(
            f"[ALIGN] Going to Spiral start point: "
            f"x={x_start:.2f}, y={y_start:.2f}, z(down)={z_start:.2f} "
            f"(duration={ALIGN_SECONDS:.1f}s)"
        )

        await goto_xyz_linear(
            drone=drone,
            state=state,
            x_target=x_start,
            y_target=y_start,
            z_target=z_start,
            duration_s=ALIGN_SECONDS,
            rate_hz=ALIGN_RATE_HZ,
            yaw_deg=DEFAULT_YAW_DEG,
            settle_s=ALIGN_SETTLE_SECONDS,
        )

    # --------------------------------------------------------
    # ✅ Shared mission start time AFTER alignment
    # --------------------------------------------------------
    t0 = time.time()

    # --------------------------------------------------------
    # Safety watchdog (uses SAME t0) ✅
    # --------------------------------------------------------
    task_safety = asyncio.create_task(
        safety_watchdog(
            drone,
            state,
            reference_xy=lambda t: trajectory.position_xyz(t)[:2],
            nominal_duration_s=trajectory.duration(),
            t0=t0,
        )
    )
    background_tasks.append(task_safety)

    # --------------------------------------------------------
    # Execute mission + robust shutdown
    # --------------------------------------------------------
    try:
        await fly_spiral(
            drone=drone,
            state=state,
            trajectory=trajectory,
            rate_hz=cfg.offboard_rate_hz,
            t0=t0,
        )

    finally:
        # Stop loops
        state.running = False

        # Let logger finish cleanly
        with suppress(Exception):
            await task_logger

        # Cancel watchers + safety
        await cancel_and_await(background_tasks)

        # Always stop offboard + land
        with suppress(Exception):
            await stop_offboard_and_land(drone)

        if getattr(state, "emergency_stop", False):
            reason = getattr(state, "emergency_reason", "UNKNOWN")
            print(f"🏁 Spiral mission ended (EMERGENCY) → {reason}")
        else:
            print("🏁 Spiral mission completed.")


if __name__ == "__main__":
    asyncio.run(main())
