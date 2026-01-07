"""
Keyboard "Velocity-Feel" Control Mission (FINAL - STABLE + LOGGED + DEBUG)

Key idea (scientific + practical):
- Use OFFBOARD POSITION setpoints for stability (PX4 loves this)
- Simulate velocity control by integrating commanded vx, vy over time:
      x_target += vx_cmd * dt
      y_target += vy_cmd * dt
- Control altitude with a real altitude target (W/S), converted to NED down:
      down_target = -alt_target

Logging:
- CSV telemetry -> logs/csv/keyboard_velocity_control.csv
- Event log (keys/warnings/stages) -> logs/telemetry/keyboard_velocity_events.txt

Controls:
- Arrows: move N/E/S/W (velocity-feel)
- W/S: up/down (altitude target)
- A/D: yaw left/right (yaw rate -> integrated yaw target)
- SPACE: stop (vx=vy=0)
- Q: quit & land
"""

import asyncio
import curses
import time
import math
from pathlib import Path
from contextlib import suppress

from mavsdk.offboard import PositionNedYaw

from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import start_offboard

from src.utils.shared_state import SharedState
from src.utils.telemetry_watchers import (
    watch_posvel,
    watch_attitude,
    watch_flight_mode,
)
from src.utils.telemetry_logger import log_telemetry_csv


# ============================================================
# Parameters
# ============================================================

# "Velocity feel" for XY (m/s)
SPEED_M_S = 1.0

# Altitude step (m per key-press tick)
ALT_STEP_M = 0.15

# Yaw rate feel (rad/s)
YAW_RATE_RAD_S = 0.5

# Safety altitude limits (m, positive up)
ALT_MIN_M = 1.0
ALT_MAX_M = 4.0

# Control frequency
CONTROL_DT_S = 0.10  # 10 Hz

# Logs
CSV_NAME = "keyboard_velocity_control.csv"
EVENTS_NAME = "keyboard_velocity_events.txt"


# ============================================================
# Paths / logging helpers
# ============================================================

def repo_root() -> Path:
    # src/missions/keyboard_velocity_control.py -> repo root = parents[2]
    return Path(__file__).resolve().parents[2]


def ensure_log_dirs():
    root = repo_root()
    (root / "logs" / "csv").mkdir(parents=True, exist_ok=True)
    (root / "logs" / "telemetry").mkdir(parents=True, exist_ok=True)
    (root / "logs" / "plots").mkdir(parents=True, exist_ok=True)


def event_log(line: str):
    path = repo_root() / "logs" / "telemetry" / EVENTS_NAME
    ts = time.time()
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"{ts:.6f} {line}\n")


# ============================================================
# Keyboard reader (SYNC, curses) -> pushes keys into queue
# ============================================================

def keyboard_reader(stdscr, command_queue: "asyncio.Queue[int]", state: SharedState):
    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0, "Keyboard Control ✅ ACTIVE (Position Setpoints)")
    stdscr.addstr(2, 0, "↑ ↓ ← → : Move (velocity-feel in XY)")
    stdscr.addstr(3, 0, "W / S   : Up / Down (altitude target)")
    stdscr.addstr(4, 0, "A / D   : Yaw Left / Right (yaw rate-feel)")
    stdscr.addstr(5, 0, "SPACE   : Stop XY")
    stdscr.addstr(6, 0, "Q       : Quit & Land")
    stdscr.addstr(8, 0, f"ALT safety: {ALT_MIN_M:.1f}m .. {ALT_MAX_M:.1f}m")
    stdscr.addstr(10, 0, ">>> KEYBOARD MODE RUNNING <<<")
    stdscr.refresh()

    event_log("CURSES_STARTED keyboard UI shown")

    while state.running:
        key = stdscr.getch()
        if key != -1:
            try:
                command_queue.put_nowait(key)
            except Exception:
                pass
        time.sleep(0.05)


def run_curses(command_queue, state):
    curses.wrapper(lambda stdscr: keyboard_reader(stdscr, command_queue, state))


# ============================================================
# Key decoding + helpers
# ============================================================

def decode_key(key: int) -> str:
    if key == curses.KEY_UP:
        return "UP_ARROW"
    if key == curses.KEY_DOWN:
        return "DOWN_ARROW"
    if key == curses.KEY_RIGHT:
        return "RIGHT_ARROW"
    if key == curses.KEY_LEFT:
        return "LEFT_ARROW"
    if key == ord("w"):
        return "W"
    if key == ord("s"):
        return "S"
    if key == ord("a"):
        return "A"
    if key == ord("d"):
        return "D"
    if key == ord("q"):
        return "Q"
    if key == ord(" "):
        return "SPACE"
    return f"KEY_{key}"


def wrap_yaw_deg(yaw_deg: float) -> float:
    # keep yaw in [-180, 180)
    y = (yaw_deg + 180.0) % 360.0 - 180.0
    return y


# ============================================================
# Async control loop (ONLY MAVSDK HERE)
# ============================================================

async def position_control_loop(drone, state: SharedState, command_queue: "asyncio.Queue[int]"):
    print("⌨️ ENTERING KEYBOARD CONTROL MODE (focus this terminal)")
    event_log("CONTROL_LOOP_STARTED")

    # Wait for telemetry position to exist (we need a start reference)
    print("⌛ Waiting for first telemetry position...")
    while state.running and state.pos_ned is None:
        await asyncio.sleep(0.05)

    if state.pos_ned is None:
        print("❌ No telemetry position received. Abort.")
        event_log("ABORT_NO_TELEMETRY")
        state.running = False
        return

    # Wait for first key press (prevents instant exit confusion)
    print("⌛ Waiting for first key press...")
    while state.running and command_queue.empty():
        await asyncio.sleep(0.1)

    if not state.running:
        return

    print("✅ First key received. Control is LIVE.")
    event_log("FIRST_KEY_RECEIVED")

    # Initialize targets from current position
    x_target, y_target, down_target = state.pos_ned

    # Altitude target (positive up)
    alt_target = -down_target

    # Yaw target: use telemetry yaw if available, else 0
    yaw_target = 0.0
    if state.attitude_deg is not None:
        yaw_target = state.attitude_deg[2]

    # Commanded "velocity feel" for XY (hold last command until new key)
    vx_cmd = 0.0
    vy_cmd = 0.0
    yaw_rate_cmd_deg_s = 0.0  # we integrate yaw target

    dt = CONTROL_DT_S

    while state.running and not state.emergency_stop:
        # default: keep previous commands (so holding direction feel is possible)
        # but if you prefer "pulse" behavior, set vx_cmd=vy_cmd=yaw_rate_cmd_deg_s=0 each tick.

        # Read key if any
        try:
            key = command_queue.get_nowait()
            event_log(f"KEY {decode_key(key)}")
        except asyncio.QueueEmpty:
            key = None

        # Map key -> command updates
        if key == curses.KEY_UP:
            vx_cmd = SPEED_M_S
            vy_cmd = 0.0
        elif key == curses.KEY_DOWN:
            vx_cmd = -SPEED_M_S
            vy_cmd = 0.0
        elif key == curses.KEY_RIGHT:
            vy_cmd = SPEED_M_S
            vx_cmd = 0.0
        elif key == curses.KEY_LEFT:
            vy_cmd = -SPEED_M_S
            vx_cmd = 0.0

        elif key == ord("w"):
            alt_target += ALT_STEP_M
        elif key == ord("s"):
            alt_target -= ALT_STEP_M

        elif key == ord("a"):
            yaw_rate_cmd_deg_s = -math.degrees(YAW_RATE_RAD_S)
        elif key == ord("d"):
            yaw_rate_cmd_deg_s = math.degrees(YAW_RATE_RAD_S)

        elif key == ord(" "):
            vx_cmd = 0.0
            vy_cmd = 0.0
            yaw_rate_cmd_deg_s = 0.0

        elif key == ord("q"):
            print("🛑 Q pressed -> exiting keyboard mode")
            event_log("USER_QUIT")
            state.running = False
            break

        # Safety altitude clamp
        if alt_target < ALT_MIN_M or alt_target > ALT_MAX_M:
            msg = f"ALT_LIMIT_EXCEEDED alt_target={alt_target:.2f}m (limits {ALT_MIN_M}-{ALT_MAX_M})"
            print(f"⚠ {msg}")
            event_log(msg)
            state.emergency_stop = True
            state.emergency_reason = msg
            state.running = False
            break

        # Integrate "velocity feel" -> position target
        x_target += vx_cmd * dt
        y_target += vy_cmd * dt

        # Integrate yaw rate -> yaw target
        yaw_target = wrap_yaw_deg(yaw_target + yaw_rate_cmd_deg_s * dt)

        # Convert altitude target -> NED down target
        down_target = -alt_target

        # Send OFFBOARD POSITION setpoint
        await drone.offboard.set_position_ned(
            PositionNedYaw(x_target, y_target, down_target, yaw_target)
        )

        await asyncio.sleep(dt)

    event_log("CONTROL_LOOP_ENDED")
    print("⌨️ EXITED KEYBOARD CONTROL MODE")


# ============================================================
# Main
# ============================================================

async def main():
    ensure_log_dirs()

    cfg = PX4Config(system_address="udpin://0.0.0.0:14540")

    print("🔌 Connecting to PX4...")
    drone = await connect_px4(cfg.system_address)

    print("🩺 Waiting for armable...")
    await wait_armable(drone)

    print("🟢 ARM...")
    await drone.action.arm()
    event_log("ARMED")

    print("🛫 TAKEOFF...")
    await drone.action.takeoff()
    event_log("TAKEOFF")
    await asyncio.sleep(5)

    # Shared state
    state = SharedState()
    state.running = True
    state.emergency_stop = False

    # Telemetry
    print("📡 Starting telemetry watchers + CSV logger...")
    t_pos = asyncio.create_task(watch_posvel(drone, state))
    t_att = asyncio.create_task(watch_attitude(drone, state))
    t_mode = asyncio.create_task(watch_flight_mode(drone, state))
    t_log = asyncio.create_task(log_telemetry_csv(drone, state, f"csv/{CSV_NAME}"))

    # Before starting offboard, send a few position setpoints near current position (if available)
    print("🧷 Pre-offboard: waiting pos, then streaming a few position setpoints...")
    while state.pos_ned is None:
        await asyncio.sleep(0.05)

    x0, y0, d0 = state.pos_ned
    yaw0 = 0.0
    if state.attitude_deg is not None:
        yaw0 = state.attitude_deg[2]

    for _ in range(20):
        await drone.offboard.set_position_ned(PositionNedYaw(x0, y0, d0, yaw0))
        await asyncio.sleep(0.05)

    print("▶ Starting OFFBOARD...")
    if not await start_offboard(drone):
        print("❌ Offboard start failed")
        event_log("OFFBOARD_START_FAILED")
        state.running = False
        return

    print("✅ OFFBOARD STARTED")
    event_log("OFFBOARD_STARTED")

    command_queue = asyncio.Queue()

    # Start curses in background thread (NO MAVSDK there)
    loop = asyncio.get_running_loop()
    curses_task = loop.run_in_executor(None, run_curses, command_queue, state)

    try:
        await position_control_loop(drone, state, command_queue)

    finally:
        # Stop everything cleanly
        state.running = False

        with suppress(Exception):
            await t_log

        for t in (t_pos, t_att, t_mode):
            t.cancel()
        for t in (t_pos, t_att, t_mode):
            with suppress(asyncio.CancelledError):
                await t

        with suppress(Exception):
            await curses_task

        print("🛑 Stopping OFFBOARD...")
        with suppress(Exception):
            await drone.offboard.stop()
        await asyncio.sleep(0.5)

        print("🛬 Landing...")
        # Land with a retry guard (timeouts can happen in SITL)
        landed = False
        for attempt in range(3):
            try:
                await drone.action.land()
                landed = True
                break
            except Exception as e:
                event_log(f"LAND_RETRY_{attempt+1} {repr(e)}")
                await asyncio.sleep(1.0)

        if not landed:
            print("⚠ Landing timeout -> trying disarm as fallback")
            event_log("LAND_FAILED_TRY_DISARM")
            with suppress(Exception):
                await drone.action.disarm()

        if state.emergency_stop:
            print(f"🏁 Mission ended (EMERGENCY) → {state.emergency_reason}")
            event_log(f"END_EMERGENCY {state.emergency_reason}")
        else:
            print("🏁 Mission completed")
            event_log("END_OK")


if __name__ == "__main__":
    asyncio.run(main())
