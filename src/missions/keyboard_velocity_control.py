"""
Keyboard Velocity Control Mission (FINAL - STABLE + LOGGED + DEBUG)

- Human-in-the-loop velocity control (NED velocities)
- Safe asyncio + curses integration (NO MAVSDK calls outside main event loop)
- CSV telemetry logging -> logs/csv/
- Event logging (key presses, warnings) -> logs/telemetry/
- Clear prints to know exactly which stage is running
- Waits for first key press (prevents instant exit)
"""

import asyncio
import curses
import time
from pathlib import Path
from contextlib import suppress

from mavsdk.offboard import VelocityNedYaw

from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import start_offboard, stop_offboard_and_land

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

SPEED_M_S = 1.0
YAW_RATE_RAD_S = 0.5

ALT_MIN_M = 1.0
ALT_MAX_M = 4.0

CSV_NAME = "keyboard_velocity_control.csv"
EVENTS_NAME = "keyboard_velocity_events.txt"

CONTROL_DT_S = 0.10  # 10 Hz


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

    stdscr.addstr(0, 0, "Keyboard Velocity Control  ✅ ACTIVE")
    stdscr.addstr(2, 0, "↑ ↓ ← → : Move (N/E/S/W)   [NED frame]")
    stdscr.addstr(3, 0, "W / S   : Up / Down")
    stdscr.addstr(4, 0, "A / D   : Yaw Left / Right (rate)")
    stdscr.addstr(5, 0, "SPACE   : Stop")
    stdscr.addstr(6, 0, "Q       : Quit & Land")
    stdscr.addstr(8, 0, f"Altitude safety: {ALT_MIN_M:.1f}m .. {ALT_MAX_M:.1f}m")
    stdscr.addstr(10, 0, ">>> KEYBOARD CONTROL MODE RUNNING <<<")
    stdscr.refresh()

    # A visible indicator that curses started
    event_log("CURSES_STARTED keyboard UI shown")

    while state.running:
        key = stdscr.getch()
        if key != -1:
            # push key into queue (non-blocking)
            try:
                command_queue.put_nowait(key)
            except Exception:
                pass
        time.sleep(0.05)


def run_curses(command_queue, state):
    curses.wrapper(lambda stdscr: keyboard_reader(stdscr, command_queue, state))


# ============================================================
# Async velocity control loop (ONLY MAVSDK HERE)
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


async def velocity_control_loop(drone, state: SharedState, command_queue: "asyncio.Queue[int]"):
    print("⌨️  ENTERING KEYBOARD CONTROL MODE (focus this terminal)")
    event_log("CONTROL_LOOP_STARTED")

    # Wait until at least one key is received (prevents instant exit confusion)
    print("⌛ Waiting for first key press...")
    while state.running:
        if not command_queue.empty():
            break
        await asyncio.sleep(0.1)

    print("✅ First key received. Control is live now!")
    event_log("FIRST_KEY_RECEIVED")

    last_key = None

    while state.running and not state.emergency_stop:
        north = east = down = yaw_rate = 0.0

        # Read key if any
        try:
            key = command_queue.get_nowait()
            last_key = key
            event_log(f"KEY {decode_key(key)}")
        except asyncio.QueueEmpty:
            key = None

        # Map key -> velocity command
        if key == curses.KEY_UP:
            north = SPEED_M_S
        elif key == curses.KEY_DOWN:
            north = -SPEED_M_S
        elif key == curses.KEY_RIGHT:
            east = SPEED_M_S
        elif key == curses.KEY_LEFT:
            east = -SPEED_M_S
        elif key == ord("w"):
            down = -SPEED_M_S
        elif key == ord("s"):
            down = SPEED_M_S
        elif key == ord("a"):
            yaw_rate = -YAW_RATE_RAD_S
        elif key == ord("d"):
            yaw_rate = YAW_RATE_RAD_S
        elif key == ord(" "):
            # stop
            pass
        elif key == ord("q"):
            print("🛑 Q pressed -> exiting keyboard mode")
            event_log("USER_QUIT")
            state.running = False
            break

        # Altitude safety (if telemetry available)
        if state.pos_ned is not None:
            _, _, down_pos = state.pos_ned
            alt = -down_pos
            if alt < ALT_MIN_M or alt > ALT_MAX_M:
                msg = f"ALT_LIMIT_EXCEEDED alt={alt:.2f}m (limits {ALT_MIN_M}-{ALT_MAX_M})"
                print(f"⚠ {msg}")
                event_log(msg)

                # zero velocity before exiting
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
                await asyncio.sleep(0.2)

                state.emergency_stop = True
                state.emergency_reason = msg
                state.running = False
                break

        # Send MAVSDK command (main loop only)
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, yaw_rate)
        )

        await asyncio.sleep(CONTROL_DT_S)

    event_log("CONTROL_LOOP_ENDED")
    print("⌨️  EXITED KEYBOARD CONTROL MODE")


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

    print("🧷 Pre-offboard: send zero velocity setpoint")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))

    print("▶ Starting OFFBOARD...")
    if not await start_offboard(drone):
        print("❌ Offboard start failed")
        event_log("OFFBOARD_START_FAILED")
        return

    print("✅ OFFBOARD STARTED")
    event_log("OFFBOARD_STARTED")

    state = SharedState()
    state.running = True
    state.emergency_stop = False

    # Telemetry watchers
    print("📡 Starting telemetry watchers + CSV logger...")
    t_pos = asyncio.create_task(watch_posvel(drone, state))
    t_att = asyncio.create_task(watch_attitude(drone, state))
    t_mode = asyncio.create_task(watch_flight_mode(drone, state))

    # CSV log saved under logs/csv/
    t_log = asyncio.create_task(
        log_telemetry_csv(drone, state, f"csv/{CSV_NAME}")
    )

    command_queue = asyncio.Queue()

    # Start curses in a background thread (NO MAVSDK there!)
    loop = asyncio.get_running_loop()
    curses_task = loop.run_in_executor(None, run_curses, command_queue, state)

    # Run the MAVSDK control loop in main event loop
    try:
        await velocity_control_loop(drone, state, command_queue)
    finally:
        # Ensure we stop even on exceptions
        state.running = False

        with suppress(Exception):
            await t_log

        for t in (t_pos, t_att, t_mode):
            t.cancel()
        for t in (t_pos, t_att, t_mode):
            with suppress(asyncio.CancelledError):
                await t

        # wait curses thread to exit
        with suppress(Exception):
            await curses_task

        print("🛬 Landing...")
        await stop_offboard_and_land(drone)

        if state.emergency_stop:
            print(f"🏁 Mission ended (EMERGENCY) → {state.emergency_reason}")
            event_log(f"END_EMERGENCY {state.emergency_reason}")
        else:
            print("🏁 Mission completed")
            event_log("END_OK")


if __name__ == "__main__":
    asyncio.run(main())
