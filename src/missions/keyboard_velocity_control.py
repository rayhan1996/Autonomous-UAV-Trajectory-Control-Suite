"""
Keyboard Velocity Control Mission (Logged)

Human-in-the-loop Offboard velocity control using keyboard input.
Adds:
- telemetry watchers (pos/vel/attitude/mode)
- robust CSV logger output to logs/csv/
- simple event log to logs/telemetry/
- cleaner curses integration (no nested asyncio.run)
"""

import asyncio
import curses
import time
from pathlib import Path
from contextlib import suppress

from mavsdk.offboard import VelocityNedYaw

# ---- Core infrastructure ----
from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import (
    start_offboard,
    stop_offboard_and_land,
)

# ---- Telemetry & logging ----
from src.utils.shared_state import SharedState
from src.utils.telemetry_watchers import (
    watch_posvel,
    watch_attitude,
    watch_flight_mode,
)
from src.utils.telemetry_logger import log_telemetry_csv


# ============================================================
# Mission parameters
# ============================================================

ALT_MIN_M = 1.0
ALT_MAX_M = 4.0
SPEED_M_S = 1.0
YAW_RATE_RAD_S = 0.5

CSV_FILENAME = "keyboard_velocity_control.csv"
EVENTS_FILENAME = "keyboard_velocity_events.txt"


# ============================================================
# Helpers: repo-rooted log paths
# ============================================================

def repo_root() -> Path:
    # src/missions/<this_file>.py -> parents[2] should be repo root
    return Path(__file__).resolve().parents[2]


def ensure_log_dirs():
    root = repo_root()
    (root / "logs" / "csv").mkdir(parents=True, exist_ok=True)
    (root / "logs" / "telemetry").mkdir(parents=True, exist_ok=True)
    (root / "logs" / "plots").mkdir(parents=True, exist_ok=True)


def log_event(line: str):
    """
    Append a simple timestamped event line to logs/telemetry/.
    """
    root = repo_root()
    path = root / "logs" / "telemetry" / EVENTS_FILENAME
    ts = time.time()
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"{ts:.6f} {line}\n")


# ============================================================
# Keyboard control loop (async, curses)
# ============================================================

async def keyboard_control_loop(drone, state: SharedState, stdscr):
    """
    Real-time keyboard control loop.
    Arrow keys control horizontal motion (N/E/S/W) in NED frame.
    W/S control altitude (up/down).
    A/D control yaw rate.
    SPACE stops motion.
    Q quits & lands.
    """

    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0, "Keyboard Velocity Control (Logged)")
    stdscr.addstr(2, 0, "Arrows : Move (N/E/S/W)  [NED frame]")
    stdscr.addstr(3, 0, "W / S  : Up / Down")
    stdscr.addstr(4, 0, "A / D  : Yaw Left / Right (rate)")
    stdscr.addstr(5, 0, "SPACE  : Stop")
    stdscr.addstr(6, 0, "Q      : Quit & Land")
    stdscr.addstr(8, 0, f"Altitude limits: {ALT_MIN_M:.1f}m .. {ALT_MAX_M:.1f}m")

    log_event("MISSION_START keyboard control loop entered")

    while state.running and not state.emergency_stop:
        key = stdscr.getch()

        north = east = down = yaw_rate = 0.0
        label = "STOP"

        if key == curses.KEY_UP:
            north = SPEED_M_S
            label = "NORTH +"
        elif key == curses.KEY_DOWN:
            north = -SPEED_M_S
            label = "NORTH -"
        elif key == curses.KEY_RIGHT:
            east = SPEED_M_S
            label = "EAST +"
        elif key == curses.KEY_LEFT:
            east = -SPEED_M_S
            label = "EAST -"
        elif key == ord("w"):
            down = -SPEED_M_S   # up (down negative)
            label = "UP +"
        elif key == ord("s"):
            down = SPEED_M_S    # down
            label = "DOWN -"
        elif key == ord("a"):
            yaw_rate = -YAW_RATE_RAD_S
            label = "YAW LEFT"
        elif key == ord("d"):
            yaw_rate = YAW_RATE_RAD_S
            label = "YAW RIGHT"
        elif key == ord(" "):
            # STOP (all zeros)
            label = "STOP"
        elif key == ord("q"):
            log_event("USER_QUIT pressed q")
            state.running = False
            break

        # Altitude safety check (if telemetry available)
        if state.pos_ned is not None:
            _, _, down_pos = state.pos_ned
            altitude = -down_pos

            if altitude < ALT_MIN_M or altitude > ALT_MAX_M:
                warn = f"ALTITUDE_LIMIT_EXCEEDED altitude={altitude:.2f}m"
                stdscr.addstr(10, 0, "⚠ ALTITUDE LIMIT EXCEEDED — STOPPING & LANDING   ")
                log_event(warn)

                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.5)

                state.emergency_stop = True
                state.emergency_reason = warn
                state.running = False
                break

        # Send velocity setpoint
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, yaw_rate)
        )

        # UI line
        stdscr.addstr(12, 0, f"CMD: {label:10s}  vN={north:+.2f} vE={east:+.2f} vD={down:+.2f} yawRate={yaw_rate:+.2f}   ")
        stdscr.refresh()

        # Optional: log command events only when key is pressed
        if key != -1:
            log_event(f"CMD {label} vN={north:+.2f} vE={east:+.2f} vD={down:+.2f} yawRate={yaw_rate:+.2f}")

        await asyncio.sleep(0.1)

    log_event("MISSION_END keyboard control loop exited")


def run_curses_keyboard_loop(drone, state: SharedState):
    """
    Run the async keyboard loop inside a dedicated event loop so
    curses.wrapper can call a normal function safely.
    """
    def _wrapped(stdscr):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(keyboard_control_loop(drone, state, stdscr))
        finally:
            loop.close()

    curses.wrapper(_wrapped)


# ============================================================
# Main mission entry point
# ============================================================

async def main():
    ensure_log_dirs()

    cfg = PX4Config(system_address="udpin://0.0.0.0:14540")
    drone = await connect_px4(cfg.system_address)
    await wait_armable(drone)

    await drone.action.arm()
    print("✔ Armed")
    log_event("ARMED")

    await drone.action.takeoff()
    print("▲ Takeoff")
    log_event("TAKEOFF")
    await asyncio.sleep(5)

    # Send zero velocity before starting offboard
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    if not await start_offboard(drone):
        print("Offboard start failed.")
        log_event("OFFBOARD_START_FAILED")
        return

    log_event("OFFBOARD_STARTED")

    # Shared state & telemetry tasks
    state = SharedState()
    state.running = True
    state.emergency_stop = False

    # ✅ watchers
    task_posvel = asyncio.create_task(watch_posvel(drone, state))
    task_att = asyncio.create_task(watch_attitude(drone, state))
    task_mode = asyncio.create_task(watch_flight_mode(drone, state))

    # ✅ CSV telemetry log -> logs/csv/
    # (log_telemetry_csv already writes under <repo_root>/logs/
    #  so we pass "csv/<name>.csv")
    task_logger = asyncio.create_task(
        log_telemetry_csv(drone, state, f"csv/{CSV_FILENAME}")
    )

    # Run keyboard loop (blocking) in a thread to not kill the main event loop
    # because curses wrapper is sync.
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, run_curses_keyboard_loop, drone, state)

    # Shutdown
    state.running = False

    with suppress(Exception):
        await task_logger

    # cancel watchers
    for t in (task_posvel, task_att, task_mode):
        t.cancel()
    for t in (task_posvel, task_att, task_mode):
        with suppress(asyncio.CancelledError):
            await t

    await stop_offboard_and_land(drone)

    if state.emergency_stop:
        print(f"🏁 Keyboard control mission ended (EMERGENCY) → {state.emergency_reason}")
        log_event(f"END_EMERGENCY {state.emergency_reason}")
    else:
        print("🏁 Keyboard control mission completed.")
        log_event("END_OK")


if __name__ == "__main__":
    asyncio.run(main())
