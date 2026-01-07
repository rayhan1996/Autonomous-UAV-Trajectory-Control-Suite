"""
Keyboard "Velocity-Feel" Control Mission (FINAL - STABLE + LOGGED + UI + SAFE)

Key idea:
- Use OFFBOARD POSITION setpoints for stability (PX4 likes this)
- Simulate velocity control by integrating commanded vx, vy over time:
      x_target += vx_cmd * dt
      y_target += vy_cmd * dt
- Control altitude with a real altitude target (W/S):
      alt_target +=/-= ALT_STEP_M
      down_target = -alt_target

Improvements added:
✅ Live UI telemetry in curses: altitude + mode + last key + targets
✅ Soft altitude limiter (clamp) instead of immediate emergency stop
✅ Pre-warning when approaching ALT_MIN/MAX
✅ Correct log paths:
   - CSV telemetry -> logs/csv/keyboard_velocity_control.csv
   - Events/stages -> logs/telemetry/keyboard_velocity_events.txt
✅ Clear stage prints and robust shutdown (offboard stop + land retry + disarm fallback)

Controls:
- Arrows: move N/E/S/W (velocity-feel)
- W/S: up/down (altitude target)
- A/D: yaw left/right (yaw rate -> integrated yaw target)
- SPACE: stop XY + yaw_rate
- Q: quit & land
"""

import asyncio
import curses
import time
import math
from pathlib import Path
from contextlib import suppress
from dataclasses import dataclass

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

# Warning margin near limits (m)
ALT_WARN_MARGIN_M = 0.25

# Control frequency
CONTROL_DT_S = 0.10  # 10 Hz

# Logs
CSV_NAME = "keyboard_velocity_control.csv"
EVENTS_NAME = "keyboard_velocity_events.txt"


# ============================================================
# Repo / logging helpers
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
# Small helpers
# ============================================================

def decode_key(key: int) -> str:
    if key == curses.KEY_UP:
        return "UP"
    if key == curses.KEY_DOWN:
        return "DOWN"
    if key == curses.KEY_RIGHT:
        return "RIGHT"
    if key == curses.KEY_LEFT:
        return "LEFT"
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
    return (yaw_deg + 180.0) % 360.0 - 180.0


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ============================================================
# UI state shared between async control loop and curses thread
# ============================================================

@dataclass
class UIState:
    last_key: str = "-"
    status: str = "INIT"
    warning: str = ""
    alt_m: float = float("nan")
    mode: str = ""
    x_t: float = 0.0
    y_t: float = 0.0
    alt_t: float = 0.0
    yaw_t: float = 0.0
    vx_cmd: float = 0.0
    vy_cmd: float = 0.0
    yawrate_cmd_deg_s: float = 0.0


# ============================================================
# Keyboard UI thread (curses) -> pushes keys into queue
# ============================================================

def keyboard_ui(stdscr, command_queue: "asyncio.Queue[int]", state: SharedState, ui: UIState):
    stdscr.nodelay(True)
    curses.curs_set(0)

    event_log("CURSES_STARTED UI shown")

    while state.running:
        stdscr.erase()

        stdscr.addstr(0, 0, "Keyboard Control ✅ ACTIVE (OFFBOARD Position Setpoints)")
        stdscr.addstr(2, 0, "↑ ↓ ← → : Move XY (velocity-feel)")
        stdscr.addstr(3, 0, "W / S   : Altitude target Up/Down")
        stdscr.addstr(4, 0, "A / D   : Yaw left/right (rate-feel)")
        stdscr.addstr(5, 0, "SPACE   : Stop XY + yaw-rate")
        stdscr.addstr(6, 0, "Q       : Quit & Land")

        stdscr.addstr(8, 0, f"ALT safety: {ALT_MIN_M:.1f} .. {ALT_MAX_M:.1f} m   (warn margin {ALT_WARN_MARGIN_M:.2f})")

        # Live telemetry + targets
        stdscr.addstr(10, 0, f"STATUS: {ui.status}")
        stdscr.addstr(11, 0, f"MODE  : {ui.mode}")
        stdscr.addstr(12, 0, f"ALT(m): {ui.alt_m:.2f}    ALT_target: {ui.alt_t:.2f}")
        stdscr.addstr(13, 0, f"XY_target: x={ui.x_t:.2f}  y={ui.y_t:.2f}")
        stdscr.addstr(14, 0, f"Yaw_target(deg): {ui.yaw_t:.1f}")
        stdscr.addstr(15, 0, f"Cmd: vx={ui.vx_cmd:.2f}  vy={ui.vy_cmd:.2f}  yawrate(deg/s)={ui.yawrate_cmd_deg_s:.1f}")
        stdscr.addstr(16, 0, f"Last key: {ui.last_key}")

        if ui.warning:
            stdscr.addstr(18, 0, f"⚠ {ui.warning}")

        stdscr.addstr(20, 0, ">>> Focus THIS terminal and press a key to control <<<")
        stdscr.refresh()

        key = stdscr.getch()
        if key != -1:
            try:
                command_queue.put_nowait(key)
            except Exception:
                pass

        time.sleep(0.05)


def run_curses(command_queue, state, ui):
    curses.wrapper(lambda stdscr: keyboard_ui(stdscr, command_queue, state, ui))


# ============================================================
# Async control loop (ONLY MAVSDK HERE)
# ============================================================

async def position_control_loop(drone, state: SharedState, ui: UIState, command_queue: "asyncio.Queue[int]"):
    ui.status = "WAIT_TELEMETRY"
    print("⌛ Waiting for first telemetry position...")
    while state.running and state.pos_ned is None:
        await asyncio.sleep(0.05)

    if state.pos_ned is None:
        print("❌ No telemetry position received. Abort.")
        event_log("ABORT_NO_TELEMETRY")
        state.running = False
        return

    ui.status = "WAIT_FIRST_KEY"
    print("⌨️ Keyboard control ready. Waiting for first key press...")
    while state.running and command_queue.empty():
        # update UI telemetry while waiting
        if state.pos_ned is not None:
            ui.alt_m = -state.pos_ned[2]
        if state.flight_mode is not None:
            ui.mode = getattr(state.flight_mode, "name", str(state.flight_mode))
        await asyncio.sleep(0.1)

    if not state.running:
        return

    print("✅ First key received. Control is LIVE.")
    event_log("FIRST_KEY_RECEIVED")
    ui.status = "CONTROL_LIVE"

    # Initialize targets from current telemetry 
    x_target, y_target, down_target = state.pos_ned
    alt_target = -down_target   

    yaw_target = 0.0
    if state.attitude_deg is not None:
        yaw_target = state.attitude_deg[2]

    # Commands (velocity-feel) - hold last command until SPACE or new direction
    vx_cmd = 0.0
    vy_cmd = 0.0
    yaw_rate_cmd_deg_s = 0.0

    dt = CONTROL_DT_S

    while state.running and not state.emergency_stop:
        # Update UI live telemetry
        if state.pos_ned is not None:
            ui.alt_m = -state.pos_ned[2]
        if state.flight_mode is not None:
            ui.mode = getattr(state.flight_mode, "name", str(state.flight_mode))

        ui.warning = ""

        # Read key if any
        try:
            key = command_queue.get_nowait()
            ui.last_key = decode_key(key)
            event_log(f"KEY {ui.last_key}")
        except asyncio.QueueEmpty:
            key = None

        # Direction keys -> update vx/vy commands
        if key == curses.KEY_UP:
            vx_cmd, vy_cmd = SPEED_M_S, 0.0
        elif key == curses.KEY_DOWN:
            vx_cmd, vy_cmd = -SPEED_M_S, 0.0
        elif key == curses.KEY_RIGHT:
            vx_cmd, vy_cmd = 0.0, SPEED_M_S
        elif key == curses.KEY_LEFT:
            vx_cmd, vy_cmd = 0.0, -SPEED_M_S

        # Altitude target changes
        elif key == ord("w"):
            alt_target += ALT_STEP_M
        elif key == ord("s"):
            alt_target -= ALT_STEP_M

        # Yaw rate commands (integrated into yaw_target)
        elif key == ord("a"):
            yaw_rate_cmd_deg_s = -math.degrees(YAW_RATE_RAD_S)
        elif key == ord("d"):
            yaw_rate_cmd_deg_s = math.degrees(YAW_RATE_RAD_S)

        # Stop
        elif key == ord(" "):
            vx_cmd, vy_cmd = 0.0, 0.0
            yaw_rate_cmd_deg_s = 0.0

        # Quit
        elif key == ord("q"):
            print("🛑 Q pressed -> exiting keyboard mode")
            event_log("USER_QUIT")
            state.running = False
            break

        # --- Soft altitude limiter (clamp) + warning near limits ---
        if alt_target >= ALT_MAX_M - ALT_WARN_MARGIN_M:
            ui.warning = f"Approaching ALT_MAX ({ALT_MAX_M:.1f}m)"
        if alt_target <= ALT_MIN_M + ALT_WARN_MARGIN_M:
            ui.warning = f"Approaching ALT_MIN ({ALT_MIN_M:.1f}m)"

        alt_target_clamped = clamp(alt_target, ALT_MIN_M, ALT_MAX_M)
        if alt_target_clamped != alt_target:
            # clamp happened
            msg = f"ALT_CLAMP alt_target={alt_target:.2f} -> {alt_target_clamped:.2f}"
            event_log(msg)
            ui.warning = msg
            alt_target = alt_target_clamped

        # Integrate velocity-feel into XY target
        x_target += vx_cmd * dt
        y_target += vy_cmd * dt

        # Integrate yaw rate into yaw target
        yaw_target = wrap_yaw_deg(yaw_target + yaw_rate_cmd_deg_s * dt)

        # Convert altitude target -> NED down
        down_target = -alt_target

        # Update UI targets
        ui.x_t, ui.y_t = x_target, y_target
        ui.alt_t = alt_target
        ui.yaw_t = yaw_target
        ui.vx_cmd, ui.vy_cmd = vx_cmd, vy_cmd
        ui.yawrate_cmd_deg_s = yaw_rate_cmd_deg_s

        # Send OFFBOARD position setpoint
        await drone.offboard.set_position_ned(
            PositionNedYaw(x_target, y_target, down_target, yaw_target)
        )

        await asyncio.sleep(dt)

    event_log("CONTROL_LOOP_ENDED")
    ui.status = "CONTROL_ENDED"
    print("⌨️ Exited keyboard control loop.")


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

    ui = UIState()

    # Telemetry watchers + CSV logger
    print("📡 Starting telemetry watchers + CSV logger...")
    t_pos = asyncio.create_task(watch_posvel(drone, state))
    t_att = asyncio.create_task(watch_attitude(drone, state))
    t_mode = asyncio.create_task(watch_flight_mode(drone, state))
    t_log = asyncio.create_task(
        log_telemetry_csv(drone, state, CSV_NAME)
      )

    # Pre-offboard: stream a few position setpoints at current pose
    print("🧷 Pre-offboard: waiting position, then streaming setpoints...")
    while state.pos_ned is None:
        await asyncio.sleep(0.05)

    x0, y0, d0 = state.pos_ned
    yaw0 = 0.0
    if state.attitude_deg is not None:
        yaw0 = state.attitude_deg[2]

    for _ in range(30):
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

    command_queue: asyncio.Queue[int] = asyncio.Queue()

    # Start curses UI in background thread (NO MAVSDK in that thread)
    loop = asyncio.get_running_loop()
    curses_task = loop.run_in_executor(None, run_curses, command_queue, state, ui)

    try:
        await position_control_loop(drone, state, ui, command_queue)

    finally:
        # Stop loops
        state.running = False

        # Logger finish
        with suppress(Exception):
            await t_log

        # Stop watchers
        for t in (t_pos, t_att, t_mode):
            t.cancel()
        for t in (t_pos, t_att, t_mode):
            with suppress(asyncio.CancelledError):
                await t

        # wait curses thread
        with suppress(Exception):
            await curses_task

        print("🛑 Stopping OFFBOARD...")
        with suppress(Exception):
            await drone.offboard.stop()
        await asyncio.sleep(0.5)

        # Land with retries (SITL can timeout)
        print("🛬 Landing...")
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
            print("⚠ Landing timeout -> trying disarm fallback")
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
