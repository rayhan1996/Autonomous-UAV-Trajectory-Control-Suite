import asyncio
import curses
import time
import math
from pathlib import Path
from dataclasses import dataclass
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


# ===================== PARAMETERS =====================

SPEED_M_S = 1.0          # forward/right speed
ALT_STEP_M = 0.15
YAW_RATE_DEG_S = 30.0

ALT_MIN_M = 1.0
ALT_MAX_M = 4.0
ALT_WARN_MARGIN = 0.25

DT = 0.1  # control loop Hz = 10


# ===================== PATHS =====================

def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def ensure_dirs():
    root = repo_root()
    (root / "logs/csv").mkdir(parents=True, exist_ok=True)
    (root / "logs/telemetry").mkdir(parents=True, exist_ok=True)
    (root / "logs/plots").mkdir(parents=True, exist_ok=True)


def timestamped_csv():
    ts = time.strftime("%Y%m%d_%H%M%S")
    return f"keyboard_velocity_control_{ts}.csv"


def event_log(msg: str):
    path = repo_root() / "logs/telemetry/keyboard_velocity_events.txt"
    with open(path, "a") as f:
        f.write(f"{time.time():.3f} {msg}\n")


# ===================== UI STATE =====================

@dataclass
class UIState:
    status: str = "INIT"
    last_key: str = "-"
    alt: float = 0.0
    alt_t: float = 0.0
    yaw: float = 0.0
    x_t: float = 0.0
    y_t: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    mode: str = ""
    warning: str = ""


# ===================== KEYBOARD =====================

def decode_key(k):
    return {
        curses.KEY_UP: "UP",
        curses.KEY_DOWN: "DOWN",
        curses.KEY_LEFT: "LEFT",
        curses.KEY_RIGHT: "RIGHT",
        ord("w"): "W",
        ord("s"): "S",
        ord("a"): "A",
        ord("d"): "D",
        ord(" "): "SPACE",
        ord("q"): "Q",
    }.get(k, str(k))


def keyboard_ui(stdscr, q: asyncio.Queue, state: SharedState, ui: UIState):
    curses.curs_set(0)
    stdscr.nodelay(True)

    while state.running:
        stdscr.erase()

        stdscr.addstr(0, 0, "Keyboard BODY-frame Control (OFFBOARD)")
        stdscr.addstr(2, 0, "↑ ↓ : forward / backward")
        stdscr.addstr(3, 0, "← → : left / right")
        stdscr.addstr(4, 0, "W/S : altitude up/down")
        stdscr.addstr(5, 0, "A/D : yaw left/right")
        stdscr.addstr(6, 0, "SPACE: stop  |  Q: quit")

        stdscr.addstr(8, 0, f"STATUS: {ui.status}")
        stdscr.addstr(9, 0, f"MODE  : {ui.mode}")
        stdscr.addstr(10, 0, f"ALT   : {ui.alt:.2f} m   target {ui.alt_t:.2f}")
        stdscr.addstr(11, 0, f"XY tgt: x={ui.x_t:.2f} y={ui.y_t:.2f}")
        stdscr.addstr(12, 0, f"Yaw   : {ui.yaw:.1f} deg")
        stdscr.addstr(13, 0, f"Cmd   : vx={ui.vx:.2f} vy={ui.vy:.2f}")
        stdscr.addstr(14, 0, f"Key   : {ui.last_key}")

        if ui.warning:
            stdscr.addstr(16, 0, f"⚠ {ui.warning}")

        stdscr.refresh()

        k = stdscr.getch()
        if k != -1:
            q.put_nowait(k)

        time.sleep(0.05)


# ===================== CONTROL LOOP =====================

async def control_loop(drone, state: SharedState, ui: UIState, q: asyncio.Queue):
    ui.status = "WAIT_TELEMETRY"
    while state.pos_ned is None:
        await asyncio.sleep(0.05)

    x_t, y_t, d_t = state.pos_ned
    alt_t = -d_t
    yaw_t = state.attitude_deg[2] if state.attitude_deg else 0.0

    vx_body = 0.0
    vy_body = 0.0
    yaw_rate = 0.0

    ui.status = "CONTROL_LIVE"
    event_log("CONTROL_START")

    while state.running:
        ui.alt = -state.pos_ned[2]
        ui.mode = getattr(state.flight_mode, "name", "")
        ui.yaw = yaw_t
        ui.warning = ""

        try:
            k = q.get_nowait()
            ui.last_key = decode_key(k)
        except asyncio.QueueEmpty:
            k = None

        if k == curses.KEY_UP:
            vx_body, vy_body = SPEED_M_S, 0.0
        elif k == curses.KEY_DOWN:
            vx_body, vy_body = -SPEED_M_S, 0.0
        elif k == curses.KEY_RIGHT:
            vx_body, vy_body = 0.0, SPEED_M_S
        elif k == curses.KEY_LEFT:
            vx_body, vy_body = 0.0, -SPEED_M_S
        elif k == ord("w"):
            alt_t += ALT_STEP_M
        elif k == ord("s"):
            alt_t -= ALT_STEP_M
        elif k == ord("a"):
            yaw_rate = -YAW_RATE_DEG_S
        elif k == ord("d"):
            yaw_rate = YAW_RATE_DEG_S
        elif k == ord(" "):
            vx_body = vy_body = yaw_rate = 0.0
        elif k == ord("q"):
            state.running = False
            break

        # altitude clamp
        alt_t = max(ALT_MIN_M, min(ALT_MAX_M, alt_t))

        # BODY → NED transform
        yaw_rad = math.radians(yaw_t)
        v_n = math.cos(yaw_rad) * vx_body - math.sin(yaw_rad) * vy_body
        v_e = math.sin(yaw_rad) * vx_body + math.cos(yaw_rad) * vy_body

        x_t += v_n * DT
        y_t += v_e * DT
        yaw_t += yaw_rate * DT

        ui.x_t, ui.y_t = x_t, y_t
        ui.alt_t = alt_t
        ui.vx, ui.vy = vx_body, vy_body

        await drone.offboard.set_position_ned(
            PositionNedYaw(x_t, y_t, -alt_t, yaw_t)
        )

        await asyncio.sleep(DT)

    event_log("CONTROL_END")


# ===================== MAIN =====================

async def main():
    ensure_dirs()
    csv_name = timestamped_csv()

    drone = await connect_px4("udpin://0.0.0.0:14540")
    await wait_armable(drone)

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)

    state = SharedState()
    state.running = True

    ui = UIState()

    t_pos = asyncio.create_task(watch_posvel(drone, state))
    t_att = asyncio.create_task(watch_attitude(drone, state))
    t_mode = asyncio.create_task(watch_flight_mode(drone, state))
    t_log = asyncio.create_task(log_telemetry_csv(drone, state, csv_name))

    while state.pos_ned is None:
        await asyncio.sleep(0.1)

    for _ in range(20):
        x, y, d = state.pos_ned
        yaw = state.attitude_deg[2]
        await drone.offboard.set_position_ned(PositionNedYaw(x, y, d, yaw))
        await asyncio.sleep(0.05)

    await start_offboard(drone)

    q = asyncio.Queue()
    loop = asyncio.get_running_loop()
    ui_task = loop.run_in_executor(None, curses.wrapper,
                                   lambda s: keyboard_ui(s, q, state, ui))

    try:
        await control_loop(drone, state, ui, q)
    finally:
        state.running = False
        await drone.offboard.stop()
        await drone.action.land()

        for t in (t_pos, t_att, t_mode, t_log):
            t.cancel()
            with suppress(asyncio.CancelledError):
                await t


if __name__ == "__main__":
    asyncio.run(main())
