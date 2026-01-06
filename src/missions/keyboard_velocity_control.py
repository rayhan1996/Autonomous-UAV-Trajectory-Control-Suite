"""
Keyboard Velocity Control Mission (FINAL - STABLE)

- Human-in-the-loop velocity control
- Safe asyncio + curses integration
- Single event loop for MAVSDK
- CSV telemetry logging
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


# ============================================================
# Keyboard reader (SYNC, curses)
# ============================================================

def keyboard_reader(stdscr, command_queue, state: SharedState):
    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0, "Keyboard Velocity Control")
    stdscr.addstr(2, 0, "↑ ↓ ← → : Move (N/E/S/W)")
    stdscr.addstr(3, 0, "W / S   : Up / Down")
    stdscr.addstr(4, 0, "A / D   : Yaw Left / Right")
    stdscr.addstr(5, 0, "SPACE   : Stop")
    stdscr.addstr(6, 0, "Q       : Quit & Land")

    while state.running:
        key = stdscr.getch()
        if key != -1:
            command_queue.put_nowait(key)
        time.sleep(0.05)


def run_curses(command_queue, state):
    curses.wrapper(lambda stdscr: keyboard_reader(stdscr, command_queue, state))


# ============================================================
# Async velocity control loop (ONLY MAVSDK HERE)
# ============================================================

async def velocity_control_loop(drone, state: SharedState, command_queue):
    while state.running:
        north = east = down = yaw = 0.0

        try:
            key = command_queue.get_nowait()
        except asyncio.QueueEmpty:
            key = None

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
            yaw = -YAW_RATE_RAD_S
        elif key == ord("d"):
            yaw = YAW_RATE_RAD_S
        elif key == ord("q"):
            state.running = False
            break

        # Altitude safety
        if state.pos_ned is not None:
            _, _, down_pos = state.pos_ned
            alt = -down_pos
            if alt < ALT_MIN_M or alt > ALT_MAX_M:
                print("⚠ Altitude limit exceeded")
                state.running = False
                break

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, yaw)
        )

        await asyncio.sleep(0.1)


# ============================================================
# Main
# ============================================================

async def main():
    cfg = PX4Config(system_address="udpin://0.0.0.0:14540")
    drone = await connect_px4(cfg.system_address)
    await wait_armable(drone)

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0, 0, 0, 0)
    )

    if not await start_offboard(drone):
        return

    state = SharedState()
    state.running = True

    # Telemetry
    t_pos = asyncio.create_task(watch_posvel(drone, state))
    t_att = asyncio.create_task(watch_attitude(drone, state))
    t_mode = asyncio.create_task(watch_flight_mode(drone, state))
    t_log = asyncio.create_task(
        log_telemetry_csv(drone, state, f"csv/{CSV_NAME}")
    )

    command_queue = asyncio.Queue()

    loop = asyncio.get_running_loop()
    curses_task = loop.run_in_executor(
        None, run_curses, command_queue, state
    )

    await velocity_control_loop(drone, state, command_queue)
    await curses_task

    state.running = False

    with suppress(Exception):
        await t_log

    for t in (t_pos, t_att, t_mode):
        t.cancel()
        with suppress(asyncio.CancelledError):
            await t

    await stop_offboard_and_land(drone)
    print("🏁 Mission completed")


if __name__ == "__main__":
    asyncio.run(main())
