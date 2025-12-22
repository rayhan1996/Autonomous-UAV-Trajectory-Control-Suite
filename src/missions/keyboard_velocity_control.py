"""
Keyboard Velocity Control Mission

Human-in-the-loop Offboard velocity control using keyboard input.
This mission demonstrates real-time UAV control via velocity setpoints
with integrated safety monitoring.

Features:
- Offboard velocity control (vx, vy, vz, yaw_rate)
- Real-time keyboard input (human-in-the-loop)
- Altitude safety limits
- Emergency stop and safe landing
"""

import asyncio
import curses
import time

from mavsdk.offboard import VelocityNedYaw

# ---- Core infrastructure ----
from src.core.config import PX4Config
from src.core.px4_connection import connect_px4, wait_armable
from src.core.offboard_helpers import (
    start_offboard,
    stop_offboard_and_land,
)
from src.utils.shared_state import SharedState


# ============================================================
# Mission parameters
# ============================================================

ALT_MIN_M = 1.0
ALT_MAX_M = 4.0
SPEED_M_S = 1.0
YAW_RATE_RAD_S = 0.5


# ============================================================
# Keyboard control loop
# ============================================================

async def keyboard_control_loop(drone, state: SharedState, stdscr):
    """
    Real-time keyboard control loop.
    Arrow keys control horizontal motion.
    W/S control altitude.
    A/D control yaw.
    """

    stdscr.nodelay(True)
    stdscr.clear()

    stdscr.addstr(0, 0, "Keyboard Velocity Control")
    stdscr.addstr(2, 0, "Arrows : Move (N/E/S/W)")
    stdscr.addstr(3, 0, "W / S  : Up / Down")
    stdscr.addstr(4, 0, "A / D  : Yaw Left / Right")
    stdscr.addstr(5, 0, "SPACE  : Stop")
    stdscr.addstr(6, 0, "Q      : Quit & Land")

    while state.running:
        key = stdscr.getch()

        north = east = down = yaw = 0.0

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
        elif key == ord(" "):
            pass
        elif key == ord("q"):
            state.running = False
            break

        # Altitude safety check (if telemetry available)
        if state.pos_ned is not None:
            _, _, down_pos = state.pos_ned
            altitude = -down_pos

            if altitude < ALT_MIN_M or altitude > ALT_MAX_M:
                stdscr.addstr(8, 0, "⚠ ALTITUDE LIMIT EXCEEDED — STOPPING")
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(1)
                state.running = False
                break

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(north, east, down, yaw)
        )

        await asyncio.sleep(0.1)


# ============================================================
# Main mission entry point
# ============================================================

async def main():
    cfg = PX4Config(system_address="udp://:14540")

    drone = await connect_px4(cfg.system_address)
    await wait_armable(drone)

    await drone.action.arm()
    print("✔ Armed")

    await drone.action.takeoff()
    print("▲ Takeoff")
    await asyncio.sleep(5)

    # Send zero velocity before starting offboard
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    if not await start_offboard(drone):
        return

    state = SharedState()

    def curses_wrapper(stdscr):
        return asyncio.run(
            keyboard_control_loop(drone, state, stdscr)
        )

    curses.wrapper(curses_wrapper)

    await stop_offboard_and_land(drone)
    print("🏁 Keyboard control mission completed.")


if __name__ == "__main__":
    asyncio.run(main())
