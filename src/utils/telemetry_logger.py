import csv
import time
from mavsdk import System
from .shared_state import SharedState

async def log_telemetry_csv(drone: System, state: SharedState, filename: str):
    print(f"Telemetry logger started → {filename}")

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t",
            "north_m", "east_m", "down_m",
            "vn_m_s", "ve_m_s", "vd_m_s",
            "roll_deg", "pitch_deg", "yaw_deg",
            "flight_mode"
        ])

        t0 = time.time()

        # We rely on watcher-updated state; sample at ~10Hz
        while state.running:
            now = time.time() - t0

            pos = state.pos_ned
            vel = state.vel_ned
            att = state.attitude_deg
            mode = state.flight_mode

            if pos and vel:
                roll = pitch = yaw = ""
                if att:
                    roll, pitch, yaw = att

                mode_str = ""
                if mode is not None:
                    mode_str = getattr(mode, "name", str(mode))

                writer.writerow([
                    f"{now:.3f}",
                    f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{pos[2]:.3f}",
                    f"{vel[0]:.3f}", f"{vel[1]:.3f}", f"{vel[2]:.3f}",
                    f"{roll}", f"{pitch}", f"{yaw}",
                    mode_str
                ])

            # 10Hz logging
            await __sleep(0.1)

# small helper to keep asyncio import local
async def __sleep(sec: float):
    import asyncio
    await asyncio.sleep(sec)
