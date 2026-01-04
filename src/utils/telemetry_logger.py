import csv
import time
from pathlib import Path
from mavsdk import System
from .shared_state import SharedState


async def log_telemetry_csv(
    drone: System,
    state: SharedState,
    filename: str,
):
    """
    Logs UAV telemetry to a CSV file inside the project-level `logs/` directory.

    CSV now includes mission markers:
    - mission_phase
    - mission_t0_unix (absolute timestamp of trajectory start)
    """

    # Resolve repo root and logs directory
    repo_root = Path(__file__).resolve().parents[2]
    logs_dir = repo_root / "logs" / "csv" 
    logs_dir.mkdir(exist_ok=True)

    log_path = logs_dir / filename
    print(f"Telemetry logger started → {log_path}")

    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "t",  # seconds since logger start
            "unix_time",  # absolute unix time for each sample
            "north_m", "east_m", "down_m",
            "vn_m_s", "ve_m_s", "vd_m_s",
            "roll_deg", "pitch_deg", "yaw_deg",
            "flight_mode",
            # ✅ markers
            "mission_phase",
            "mission_t0_unix",
        ])

        t0_logger = time.time()

        while state.running:
            unix_now = time.time()
            now = unix_now - t0_logger

            pos = state.pos_ned
            vel = state.vel_ned
            att = state.attitude_deg
            mode = state.flight_mode

            if pos is not None and vel is not None:
                roll = pitch = yaw = ""
                if att is not None:
                    roll, pitch, yaw = att

                mode_str = ""
                if mode is not None:
                    mode_str = getattr(mode, "name", str(mode))

                writer.writerow([
                    f"{now:.3f}",
                    f"{unix_now:.6f}",
                    f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{pos[2]:.3f}",
                    f"{vel[0]:.3f}", f"{vel[1]:.3f}", f"{vel[2]:.3f}",
                    f"{roll}", f"{pitch}", f"{yaw}",
                    mode_str,
                    # ✅ markers
                    state.mission_phase,
                    "" if state.mission_t0_unix is None else f"{state.mission_t0_unix:.6f}",
                ])

            await _sleep(0.1)


async def _sleep(sec: float):
    import asyncio
    await asyncio.sleep(sec)
