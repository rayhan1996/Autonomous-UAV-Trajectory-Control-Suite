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

    This function is intentionally filesystem-robust:
    - CSV is always written to <repo_root>/logs/
    - Independent of current working directory
    """

    # --------------------------------------------------
    # Resolve project root and logs directory
    # src/utils/telemetry_logger.py -> repo_root = parents[2]
    # --------------------------------------------------
    repo_root = Path(__file__).resolve().parents[2]
    logs_dir = repo_root / "logs"
    logs_dir.mkdir(exist_ok=True)

    log_path = logs_dir / filename

    print(f"Telemetry logger started → {log_path}")

    # --------------------------------------------------
    # Open CSV file
    # --------------------------------------------------
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)

        # CSV header
        writer.writerow([
            "t",
            "north_m", "east_m", "down_m",
            "vn_m_s", "ve_m_s", "vd_m_s",
            "roll_deg", "pitch_deg", "yaw_deg",
            "flight_mode",
        ])

        t0 = time.time()

        # --------------------------------------------------
        # Logging loop (~10 Hz)
        # --------------------------------------------------
        while state.running:
            now = time.time() - t0

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
                    f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{pos[2]:.3f}",
                    f"{vel[0]:.3f}", f"{vel[1]:.3f}", f"{vel[2]:.3f}",
                    f"{roll}", f"{pitch}", f"{yaw}",
                    mode_str,
                ])

            # ~10 Hz logging rate
            await _sleep(0.1)


# --------------------------------------------------
# Small helper to keep asyncio import local
# --------------------------------------------------
async def _sleep(sec: float):
    import asyncio
    await asyncio.sleep(sec)
