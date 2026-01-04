"""
Spiral Mission — Corrected Time-Series Analysis

What this script does (scientifically correct):
1) Uses mission_phase == "TRAJECTORY" only
2) Time aligned using mission_t0_unix (ground-truth start)
3) Computes TRUE geometric 3D drift (closest-point on trajectory)
4) Separates shape error from timing / phase effects
5) Produces a clean 2x3 dashboard
"""

import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt

from src.trajectories.spiral import SpiralTrajectory


# --------------------------------------------------
# Paths
# --------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]
CSV_PATH = REPO_ROOT / "logs" / "spiral_position_mission_log.csv"

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

OUTPUT_PNG = OUTPUT_DIR / "spiral_time_series_corrected.png"


# --------------------------------------------------
# Trajectory parameters (MUST match mission)
# --------------------------------------------------

ALTITUDE_M = 2.5
CLIMB_M = 5.0
RADIUS_M = 3.0
OMEGA = 0.3


# --------------------------------------------------
# Load CSV (TRAJECTORY phase only)
# --------------------------------------------------

t = []
x = []
y = []
z = []

vx = []
vy = []
vz = []

roll = []
pitch = []
yaw = []

mission_t0 = None

with open(CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)

    for row in reader:
        if row["mission_phase"] != "TRAJECTORY":
            continue

        if mission_t0 is None:
            mission_t0 = float(row["mission_t0_unix"])

        unix_time = float(row["unix_time"])
        t.append(unix_time - mission_t0)

        x.append(float(row["north_m"]))
        y.append(float(row["east_m"]))
        z.append(float(row["down_m"]))

        vx.append(float(row["vn_m_s"]))
        vy.append(float(row["ve_m_s"]))
        vz.append(float(row["vd_m_s"]))

        roll.append(float(row["roll_deg"]))
        pitch.append(float(row["pitch_deg"]))
        yaw.append(float(row["yaw_deg"]))

assert mission_t0 is not None, "No TRAJECTORY phase found in CSV"


# --------------------------------------------------
# Reference trajectory (single source of truth)
# --------------------------------------------------

trajectory = SpiralTrajectory(
    radius=RADIUS_M,
    start_z=-ALTITUDE_M,
    end_z=-(ALTITUDE_M + CLIMB_M),
    omega=OMEGA,
)


# --------------------------------------------------
# Helper: closest-point geometric drift (3D)
# --------------------------------------------------

def geometric_drift_3d(xa, ya, za, trajectory, t_center, window=0.5, samples=40):
    """
    Compute geometric drift as minimum distance to reference trajectory
    around time t_center (sliding window search).
    """
    t_start = max(0.0, t_center - window)
    t_end = t_center + window

    min_dist = float("inf")

    for i in range(samples):
        tau = t_start + (t_end - t_start) * i / (samples - 1)
        xr, yr, zr = trajectory.position_xyz(tau)

        d = math.sqrt(
            (xa - xr) ** 2 +
            (ya - yr) ** 2 +
            (za - zr) ** 2
        )
        min_dist = min(min_dist, d)

    return min_dist


# --------------------------------------------------
# Compute metrics
# --------------------------------------------------

drift_geo = []
speed = []
altitude = []

for i in range(len(t)):
    drift_geo.append(
        geometric_drift_3d(
            x[i], y[i], z[i],
            trajectory,
            t[i]
        )
    )

    speed.append(
        math.sqrt(vx[i] ** 2 + vy[i] ** 2 + vz[i] ** 2)
    )

    altitude.append(-z[i])  # positive up


# --------------------------------------------------
# Plot: 2x3 corrected dashboard
# --------------------------------------------------

fig, axs = plt.subplots(2, 3, figsize=(15, 8), sharex=True)

# ---- Row 1 ----
axs[0, 0].plot(t, drift_geo)
axs[0, 0].set_title("Geometric Drift (3D)")
axs[0, 0].set_ylabel("m")
axs[0, 0].grid(True)

axs[0, 1].plot(t, speed)
axs[0, 1].set_title("Speed Magnitude")
axs[0, 1].set_ylabel("m/s")
axs[0, 1].grid(True)

axs[0, 2].plot(t, yaw)
axs[0, 2].set_title("Yaw")
axs[0, 2].set_ylabel("deg")
axs[0, 2].grid(True)

# ---- Row 2 ----
axs[1, 0].plot(t, roll)
axs[1, 0].set_title("Roll")
axs[1, 0].set_ylabel("deg")
axs[1, 0].grid(True)

axs[1, 1].plot(t, pitch)
axs[1, 1].set_title("Pitch")
axs[1, 1].set_ylabel("deg")
axs[1, 1].grid(True)

axs[1, 2].plot(t, altitude)
axs[1, 2].set_title("Altitude")
axs[1, 2].set_ylabel("m")
axs[1, 2].grid(True)

for ax in axs[1, :]:
    ax.set_xlabel("Time since trajectory start [s]")

fig.suptitle(
    "Spiral (Helical) Autonomous Flight — Corrected Time-Series Analysis",
    fontsize=14
)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig(OUTPUT_PNG, dpi=200)
plt.close()

print(f"Saved corrected analysis plot → {OUTPUT_PNG}")
