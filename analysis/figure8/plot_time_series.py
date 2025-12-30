import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

from src.trajectories.figure8 import Figure8Trajectory

# --------------------------------------------------
# Paths (mission-local outputs)
# --------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]

CSV_PATH = REPO_ROOT / "logs" / "csv" / "figure8_autonomous_flight_log.csv"

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

OUTPUT_PNG = OUTPUT_DIR / "figure8_time_series_summary.png"

# --------------------------------------------------
# Trajectory parameters (must match mission)
# --------------------------------------------------

RADIUS_M = 3.0
OMEGA = 0.3


# --------------------------------------------------
# Load telemetry CSV
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

with open(CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["time_s"]))

        x.append(float(row["pos_north_m"]))
        y.append(float(row["pos_east_m"]))
        z.append(float(row["pos_down_m"]))

        vx.append(float(row["vel_north_m"]))
        vy.append(float(row["vel_east_m"]))
        vz.append(float(row["vel_down_m"]))

        roll.append(float(row["roll_deg"]))
        pitch.append(float(row["pitch_deg"]))
        yaw.append(float(row["yaw_deg"]))


# --------------------------------------------------
# Normalize time
# --------------------------------------------------

t0 = t[0]
t_rel = [ti - t0 for ti in t]


# --------------------------------------------------
# Reference trajectory (single source of truth)
# --------------------------------------------------

trajectory = Figure8Trajectory(
    radius=RADIUS_M,
    omega=OMEGA,
    center_x=0.0,
    center_y=0.0,
)

x_ref = []
y_ref = []

for ti in t_rel:
    xr, yr = trajectory.position_xy(ti)
    x_ref.append(xr)
    y_ref.append(yr)


# --------------------------------------------------
# Derived metrics
# --------------------------------------------------

# Drift (position error in XY)
drift = [
    math.hypot(x[i] - x_ref[i], y[i] - y_ref[i])
    for i in range(len(t_rel))
]

# Speed magnitude
speed = [
    math.sqrt(vx[i] ** 2 + vy[i] ** 2 + vz[i] ** 2)
    for i in range(len(t_rel))
]

# Altitude (positive up)
altitude = [-zi for zi in z]


# --------------------------------------------------
# Plot: 2x3 time-series dashboard
# --------------------------------------------------

fig, axs = plt.subplots(2, 3, figsize=(15, 8), sharex=True)

# ---- Row 1 ----
axs[0, 0].plot(t_rel, drift)
axs[0, 0].set_title("Position Error (Drift)")
axs[0, 0].set_ylabel("m")
axs[0, 0].grid(True)

axs[0, 1].plot(t_rel, speed)
axs[0, 1].set_title("Speed")
axs[0, 1].set_ylabel("m/s")
axs[0, 1].grid(True)

axs[0, 2].plot(t_rel, yaw)
axs[0, 2].set_title("Yaw")
axs[0, 2].set_ylabel("deg")
axs[0, 2].grid(True)

# ---- Row 2 ----
axs[1, 0].plot(t_rel, roll)
axs[1, 0].set_title("Roll")
axs[1, 0].set_ylabel("deg")
axs[1, 0].grid(True)

axs[1, 1].plot(t_rel, pitch)
axs[1, 1].set_title("Pitch")
axs[1, 1].set_ylabel("deg")
axs[1, 1].grid(True)

axs[1, 2].plot(t_rel, altitude)
axs[1, 2].set_title("Altitude")
axs[1, 2].set_ylabel("m")
axs[1, 2].grid(True)

for ax in axs[1, :]:
    ax.set_xlabel("Time [s]")

fig.suptitle("Figure-8 Autonomous Flight — Time-Series Summary", fontsize=14)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig(OUTPUT_PNG, dpi=200)
plt.close()

print(f"Saved summary plot → {OUTPUT_PNG}")


