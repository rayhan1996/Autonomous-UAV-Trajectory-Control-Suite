import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt

from src.trajectories.spiral import SpiralTrajectory


# --------------------------------------------------
# Column mapping (CSV schema)
# --------------------------------------------------

COL_T = "t"
COL_X = "north_m"
COL_Y = "east_m"
COL_Z = "down_m"

COL_VX = "vn_m_s"
COL_VY = "ve_m_s"
COL_VZ = "vd_m_s"

COL_ROLL = "roll_deg"
COL_PITCH = "pitch_deg"
COL_YAW = "yaw_deg"


# --------------------------------------------------
# Paths
# --------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]
CSV_PATH = REPO_ROOT / "logs" / "csv" / "spiral_position_mission_log.csv"

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

OUTPUT_PNG = OUTPUT_DIR / "spiral_time_series_summary.png"


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
        t.append(float(row[COL_T]))

        x.append(float(row[COL_X]))
        y.append(float(row[COL_Y]))
        z.append(float(row[COL_Z]))

        vx.append(float(row[COL_VX]))
        vy.append(float(row[COL_VY]))
        vz.append(float(row[COL_VZ]))

        roll.append(float(row[COL_ROLL]))
        pitch.append(float(row[COL_PITCH]))
        yaw.append(float(row[COL_YAW]))


# --------------------------------------------------
# Normalize time
# --------------------------------------------------

t0 = t[0]
t_rel = [ti - t0 for ti in t]


# --------------------------------------------------
# Reference trajectory (single source of truth)
# --------------------------------------------------

ALTITUDE_M = 2.5
CLIMB_M = 5.0

trajectory = SpiralTrajectory(
    radius=3.0,
    center_x=0.0,
    center_y=0.0,
    start_z=-ALTITUDE_M,
    end_z=-(ALTITUDE_M + CLIMB_M),
    omega=0.3,
)

x_ref = []
y_ref = []
z_ref = []

for ti in t_rel:
    if ti > trajectory.duration():
        break
    xr, yr, zr = trajectory.position_xyz(ti)
    x_ref.append(xr)
    y_ref.append(yr)
    z_ref.append(zr)


# --------------------------------------------------
# Derived metrics
# --------------------------------------------------

# XY Drift
drift = [
    math.hypot(x[i] - x_ref[i], y[i] - y_ref[i])
    for i in range(len(x_ref))
]

# Speed magnitude
speed = [
    math.sqrt(vx[i] ** 2 + vy[i] ** 2 + vz[i] ** 2)
    for i in range(len(vx))
]

# Altitude (positive up)
altitude = [-zi for zi in z]


# --------------------------------------------------
# Plot: 2x3 dashboard
# --------------------------------------------------

fig, axs = plt.subplots(2, 3, figsize=(15, 8), sharex=True)

# ---- Row 1 ----
axs[0, 0].plot(t_rel[:len(drift)], drift)
axs[0, 0].set_title("XY Position Error (Drift)")
axs[0, 0].set_ylabel("m")
axs[0, 0].grid(True)

axs[0, 1].plot(t_rel, speed)
axs[0, 1].set_title("Speed Magnitude")
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

fig.suptitle(
    "Spiral (Helical) Autonomous Flight — Time-Series Summary",
    fontsize=14
)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig(OUTPUT_PNG, dpi=200)
plt.close()

print(f"Saved summary plot → {OUTPUT_PNG}")
