"""
Spiral (Helical) Mission — Time-Series Summary (FIXED)

Fixes applied:
1) 3D drift (XYZ), not just XY
2) Spatial alignment: reference shifted to match actual start
3) Temporal alignment: warm-up phase removed
4) Clean, reproducible, single-source-of-truth reference
"""

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

OUTPUT_PNG = OUTPUT_DIR / "spiral_time_series.png"


# --------------------------------------------------
# Parameters (explicit & tunable)
# --------------------------------------------------

ALTITUDE_M = 2.5
CLIMB_M = 5.0
RADIUS_M = 3.0
OMEGA = 0.3

WARMUP_SECONDS = 10.0   # <-- temporal alignment (remove alignment phase)


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
# Temporal alignment (remove warm-up)
# --------------------------------------------------

t0 = t[0]
t_rel = [ti - t0 for ti in t]

start_idx = next(
    (i for i, ti in enumerate(t_rel) if ti >= WARMUP_SECONDS),
    0
)

t_u = t_rel[start_idx:]
x_u = x[start_idx:]
y_u = y[start_idx:]
z_u = z[start_idx:]

vx_u = vx[start_idx:]
vy_u = vy[start_idx:]
vz_u = vz[start_idx:]

roll_u = roll[start_idx:]
pitch_u = pitch[start_idx:]
yaw_u = yaw[start_idx:]


# --------------------------------------------------
# Reference trajectory (single source of truth)
# --------------------------------------------------

trajectory = SpiralTrajectory(
    radius=RADIUS_M,
    center_x=0.0,
    center_y=0.0,
    start_z=-ALTITUDE_M,
    end_z=-(ALTITUDE_M + CLIMB_M),
    omega=OMEGA,
)

# --- Reference aligned to actual start position ---
x0, y0, z0 = x_u[0], y_u[0], z_u[0]
xr0, yr0, zr0 = trajectory.position_xyz(t_u[0])

dx = x0 - xr0
dy = y0 - yr0
dz = z0 - zr0


x_ref = []
y_ref = []
z_ref = []

for ti in t_u:
    if ti > trajectory.duration():
        break
    xr, yr, zr = trajectory.position_xyz(ti)
    x_ref.append(xr + dx)
    y_ref.append(yr + dy)
    z_ref.append(zr + dz)


# --------------------------------------------------
# Derived metrics (FIXED)
# --------------------------------------------------

N = len(x_ref)

# 3D Drift (XYZ)
drift_xyz = [
    math.sqrt(
        (x_u[i] - x_ref[i]) ** 2 +
        (y_u[i] - y_ref[i]) ** 2 +
        (z_u[i] - z_ref[i]) ** 2
    )
    for i in range(N)
]

# Speed magnitude
speed = [
    math.sqrt(vx_u[i] ** 2 + vy_u[i] ** 2 + vz_u[i] ** 2)
    for i in range(N)
]

# Altitude (positive up)
altitude = [-z_u[i] for i in range(N)]


# --------------------------------------------------
# Plot: 2x3 dashboard (final)
# --------------------------------------------------

fig, axs = plt.subplots(2, 3, figsize=(15, 8), sharex=True)

# ---- Row 1 ----
axs[0, 0].plot(t_u[:N], drift_xyz)
axs[0, 0].set_title("3D Position Error (Drift XYZ)")
axs[0, 0].set_ylabel("m")
axs[0, 0].grid(True)

axs[0, 1].plot(t_u[:N], speed)
axs[0, 1].set_title("Speed Magnitude")
axs[0, 1].set_ylabel("m/s")
axs[0, 1].grid(True)

axs[0, 2].plot(t_u[:N], yaw_u[:N])
axs[0, 2].set_title("Yaw")
axs[0, 2].set_ylabel("deg")
axs[0, 2].grid(True)

# ---- Row 2 ----
axs[1, 0].plot(t_u[:N], roll_u[:N])
axs[1, 0].set_title("Roll")
axs[1, 0].set_ylabel("deg")
axs[1, 0].grid(True)

axs[1, 1].plot(t_u[:N], pitch_u[:N])
axs[1, 1].set_title("Pitch")
axs[1, 1].set_ylabel("deg")
axs[1, 1].grid(True)

axs[1, 2].plot(t_u[:N], altitude)
axs[1, 2].set_title("Altitude")
axs[1, 2].set_ylabel("m")
axs[1, 2].grid(True)

for ax in axs[1, :]:
    ax.set_xlabel("Time [s]")

fig.suptitle(
    "Spiral (Helical) Autonomous Flight — Time-Series Summary (Corrected Drift)",
    fontsize=14
)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig(OUTPUT_PNG, dpi=200)
plt.close()

print(f"Saved corrected summary plot → {OUTPUT_PNG}")
