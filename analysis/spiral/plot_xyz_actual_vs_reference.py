"""
3D Helical Trajectory Plot
Actual UAV path vs Reference trajectory

- Reference: from SpiralTrajectory (single source of truth)
- Actual: from PX4 telemetry CSV
"""

import csv
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# --------------------------------------------------
# Import trajectory (reference)
# --------------------------------------------------
from src.trajectories.spiral import SpiralTrajectory


# --------------------------------------------------
# Paths (repo-consistent)
# --------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parents[2]

CSV_PATH = REPO_ROOT / "logs" / "csv" / "spiral_position_mission_log.csv"
OUTPUT_DIR = REPO_ROOT / "analysis" / "spiral" / "outputs"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

OUTPUT_PNG = OUTPUT_DIR / "xyz_actual_vs_reference.png"


# --------------------------------------------------
# Load telemetry CSV (actual trajectory)
# --------------------------------------------------
t = []
x_act = []
y_act = []
z_act = []

with open(CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["t"]))
        x_act.append(float(row["north_m"]))
        y_act.append(float(row["east_m"]))
        z_act.append(float(row["down_m"]))

# --------------------------------------------------
# Normalize time (mission-relative)
# --------------------------------------------------
t0 = t[0]
t_rel = [ti - t0 for ti in t]


# --------------------------------------------------
# Reference trajectory (EXPECTED path)
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
    x, y, z = trajectory.position_xyz(ti)
    x_ref.append(x)
    y_ref.append(y)
    z_ref.append(z)


# --------------------------------------------------
# Plot (3D)
# --------------------------------------------------
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")

ax.plot(
    x_ref,
    y_ref,
    z_ref,
    linestyle="--",
    linewidth=2,
    label="Reference Helical Trajectory",
)

ax.plot(
    x_act,
    y_act,
    z_act,
    linewidth=2,
    label="Actual UAV Trajectory",
)

ax.set_xlabel("North [m]")
ax.set_ylabel("East [m]")
ax.set_zlabel("Down [m]")

ax.set_title("3D Helical Trajectory — Actual vs Reference")

ax.legend()
ax.view_init(elev=25, azim=135)

plt.tight_layout()
plt.savefig(OUTPUT_PNG, dpi=200)
plt.close()

print(f"Saved 3D plot → {OUTPUT_PNG}")
