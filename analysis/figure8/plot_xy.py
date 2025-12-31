import csv
from pathlib import Path
import matplotlib.pyplot as plt

# --------------------------------------------------
# Import trajectory (single source of truth)
# --------------------------------------------------

from src.trajectories.figure8 import Figure8Trajectory


# --------------------------------------------------
# Paths (aligned with repo structure)
# --------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]

CSV_PATH = REPO_ROOT / "logs" / "csv" / "figure8_autonomous_flight_log.csv"
OUTPUT_DIR = REPO_ROOT / "analysis" / "figure8" / "outputs"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

OUTPUT_PNG = OUTPUT_DIR / "xy_actual_vs_reference.png"


# --------------------------------------------------
# Load telemetry CSV (actual trajectory)
# --------------------------------------------------

t = []
x_actual = []
y_actual = []

with open(CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["t"]))
        x_actual.append(float(row["north_m"]))
        y_actual.append(float(row["east_m"]))

# --------------------------------------------------
# Normalize time (t = 0 at mission start)
# --------------------------------------------------

t0 = t[0]
t_rel = [ti - t0 for ti in t]


# --------------------------------------------------
# Reference trajectory (from trajectory module)
# --------------------------------------------------

trajectory = Figure8Trajectory(
    radius=3.0,
    omega=0.3,
    center_x=0.0,
    center_y=0.0,
)

x_ref = []
y_ref = []

for ti in t_rel:
    x, y = trajectory.position_xy(ti)
    x_ref.append(x)
    y_ref.append(y)


# --------------------------------------------------
# Plot
# --------------------------------------------------

plt.figure(figsize=(7, 7))

plt.plot(
    x_ref,
    y_ref,
    linestyle="--",
    linewidth=2,
    label="Reference Figure-8"
)

plt.plot(
    x_actual,
    y_actual,
    linewidth=2,
    label="Actual UAV Trajectory"
)

plt.xlabel("North [m]")
plt.ylabel("East [m]")
plt.title("Figure-8 XY Trajectory — Actual vs Reference")
plt.axis("equal")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig(OUTPUT_PNG, dpi=200)
plt.close()

print(f"Saved plot → {OUTPUT_PNG}")
