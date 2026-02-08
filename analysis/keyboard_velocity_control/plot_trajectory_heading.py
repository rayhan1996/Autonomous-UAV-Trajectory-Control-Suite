import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ================= CONFIG =================

CSV_PATH = Path("logs/csv/keyboard_velocity_control_20260206_202037.csv")
SAVE_DIR = Path("analysis/keyboard_velocity_control/outputs")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# how often to draw arrows (reduce clutter)
ARROW_EVERY = 20

# ================= LOAD =================

df = pd.read_csv(CSV_PATH)

x = df["north_m"]
y = df["east_m"]
yaw_deg = df["yaw_deg"]

# convert yaw → radians
yaw = np.deg2rad(yaw_deg)

# ================= PLOT =================

plt.figure(figsize=(8, 8))

# trajectory line
plt.plot(x, y, linewidth=2, label="Trajectory")

# start & end
plt.scatter(x.iloc[0], y.iloc[0], s=100, label="Start")
plt.scatter(x.iloc[-1], y.iloc[-1], s=100, label="End")

# heading arrows
plt.quiver(
    x[::ARROW_EVERY],
    y[::ARROW_EVERY],
    np.cos(yaw[::ARROW_EVERY]),
    np.sin(yaw[::ARROW_EVERY]),
    angles="xy",
    scale_units="xy",
    scale=1,
    width=0.003,
)

plt.xlabel("North (m)")
plt.ylabel("East (m)")
plt.title("Trajectory with Heading Direction")
plt.axis("equal")
plt.grid(True)
plt.legend()

# ================= SAVE =================

out_path = SAVE_DIR / "trajectory_with_heading.png"
plt.savefig(out_path, dpi=200)
print(f"Saved → {out_path}")
