import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = REPO_ROOT / "logs" / "circle_position_mission_log.csv"
OUT_DIR = REPO_ROOT / "analysis" / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

OUT_PNG = OUT_DIR / "circle_xy_actual_vs_reference.png"

# ---- Mission parameters (EXACT reference) ----
R = 3.0
OMEGA = 0.3

df = pd.read_csv(CSV_PATH)

t = df["t"].values
x = df["north_m"].values
y = df["east_m"].values

# Use initial position as center (same as mission)
cx = x[0]
cy = y[0]

# Ideal reference trajectory
x_ref = cx + R * np.cos(OMEGA * t)
y_ref = cy + R * np.sin(OMEGA * t)

plt.figure(figsize=(6, 6))

plt.plot(x, y, label="Actual UAV trajectory", linewidth=2)
plt.plot(x_ref, y_ref, "--", label="Reference (ideal) trajectory", alpha=0.8)

plt.scatter(x[0], y[0], c="green", s=60, label="Start")
plt.scatter(x[-1], y[-1], c="red", s=60, label="End")

plt.axis("equal")
plt.grid(True)
plt.xlabel("North [m]")
plt.ylabel("East [m]")
plt.title("Actual vs Reference XY Trajectory (Offboard Position Control)")
plt.legend()

plt.savefig(OUT_PNG, dpi=200, bbox_inches="tight")
print(f"Saved → {OUT_PNG}")
