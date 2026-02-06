import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ================= CONFIG =================

CSV_PATH = Path("logs/csv/keyboard_velocity_control_20260206_202037.csv")
SAVE_DIR = Path("analysis/keyboard_velocity_control/outputs")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# ================= LOAD =================

df = pd.read_csv(CSV_PATH)

# expected columns (based on MAVSDK logging)
# timestamp, x_n, y_e, z_d, vx, vy, vz, yaw_deg

x = df["x_n"]
y = df["y_e"]
alt = -df["z_d"]   # NED → altitude

# ================= PLOT XY =================

plt.figure(figsize=(7, 7))
plt.plot(x, y, label="Trajectory", linewidth=2)
plt.scatter(x.iloc[0], y.iloc[0], c="green", s=80, label="Start")
plt.scatter(x.iloc[-1], y.iloc[-1], c="red", s=80, label="End")

plt.xlabel("North (m)")
plt.ylabel("East (m)")
plt.title("Keyboard Offboard Trajectory (Top View)")
plt.axis("equal")
plt.grid(True)
plt.legend()

plt.savefig(SAVE_DIR / "keyboard_xy_trajectory.png", dpi=200)
plt.show()

# ================= ALTITUDE =================

plt.figure(figsize=(8, 4))
plt.plot(df["timestamp"], alt, linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Altitude Profile")
plt.grid(True)

plt.savefig(SAVE_DIR / "xy_altitude.png", dpi=200)
plt.show()
