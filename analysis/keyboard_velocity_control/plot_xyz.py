import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from mpl_toolkits.mplot3d import Axes3D

# ================= CONFIG =================
CSV_PATH = Path("logs/csv/keyboard_velocity_control_20260206_202037.csv")
SAVE_DIR = Path("analysis/keyboard_velocity_control/outputs")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# ================= LOAD CSV =================
df = pd.read_csv(CSV_PATH)

# ======= column =======
x = df["north_m"]
y = df["east_m"]
z = -df["down_m"]  # NED → altitude

# ================= PLOT 3D =================
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z, label="Trajectory", linewidth=2, color='blue')
ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], c="green", s=80, label="Start")
ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], c="red", s=80, label="End")

ax.set_xlabel("North (m)")
ax.set_ylabel("East (m)")
ax.set_zlabel("Altitude (m)")
ax.set_title("Keyboard Offboard Trajectory (3D)")
ax.legend()
plt.grid(True)

# save 3D plot
plt.savefig(SAVE_DIR / "keyboard_3d_trajectory.png", dpi=200)
plt.close()
