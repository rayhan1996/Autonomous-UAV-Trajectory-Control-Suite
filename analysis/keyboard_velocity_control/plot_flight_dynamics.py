import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ================= CONFIG =================

CSV_PATH = Path("logs/csv/keyboard_velocity_control_20260206_202037.csv")
SAVE_DIR = Path("analysis/keyboard_velocity_control/outputs")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# ================= LOAD =================

df = pd.read_csv(CSV_PATH)

t = df["t"]

vn = df["vn_m_s"]
ve = df["ve_m_s"]
vd = df["vd_m_s"]

yaw = df["yaw_deg"]

# speed magnitude
speed = np.sqrt(vn**2 + ve**2 + vd**2)

# ================= PLOT =================

fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# --- velocities ---
axs[0].plot(t, vn, label="Vn")
axs[0].plot(t, ve, label="Ve")
axs[0].plot(t, vd, label="Vd")
axs[0].set_ylabel("Velocity (m/s)")
axs[0].set_title("Velocity Components")
axs[0].grid(True)
axs[0].legend()

# --- yaw ---
axs[1].plot(t, yaw)
axs[1].set_ylabel("Yaw (deg)")
axs[1].set_title("Heading (Yaw)")
axs[1].grid(True)

# --- speed magnitude ---
axs[2].plot(t, speed)
axs[2].set_ylabel("Speed (m/s)")
axs[2].set_title("Speed Magnitude")
axs[2].set_xlabel("Time (s)")
axs[2].grid(True)

plt.tight_layout()

# ================= SAVE =================

out_path = SAVE_DIR / "flight_dynamics_overview.png"
plt.savefig(out_path, dpi=200)
print(f"Saved → {out_path}")
