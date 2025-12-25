import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# --------------------------------------------------
# Paths
# --------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = REPO_ROOT / "logs" / "circle_position_mission_log.csv"

OUT_DIR = REPO_ROOT / "analysis" / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

OUT_PNG = OUT_DIR / "circle_xy_trajectory.png"

# --------------------------------------------------
# Load telemetry
# --------------------------------------------------

df = pd.read_csv(CSV_PATH)

# Columns from your CSV:
# t,north_m,east_m,down_m,vn_m_s,ve_m_s,vd_m_s,roll_deg,pitch_deg,yaw_deg,flight_mode

x = df["north_m"]
y = df["east_m"]

# --------------------------------------------------
# Plot XY trajectory
# --------------------------------------------------

plt.figure(figsize=(6, 6))

plt.plot(x, y, linewidth=2, label="UAV trajectory")
plt.scatter(x.iloc[0], y.iloc[0], color="green", s=60, label="Start")
plt.scatter(x.iloc[-1], y.iloc[-1], color="red", s=60, label="End")

plt.axis("equal")
plt.grid(True)

plt.xlabel("North [m]")
plt.ylabel("East [m]")
plt.title("Circular Offboard Trajectory (PX4 + MAVSDK)")

plt.legend()

# --------------------------------------------------
# Save
# --------------------------------------------------

plt.savefig(OUT_PNG, dpi=200, bbox_inches="tight")
print(f"Saved plot → {OUT_PNG}")

plt.show()
