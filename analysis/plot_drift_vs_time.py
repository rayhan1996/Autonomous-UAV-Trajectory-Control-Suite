import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = REPO_ROOT / "logs" / "circle_position_mission_log.csv"

OUT_DIR = REPO_ROOT / "analysis" / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

OUT_PNG = OUT_DIR / "drift_vs_time.png"

df = pd.read_csv(CSV_PATH)

t = df["t"].values
x = df["north_m"].values
y = df["east_m"].values

cx = np.mean(x)
cy = np.mean(y)
r = np.mean(np.sqrt((x - cx)**2 + (y - cy)**2))

drift = np.abs(np.sqrt((x - cx)**2 + (y - cy)**2) - r)

plt.figure(figsize=(8, 4))
plt.plot(t, drift, linewidth=2)
plt.grid(True)

plt.xlabel("Time [s]")
plt.ylabel("Radial Drift [m]")
plt.title("Radial Drift from Reference Circle")

plt.savefig(OUT_PNG, dpi=200, bbox_inches="tight")
print(f"Saved plot → {OUT_PNG}")
