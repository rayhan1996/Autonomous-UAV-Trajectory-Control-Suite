# Trajectory Library

This directory contains **pure mathematical trajectory generators**
used for autonomous UAV flight planning.

Trajectories in this folder are **independent of PX4, MAVSDK, or flight control logic**.
They only describe *where the UAV should be* as a function of time.

This separation allows:
- clean mission design
- reusable path planning
- easier testing and analysis
- future extension to AI-based planners

---

## 📐 Design Philosophy

**Trajectory ≠ Mission**

- A **trajectory** defines a reference path:

f(t) = (x, y, z, yaw)

- A **mission** decides:
- how to follow the trajectory
- which control mode to use
- what safety constraints apply
- how telemetry is logged

Keeping trajectories pure and stateless makes the system modular,
testable, and closer to real UAV autonomy architectures.

---

## 📂 Available Trajectories

### ▶ Figure-8 Trajectory

**File:** `figure8.py`

A smooth horizontal figure-8 trajectory defined by sinusoidal functions:

- Continuous curvature
- No sharp corners
- Ideal for testing:
- trajectory tracking accuracy
- safety watchdogs
- drift and control stability

#### Mathematical Form

```text
x(t) = R · sin(ωt)
y(t) = 0.5 · R · sin(2ωt)
```


Where:
- `R` = trajectory radius
- `ω` = angular frequency

---
## 🧩 Usage Example

```python
from src.trajectories.figure8 import Figure8Trajectory

# Create a trajectory instance
trajectory = Figure8Trajectory(radius=3.0, omega=0.3)

# Query reference position at time t (seconds)
t = 1.25
x, y = trajectory.position_xy(t)

# Nominal duration of one full figure-8
T = trajectory.duration()

print(f"x={x:.2f}, y={y:.2f}, duration={T:.2f}s")

```

---

## 🔗 Integration with Mission Layer

The trajectory modules in this directory only generate **reference paths**.

The mission layer:
- consumes these trajectory references
- converts them into PX4 Offboard position setpoints
- applies safety constraints and telemetry logging

This separation allows missions to focus on execution logic,
while trajectories remain reusable and independent from flight control.

---

## 🚀 Why This Matters

This structure reflects real-world UAV autonomy systems, where:

- planners generate reference paths
- controllers track those paths
- safety systems continuously monitor deviations

It also enables future extensions such as:
- dynamic trajectory switching
- obstacle-aware planning
- AI-based path generation



