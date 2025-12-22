# ✈️ Trajectory Library

This directory contains **pure mathematical trajectory generators**
used for autonomous UAV flight planning.

All trajectories are **independent of PX4, MAVSDK, and control logic**.
They only define *reference paths* as a function of time.

---

## 🎯 Design Principle

**Trajectory ≠ Mission**

A **trajectory** answers:

> Where should the UAV be at time *t*?

A **mission** decides:
- how to track the trajectory
- which control mode to use (position / velocity)
- what safety constraints apply
- how telemetry is logged

This separation reflects real-world UAV autonomy architectures  
(planner → controller → safety).

---

## 📂 Available Trajectories

### ◾ Circle Trajectory
**File:** `circle.py`

- Constant curvature motion
- Fixed altitude
- Baseline for tracking accuracy and drift analysis

Mathematical form:

```text
x(t) = cx + R · cos(ωt)
y(t) = cy + R · sin(ωt)
```


---

### ◾ Figure-8 Trajectory
**File:** `figure8.py`

- Variable curvature
- Direction changes
- Tests controller stability and crossover behavior

Mathematical form:

```text
x(t) = R · sin(ωt)
y(t) = 0.5 · R · sin(2ωt)
```

---

### ◾ Spiral (Helix) Trajectory
**File:** `spiral.py`

- Full 3D path
- Circular motion with monotonic altitude change
- Common in mapping, inspection, and search missions

Form:

```text

x(t), y(t) → circular motion
z(t) → linear altitude progression

yaml
Copy code
```


---

## 🔗 Integration

Trajectory modules:
- generate reference paths only
- contain no state
- have no PX4 or MAVSDK dependencies

They are consumed by:
- position-based autonomous missions
- safety watchdogs for drift monitoring
- telemetry analysis pipelines

---

## 🚀 Why This Matters

This structure enables:
- reusable path planning
- clean mission logic
- easier testing and extension
- future AI-based planners (vision, SLAM, RL)

