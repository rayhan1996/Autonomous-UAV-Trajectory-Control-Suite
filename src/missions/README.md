# Mission Layer

This directory contains **mission-level autonomous flight scenarios**
built on top of the core PX4 Offboard infrastructure and trajectory planners.

A mission defines **how the UAV flies safely and autonomously**.

---

## 🧠 Mission vs Trajectory

- **Trajectory** → defines *where the UAV should be* as a function of time  
- **Mission** → defines *how the UAV executes that trajectory*

Missions combine:
- trajectory references (`src/trajectories`)
- control and safety logic (`src/core`)
- telemetry and logging tools (`src/utils`)

---

## 🧩 Mission Responsibilities

Each mission typically handles:
- PX4 connection, arming, takeoff, landing
- Offboard mode lifecycle
- trajectory execution (position or velocity control)
- safety monitoring and fail-safe behavior
- telemetry logging for post-flight analysis

---

## 📂 Available Missions

### ▶ Autonomous Figure-8 Flight  
**File:** `figure8_autonomous_flight.py`

A fully autonomous Position Offboard mission executing a smooth
figure-8 trajectory with integrated safety monitoring.

**Features:**
- position-based Offboard control
- reusable trajectory planning
- real-time safety watchdog
- telemetry logging
- automatic landing on completion or emergency

---

## 🚀 Extensibility

New missions (circle, waypoint, keyboard control, AI-guided flight)
can be added without modifying the core infrastructure.

---

**This directory represents the execution layer of the
Autonomous UAV Trajectory Control Suite.**

