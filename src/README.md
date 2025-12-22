# 🚁 Autonomous UAV Trajectory Control Suite

A modular Python-based framework for **autonomous UAV flight control**
built on **PX4 + MAVSDK**, focusing on **trajectory planning, offboard control,
telemetry analysis, and safety-aware execution**.

This project demonstrates a clean separation between:
- trajectory planning (pure math)
- mission execution (offboard control)
- safety monitoring
- telemetry logging and analysis

Designed as a **portfolio-grade UAV autonomy project**.

---

## 🎯 Project Goals

- Design reusable **mathematical trajectory generators**
- Execute trajectories using **PX4 Offboard Position / Velocity control**
- Implement **safety-aware autonomous flight**
- Log and analyze telemetry for flight accuracy evaluation
- Provide a clean architecture aligned with real UAV autonomy systems

---

## 🧠 Core Concepts

- **Trajectory ≠ Mission**
- Trajectories define *where the UAV should be*
- Missions define *how the UAV gets there safely*
- PX4 handles low-level stabilization
- High-level planning lives outside the flight controller

---

## 📂 Repository Structure

```text
src/
├── core/ # PX4 connection, offboard helpers, safety logic
├── trajectories/ # Pure mathematical trajectory generators
│ ├── circle.py
│ ├── figure8.py
│ ├── spiral.py
│ └── README.md
├── missions/ # Executable UAV missions (offboard control)
│ ├── circle_position_mission.py
│ ├── figure8_position_mission.py
│ ├── spiral_position_mission.py
│ └── keyboard_velocity_control.py
├── utils/ # Telemetry logging, shared state, watchers
```

---

## ✈️ Trajectory Library

Available reference trajectories:
- **Circle** — constant curvature, baseline tracking
- **Figure-8** — variable curvature, direction changes
- **Spiral (Helix)** — full 3D path with altitude progression

All trajectories are:
- stateless
- PX4-independent
- reusable across missions

See `src/trajectories/README.md` for details.

---

## 🚀 Missions

### Position-Based Autonomous Missions
- Smooth trajectory tracking using **Offboard Position Control**
- Continuous setpoint streaming
- Integrated safety watchdogs
- Automatic landing on completion or emergency

Examples:
- Circular autonomous flight
- Figure-8 autonomous flight
- 3D spiral (helix) ascent/descent

### Human-in-the-Loop Control
- **Keyboard Velocity Control**
- Real-time offboard velocity commands
- Demonstrates teleoperation and manual intervention scenarios

---

## 🛡 Safety Features

- Altitude limits
- Speed limits
- Attitude (roll/pitch) bounds
- Trajectory drift monitoring
- Mission timeout protection
- Automatic landing on violation

Safety logic runs **in parallel** with mission execution.

---

## 📊 Telemetry & Analysis

- Logs position, velocity, attitude, and flight mode
- CSV output for offline analysis
- Enables trajectory tracking error and drift evaluation
- Suitable for future visualization and ML-based analysis

---

## 🧩 Technology Stack

- **PX4 Autopilot**
- **MAVSDK (Python)**
- Python 3.10+
- AsyncIO-based control loops
- Gazebo / SITL compatible

---

## ▶️ Running a Mission

Example:
```bash
python -m src.missions.circle_position_mission
```

PX4 SITL or a real PX4-based UAV must be running.

---

## 🧭 Project Status

✔ Trajectory planning library complete

✔ Position & velocity offboard missions implemented

✔ Safety and telemetry integrated

Future extensions:

- Obstacle-aware planning

- Vision-based navigation

- AI-driven trajectory generation

- SLAM / mapping integration

  ---

## 👤 Author

Developed as a portfolio and research-oriented project
focused on autonomous UAV navigation and control.

This repository is intended to demonstrate:

- UAV autonomy fundamentals

- clean software architecture

- safety-conscious flight control design

