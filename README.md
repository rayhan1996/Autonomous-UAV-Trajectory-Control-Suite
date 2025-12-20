# Autonomous-UAV-Trajectory-Control-Suite

A collection of **PX4 Offboard** experiments for autonomous UAV trajectory control,  
built with MAVSDK + PX4 SITL .

The goal of this project is to explore:
- Position & velocity offboard control
- Math-based trajectory generation (circle, figure-8, spiral, multi-waypoint)
- Telemetry logging and flight path visualization
- Safety logic and fail-safe behavior during autonomous flight

---

##  Features

- Modular PX4 connection & offboard controller
- Position & velocity setpoint missions
- Circle, figure-8, spiral and waypoint-based trajectories
- Real-time telemetry logging to CSV
- 2D/3D flight path plotting (matplotlib)
- Basic & advanced safety logic (speed, altitude, geofence)

---

##  Project Structure

```text
src/
  core/
    px4_connection.py      # Connects to PX4 SITL (MAVSDK)
    offboard_controller.py # Generic offboard control helpers
    safety.py              # Safety checks and fail-safe logic

  missions/
    mission_position_hold.py  # Basic position hold
    mission_multi_waypoint.py # Smooth multi-waypoint navigation
    mission_circle.py         # Circle trajectory (position-based)
    mission_figure8.py        # Figure-8 trajectory
    mission_spiral.py         # Spiral trajectory

  utils/
    telemetry_logger.py  # CSV logging utilities
    plotter.py           # Path plotting and analysis
    config.py            # Central config (ports, rates, etc.)

logs/
  telemetry/   # Recorded flight data
  plots/       # Generated figures

notebooks/
  trajectory_analysis.ipynb  # Optional: data analysis & experiments

examples/
  run_circle_demo.py
  run_figure8_demo.py

