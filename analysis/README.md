# 📊 Flight Analysis & Evaluation Layer

This directory contains the **post-flight analytics framework** for the  
**Autonomous UAV Trajectory Control Suite**.

It transforms raw telemetry logs into quantitative insights, performance
metrics, and professional visualizations suitable for:

- engineering validation  
- controller benchmarking  
- research reporting  
- recruiter / portfolio demonstration  

---

##  Objectives

After each mission, we want to answer:

- Did the UAV follow the reference trajectory correctly?
- How large was the tracking error?
- Was the motion smooth and dynamically feasible?
- Did altitude, velocity, and yaw remain stable?
- How does performance vary between missions?

---

##  What is Analyzed?

We evaluate:

✅ geometric tracking accuracy  
✅ drift over time  
✅ velocity consistency  
✅ altitude regulation  
✅ yaw / heading behavior  
✅ 3D path quality  

---

## Data Source

All plots are generated from CSV logs recorded during flight via MAVSDK.

Typical fields include:

north_m, east_m, down_m
vn_m_s, ve_m_s, vd_m_s
roll_deg, pitch_deg, yaw_deg
flight_mode, mission_phase


---

## Directory Structure

analysis/
├── circle/
├── figure8/
├── spiral/
├── keyboard_velocity_control/
└── outputs/


Each mission folder contains:

- plotting scripts  
- generated figures  
- mission-specific comparisons  

---

---

# Missions Overview

---

## Circle Mission

**Goal:** Validate constant-radius position tracking and phase alignment.

### Key Evaluations
- actual vs reference XY path  
- drift vs time  
- circular symmetry  

### Demonstrates
- precise position control  
- correct offboard reference timing  
- proper spatial initialization  

---

## Figure-8 Mission

**Goal:** Test controller behavior on curvature reversals and dynamic transitions.

### Key Evaluations
- crossover accuracy  
- symmetry  
- time series stability  

### Demonstrates
- handling of aggressive direction changes  
- smooth reference tracking  
- robustness of watchdog & offboard lifecycle  

---

## Spiral (Helix) Mission

**Goal:** Evaluate simultaneous lateral + vertical tracking.

### Key Evaluations
- 3D path reconstruction  
- altitude evolution  
- drift accumulation  

### Demonstrates
- coordinated motion in 3D  
- altitude consistency  
- long-duration stability  

---

## Keyboard Velocity Control

**Goal:** Human-in-the-loop validation of body-frame velocity commands.

### Generated Analytics

- XY trajectory  
- altitude profile  
- heading along path  
- 3D reconstruction  
- flight dynamics overview (velocities + attitude)

### Demonstrates
- real-time offboard control  
- coordinate transformations (Body → NED)  
- responsiveness to commands  
- suitability for tele-operation / shared autonomy  

---

---

# 📈 Example Output Types

Depending on the mission, the framework can produce:

- top-view trajectory maps  
- reference vs actual overlays  
- drift curves  
- velocity time series  
- yaw evolution  
- altitude regulation  
- 3D flight paths  
- combined dynamics dashboards  

All figures are publication-ready.

---

---

# Methodology

1. Record telemetry during flight  
2. Parse CSV into structured arrays  
3. reconstruct position & orientation  
4. compute derived metrics  
5. generate high-resolution plots  
6. export into version-controlled artifacts  

This ensures experiments are:

✅ reproducible  
✅ comparable  
✅ reviewable  
✅ suitable for academic or industrial audit  

---

---

# Skills Demonstrated

This layer showcases practical ability in:

- robotics data pipelines  
- autonomy validation  
- PX4 + MAVSDK integration  
- coordinate systems (NED / body frames)  
- experiment reproducibility  
- scientific visualization  
- post-mission diagnostics  

---

---

# How to Reproduce a Plot

Example:

```bash
python3 analysis/keyboard_velocity_control/plot_xy.py
```

Outputs are saved automatically into:

analysis/<mission>/outputs/

 Why This Matters

Flying is only half of autonomy.

Understanding and proving how well the vehicle flew is what makes a
system engineering-grade.

This directory provides that proof.


