# Flight Logs

This directory contains recorded telemetry from executed UAV missions.

The logs are generated automatically during flight using MAVSDK telemetry
streams and are later used for post-flight analysis, evaluation, and
visualization inside the `/analysis` module.

These datasets provide a reproducible foundation for validating controller
performance, trajectory tracking accuracy, and vehicle behavior.

---

## 📁 Directory Structure

logs/
├── csv/ # Structured flight telemetry
└── telemetry/ # Mission events & state transitions


---

## 🧠 What Gets Logged

Each CSV file represents one complete mission run.

Data is sampled in real-time during flight and stored in **NED frame**
(North-East-Down), which is standard in PX4 and robotics research.

Typical signals include:

- vehicle position  
- linear velocity  
- attitude (roll/pitch/yaw)  
- flight mode  
- mission timing  

---

## 📄 CSV Format

Example columns:

| Column | Description |
|-------|-------------|
| `t` | mission elapsed time (s) |
| `north_m` | north position relative to home (m) |
| `east_m` | east position relative to home (m) |
| `down_m` | down position in NED (negative = altitude) |
| `vn_m_s` | north velocity (m/s) |
| `ve_m_s` | east velocity (m/s) |
| `vd_m_s` | vertical speed (m/s) |
| `roll_deg` | roll angle |
| `pitch_deg` | pitch angle |
| `yaw_deg` | heading |
| `flight_mode` | PX4 flight mode at that time |

---

## ✈ Available Mission Logs

| Mission | Description |
|--------|-------------|
| circle | constant-radius circular trajectory |
| figure8 | autonomous figure-8 tracking |
| spiral | helix climb / descent |
| keyboard_velocity_control | manual body-frame velocity control |

---

## 🔬 How These Logs Are Used

The analysis layer uses them to compute:

- trajectory plots  
- tracking error  
- drift  
- altitude profiles  
- velocity stability  
- heading evolution  

Resulting figures are saved under:

analysis/<mission>/outputs/


---

## 🧪 Reproducibility

To regenerate a log:

1. run the mission from `src/missions`
2. a new timestamped CSV will appear in `logs/csv`
3. run the corresponding plotting scripts in `/analysis`

---

## 🎯 Engineering Value

This logging architecture demonstrates:

- deterministic experiment recording  
- separation of execution vs evaluation  
- dataset-driven validation  
- research-grade traceability  

Exactly what real robotics & autonomy teams expect.


