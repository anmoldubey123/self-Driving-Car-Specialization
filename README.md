# 🚗 Introduction to Self-Driving Cars
### Vehicle Dynamics Modeling | Coursera — University of Toronto

Python implementations of foundational vehicle dynamics models used in autonomous driving systems. These form the mathematical backbone of any self-driving stack — before planning or perception, you need to know how a vehicle actually moves.

---

## 📐 Models Implemented

### 1. Kinematic Bicycle Model

**File**: `vehicleDynamicModeling/Kinematic_Bicycle_Model.py`

Models lateral vehicle motion using a simplified two-wheel (bicycle) representation. Captures how steering angle, speed, and slip angle interact to produce a trajectory — without needing full dynamic tire force modeling.

**State variables**

| Variable | Description |
|---|---|
| `xc`, `yc` | Center of mass position (m) |
| `theta` | Heading angle (rad) |
| `delta` | Steering angle (rad) |
| `beta` | Slip angle (rad) |

**Equations of motion**

```
ẋc    = v · cos(θ + β)
ẏc    = v · sin(θ + β)
θ̇     = (v · cos(β) · tan(δ)) / L
β     = arctan((lr / L) · tan(δ))
δ̇     = ω   (clipped to ±ω_max)
```

**Trajectories validated**

- **Circle** — constant speed and steering angle, with and without slip angle correction
- **Square** — piecewise angular rate inputs at corners
- **Spiral** — high initial ω decaying to small negative
- **Wave** — square wave ω input
- **Figure-8** — binary solver used to find `delta_star` for exact curvature at radius R; time-scheduled left/right arc segments

The figure-8 trajectory is the graded assignment output, exported to `figure8.txt` for autograder submission.

---

### 2. Longitudinal Vehicle Model

**File**: `vehicleDynamicModeling/Longitudinal_Vehicle_Model.py`

Models the fore-aft dynamics of a vehicle — how throttle input, road grade, aerodynamic drag, rolling resistance, and tire slip determine acceleration and position over time.

**State variables**

| Variable | Description |
|---|---|
| `x` | Position (m) |
| `v` | Longitudinal velocity (m/s) |
| `a` | Acceleration (m/s²) |
| `w_e` | Engine angular speed (rad/s) |
| `w_e_dot` | Engine angular acceleration (rad/s²) |

**Model pipeline (each timestep)**

```
1. Engine torque:     Te = throttle · (a₀ + a₁·ωe + a₂·ωe²)
2. Wheel speed:       ωw = GR · ωe
3. Slip ratio:        s  = (ωw·re − v) / v
4. Tire force:        Fx = c·s  if |s| < 1, else Fmax·sign(s)
5. Load forces:       Faero = ca·v²,  Rx = cr1·v,  Fg = m·g·sin(α)
6. Engine dynamics:   Je·ω̇e = Te − GR·re·Fload
7. Longitudinal accel: m·ẍ  = Fx − Fload
```

**Scenarios simulated**

- **Constant throttle** — velocity converges to terminal speed as drag and tire force balance
- **Downhill coast** — gravity accelerates vehicle to a drag-limited terminal velocity
- **Ramp profile** — piecewise throttle ramp (0.2 → 0.5 → 0) over a two-segment incline (3m rise over 60m, then 9m rise over 90m); position exported to `xdata.txt`

---

## 🧮 Key Concepts

**Kinematic vs. Dynamic modeling** — The bicycle model is *kinematic*: it assumes the tires always point the way the wheel is steered (no force modeling). The longitudinal model is *dynamic*: it explicitly computes tire slip forces and engine torque.

**Slip angle (β)** — The offset between where the vehicle is pointed and where it's actually going due to lateral tire deformation. Even in the kinematic model, β introduces a trajectory offset visible in the circular path test.

**Slip ratio (s)** — The mismatch between wheel rotational speed and vehicle speed. Drives the tire force calculation; above `|s| = 1` the tire saturates at `F_max`.

**Euler integration** — Both models use forward Euler (`state += derivative · dt`) at `dt = 0.01s` (10ms sample rate), consistent with real-time automotive control loops.

**Steering rate control** — The bicycle model accepts angular rate `ω` rather than direct steering angle, requiring a rate-limited command to drive `delta → delta_des`, mirroring real actuator constraints.

---

## 🚀 Running the Code

**Dependencies**
```bash
pip install numpy matplotlib
```

**Run**
```bash
python vehicleDynamicModeling/Kinematic_Bicycle_Model.py
python vehicleDynamicModeling/Longitudinal_Vehicle_Model.py
```

Each script produces matplotlib trajectory plots inline. The bicycle model also exports `figure8.txt` and the longitudinal model exports `xdata.txt`.

> **Note**: The bicycle model imports `notebook_grader` (course autograder module) for the graded figure-8 section. This module is provided by the Coursera course environment. All other code runs independently.

---

## 📁 Structure

```
introToSelfDrivingCars/
└── vehicleDynamicModeling/
    ├── Kinematic_Bicycle_Model.py    # Lateral dynamics, figure-8 trajectory
    ├── Longitudinal_Vehicle_Model.py # Throttle/grade/engine dynamics
    ├── figure8.txt                   # Grader output: [t, v, w] trajectory
    └── xdata.txt                     # Grader output: [t, x] position trace
```

---

## 📝 Notes

- Part of the **Self-Driving Cars Specialization** — University of Toronto / Coursera
- Models written in pure NumPy — no external dynamics libraries
- Designed as the mathematical foundation before adding controllers (PID, MPC) and state estimators (Kalman filtering) in later courses

---

*Part of my engineering portfolio — covers the vehicle modeling layer that sits beneath planning, perception, and control in a full autonomous driving stack.*
