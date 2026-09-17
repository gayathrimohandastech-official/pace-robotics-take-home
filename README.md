# Pace Robotics Take-Home: The Shifting Wall & Slipping Base Challenge

## Important note on environment: Gazebo/Isaac Sim substitution

This assignment specified Gazebo or Isaac Sim as the simulation environment.
Given a hard same-day deadline combined with a fresh Windows 11 machine where
WSL2 installation failed on a Windows component-store corruption issue
(DISM error 0x800f0915) that would have required a full OS repair to fix
safely, I made the deliberate decision to implement the complete assignment
as a Python-only kinematic simulation instead of losing hours to an
uncertain environment fix.

**Everything the assignment actually asks you to evaluate is implemented in
full**: the corridor/wall-shift scenario, both noise injection models exactly
as specified, EKF sensor fusion, degeneracy-aware scan-matching information
tracking, and ghost-obstacle map clearing. What's substituted is the physics
engine and rendering layer (Gazebo), not the algorithmic work.

If given more time, the natural next step would be porting this pipeline's
noise/EKF/diagnostic modules directly into ROS2 nodes running against a real
Gazebo world with an actual LiDAR plugin and scan-matching library (e.g.
`scan_matcher` or NDT/ICP via PCL) -- the module boundaries here were kept
clean specifically to make that port straightforward.

## What's implemented

- **`sim/plant.py`** -- Ground-truth kinematic simulation: 20m corridor,
  differential-drive robot at constant velocity, wall panel shifting ±1.5m
  every 15s.
- **`noise/inject.py`** -- Both noise models from Section 3, implemented
  exactly as specified:
  - Linear odometry slip: `v_recorded(t) = s(t) * v_true(t) + w_v(t)`,
    with `s(t) = 0.70` for `10 <= X <= 15`.
  - IMU bias drift: `omega_measured(t) = omega_true(t) + b(t) + eta_g(t)`,
    with `b_dot(t) = eta_b(t)` (random walk).
- **`estimation/scan_info.py`** -- Synthetic scan-matching information
  matrix. Models the real physics of a featureless corridor: strong Y/theta
  constraint from parallel walls, near-zero X constraint except near the
  wall panel's edge, which locally restores it. This is the Python-fallback
  substitute for real LiDAR scan-matching (see substitution note above).
- **`estimation/ekf.py`** -- Extended Kalman Filter fusing noisy odometry +
  IMU (predict step) with the scan-matching pseudo-measurement (update step),
  using the information matrix as the measurement's inverse covariance.
- **`estimation/diagnostics.py`** -- Degeneracy diagnostic based on the
  scan-matching information matrix's smallest eigenvalue (per the
  assignment's Task 2 wording), logging `LOCALIZATION_DEGENERACY_WARNING`
  when an axis is weakly constrained. Deliberately NOT based on the EKF's
  own fused covariance, which naturally shrinks even under weak individual
  measurements and would mask the real degeneracy.
- **`estimation/mapping.py`** -- Ghost-obstacle clearing: simulates a LiDAR
  sweep at the wall panel's location each step, marking stale occupied cells
  free once the panel moves.
- **`scripts/run_pipeline.py`** -- Runs all stages end-to-end.
- **`scripts/generate_outputs.py`** / **`scripts/generate_video.py`** --
  Produce the plot, log, and video deliverables.

## Robot URDF and simulation world (design/reference artifacts)

Per the deliverables list, `robot_description/differential_drive_robot.urdf`
and `world/corridor_world.sdf` are included. **These are design specifications,
not executed simulations** -- consistent with the Gazebo substitution
explained above, they were not run in Gazebo for this submission. They
specify exactly the robot (diff-drive, 16-ch LiDAR, 6-axis IMU) and
environment (20m corridor, wall panel shifting ±1.5m at X=10) that the
Python simulation's parameters already model numerically, and are intended
as the direct starting point for the ROS2/Gazebo port described above.

## Results

- Final position error, fused estimate: **0.127 m** (over a 20m run)
- Final position error, raw slipping odometry alone: **1.500 m**
- Degeneracy correctly flagged for **56.5%** of the trajectory (the
  featureless sections), with the X axis correctly identified as the weak
  axis throughout
- 16 ghost-obstacle clear events correctly logged at both wall-shift times

## How to run

```powershell
python -m venv venv
venv\Scripts\activate
pip install numpy scipy matplotlib filterpy
python scripts\generate_outputs.py
python scripts\generate_video.py
```

Outputs land in `output/`: `trajectory_plot.png`, `run_log.txt`,
`simulation_video.gif` (falls back to GIF automatically if ffmpeg isn't
installed; MP4 otherwise).