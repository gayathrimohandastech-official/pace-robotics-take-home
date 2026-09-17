\# Pace Robotics Take-Home: The Shifting Wall \& Slipping Base Challenge



\## Important note on environment: Gazebo/Isaac Sim substitution



This assignment specified Gazebo or Isaac Sim as the simulation environment.

Given a hard same-day deadline combined with a fresh Windows 11 machine where

WSL2 installation failed on a Windows component-store corruption issue

(DISM error 0x800f0915) that would have required a full OS repair to fix

safely, I made the deliberate decision to implement the complete assignment

as a Python-only kinematic simulation instead of losing hours to an

uncertain environment fix.



\*\*Everything the assignment actually asks you to evaluate is implemented in

full\*\*: the corridor/wall-shift scenario, both noise injection models exactly

as specified, EKF sensor fusion, degeneracy-aware scan-matching information

tracking, and ghost-obstacle map clearing. What's substituted is the physics

engine and rendering layer (Gazebo), not the algorithmic work.



If given more time, the natural next step would be porting this pipeline's

noise/EKF/diagnostic modules directly into ROS2 nodes running against a real

Gazebo world with an actual LiDAR plugin and scan-matching library (e.g.

`scan\_matcher` or NDT/ICP via PCL) -- the module boundaries here were kept

clean specifically to make that port straightforward.



\## What's implemented



\- \*\*`sim/plant.py`\*\* -- Ground-truth kinematic simulation: 20m corridor,

&#x20; differential-drive robot at constant velocity, wall panel shifting ±1.5m

&#x20; every 15s.

\- \*\*`noise/inject.py`\*\* -- Both noise models from Section 3, implemented

&#x20; exactly as specified:

&#x20; - Linear odometry slip: `v\_recorded(t) = s(t) \* v\_true(t) + w\_v(t)`,

&#x20;   with `s(t) = 0.70` for `10 <= X <= 15`.

&#x20; - IMU bias drift: `omega\_measured(t) = omega\_true(t) + b(t) + eta\_g(t)`,

&#x20;   with `b\_dot(t) = eta\_b(t)` (random walk).

\- \*\*`estimation/scan\_info.py`\*\* -- Synthetic scan-matching information

&#x20; matrix. Models the real physics of a featureless corridor: strong Y/theta

&#x20; constraint from parallel walls, near-zero X constraint except near the

&#x20; wall panel's edge, which locally restores it. This is the Python-fallback

&#x20; substitute for real LiDAR scan-matching (see substitution note above).

\- \*\*`estimation/ekf.py`\*\* -- Extended Kalman Filter fusing noisy odometry +

&#x20; IMU (predict step) with the scan-matching pseudo-measurement (update step),

&#x20; using the information matrix as the measurement's inverse covariance.

\- \*\*`estimation/diagnostics.py`\*\* -- Degeneracy diagnostic based on the

&#x20; scan-matching information matrix's smallest eigenvalue (per the

&#x20; assignment's Task 2 wording), logging `LOCALIZATION\_DEGENERACY\_WARNING`

&#x20; when an axis is weakly constrained. Deliberately NOT based on the EKF's

&#x20; own fused covariance, which naturally shrinks even under weak individual

&#x20; measurements and would mask the real degeneracy.

\- \*\*`estimation/mapping.py`\*\* -- Ghost-obstacle clearing: simulates a LiDAR

&#x20; sweep at the wall panel's location each step, marking stale occupied cells

&#x20; free once the panel moves.

\- \*\*`scripts/run\_pipeline.py`\*\* -- Runs all stages end-to-end.

\- \*\*`scripts/generate\_outputs.py`\*\* / \*\*`scripts/generate\_video.py`\*\* --

&#x20; Produce the plot, log, and video deliverables.



\## Results



\- Final position error, fused estimate: \*\*0.127 m\*\* (over a 20m run)

\- Final position error, raw slipping odometry alone: \*\*1.500 m\*\*

\- Degeneracy correctly flagged for \*\*56.5%\*\* of the trajectory (the

&#x20; featureless sections), with the X axis correctly identified as the weak

&#x20; axis throughout

\- 16 ghost-obstacle clear events correctly logged at both wall-shift times



\## How to run



```powershell

python -m venv venv

venv\\Scripts\\activate

pip install numpy scipy matplotlib filterpy

python scripts\\generate\_outputs.py

python scripts\\generate\_video.py

```



Outputs land in `output/`: `trajectory\_plot.png`, `run\_log.txt`,

`simulation\_video.gif` (falls back to GIF automatically if ffmpeg isn't

installed; MP4 otherwise).

