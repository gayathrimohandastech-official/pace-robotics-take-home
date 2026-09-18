# The Shifting Wall & Slipping Base Challenge

## Overview
Simulation stress-test for a degenerate residential construction corridor, built in Gazebo Sim (headless, CPU-only physics via gz-physics-dartsim) with ROS 2 Lyrical on Ubuntu 26.04.

## Repo structure
- `worlds/corridor.sdf` — 20m corridor, two flat concrete walls, shifting wall panel, differential-drive robot with 16-channel simulated 3D LiDAR (gpu_lidar, CPU-rendered via ogre2) and 6-axis IMU
- `src/noise_injector.py` — implements the linear odometry slip model (patch-based scale factor 0.70 over x in [10,15]) and IMU gyro bias random-walk + white noise model, per the assignment's Section 3 math
- `config/ekf.yaml` — robot_localization EKF fusing noisy odom + noisy IMU
- `src/degeneracy_monitor.py` — computes the information matrix (inverse of position covariance) from the EKF's running covariance, tracks its eigenvalues, and logs `LOCALIZATION_DEGENERACY_WARNING` when the minimum eigenvalue drops below threshold
- `scripts/plot_trajectory.py` — reads the recorded bag and plots Ground Truth vs Slipping Unfused Odometry vs Fused State Estimate
- `bags/run1/` — full rosbag recording of the run (all topics)
- `degeneracy_log.txt` — timestamped eigenvalue log and degeneracy warnings
- `trajectory_plot.png` — the deliverable summary plot

## Design choices & simplifications (given time constraints)
- **EKF instead of a custom factor graph.** The assignment allows "EKF/Factor Graph pipeline." I used `robot_localization`'s EKF for time reasons; the running pose covariance matrix satisfies the deliverable directly. A factor-graph (GTSAM) implementation would let the Information Matrix be read directly off each factor's precision rather than inverted from the EKF's output covariance — noted as a natural extension.
- **Degeneracy metric is a proxy.** Rather than computing the Information Matrix of scan-matching directly (which would need a dedicated ICP/scan-matcher node), I derive it from the EKF's own position covariance (`info = inv(pos_cov)`), which captures the same qualitative signal — degraded certainty along the corridor's long axis — without a separate scan-matching pipeline.
- **Wall shift as instant teleport, not smooth slide.** The 1.5m shift at T=15s is implemented (intended) via direct pose-set rather than continuous animation; the panel modeled as a dynamic rigid body so it can be repositioned via Gazebo's `/world/.../set_pose` service.
- **Ghost-obstacle ray-cast clearing not implemented** due to time. This would be the next addition — occupancy-grid clearing via ray-casting through previously-hit cells once the panel moves.
- **Headless Gazebo (`-s`, server-only).** The GUI client repeatedly crashed under VirtualBox's software 3D rendering during this session; running server-only avoided further instability and is fully sufficient since all deliverables (bag, log, plot) are topic/file-based, not visual.
- **No live screen-recorded video.** Given the GUI instability above, the trajectory plot and rosbag serve as the primary run evidence in place of a live-capture video.

## How to reproduce
```bash
cd ~/shifting-wall-slam
gz sim worlds/corridor.sdf -r -s &
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /model/delta/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry /imu@sensor_msgs/msg/Imu[gz.msgs.IMU /points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked &
python3 src/noise_injector.py &
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf.yaml -p use_sim_time:=true &
python3 src/degeneracy_monitor.py --ros-args -p use_sim_time:=true &
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" &
ros2 bag record -a -o bags/run1
```
