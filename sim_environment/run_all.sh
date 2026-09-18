#!/bin/bash
cd ~/shifting-wall-slam
LOGDIR=~/shifting-wall-slam/logs

pkill -f "gz sim" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "noise_injector" 2>/dev/null
pkill -f "ekf_node" 2>/dev/null
pkill -f "degeneracy_monitor" 2>/dev/null
pkill -f "wall_shifter" 2>/dev/null
pkill -f "ghost_clearer" 2>/dev/null
pkill -f "topic pub" 2>/dev/null
sleep 2

nohup gz sim worlds/corridor.sdf -r > $LOGDIR/gazebo.log 2>&1 &
sleep 6

nohup ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /model/delta/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  > $LOGDIR/bridge.log 2>&1 &
sleep 3

nohup python3 src/noise_injector.py > $LOGDIR/noise.log 2>&1 &
sleep 1

nohup ros2 run robot_localization ekf_node --ros-args --params-file config/ekf.yaml -p use_sim_time:=true > $LOGDIR/ekf.log 2>&1 &
sleep 2

nohup python3 src/degeneracy_monitor.py --ros-args -p use_sim_time:=true > $LOGDIR/degeneracy.log 2>&1 &
sleep 1

nohup python3 src/wall_shifter.py --ros-args -p use_sim_time:=true > $LOGDIR/wallshift.log 2>&1 &
sleep 1

nohup python3 src/ghost_clearer.py --ros-args -p use_sim_time:=true > $LOGDIR/ghost.log 2>&1 &
sleep 1

nohup ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" > $LOGDIR/cmdvel.log 2>&1 &

echo "All processes launched."
