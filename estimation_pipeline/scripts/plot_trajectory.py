#!/usr/bin/env python3
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

bag_path = sys.argv[1] if len(sys.argv) > 1 else '/home/gayathri-mohandas-k/shifting-wall-slam/bags/run1'

def extract(topic_name):
    xs, ys = [], []
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, Odometry)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
    return xs, ys

gt_x, gt_y = extract('/model/delta/odometry')
noisy_x, noisy_y = extract('/odom_noisy')
fused_x, fused_y = extract('/odometry/filtered')

plt.figure(figsize=(12, 6))
plt.plot(gt_x, gt_y, label='Ground Truth', linewidth=2, color='green')
plt.plot(noisy_x, noisy_y, label='Slipping Unfused Odometry', linewidth=1, color='red', alpha=0.7)
plt.plot(fused_x, fused_y, label='Fused State Estimate (EKF)', linewidth=1.5, color='blue')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison: GT vs Slipping Unfused Odometry vs EKF Fused Estimate')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('/home/gayathri-mohandas-k/shifting-wall-slam/trajectory_plot.png', dpi=150)
print(f"Saved plot. GT: {len(gt_x)}, Noisy: {len(noisy_x)}, Fused: {len(fused_x)}")
