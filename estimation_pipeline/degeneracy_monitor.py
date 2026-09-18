#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry

THRESH = 2.0

class DegeneracyMonitor(Node):
    def __init__(self):
        super().__init__('degeneracy_monitor')
        self.create_subscription(Odometry, '/odometry/filtered', self.cb, 10)
        self.log = open('/home/gayathri-mohandas-k/shifting-wall-slam/degeneracy_log.txt', 'w')
        self.get_logger().info('Degeneracy monitor running')

    def cb(self, msg):
        c = np.array(msg.pose.covariance).reshape(6, 6)
        pos_cov = c[0:2, 0:2]
        info = np.linalg.inv(pos_cov + 1e-9 * np.eye(2))
        eigvals = np.linalg.eigvalsh(info)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        line = str(t) + " eig_min=" + str(eigvals.min()) + " eig_max=" + str(eigvals.max()) + "\n"
        if eigvals.min() < THRESH:
            line += str(t) + " LOCALIZATION_DEGENERACY_WARNING\n"
            self.get_logger().warning('LOCALIZATION_DEGENERACY_WARNING')
        self.log.write(line)
        self.log.flush()

def main():
    rclpy.init()
    node = DegeneracyMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
