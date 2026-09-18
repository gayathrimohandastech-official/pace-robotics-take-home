#!/usr/bin/env python3
import rclpy, math, random
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

SIGMA_V = 0.02
SIGMA_G = 0.005
SIGMA_B = 0.0005

class NoiseInjector(Node):
    def __init__(self):
        super().__init__('noise_injector')
        self.bias = 0.0
        self.last_t = None
        self.create_subscription(Odometry, '/model/delta/odometry', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_noisy', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_noisy', 10)
        self.get_logger().info('Noise injector running')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        s = 0.70 if 10.0 <= x <= 15.0 else 1.00
        v_true = msg.twist.twist.linear.x
        msg.twist.twist.linear.x = s * v_true + random.gauss(0, SIGMA_V)
        pc = [0.0]*36
        pc[0] = pc[7] = pc[14] = 0.05
        pc[21] = pc[28] = pc[35] = 0.05
        msg.pose.covariance = pc
        tc = [0.0]*36
        tc[0] = tc[7] = SIGMA_V**2 + 0.001
        tc[35] = 0.01
        msg.twist.covariance = tc
        self.odom_pub.publish(msg)

    def imu_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        self.bias += random.gauss(0, SIGMA_B) * math.sqrt(max(dt, 1e-3))
        msg.angular_velocity.z += self.bias + random.gauss(0, SIGMA_G)
        oc = [0.0]*9
        oc[8] = 0.02
        msg.orientation_covariance = oc
        avc = [0.0]*9
        avc[8] = SIGMA_G**2 + 0.0001
        msg.angular_velocity_covariance = avc
        msg.linear_acceleration_covariance = [0.0]*9
        self.imu_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(NoiseInjector())

if __name__ == '__main__':
    main()
