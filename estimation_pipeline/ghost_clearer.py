#!/usr/bin/env python3
"""
Ghost-obstacle clearing via ray-casting occupancy grid.
Maintains a 2D occupancy grid; each new LiDAR scan ray-casts from the
robot's estimated pose to each hit point, clearing any previously-occupied
cell the ray passes through before the hit. This removes the stale 'ghost'
of the wall panel once it physically moves.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

RES = 0.1
W, H = 300, 100
ORIGIN_X, ORIGIN_Y = -5.0, -5.0

class GhostClearer(Node):
    def __init__(self):
        super().__init__('ghost_clearer')
        self.grid = np.full((H, W), -1, dtype=np.int8)
        self.pose = (0.0, 0.0)
        self.create_subscription(Odometry, '/odometry/filtered', self.pose_cb, 10)
        self.create_subscription(PointCloud2, '/points', self.scan_cb, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_map', 10)
        self.get_logger().info('Ghost clearer running')

    def pose_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def world_to_grid(self, x, y):
        gx = int((x - ORIGIN_X) / RES)
        gy = int((y - ORIGIN_Y) / RES)
        return gx, gy

    def bresenham_clear(self, x0, y0, x1, y1):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while (x0, y0) != (x1, y1):
            if 0 <= x0 < W and 0 <= y0 < H:
                if self.grid[y0, x0] == 100:
                    self.grid[y0, x0] = 0
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def scan_cb(self, msg):
        rx, ry = self.world_to_grid(*self.pose)
        count = 0
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if count % 20 != 0:
                count += 1
                continue
            count += 1
            hx = self.pose[0] + p[0]
            hy = self.pose[1] + p[1]
            gx, gy = self.world_to_grid(hx, hy)
            self.bresenham_clear(rx, ry, gx, gy)
            if 0 <= gx < W and 0 <= gy < H:
                self.grid[gy, gx] = 100
        og = OccupancyGrid()
        og.header.frame_id = 'odom'
        og.header.stamp = self.get_clock().now().to_msg()
        og.info.resolution = RES
        og.info.width = W
        og.info.height = H
        og.info.origin.position.x = ORIGIN_X
        og.info.origin.position.y = ORIGIN_Y
        og.data = self.grid.flatten().tolist()
        self.pub.publish(og)

def main():
    rclpy.init()
    rclpy.spin(GhostClearer())

if __name__ == '__main__':
    main()
