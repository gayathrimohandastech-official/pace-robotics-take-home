#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess

class WallShifter(Node):
    def __init__(self):
        super().__init__('wall_shifter')
        self.shifted = False
        self.timer = self.create_timer(1.0, self.check)
        self.start_time = self.get_clock().now()
        self.get_logger().info('Wall shifter running - shifts panel every 15s')

    def check(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= 15.0 and not self.shifted:
            self.shift_panel(reverse=False)
            self.shifted = True
            self.start_time = self.get_clock().now()
        elif elapsed >= 15.0 and self.shifted:
            self.shift_panel(reverse=True)
            self.shifted = False
            self.start_time = self.get_clock().now()

    def shift_panel(self, reverse):
        y = 0.7 if reverse else (0.7 - 1.5)
        req = f'name: "shifting_panel", position: {{x: 10, y: {y}, z: 1}}'
        subprocess.run([
            'gz', 'service', '-s', '/world/corridor_world/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000', '--req', req
        ])
        self.get_logger().info(f'Panel shifted: reverse={reverse}, new y={y}')

def main():
    rclpy.init()
    rclpy.spin(WallShifter())

if __name__ == '__main__':
    main()
