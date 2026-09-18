"""
test_move2.py
Same as test_move.py but with longer discovery wait and step-by-step
diagnostics, to isolate whether this is a timing issue or something deeper.
"""

import sys
import time
from gz.sim8 import TestFixture
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.odometry_pb2 import Odometry

world_path = r"D:\GAYATHRI MOHANDAS\personal projects\pace_robotics\gazebo_sim\worlds\corridor_world.sdf"

odom_readings = []


def odom_cb(msg):
    odom_readings.append((msg.pose.position.x, msg.pose.position.y))
    print(f"  odom callback fired: x={msg.pose.position.x:.4f}")


try:
    fixture = TestFixture(world_path)
    fixture.finalize()
    server = fixture.server()
    server.run(False, 10, True)

    node = Node()

    print("Subscribing to /odometry...")
    sub_ok = node.subscribe(Odometry, "/odometry", odom_cb)
    print(f"Subscribe call returned: {sub_ok}")

    print("Advertising /cmd_vel...")
    pub = node.advertise("/cmd_vel", Twist)

    print("Waiting 3s for topic discovery...")
    time.sleep(3.0)

    print("Checking subscribed topics list...")
    print(f"Node topic list: {node.topic_list()}")

    twist = Twist()
    twist.linear.x = 0.4
    twist.angular.z = 0.0

    print("Publishing forward velocity command and stepping simulation (200 iterations)...")
    for i in range(200):
        pub.publish(twist)
        server.run(False, 5, True)
        time.sleep(0.02)
        if i % 50 == 0:
            print(f"  iteration {i}, odom readings so far: {len(odom_readings)}")

    print(f"\nFinal: Collected {len(odom_readings)} odometry readings.")
    if odom_readings:
        print(f"First odom position: {odom_readings[0]}")
        print(f"Last odom position: {odom_readings[-1]}")
        moved = abs(odom_readings[-1][0] - odom_readings[0][0])
        print(f"Distance moved in X: {moved:.3f} m")
        if moved > 0.1:
            print("SUCCESS: Robot genuinely moved via real physics + diff-drive control.")
        else:
            print("WARNING: Robot did not move significantly.")
    else:
        print("WARNING: No odometry readings received even with longer wait.")

except Exception as e:
    print(f"FAILED: {type(e).__name__}: {e}")
    sys.exit(1)
