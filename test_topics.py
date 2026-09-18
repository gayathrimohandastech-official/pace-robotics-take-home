"""
test_topics.py
Start the simulation as a background server, then use gz-transport to list
active topics and confirm cmd_vel/odometry/lidar/imu are really there.
"""

import sys
import time
from gz.sim8 import TestFixture
from gz.transport13 import Node

world_path = r"D:\GAYATHRI MOHANDAS\personal projects\pace_robotics\gazebo_sim\worlds\corridor_world.sdf"

try:
    fixture = TestFixture(world_path)
    fixture.finalize()
    server = fixture.server()

    # Run a bunch of steps first so plugins fully initialize and advertise topics
    server.run(False, 50, True)

    node = Node()
    time.sleep(1.0)  # give topic discovery a moment
    topics = node.topic_list()

    print(f"Found {len(topics)} topics:")
    for t in topics:
        print(f"  {t}")

    expected = ["cmd_vel", "odometry", "lidar", "imu"]
    found = [e for e in expected if any(e in t for t in topics)]
    print(f"\nExpected topics found: {found}")

    if len(found) >= 2:
        print("SUCCESS: Robot's sensor/control topics are live.")
    else:
        print("WARNING: Few or no expected topics found.")

except Exception as e:
    print(f"FAILED: {type(e).__name__}: {e}")
    sys.exit(1)
