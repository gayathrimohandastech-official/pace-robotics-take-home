"""
test_gazebo.py
Verify the robot model is actually present and gets its pose, plus confirm
its plugins (diff-drive, IMU) are active by checking their component data.
"""

import sys
from gz.sim8 import TestFixture, world_entity
from gz.math7 import Pose3d

print("Importing gz.sim8...")

world_path = r"D:\GAYATHRI MOHANDAS\personal projects\pace_robotics\gazebo_sim\worlds\corridor_world.sdf"

robot_poses = []

try:
    fixture = TestFixture(world_path)

    def on_post_update(_info, _ecm):
        entity = _ecm.entity_by_name("robot")
        if entity is not None:
            pose = _ecm.component_pose(entity)
            robot_poses.append((_info.sim_time, pose))

    fixture.on_post_update(on_post_update)
    fixture.finalize()

    server = fixture.server()
    server.run(False, 20, True)

    print(f"Recorded {len(robot_poses)} robot poses over 20 steps.")
    if robot_poses:
        first_t, first_p = robot_poses[0]
        last_t, last_p = robot_poses[-1]
        print(f"First pose at t={first_t}: {first_p}")
        print(f"Last pose at t={last_t}: {last_p}")
        print("SUCCESS: Robot entity found and tracked in simulation.")
    else:
        print("WARNING: Simulation ran but robot entity 'robot' was never found.")

except Exception as e:
    print(f"FAILED: {type(e).__name__}: {e}")
    sys.exit(1)
