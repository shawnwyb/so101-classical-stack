from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

import json
from pathlib import Path
import numpy as np
from so101_kinematics import JOINT_CHAIN, forward_kinematics

robot = SO101Follower(SO101FollowerConfig(
    port="/dev/ttyACM1",
    id="so101_follower",
    use_degrees=True,
))
bus = robot.bus

try:
    bus.connect()
    readings = bus.sync_read("Present_Position")
    print(readings)

    joint_angles = {
        name: float(np.deg2rad(readings[name]))
        for name in JOINT_CHAIN[:-1]
    }

    pose = forward_kinematics(joint_angles)

    print("Grasp position [mm]:", pose.T[:3, 3] * 1000)

    grasp = {
        "joint_angles_deg": readings,
        "tool_transform_m": pose.T.tolist(),
    }

    Path("grasp_pose.json").write_text(json.dumps(grasp, indent=2) + "\n")
finally:
    if bus.is_connected:
        bus.disconnect(disable_torque=False)