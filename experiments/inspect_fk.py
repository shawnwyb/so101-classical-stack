"""Read the arm and predict its gripper pose without commanding movement."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from so101_kinematics import JOINT_CHAIN, forward_kinematics
from experiments import RESULTS_DIR


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--id", default="so101_follower")
    parser.add_argument("--save", type=Path,
                        help="Save JSON under experiments/results/ (or use an absolute path)")
    args = parser.parse_args()

    robot = SO101Follower(SO101FollowerConfig(
        port=args.port, id=args.id, use_degrees=True,
    ))
    # Open only the motor bus: robot.connect() also configures the servos.
    try:
        robot.bus.connect()
        if not robot.bus.is_calibrated:
            raise RuntimeError("Motor calibration does not match the saved file")
        readings = robot.bus.sync_read("Present_Position")
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)

    angles = {name: float(np.deg2rad(readings[name])) for name in JOINT_CHAIN[:-1]}
    pose = forward_kinematics(angles)
    for name in angles:
        print(f"{name:16s} {readings[name]:8.2f} deg")
    print(f"Gripper opening: {readings['gripper']:.2f}% (normalized reading)")
    print("\nPredicted gripper_frame_link position in base_link [mm]:")
    print(np.round(pose.T[:3, 3] * 1000, 2))
    print("\nOrientation matrix (gripper axes expressed in base coordinates):")
    print(np.round(pose.T[:3, :3], 5))
    print("\nAssumption: LeRobot arm angles equal URDF angles; offsets remain unverified.")

    if args.save:
        if not args.save.is_absolute():
            args.save = RESULTS_DIR / args.save
        result = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "robot_id": args.id,
            "port": args.port,
            "joint_angles_deg": {name: float(readings[name]) for name in angles},
            "gripper_percent": float(readings["gripper"]),
            "base_to_gripper_transform_m": pose.T.tolist(),
            "measured_position_mm": None,
            "note": "Model prediction; physical measurement and angle offsets unverified.",
        }
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_text(json.dumps(result, indent=2) + "\n")
        print(f"Saved: {args.save}")


if __name__ == "__main__":
    main()
