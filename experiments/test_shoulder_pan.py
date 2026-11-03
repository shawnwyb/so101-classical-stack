"""Small shoulder-pan experiment. Run beside the arm; support the other joints."""

import argparse
import json
import time
from datetime import datetime, timezone

import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from so101_kinematics import JOINT_CHAIN, forward_kinematics
from experiments import RESULTS_DIR


def tool_position(readings):
    angles = {name: np.deg2rad(readings[name]) for name in JOINT_CHAIN[:-1]}
    return forward_kinematics(angles).T[:3, 3] * 1000


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    robot = SO101Follower(SO101FollowerConfig(
        port="/dev/ttyACM1", id="so101_follower", use_degrees=True,
    ))
    bus = robot.bus
    joint = "shoulder_pan"
    enabled = False
    samples = []
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError("Calibration mismatch")
        if bus.read("Operating_Mode", joint) != 0:
            raise RuntimeError("Shoulder pan must be in position mode")
        if any(bus.sync_read("Torque_Enable").values()):
            raise RuntimeError("Start with all motors released; support the arm")

        input("Support the arm; clear 5 degrees of clockwise travel. Enter to read start: ")
        before = bus.sync_read("Present_Position")
        start = before[joint]
        target = start + 5.0
        # Conservative pan envelope inside the URDF limits (approximately +/-110 deg).
        if not (-100 < start < 100 and -100 < target < 100):
            raise RuntimeError("Shoulder pan is too close to its travel limits")
        expected = dict(before, shoulder_pan=target)
        displacement = tool_position(expected) - tool_position(before)
        print(f"Pan: {start:.2f} -> {target:.2f} degrees (clockwise from above)")
        print("Predicted tool displacement [mm]:", np.round(displacement, 2))
        print(f"Predicted straight-line travel: {np.linalg.norm(displacement):.1f} mm")
        input("Mark the initial fingertip location. Enter to move; Ctrl+C to cancel: ")

        # Refuse to enable if the supported pose shifted while marking it.
        current = bus.sync_read("Present_Position")
        if any(abs(current[name] - before[name]) > 2 for name in JOINT_CHAIN[:-1]):
            raise RuntimeError("Pose changed by more than 2 degrees; restart")
        bus.write("Goal_Position", joint, current[joint])
        enabled = True
        bus.enable_torque(joint)

        def move_to(destination):
            initial = bus.read("Present_Position", joint)
            for step in range(1, 61):
                command = initial + (destination - initial) * step / 60
                bus.write("Goal_Position", joint, command)
                actual = bus.read("Present_Position", joint)
                samples.append({"time": time.time(), "command_deg": command, "actual_deg": actual})
                if abs(command - actual) > 3:
                    raise RuntimeError("Pan tracking error exceeded 3 degrees")
                time.sleep(0.05)
            time.sleep(0.5)

        move_to(target)
        after = bus.sync_read("Present_Position")
        print(f"Measured pan change: {after[joint] - start:.2f} degrees")
        print("Other arm-joint changes [deg]:", {
            name: round(after[name] - before[name], 2) for name in JOINT_CHAIN[1:-1]
        })
        result = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "before": before, "after": after,
            "predicted_tool_displacement_mm": displacement.tolist(),
            "physical_displacement_mm": None, "tracking": samples,
        }
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        path = RESULTS_DIR / f"shoulder_pan_{stamp}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2) + "\n")
        print(f"Saved {path}. Mark the final fingertip location and measure the distance.")
        input("Enter to return slowly to the starting pan angle: ")
        move_to(start)
    finally:
        if bus.is_connected:
            try:
                if enabled:
                    bus.disable_torque(joint)
            finally:
                bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
