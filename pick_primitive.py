"""Pick, lift, and return using a taught grasp and smooth joint trajectories."""

import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from experiments import RESULTS_DIR
from experiments.test_trajectory_tracking import execute, make_trajectory
from gripper_control import release_gripper
from so101_kinematics import (
    JOINT_CHAIN, calibrated_joint_limits, forward_kinematics, inverse_kinematics,
)

ARM_JOINTS = JOINT_CHAIN[:-1]
ALL_MOTORS = (*ARM_JOINTS, "gripper")
SAMPLE_PERIOD = 0.02
MAX_VELOCITY = 60       # degrees/second; trial setting, not a motor safety rating
MAX_ACCELERATION = 60  # degrees/second²; trial setting
ELBOW_P = 32
APPROACH_HEIGHT = 0.03  # metres above the taught grasp
LIFT_HEIGHT = 0.20      # metres above the taught grasp
DURATION = 1.0         # minimum seconds per arm segment
OPEN_PERCENT = 70
GRASP_FILE = Path(__file__).with_name("grasp_pose.json")


def read_joint_angles(bus):
    """Read arm angles and convert LeRobot degrees to our math's radians."""
    readings = bus.sync_read("Present_Position")
    return {name: float(np.deg2rad(readings[name])) for name in ARM_JOINTS}


def build_trajectory(start_angles, end_angles, minimum_duration):
    """Build smooth motion, extending duration if speed/acceleration require it."""
    start = np.array([start_angles[name] for name in ARM_JOINTS])
    end = np.array([end_angles[name] for name in ARM_JOINTS])
    largest_change = np.max(np.abs(np.rad2deg(end - start)))
    time_for_speed = 1.875 * largest_change / MAX_VELOCITY
    time_for_acceleration = np.sqrt((10 / np.sqrt(3)) * largest_change / MAX_ACCELERATION)
    duration = max(minimum_duration, time_for_speed, time_for_acceleration) * 1.001
    return make_trajectory(
        start, end, duration, SAMPLE_PERIOD, MAX_VELOCITY, MAX_ACCELERATION,
    )


def plan_waypoints(grasp_file, duration):
    """Load the grasp and solve the configured approach and lift poses."""
    saved = json.loads(grasp_file.read_text())
    close_percent = saved.get("gripper_percent")
    if close_percent is None or not np.isfinite(close_percent) or not 0 <= close_percent < OPEN_PERCENT:
        raise ValueError("Teach a grasp with gripper_percent below OPEN_PERCENT using experiments.inspect_fk")
    close_percent = float(close_percent)
    grasp = {name: float(np.deg2rad(saved["joint_angles_deg"][name])) for name in ARM_JOINTS}
    grasp_position = forward_kinematics(grasp).T[:3, 3]
    approach_position = grasp_position + [0, 0, APPROACH_HEIGHT]
    solution = inverse_kinematics(approach_position, grasp)
    if not solution["success"]:
        raise RuntimeError("Approach IK failed")
    approach = solution["joint_angles"]
    build_trajectory(approach, grasp, duration)  # Check the planned descent limits.
    lift_solution = inverse_kinematics(grasp_position + [0, 0, LIFT_HEIGHT], approach)
    if not lift_solution["success"]:
        raise RuntimeError(f"{LIFT_HEIGHT * 100:g} cm lift IK failed")
    lift = lift_solution["joint_angles"]
    build_trajectory(grasp, lift, duration)  # Also validates the reverse lowering path.
    relative_rotation = forward_kinematics(grasp).T[:3, :3].T @ forward_kinematics(lift).T[:3, :3]
    orientation_change = np.rad2deg(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1)))
    print(f"Approach: {APPROACH_HEIGHT * 1000:g} mm; lift: {LIFT_HEIGHT * 1000:g} mm above grasp; "
          f"lift IK error {lift_solution['error_m'] * 1000:.2f} mm")
    print(f"Position-only IK changes gripper orientation by {orientation_change:.2f} deg during lift")
    print(f"Taught gripper close: {close_percent:.2f}%")
    return grasp, approach, lift, close_percent


def move_to(bus, target_angles, label, duration, trial):
    """Move from measured angles, recording tracking even if execution fails."""
    motion = build_trajectory(read_joint_angles(bus), target_angles, duration)
    record = {"label": label, "duration_s": float(motion["time_s"][-1]), "samples": []}
    trial["phases"].append(record)
    print(f"Moving: {label} ({record['duration_s']:.1f} s)")
    execute(bus, motion, record, settle_time=0, verbose=False)
    record["actual_tool_position_m"] = forward_kinematics(read_joint_angles(bus)).T[:3, 3].tolist()


def set_gripper(bus, percent, label, trial):
    """Send a position target without changing motor effort or protection settings."""
    before = bus.read("Present_Position", "gripper")
    bus.write("Goal_Position", "gripper", percent)
    time.sleep(0.5)
    record = {
        "phase": label, "before_percent": before, "command_percent": percent,
        "actual_percent": bus.read("Present_Position", "gripper"),
    }
    trial["gripper"].append(record)
    print(f"Gripper {label}: {before:.1f}% -> {record['actual_percent']:.1f}% (target {percent:.1f}%)")


def prepare_motors(bus, approach, session, log):
    """Connect, save settings, and enable holding at the current position."""
    bus.connect()
    if not bus.is_calibrated or any(bus.sync_read("Torque_Enable").values()):
        raise RuntimeError("Need matching calibration and initially released motors")
    if any(bus.read("Operating_Mode", name) != 0 for name in ALL_MOTORS):
        raise RuntimeError("All motors must be in position mode")

    input("Support arm; clear its route to the taught grasp. Enter to enable holding: ")
    current = bus.sync_read("Present_Position")
    startup_angles = {name: float(np.deg2rad(current[name])) for name in ARM_JOINTS}
    log["original_position_rad"] = startup_angles
    build_trajectory(startup_angles, approach, DURATION)

    # Save before writing, so a partial setup can still be cleaned up.
    session["original_settings"] = {
        "elbow_p": bus.read("P_Coefficient", "elbow_flex"),
        "elbow_lock": bus.read("Lock", "elbow_flex"),
    }
    bus.write("Lock", "elbow_flex", 0)
    bus.write("P_Coefficient", "elbow_flex", ELBOW_P)
    if bus.read("P_Coefficient", "elbow_flex") != ELBOW_P:
        raise RuntimeError("Elbow P readback mismatch")

    bus.sync_write("Goal_Position", {name: current[name] for name in ALL_MOTORS})
    session["motors_enabled"] = True  # Cleanup also handles partial enable failure.
    bus.enable_torque(list(ALL_MOTORS))
    return startup_angles


def run_trial(bus, grasp, approach, lift, startup_angles, trial, close_percent):
    """Open, pick, lift, lower, release, and return without a top hold."""
    set_gripper(bus, OPEN_PERCENT, "open", trial)
    move_to(bus, approach, "approach", DURATION, trial)
    move_to(bus, grasp, "descend", DURATION, trial)
    set_gripper(bus, close_percent, "close", trial)
    move_to(bus, lift, "lift", DURATION, trial)
    move_to(bus, grasp, "lower", DURATION, trial)
    set_gripper(bus, OPEN_PERCENT, "release_object", trial)
    move_to(bus, approach, "retreat", DURATION, trial)
    move_to(bus, startup_angles, "return_start", DURATION, trial)

    answer = input(f"Did the object lift about {LIFT_HEIGHT * 100:g} cm, then lower and release successfully? "
                   "[y/n/Enter to skip scoring]: ").strip().lower()
    if not answer:
        return
    if answer not in ("y", "n"):
        raise ValueError("Outcome must be y or n; trial remains unscored")
    trial["success"] = answer == "y"
    trial["failure_reason"] = "" if trial["success"] else input("Failure reason: ")


def cleanup_motors(bus, session, log):
    """Release each motor, restore settings, and close the USB connection."""
    if not bus.is_connected:
        return
    if session["motors_enabled"]:
        try:
            input("Support object and full arm weight. Enter to release all motors: ")
        except (KeyboardInterrupt, EOFError):
            log["cleanup_note"] = "Release prompt interrupted; cleanup still attempted"
        for name in ALL_MOTORS:
            try:
                if name == "gripper":
                    shutdown = release_gripper(bus)
                    log["gripper_shutdown"] = shutdown
                    if shutdown['fault_flags']:
                        print('Gripper torque-off confirmed, but its fault remains active; '
                              'inspect/recover the gripper before another run.')
                else:
                    bus.disable_torque(name)
            except Exception as error:
                log.setdefault("cleanup_errors", []).append(f"{name}: {error}")
                print(f"Could not confirm release of {name}: {error}")

    original = session["original_settings"]
    if original:
        settings_to_restore = [
            ("Lock", "elbow_flex", 0),
            ("P_Coefficient", "elbow_flex", original["elbow_p"]),
            ("Lock", "elbow_flex", original["elbow_lock"]),
        ]
        for register, motor, value in settings_to_restore:
            try:
                bus.write(register, motor, value)
                if bus.read(register, motor) != value:
                    raise RuntimeError("Restoration readback mismatch")
            except Exception as error:
                log.setdefault("cleanup_errors", []).append(f"Restore {register}: {error}")
                print(f"Could not restore {register}: {error}")
    try:
        bus.disconnect(disable_torque=False)
    except Exception as error:
        log.setdefault("cleanup_errors", []).append(f"Disconnect: {error}")


def save_log(path, log):
    scored_trials = [trial for trial in log["trials"] if trial["success"] is not None]
    log["successes"] = sum(trial["success"] for trial in scored_trials)
    log["scored_trials"] = len(scored_trials)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(log, indent=2) + "\n")
    print(f"Saved {path}; successes {log['successes']}/{len(scored_trials)} scored trials")
    print("Run status:", log["status"])


def main():
    grasp, approach, lift, close_percent = plan_waypoints(GRASP_FILE, DURATION)

    robot = SO101Follower(SO101FollowerConfig(
        port="/dev/ttyACM1", id="so101_follower", use_degrees=True,
    ))
    log = {
        "settings": {
            "duration_s": DURATION,
            "max_velocity_deg_s": MAX_VELOCITY,
            "max_acceleration_deg_s2": MAX_ACCELERATION,
            "joint_limits_deg": np.rad2deg(calibrated_joint_limits()).tolist(),
            "open_percent": OPEN_PERCENT,
            "close_percent": close_percent,
            "grasp": str(GRASP_FILE),
        },
        "grasp_angles_rad": grasp,
        "approach_angles_rad": approach,
        "lift_angles_rad": lift,
        "lift_height_m": LIFT_HEIGHT,
        "trials": [],
        "status": "started",
    }
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    path = RESULTS_DIR / f"pick_trials_{timestamp}.json"
    session = {"motors_enabled": False, "original_settings": {}}

    try:
        startup_angles = prepare_motors(robot.bus, approach, session, log)
        input("Place object at its mark; hands clear. Enter to start the full cycle: ")
        trial = {"number": 1, "phases": [], "gripper": [], "success": None}
        log["trials"].append(trial)
        run_trial(robot.bus, grasp, approach, lift, startup_angles, trial, close_percent)
        log["status"] = "completed"
    except BaseException as error:
        log["status"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        try:
            cleanup_motors(robot.bus, session, log)
        finally:
            save_log(path, log)


if __name__ == "__main__":
    main()
