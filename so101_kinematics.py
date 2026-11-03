"""SO-101 forward kinematics, using metres and radians."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from se3 import SE3


URDF_PATH = Path(__file__).parent / "models/so101/so101_new_calib.urdf"
JOINT_CHAIN = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper_frame_joint",
)


def axis_rotation(axis, angle):
    """Rotate around a local axis using Rodrigues' rotation formula."""
    axis = np.asarray(axis, dtype=float)
    if axis.shape != (3,) or not np.all(np.isfinite(axis)):
        raise ValueError("Axis must have three finite entries")
    length = np.linalg.norm(axis)
    if length == 0 or not np.isfinite(angle):
        raise ValueError("Axis must be nonzero and angle must be finite")
    x, y, z = axis / length
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def origin_transform(joint):
    """Fixed mounting transform from the parent frame to the joint frame."""
    origin = joint.find("origin")
    attributes = origin.attrib if origin is not None else {}
    xyz = np.fromstring(attributes.get("xyz", "0 0 0"), sep=" ")
    roll, pitch, yaw = np.fromstring(attributes.get("rpy", "0 0 0"), sep=" ")
    # URDF uses fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
    rotation = (
        axis_rotation([0, 0, 1], yaw)
        @ axis_rotation([0, 1, 0], pitch)
        @ axis_rotation([1, 0, 0], roll)
    )
    return SE3(rotation, xyz)


def forward_kinematics(joint_angles):
    """Return gripper_frame_link pose in base_link as an SE3.

    joint_angles maps the five arm joint names to URDF angles in radians.
    This calculates geometry only; it does not command hardware or enforce limits.
    """
    robot = ET.parse(URDF_PATH).getroot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    pose = SE3(np.eye(3), np.zeros(3))
    current_link = "base_link"

    for name in JOINT_CHAIN:
        joint = joints[name]
        if joint.find("parent").attrib["link"] != current_link:
            raise ValueError(f"Broken joint chain at {name}")

        pose = pose.compose(origin_transform(joint))
        if joint.attrib["type"] == "revolute":
            axis_element = joint.find("axis")
            axis = np.fromstring(
                axis_element.attrib["xyz"] if axis_element is not None else "1 0 0",
                sep=" ",
            )
            rotation = axis_rotation(axis, joint_angles[name])
            pose = pose.compose(SE3(rotation, np.zeros(3)))
        elif joint.attrib["type"] != "fixed":
            raise ValueError(f"Unsupported joint type at {name}")

        current_link = joint.find("child").attrib["link"]

    return pose


def position_jacobian(joint_angles, epsilon=1e-5):
    """Return a 3x5 position Jacobian in metres per radian.

    Columns follow JOINT_CHAIN's five moving arm joints. Central differences
    estimate each column by perturbing one joint while holding the rest fixed.
    This describes position only, not orientation, and never commands hardware.
    """
    if not np.isfinite(epsilon) or epsilon <= 0:
        raise ValueError("epsilon must be positive and finite")

    jacobian = np.zeros((3, 5))
    for column, name in enumerate(JOINT_CHAIN[:-1]):
        plus = dict(joint_angles)
        minus = dict(joint_angles)
        plus[name] += epsilon
        minus[name] -= epsilon

        position_plus = forward_kinematics(plus).T[:3, 3]
        position_minus = forward_kinematics(minus).T[:3, 3]
        jacobian[:, column] = (position_plus - position_minus) / (2 * epsilon)

    return jacobian


def inverse_kinematics(
    target_position, initial_angles, *, tolerance=0.001, damping=0.01,
    max_step=np.deg2rad(5), max_iterations=100,
):
    """Solve position-only IK with damped least squares and URDF joint limits.

    Positions/tolerance are in metres; angles/max_step are in radians.
    Returns angles, success, and error history. Orientation is unconstrained.
    A successful result does not guarantee collision-free hardware motion.
    """
    target = np.asarray(target_position, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("Target must contain three finite coordinates")
    if any(not np.isfinite(x) or x <= 0 for x in (tolerance, damping, max_step)):
        raise ValueError("Tolerance, damping, and max_step must be positive and finite")
    if not isinstance(max_iterations, int) or max_iterations < 0:
        raise ValueError("max_iterations must be a nonnegative integer")

    names = JOINT_CHAIN[:-1]
    joints = {j.attrib["name"]: j for j in ET.parse(URDF_PATH).getroot().findall("joint")}
    lower = np.array([float(joints[n].find("limit").attrib["lower"]) for n in names])
    upper = np.array([float(joints[n].find("limit").attrib["upper"]) for n in names])
    q = np.array([initial_angles[n] for n in names], dtype=float)
    if not np.all(np.isfinite(q)) or np.any(q < lower) or np.any(q > upper):
        raise ValueError("Initial angles must be finite and within URDF limits")

    errors = []
    for iteration in range(max_iterations + 1):
        angles = dict(zip(names, q))
        error = target - forward_kinematics(angles).T[:3, 3]
        error_norm = float(np.linalg.norm(error))
        errors.append(error_norm)
        if error_norm <= tolerance or iteration == max_iterations:
            break

        jacobian = position_jacobian(angles)
        # Damping keeps the solve bounded near configurations with poor mobility.
        system = jacobian @ jacobian.T + damping**2 * np.eye(3)
        step = jacobian.T @ np.linalg.solve(system, error)
        step *= min(1.0, max_step / max(np.max(np.abs(step)), 1e-12))

        # Try smaller steps if the linear prediction fails to improve actual FK.
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            candidate = np.clip(q + scale * step, lower, upper)
            position = forward_kinematics(dict(zip(names, candidate))).T[:3, 3]
            if np.linalg.norm(target - position) < error_norm:
                q = candidate
                break
        else:
            break

    return {
        "joint_angles": dict(zip(names, q)),
        "success": errors[-1] <= tolerance,
        "error_m": errors[-1],
        "error_history_m": errors,
        "iterations": len(errors) - 1,
    }


def print_joint_chain():
    robot = ET.parse(URDF_PATH).getroot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    for name in JOINT_CHAIN:
        joint = joints[name]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        xyz = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        axis = joint.find("axis")

        print(f"{name} ({joint.attrib['type']}): {parent} -> {child}")
        print(f"  xyz [m]:   {xyz}")
        print(f"  rpy [rad]: {rpy}")
        if joint.attrib["type"] != "fixed":
            print(f"  axis:      {axis.attrib['xyz'] if axis is not None else '1 0 0'}")
        print()


if __name__ == "__main__":
    print_joint_chain()
    zero_angles = {name: 0.0 for name in JOINT_CHAIN[:-1]}
    pose = forward_kinematics(zero_angles)
    print("Zero-angle gripper position [m]:", pose.T[:3, 3])
    print("Zero-angle gripper transform:\n", pose.T)
