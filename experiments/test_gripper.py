from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

robot = SO101Follower(SO101FollowerConfig(
    port="/dev/ttyACM1",
    id="so101_follower",
    use_degrees=True,
))
bus = robot.bus
enabled = False
original_limit = None

try:
    bus.connect()

    if not bus.is_calibrated:
        raise RuntimeError("Calibration mismatch")
    if bus.read("Torque_Enable", "gripper") != 0:
        raise RuntimeError("Start with the gripper motor released")
    if bus.read("Operating_Mode", "gripper") != 0:
        raise RuntimeError("Gripper must be in position mode")

    current = bus.read("Present_Position", "gripper")
    target = current + 5 if current <= 95 else current - 5

    print(f"Gripper target: {current:.1f} → {target:.1f}%")
    input("Keep fingers clear. Enter to move: ")

    # Temporary effort cap: 250/1000 = 25%; not a precise grip-force setting.
    original_limit = bus.read("Torque_Limit", "gripper")
    bus.write("Torque_Limit", "gripper", 250)

    # Seed its current position before enabling holding.
    bus.write("Goal_Position", "gripper", current)
    enabled = True
    bus.enable_torque("gripper")
    bus.write("Goal_Position", "gripper", target)

    input("Did it open or close? Enter to return: ")
    print("Reached:", bus.read("Present_Position", "gripper"))

    bus.write("Goal_Position", "gripper", current)
    input("Once it has returned, Enter to release: ")

finally:
    if bus.is_connected:
        try:
            if enabled:
                bus.disable_torque("gripper")
        finally:
            try:
                if original_limit is not None:
                    bus.write("Torque_Limit", "gripper", original_limit)
            finally:
                bus.disconnect(disable_torque=False)
