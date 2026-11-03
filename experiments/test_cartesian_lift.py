"""Try a small position-only IK lift. Run beside the arm with space to move."""

import argparse
import json
import time
from datetime import datetime, timezone

import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from so101_kinematics import JOINT_CHAIN, forward_kinematics, inverse_kinematics
from experiments import RESULTS_DIR


NAMES = JOINT_CHAIN[:-1]


def plan(readings, lift_m=0.05):
    angles = {n: np.deg2rad(readings[n]) for n in NAMES}
    target = forward_kinematics(angles).T[:3, 3] + [0, 0, lift_m]
    result = inverse_kinematics(target, angles)
    goal = {n: float(np.rad2deg(result['joint_angles'][n])) for n in NAMES}
    if not result['success']:
        raise RuntimeError("IK did not reach the target within tolerance")
    print(f"Target: {lift_m * 1000:.0f} mm upward; orientation is free.")
    for n in NAMES:
        print(f"{n:16s} {readings[n]:7.2f} -> {goal[n]:7.2f} deg")
    return target, result, goal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='Use saved angles; do not connect')
    parser.add_argument('--elbow-p', type=int, choices=(16, 24, 32),
                        help='Temporarily set elbow proportional gain; restore on exit')
    args = parser.parse_args()
    if args.dry_run:
        print(f'Elbow P test: {args.elbow_p if args.elbow_p is not None else "unchanged"} (no writes in dry run)')
        saved = json.loads((RESULTS_DIR / 'ik_start_pose.json').read_text())
        plan(saved['joint_angles_deg'])
        return

    robot = SO101Follower(SO101FollowerConfig(
        port='/dev/ttyACM1', id='so101_follower', use_degrees=True,
    ))
    bus = robot.bus
    enabled = False
    original_p = None
    original_lock = None
    log = {'tracking': [], 'status': 'started'}
    # Preserve raw reply codes: the SDK leaves some servo errors unnamed.
    original_write = bus.packet_handler.writeTxRx

    def diagnostic_write(port, motor_id, address, length, data):
        comm, error = original_write(port, motor_id, address, length, data)
        if comm != 0 or error != 0:
            detail = {
                'motor_id': motor_id, 'register_address': address,
                'data': list(data), 'communication_result': comm,
                'packet_error': error,
                'packet_error_hex': f'0x{error:02X}',
                'sdk_message': bus.packet_handler.getRxPacketError(error),
            }
            log.setdefault('write_errors', []).append(detail)
            print('RAW SERVO REPLY:', json.dumps(detail))
        return comm, error

    bus.packet_handler.writeTxRx = diagnostic_write
    path = RESULTS_DIR / datetime.now(timezone.utc).strftime('cartesian_lift_%Y%m%dT%H%M%S_%fZ.json')
    started = time.monotonic()
    try:
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError('Calibration mismatch')
        if any(bus.sync_read('Torque_Enable').values()):
            raise RuntimeError('Start with motors released; support the arm')
        if any(bus.read('Operating_Mode', n) != 0 for n in NAMES):
            raise RuntimeError('All arm joints must be in position mode')
        log['elbow_p_original'] = bus.read('P_Coefficient', 'elbow_flex')
        log['elbow_p_test'] = args.elbow_p if args.elbow_p is not None else log['elbow_p_original']

        input('Support the current pose; clear space above and around the arm. Enter to plan: ')
        before = bus.sync_read('Present_Position')
        target, result, goal = plan(before)
        log.update(before=before, target_position_m=target.tolist(), ik=result)
        input('Enter to enable holding at this pose ONLY. Ctrl+C cancels: ')
        current = bus.sync_read('Present_Position')
        if any(abs(current[n] - before[n]) > 0.5 for n in NAMES):
            raise RuntimeError('Pose shifted while waiting; restart')
        if args.elbow_p is not None:
            original_p = log['elbow_p_original']
            original_lock = bus.read('Lock', 'elbow_flex')
            bus.write('Lock', 'elbow_flex', 0)
            bus.write('P_Coefficient', 'elbow_flex', args.elbow_p)
            readback = bus.read('P_Coefficient', 'elbow_flex')
            if readback != args.elbow_p:
                raise RuntimeError('Elbow P readback did not match requested value')
            print(f'Elbow P: {original_p} -> {readback}; will restore on exit')
        # Seed every arm goal before enabling holding force; leave gripper untouched.
        bus.sync_write('Goal_Position', {n: current[n] for n in NAMES})
        enabled = True
        bus.enable_torque(list(NAMES))

        input('Gently stop supporting its weight; stay ready to catch it. Enter when hands are clear: ')
        held_start = bus.sync_read('Present_Position')
        time.sleep(1)
        held_end = bus.sync_read('Present_Position')
        log['hold_check'] = {'command': current, 'start': held_start, 'end': held_end}
        drift = max(abs(held_end[n] - held_start[n]) for n in NAMES)
        hold_error = max(abs(held_end[n] - current[n]) for n in NAMES)
        print(f'Hold drift over 1 second: {drift:.2f} deg; maximum hold error: {hold_error:.2f} deg')
        if drift > 0.5 or hold_error > 2:
            input('Holding check failed. Support the arm, then Enter to release and stop: ')
            raise RuntimeError('Arm did not hold its starting pose; no IK movement attempted')

        # Plan from the actual unsupported pose, so hand release is not counted as motion.
        before = held_end
        target, result, goal = plan(before)
        log.update(before=before, target_position_m=target.tolist(), ik=result)
        input('Keep hands clear. Enter to send the lift target directly: ')
        current_pose = bus.sync_read('Present_Position')
        if any(abs(current_pose[n] - before[n]) > 0.5 for n in NAMES):
            raise RuntimeError('Pose drifted before the lift; restart')

        def move_to(destination):
            initial = bus.sync_read('Present_Position')
            command = {n: destination[n] for n in NAMES}
            bus.sync_write('Goal_Position', command)
            # Read the targets back from the motors, rather than assuming receipt.
            goal_readback = {n: bus.read('Goal_Position', n) for n in NAMES}
            print('Motor target readback errors [deg]:', {
                n: round(goal_readback[n] - command[n], 3) for n in NAMES
            })
            telemetry = []
            next_telemetry = time.monotonic()
            # Observe for five seconds; this does not set the servo movement speed.
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                actual = bus.sync_read('Present_Position')
                log['tracking'].append({
                    'elapsed_s': time.monotonic() - started,
                    'command_deg': command, 'actual': actual,
                })
                if time.monotonic() >= next_telemetry:
                    sample = {'elapsed_s': time.monotonic() - started}
                    for register in ('Present_Load', 'Present_Current', 'Present_Voltage',
                                     'Present_Temperature', 'Status'):
                        try:
                            sample[register] = bus.read(register, 'elbow_flex', normalize=False)
                        except Exception as error:
                            sample[register] = str(error)
                    telemetry.append(sample)
                    # Save immediately so fault/abort runs retain these diagnostics.
                    log.setdefault('elbow_telemetry', []).append(sample)
                    next_telemetry = time.monotonic() + 0.5
                # A step command initially has a large error by design. Check for
                # excursions outside its start-to-target interval instead.
                if any(actual[n] < min(initial[n], command[n]) - 3 or
                       actual[n] > max(initial[n], command[n]) + 3 for n in NAMES):
                    raise RuntimeError('Joint moved outside the commanded interval by over 3 degrees')
                time.sleep(0.05)
            actual = bus.sync_read('Present_Position')
            errors = {n: float(actual[n] - command[n]) for n in NAMES}
            log.setdefault('move_results', []).append({
                'command_deg': command, 'actual': actual,
                'final_error_deg': errors, 'observation_seconds': 5,
                'goal_readback_deg': goal_readback, 'elbow_telemetry': telemetry,
            })
            print('After 5 seconds, target errors [deg] (actual - command):')
            for n, error in errors.items():
                print(f'  {n:16s} {error:+.2f}')
            print('Elbow telemetry (raw register units):', telemetry[-1] if telemetry else {})
            return actual

        after = move_to(goal)
        log['after'] = after
        def position(q):
            return forward_kinematics({n: np.deg2rad(q[n]) for n in NAMES}).T[:3, 3]
        print('FK displacement from actual angles [mm]:', np.round((position(after) - position(before)) * 1000, 2))
        print('No automatic return: the arm is holding the lifted pose.')
        input('Securely support its weight, then Enter to release holding force and finish: ')
        log['status'] = 'completed'
    except BaseException as error:
        log['status'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            if bus.is_connected:
                try:
                    if enabled:
                        # Attempt every motor even if one reports a servo fault.
                        for name in NAMES:
                            try:
                                bus.disable_torque(name)
                            except Exception as cleanup_error:
                                message = f'{name}: {cleanup_error}'
                                log.setdefault('release_errors', []).append(message)
                                print(f'Could not confirm motor release: {message}')
                finally:
                    try:
                        if original_p is not None:
                            try:
                                bus.write('Lock', 'elbow_flex', 0)
                                bus.write('P_Coefficient', 'elbow_flex', original_p)
                                restored = bus.read('P_Coefficient', 'elbow_flex')
                                if restored != original_p:
                                    raise RuntimeError('Restored P readback mismatch')
                                bus.write('Lock', 'elbow_flex', original_lock)
                                log['elbow_p_restored'] = restored
                                print(f'Restored elbow P={restored}')
                            except Exception as restore_error:
                                log['gain_restore_error'] = str(restore_error)
                                print(f'P RESTORE FAILED: {restore_error}; verify gain before next run')
                    finally:
                        bus.disconnect(disable_torque=False)
        finally:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(log, indent=2) + '\n')
            print(f'Saved {path}')


if __name__ == '__main__':
    main()
