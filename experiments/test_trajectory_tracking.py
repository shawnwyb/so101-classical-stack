"""Track one quintic outward move and a smooth reset; run beside the arm."""

import argparse
import csv
import json
import time
from datetime import datetime, timezone

import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from experiments import RESULTS_DIR
from so101_kinematics import JOINT_CHAIN, calibrated_joint_limits
from trajectories import quintic_trajectory


NAMES = JOINT_CHAIN[:-1]


def angles(readings):
    return np.deg2rad([readings[n] for n in NAMES])


def make_trajectory(start, end, duration, period, velocity_limit, acceleration_limit):
    trajectory = quintic_trajectory(start, end, duration, period)
    lower, upper = calibrated_joint_limits()
    for column, name in enumerate(NAMES):
        positions = trajectory['position_rad'][:, column]
        if positions.min() < lower[column] or positions.max() > upper[column]:
            raise ValueError(f'{name}: trajectory exceeds calibrated limits')
    # Exact continuous maxima of this particular zero-endpoint quintic.
    peak_speed = np.rad2deg(1.875 * np.abs(end - start) / duration)
    peak_acceleration = np.rad2deg((10 / np.sqrt(3)) * np.abs(end - start) / duration**2)
    if np.max(peak_speed) > velocity_limit or np.max(peak_acceleration) > acceleration_limit:
        raise ValueError('Trajectory exceeds the requested velocity/acceleration limits; increase duration')
    return trajectory


def execute(bus, trajectory, record, *, settle_time=2.0, verbose=True):
    """Schedule against a monotonic clock; skip missed samples instead of bursting."""
    if not np.isfinite(settle_time) or settle_time < 0:
        raise ValueError('Settling time must be nonnegative and finite')
    reference_times = trajectory['time_s']
    positions = np.rad2deg(trajectory['position_rad'])
    start_time = time.monotonic()
    index = 0
    while index < len(reference_times):
        deadline = start_time + reference_times[index]
        time.sleep(max(0, deadline - time.monotonic()))
        command_time = time.monotonic() - start_time
        # Advance to the most recent reference if this computer fell behind.
        index = max(index, int(np.searchsorted(reference_times, command_time, side='right')) - 1)
        index = min(index, len(reference_times) - 1)
        command = positions[index]
        bus.sync_write('Goal_Position', dict(zip(NAMES, command)))
        actual = np.rad2deg(angles(bus.sync_read('Present_Position')))
        sample = {
            'reference_time_s': float(reference_times[index]),
            'command_time_s': command_time,
            'feedback_time_s': time.monotonic() - start_time,
            'command_deg': command.tolist(), 'actual_deg': actual.tolist(),
            'error_deg': (actual - command).tolist(),
        }
        record['samples'].append(sample)
        # Detect unexpected excursions; ordinary target error remains a result.
        low = np.minimum(positions[0], positions[-1]) - 3
        high = np.maximum(positions[0], positions[-1]) + 3
        if np.any(actual < low) or np.any(actual > high):
            raise RuntimeError('Actual joint position left the trajectory envelope by over 3 degrees')
        index += 1

    errors = np.array([s['error_deg'] for s in record['samples']])
    record['rms_error_deg'] = dict(zip(NAMES, np.sqrt(np.mean(errors**2, axis=0)).tolist()))
    record['peak_error_deg'] = dict(zip(NAMES, np.max(np.abs(errors), axis=0).tolist()))
    record['skipped_samples'] = len(reference_times) - len(record['samples'])
    # Tracking experiments wait; pick motion can continue immediately.
    if settle_time:
        time.sleep(settle_time)
    final = np.rad2deg(angles(bus.sync_read('Present_Position')))
    error_key = 'settled_error_deg' if settle_time else 'end_error_deg'
    record['settle_time_s'] = settle_time
    record[error_key] = dict(zip(NAMES, (final - positions[-1]).tolist()))
    if verbose:
        print(f"{record['label']} RMS errors [deg]:", {n: round(v, 3) for n, v in record['rms_error_deg'].items()})
        print('Final errors [deg]:', {n: round(v, 3) for n, v in record[error_key].items()})
        print('Skipped reference samples:', record['skipped_samples'])


def save_results(path, log):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(log, indent=2) + '\n')
    with path.with_suffix('.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['phase', 'reference_s', 'command_s', 'feedback_s'] +
                        [f'{n}_{field}' for field in ('command_deg', 'actual_deg', 'error_deg') for n in NAMES])
        for phase in log['phases']:
            for s in phase['samples']:
                writer.writerow([phase['label'], s['reference_time_s'], s['command_time_s'], s['feedback_time_s']] +
                                s['command_deg'] + s['actual_deg'] + s['error_deg'])
    # Overlay outward and reset errors; direction differs, so this is not a speed comparison.
    phases = [p for p in log['phases'] if p['samples']]
    legend = '; '.join(f'{color}: {phase["label"]}, {phase["duration_s"]:g} s'
                       for color, phase in zip(('Blue', 'orange'), phases))
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="850" height="1000">',
           '<rect width="850" height="1000" fill="white"/>',
           '<text x="65" y="25" font-family="sans-serif">Joint tracking error [degrees], actual - command</text>',
           f'<text x="65" y="45" font-family="sans-serif">{legend}. X axis: trajectory progress, 0–100%.</text>']
    for i, name in enumerate(NAMES):
        bound = max([0.5] + [abs(s['error_deg'][i]) for p in phases for s in p['samples']]) * 1.1
        top = 85 + i * 180
        svg.extend([f'<text x="65" y="{top-10}" font-family="sans-serif">{name} (scale ±{bound:.2f}°)</text>',
                    f'<path d="M65 {top} V{top+125} H800 M65 {top+62.5} H800" stroke="#aaa" fill="none"/>'])
        for color, phase in zip(('#2563eb', '#d97706'), phases):
            points = ' '.join(f"{65+735*s['reference_time_s']/phase['duration_s']:.2f},{top+62.5-62.5*s['error_deg'][i]/bound:.2f}" for s in phase['samples'])
            svg.append(f'<polyline points="{points}" stroke="{color}" stroke-width="2" fill="none"/>')
    svg.append('</svg>')
    path.with_suffix('.svg').write_text('\n'.join(svg) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='Check plans from saved angles; no connection')
    parser.add_argument('--duration', type=float, default=2,
                        help='Outward move duration in seconds (default: 2)')
    parser.add_argument('--reset-duration', type=float, default=2,
                        help='Smooth reset duration in seconds (default: 2)')
    parser.add_argument('--delta-deg', nargs=5, type=float, default=[0, 5, -10, 5, 0], metavar='ANGLE')
    parser.add_argument('--period', type=float, default=0.02, help='Requested command interval in seconds')
    parser.add_argument('--max-velocity', type=float, default=20, help='Experiment limit in degrees/s')
    parser.add_argument('--max-acceleration', type=float, default=40, help='Experiment limit in degrees/s²')
    args = parser.parse_args()
    if any(not np.isfinite(x) or x <= 0 for x in (args.duration, args.reset_duration)):
        parser.error('Move and reset durations must be positive and finite')
    if any(not np.isfinite(x) or x <= 0 for x in (args.max_velocity, args.max_acceleration)):
        parser.error('Velocity and acceleration limits must be positive and finite')

    def plans(start):
        end = start + np.deg2rad(args.delta_deg)
        specifications = [(f'outward_{args.duration:g}s', start, end, args.duration),
                          (f'reset_{args.reset_duration:g}s', end, start, args.reset_duration)]
        return [(label, duration, make_trajectory(a, b, duration, args.period, args.max_velocity, args.max_acceleration))
                for label, a, b, duration in specifications]

    if args.dry_run:
        saved = json.loads((RESULTS_DIR / 'ik_start_pose.json').read_text())
        for label, duration, trajectory in plans(angles(saved['joint_angles_deg'])):
            print(label, 'samples:', len(trajectory['time_s']), 'duration:', duration)
        print('Software-only validation passed; no hardware connection.')
        return

    bus = SO101Follower(SO101FollowerConfig(
        port='/dev/ttyACM1', id='so101_follower', use_degrees=True,
    )).bus
    path = RESULTS_DIR / datetime.now(timezone.utc).strftime('trajectory_tracking_%Y%m%dT%H%M%S_%fZ.json')
    log = {'status': 'started', 'joint_order': list(NAMES), 'settings': vars(args), 'phases': []}
    original_p = original_lock = None
    enabled = False
    try:
        bus.connect()
        if not bus.is_calibrated or any(bus.sync_read('Torque_Enable').values()):
            raise RuntimeError('Need matching calibration and initially released motors')
        if any(bus.read('Operating_Mode', n) != 0 for n in NAMES):
            raise RuntimeError('All arm joints must be in position mode')
        input('Support the current pose, with room for a small move. Enter to preview: ')
        before = bus.sync_read('Present_Position')
        plans(angles(before))  # Validate before enabling holding force.
        print('Joint changes [deg]:', dict(zip(NAMES, args.delta_deg)))
        print(f'Sequence: {args.duration:g} s outward, {args.reset_duration:g} s smooth reset. Elbow P=32.')
        print(f'Experiment limits: {args.max_velocity:g} deg/s, {args.max_acceleration:g} deg/s²')
        input('Enter to enable holding only: ')
        current = bus.sync_read('Present_Position')
        if np.max(np.abs(angles(current) - angles(before))) > np.deg2rad(0.5):
            raise RuntimeError('Starting pose changed; restart')
        original_p = bus.read('P_Coefficient', 'elbow_flex')
        original_lock = bus.read('Lock', 'elbow_flex')
        bus.write('Lock', 'elbow_flex', 0)
        bus.write('P_Coefficient', 'elbow_flex', 32)
        if bus.read('P_Coefficient', 'elbow_flex') != 32:
            raise RuntimeError('P readback mismatch')
        log['original_elbow_p'] = original_p
        log['test_elbow_p'] = 32
        bus.sync_write('Goal_Position', {n: current[n] for n in NAMES})
        enabled = True
        bus.enable_torque(list(NAMES))
        input('Stop supporting its weight; keep hands clear. Enter when it holds steady: ')
        held = bus.sync_read('Present_Position')
        time.sleep(1)
        start = bus.sync_read('Present_Position')
        if np.max(np.abs(angles(start) - angles(held))) > np.deg2rad(0.5):
            input('Arm drifted. Support its weight, then Enter to release: ')
            raise RuntimeError('Holding check failed')
        log['start_readings'] = start
        for label, duration, trajectory in plans(angles(start)):
            print(f'Next: {label}. Keep the movement area clear.')
            input('Enter to execute this smooth trajectory: ')
            actual_start = np.rad2deg(angles(bus.sync_read('Present_Position')))
            reference_start = np.rad2deg(trajectory['position_rad'][0])
            print('Starting tracking offsets [deg]:', np.round(actual_start - reference_start, 2))
            phase = {'label': label, 'duration_s': duration, 'samples': [],
                     'actual_start_deg': actual_start.tolist(), 'reference_start_deg': reference_start.tolist()}
            log['phases'].append(phase)
            execute(bus, trajectory, phase)
        input('Finished; holding the endpoint. Support its full weight, then Enter to release: ')
        log['status'] = 'completed'
    except BaseException as error:
        log['status'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            if bus.is_connected:
                try:
                    if enabled:
                        for name in NAMES:
                            try:
                                bus.disable_torque(name)
                            except Exception as error:
                                log.setdefault('cleanup_errors', []).append(f'{name}: {error}')
                                print(f'Could not confirm release of {name}: {error}')
                    if original_p is not None:
                        try:
                            bus.write('Lock', 'elbow_flex', 0)
                            bus.write('P_Coefficient', 'elbow_flex', original_p)
                            if bus.read('P_Coefficient', 'elbow_flex') != original_p:
                                raise RuntimeError('P restoration readback mismatch')
                            bus.write('Lock', 'elbow_flex', original_lock)
                            log['restored_elbow_p'] = original_p
                            print(f'Restored elbow P={original_p}')
                        except Exception as error:
                            log['restore_error'] = str(error)
                            print(f'P RESTORE FAILED: {error}')
                finally:
                    bus.disconnect(disable_torque=False)
        finally:
            save_results(path, log)
            print(f'Saved {path} and matching CSV/SVG files')


if __name__ == '__main__':
    main()
