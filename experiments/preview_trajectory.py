"""Preview a quintic trajectory in software; no hardware connection."""

import argparse
import csv
import json

import numpy as np

from experiments import RESULTS_DIR
from so101_kinematics import JOINT_CHAIN
from trajectories import quintic_trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=4.0)
    args = parser.parse_args()
    saved = json.loads((RESULTS_DIR / 'ik_start_pose.json').read_text())
    names = JOINT_CHAIN[:-1]
    start = np.array([saved['joint_angles_rad'][n] for n in names])
    end = start + np.deg2rad([5, -5, -5, 5, 0])
    trajectory = quintic_trajectory(start, end, args.duration)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = RESULTS_DIR / 'trajectory_preview.csv'
    fields = ('position_rad', 'velocity_rad_s', 'acceleration_rad_s2')
    with csv_path.open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s'] + [f'{n}_{field}' for field in fields for n in names])
        for i, time in enumerate(trajectory['time_s']):
            writer.writerow([time] + [value for field in fields for value in trajectory[field][i]])

    # Simple standalone SVG: show the elbow's position, speed, and acceleration.
    time = trajectory['time_s']
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="800" height="650" viewBox="0 0 800 650">',
           '<rect width="800" height="650" fill="white"/>',
           '<text x="60" y="25" font-family="sans-serif" font-size="18">Elbow quintic preview — software only</text>']
    labels = ('Position [degrees]', 'Velocity [degrees/s]', 'Acceleration [degrees/s²]')
    for panel, (field, label) in enumerate(zip(fields, labels)):
        values = np.rad2deg(trajectory[field][:, 2])
        low, high = float(values.min()), float(values.max())
        padding = max((high - low) * 0.15, 0.1)
        low, high = low - padding, high + padding
        top = 65 + panel * 190
        x = 80 + time / args.duration * 650
        y = top + 130 - (values - low) / (high - low) * 130
        points = ' '.join(f'{a:.2f},{b:.2f}' for a, b in zip(x, y))
        svg.extend([
            f'<text x="80" y="{top-10}" font-family="sans-serif" font-size="14">{label}</text>',
            f'<path d="M80 {top} V{top+130} H730" fill="none" stroke="#777"/>',
            f'<polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="2"/>',
            f'<text x="10" y="{top+10}" font-family="sans-serif" font-size="12">{high:.2f}</text>',
            f'<text x="10" y="{top+130}" font-family="sans-serif" font-size="12">{low:.2f}</text>',
            f'<text x="80" y="{top+150}" font-family="sans-serif" font-size="12">0 s</text>',
            f'<text x="690" y="{top+150}" font-family="sans-serif" font-size="12">{args.duration:g} s</text>',
        ])
    svg.append('</svg>')
    plot_path = RESULTS_DIR / 'trajectory_preview.svg'
    plot_path.write_text('\n'.join(svg) + '\n')
    print(f'Software only: {len(time)} samples, interval {time[1]-time[0]:.4f} s')
    print('Start/end velocities:', trajectory['velocity_rad_s'][[0, -1]])
    print('Start/end accelerations:', trajectory['acceleration_rad_s2'][[0, -1]])
    print(f'Saved {csv_path}\nSaved {plot_path}')


if __name__ == '__main__':
    main()
