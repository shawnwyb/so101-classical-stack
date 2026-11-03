"""Compare PD holding with/without gravity compensation in MuJoCo only."""

import argparse
import csv
import json
from datetime import datetime, timezone

import mujoco
import numpy as np

from experiments import RESULTS_DIR
from mujoco_dynamics import load_model, inspect_pose, NAMES


def simulate(compensate, duration=5, timestep=0.0005):
    model, data = load_model()
    model.opt.timestep = timestep
    # Isolate gravity/control from self-contact. This is not a collision demo.
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    names = [model.joint(i).name for i in range(model.njnt)]
    gravity_data = mujoco.MjData(model)
    target = np.zeros(model.nq)
    _, mass = inspect_pose(model, data, dict.fromkeys(NAMES, 0.0))
    kp = np.array([8, 8, 8, 2, 0.3, 0.3], dtype=float)
    # Starting estimate for damping, using each joint's diagonal inertia.
    kd = 2 * np.sqrt(kp * np.diag(mass))
    rows = []
    stride = max(1, round(0.01 / timestep))
    count = round(duration / timestep)
    for step in range(count + 1):
        mujoco.mj_forward(model, data)
        # Compute gravity at the current position with velocity explicitly zero.
        # Using moving data.qfrc_bias directly would also cancel Coriolis terms.
        gravity_data.qpos[:] = data.qpos
        gravity_data.qvel[:] = 0
        mujoco.mj_forward(model, gravity_data)
        gravity = gravity_data.qfrc_bias.copy()
        error = target - data.qpos
        pd_torque = kp * error - kd * data.qvel
        torque = pd_torque + (gravity if compensate else 0)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(torque)):
            raise RuntimeError('Nonfinite simulated state')
        if data.ncon:
            raise RuntimeError('Unexpected contact in isolated experiment')
        for i in range(model.njnt):
            if not model.joint(i).range[0] < data.qpos[i] < model.joint(i).range[1]:
                raise RuntimeError(f'{names[i]} reached a joint limit; comparison invalid')
        if step % stride == 0 or step == count:
            rows.append({'time_s': float(data.time), 'position_rad': data.qpos.copy(),
                         'velocity_rad_s': data.qvel.copy(), 'error_rad': error.copy(),
                         'pd_torque_Nm': pd_torque.copy(), 'gravity_Nm': gravity.copy(),
                         'applied_torque_Nm': torque.copy()})
        if step < count:
            # Direct ideal joint torques; not Feetech position commands.
            data.qfrc_applied[:] = torque
            mujoco.mj_step(model, data)
    errors = np.array([r['error_rad'] for r in rows])
    tail = errors[np.array([r['time_s'] for r in rows]) >= duration - 1]
    metrics = {
        'final_error_deg': dict(zip(names, np.rad2deg(errors[-1]).tolist())),
        'last_second_rms_error_deg': dict(zip(names, np.rad2deg(np.sqrt(np.mean(tail**2, axis=0))).tolist())),
        'final_velocity_rad_s': dict(zip(names, rows[-1]['velocity_rad_s'].tolist())),
        'final_equilibrium_residual_Nm': (rows[-1]['applied_torque_Nm'] - rows[-1]['gravity_Nm']).tolist(),
    }
    settings = {'joint_order': names, 'target_angles_rad': target.tolist(),
                'kp_Nm_per_rad': kp.tolist(), 'kd_Nm_s_per_rad': kd.tolist(),
                'duration_s': duration, 'timestep_s': timestep}
    return rows, metrics, settings


def save_plot(path, runs, names):
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="950" height="780">',
           '<rect width="950" height="780" fill="white"/>',
           '<text x="75" y="25" font-family="sans-serif">Simulated PD holding: blue = gravity compensation OFF; orange = ON</text>',
           '<text x="75" y="47" font-family="sans-serif">Solid = actual angle; dashed = target minus actual. Target angle = 0 degrees.</text>']
    duration = runs['off'][-1]['time_s']
    for panel, name in enumerate(('shoulder_lift', 'elbow_flex', 'wrist_flex')):
        i = names.index(name)
        bound = max(0.1, *(abs(np.rad2deg(row['position_rad'][i])) for rows in runs.values() for row in rows)) * 1.15
        top = 90 + panel*220
        svg.extend([f'<text x="75" y="{top-12}" font-family="sans-serif">{name}: angle and error [degrees], range ±{bound:.2f}</text>',
                    f'<path d="M75 {top} V{top+160} H870 M75 {top+80} H870" stroke="#aaa" fill="none"/>',
                    f'<text x="75" y="{top+180}" font-size="12">0 s</text>',
                    f'<text x="830" y="{top+180}" font-size="12">{duration:g} s</text>'])
        for label, color in [('off', '#2563eb'), ('on', '#d97706')]:
            for field, dash in [('position_rad', ''), ('error_rad', 'stroke-dasharray="5 4"')]:
                points = ' '.join(f"{75+795*row['time_s']/duration:.2f},{top+80-80*np.rad2deg(row[field][i])/bound:.2f}" for row in runs[label])
                svg.append(f'<polyline points="{points}" stroke="{color}" stroke-width="2" fill="none" {dash}/>')
    svg.append('</svg>')
    path.write_text('\n'.join(svg)+'\n')


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    runs, metrics, settings = {}, {}, None
    for label, compensate in [('off', False), ('on', True)]:
        runs[label], metrics[label], settings = simulate(compensate)
        print(f'Gravity compensation {label.upper()}: final errors [deg]')
        for name in NAMES:
            print(f'  {name:16s} {metrics[label]["final_error_deg"][name]:+.5f}')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    base = RESULTS_DIR / f'gravity_compensation_{stamp}'
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    summary = {'mujoco_version': mujoco.__version__, 'settings': settings, 'metrics': metrics,
               'prediction': 'Gravity compensation reduces static error with identical PD gains.',
               'limitations': 'Ideal unlimited torques, exact model, no contacts or identified servo/gear/friction dynamics. Not hardware accuracy.'}
    base.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
    fields = ['position_rad', 'velocity_rad_s', 'error_rad', 'pd_torque_Nm', 'gravity_Nm', 'applied_torque_Nm']
    with base.with_suffix('.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['compensation', 'time_s'] + [f'{n}_{f}' for f in fields for n in settings['joint_order']])
        for label, rows in runs.items():
            for row in rows:
                writer.writerow([label, row['time_s']] + [x for f in fields for x in row[f]])
    save_plot(base.with_suffix('.svg'), runs, settings['joint_order'])
    print(f'Saved {base} .json / .csv / .svg; simulation only.')


if __name__ == '__main__':
    main()
