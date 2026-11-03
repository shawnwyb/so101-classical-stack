"""Inspect official URDF dynamics in MuJoCo; never connects to hardware."""

import argparse
import csv
import json
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from experiments import RESULTS_DIR
from so101_kinematics import JOINT_CHAIN, URDF_PATH


NAMES = JOINT_CHAIN[:-1]


def load_model():
    root = ET.parse(URDF_PATH).getroot()
    # Adapt import options in memory; keep the official file unchanged.
    extension = ET.SubElement(root, 'mujoco')
    ET.SubElement(extension, 'compiler', meshdir=str(URDF_PATH.resolve().parent),
                  discardvisual='false', fusestatic='false')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    model.opt.gravity[:] = [0, 0, -9.81]
    return model, mujoco.MjData(model)


def inspect_pose(model, data, angles):
    """Return g(q) and M(q) for all six joints at zero velocity."""
    mujoco.mj_resetData(model, data)
    for name, angle in angles.items():
        joint = model.joint(name)
        if not joint.range[0] <= angle <= joint.range[1]:
            raise ValueError(f'{name}: angle outside model limits')
        data.qpos[joint.qposadr[0]] = angle
    # With qvel=0, the gravity/Coriolis bias contains only gravity terms.
    mujoco.mj_forward(model, data)
    mass_matrix = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass_matrix)
    return data.qfrc_bias.copy(), mass_matrix


def sweep_elbow(model, data, angles):
    """Vary elbow angle while keeping all other joint angles fixed."""
    limits = model.joint('elbow_flex').range
    sweep = np.linspace(limits[0] + 0.05, limits[1] - 0.05, 61)
    gravity, inertia = [], []
    indices = [int(model.joint(n).dofadr[0]) for n in NAMES]
    for angle in sweep:
        g, mass = inspect_pose(model, data, dict(angles, elbow_flex=angle))
        gravity.append(g[indices])
        inertia.append(np.diag(mass)[indices])
    gravity, inertia = np.array(gravity), np.array(inertia)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    path = RESULTS_DIR / 'dynamics_elbow_sweep'
    with path.with_suffix('.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['elbow_deg'] + [f'{n}_gravity_Nm' for n in NAMES] + [f'{n}_Mii_kg_m2' for n in NAMES])
        for angle, g, diagonal in zip(sweep, gravity, inertia):
            writer.writerow([np.rad2deg(angle), *g, *diagonal])
    colors = ['#2563eb', '#d97706', '#16a34a', '#dc2626', '#9333ea']
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="950" height="650">',
           '<rect width="950" height="650" fill="white"/>',
           '<text x="70" y="25" font-family="sans-serif">MuJoCo model prediction: elbow sweep, other joints fixed</text>']
    for panel, (values, label) in enumerate([(gravity, 'Gravity compensation torque [N m]'),
                                            (inertia, 'Inertia matrix diagonal [kg m²]')]):
        low, high = float(values.min()), float(values.max())
        padding = max((high-low)*0.1, 1e-6)
        low, high = low-padding, high+padding
        top = 70 + panel*280
        svg.extend([f'<text x="70" y="{top-10}" font-family="sans-serif">{label}</text>',
                    f'<path d="M70 {top} V{top+190} H710" stroke="#888" fill="none"/>',
                    f'<text x="5" y="{top+12}" font-size="12">{high:.4g}</text>',
                    f'<text x="5" y="{top+190}" font-size="12">{low:.4g}</text>',
                    f'<text x="70" y="{top+210}" font-size="12">{np.rad2deg(sweep[0]):.1f}°</text>',
                    f'<text x="600" y="{top+210}" font-size="12">{np.rad2deg(sweep[-1]):.1f}° elbow</text>'])
        for i, (name, color) in enumerate(zip(NAMES, colors)):
            x = 70 + 640*(sweep-sweep[0])/(sweep[-1]-sweep[0])
            y = top + 190 - 190*(values[:,i]-low)/(high-low)
            points = ' '.join(f'{a:.2f},{b:.2f}' for a,b in zip(x,y))
            svg.extend([f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>',
                        f'<text x="730" y="{top+20+i*25}" fill="{color}" font-family="sans-serif">{name}</text>'])
    svg.append('</svg>')
    path.with_suffix('.svg').write_text('\n'.join(svg)+'\n')
    print(f'Saved {path}.csv and .svg (software predictions only)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angles-deg', type=float, nargs=5, default=[0]*5,
                        help='Pan, lift, elbow, wrist flex, wrist roll (default all zero)')
    parser.add_argument('--viewer', action='store_true', help='Display a static pose; no integration')
    parser.add_argument('--sweep', action='store_true', help='Plot gravity and inertia versus elbow angle')
    args = parser.parse_args()
    model, data = load_model()
    angles = dict(zip(NAMES, np.deg2rad(args.angles_deg)))
    gravity, mass = inspect_pose(model, data, angles)
    joint_order = [model.joint(i).name for i in range(model.njnt)]
    print(f'MuJoCo {mujoco.__version__}: {model.njnt} joints; gripper angle fixed at zero')
    print('Joint order:', joint_order)
    print('Gravity compensation torque [N m]:', np.round(gravity, 5))
    print('Inertia matrix M(q) [kg m²]:\n', np.round(mass, 6))
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    (RESULTS_DIR/'dynamics_pose.json').write_text(json.dumps({
        'mujoco_version': mujoco.__version__, 'joint_order': joint_order,
        'arm_angles_deg': args.angles_deg, 'gravity_compensation_Nm': gravity.tolist(),
        'mass_matrix_kg_m2': mass.tolist(),
        'note': 'URDF prediction, no actuator/gear/friction identification; no real-arm torque measurement.',
    }, indent=2)+'\n')
    if args.sweep:
        sweep_elbow(model, data, angles)
        inspect_pose(model, data, angles)
    if args.viewer:
        from mujoco import viewer as mujoco_viewer
        with mujoco_viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0.15, 0, 0.15]
            viewer.cam.distance = 0.8
            while viewer.is_running():
                viewer.sync()
                time.sleep(1/60)


if __name__ == '__main__':
    main()
