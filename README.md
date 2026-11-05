# SO-101 Classical Stack

Controls an SO-101 arm to pick up an object at a saved grasp pose, lift it,
lower it, and return to its starting position. Forward kinematics (FK) calculates
the gripper position, inverse kinematics (IK) calculates approach and lift joint
angles, and quintic trajectories generate smooth joint commands sent through
LeRobot. Separate MuJoCo experiments examine gravity, inertia, and gravity compensation.

## Setup

From the repository root, with Conda installed:

```bash
conda create -n lerobot python=3.12
conda activate lerobot
python -m pip install "lerobot[feetech]==0.6.1" numpy "mujoco==3.17.0"
```

Follow the [LeRobot SO-101 setup and calibration guide](https://huggingface.co/docs/lerobot/en/so101).
The scripts expect follower ID `so101_follower`, USB port `/dev/ttyACM1`, and
Linux serial-port access. Calibration is loaded from
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json`.

The official `so101_new_calib.urdf` and mesh assets are under `models/so101/`.
The arm has five positioning joints plus a gripper.

## Run

Teach your own grasp before picking: position the released arm around the object,
close the fingers to a snug fit, then record the arm and gripper positions without
commanding motion:

```bash
python -m experiments.inspect_fk --save "$PWD/grasp_pose.json"
```

Place the object at that taught location, clear the route, then run:

```bash
python pick_primitive.py
```

Sequence: open → approach 3 cm above grasp → descend → close → lift 20 cm
→ lower → release → retreat → return to startup. Follow the holding/release prompts.
Motion settings and the gripper opening are constants at the top of `pick_primitive.py`;
the closing position is loaded from `gripper_percent` in `grasp_pose.json`.

## Scope and limitations

For desktop camera previews with optional recording, run
`python -m experiments.view_cameras`. Click **Record all** or press **R** to
start/stop recording. The viewer reapplies saved Linux camera profiles from
`experiments/camera_settings.json` and logs settings with each recording.
See [camera viewer options](experiments/README.md).

Picks require a taught grasp and fixed object placement. IK controls position only;
gripper orientation may change. Motion has no collision checking. Gravity
compensation is explored in simulation only. Measurements and trial logs are
documented in [experiments](experiments/README.md).
