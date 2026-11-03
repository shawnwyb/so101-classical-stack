Run experiments from the repository root with the `lerobot` environment active:

```bash
# Read joint angles and calculate FK; does not command motion.
python -m experiments.inspect_fk --save fk_reading.json

# Hardware shoulder-pan test.
python -m experiments.test_shoulder_pan

# Software-only IK check using results/ik_start_pose.json; no hardware connection.
python -m experiments.test_cartesian_lift --dry-run --elbow-p 32

# Hardware 50 mm lift; temporarily change elbow P and restore it on exit.
python -m experiments.test_cartesian_lift --elbow-p 32
```

`results/` contains the recorded JSON data, including failed runs. Relative
`inspect_fk --save` paths are resolved inside this directory; absolute paths are
used as given. Other experiments generate timestamped filenames there.

Scripts were renamed from `check_fk.py`, `pan_step.py`, and `ik_step.py`.
Historical log filenames are preserved, so references to individual trials remain
recognizable; new motion logs use `shoulder_pan_` and `cartesian_lift_` prefixes.
`ik_up_10mm.json` references its starting pose by repository-relative path.

The lift sends targets directly and holds at the lifted pose until release.
Support the arm's full weight before releasing motors. The shoulder-pan test
returns to its starting pan angle automatically before releasing that motor.

Current observations: rough FK agreement; position-only IK demonstrated on the
arm. Unsupported elbow error was about 3.62 degrees at P=16, 2.06 at P=24, and
0.80 at P=32 in slightly different starting poses. FK-estimated rises were 31.27,
39.17, and 45.43 mm for 50 mm targets. These are not precise physical tip accuracy
measurements; calibration offsets and full pose accuracy remain unverified.
