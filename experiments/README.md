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

# Software-only quintic preview; writes a CSV and SVG in results/.
python -m experiments.preview_trajectory --duration 4

# Validate the tracking experiment without connecting to hardware.
python -m experiments.test_trajectory_tracking --dry-run

# Hardware default: 2 s outward, then 2 s smooth reset.
python -m experiments.test_trajectory_tracking

# Override the outward and reset durations independently.
python -m experiments.test_trajectory_tracking --duration 4 --reset-duration 2

# MuJoCo model inspection and a gravity/inertia sweep; no hardware connection.
python mujoco_dynamics.py
python mujoco_dynamics.py --viewer
python mujoco_dynamics.py --sweep

# Simulation-only PD holding comparison, with gravity compensation off/on.
python -m experiments.simulate_gravity_compensation
```

`results/` contains curated recorded JSON data, including representative failed runs. Relative
`inspect_fk --save` paths are resolved inside this directory; absolute paths are
used as given. Other experiments generate timestamped filenames there.

Scripts were renamed from `check_fk.py`, `pan_step.py`, and `ik_step.py`.
Historical log filenames are preserved, so references to individual trials remain
recognizable; new motion logs use `shoulder_pan_` and `cartesian_lift_` prefixes.
The software preview overwrites `trajectory_preview.csv` and `trajectory_preview.svg`.
Trajectory tracking saves timestamped JSON, CSV, and SVG files with a
`trajectory_tracking_` prefix. The SVG compares per-joint tracking errors during
the outward move and reset. They travel in opposite directions, so this plot is
not a controlled speed comparison. RMS and peak errors use actual-minus-command samples;
final holding errors are measured separately after two seconds. Logged command
and feedback times reveal host/serial delays; late reference samples are skipped
rather than sent in a burst. The default requested rate is 50 Hz, not a real-time
guarantee. Start offsets are reported because the reset may leave a holding error.
The outward move defaults to `--duration 2`; the reset defaults to
`--reset-duration 2`. There is no third movement. The two-second final observation pause is separate
from the movement duration.

Tracking uses elbow P=32 temporarily and verifies restoration on normal cleanup.
Default joint changes, in chain order, are `[0, 5, -10, 5, 0]` degrees. The CLI
accepts `--delta-deg` with five values. Reference trajectories are checked against
URDF angle limits and experiment limits of 20 degrees/s and 40 degrees/s²;
these are chosen test settings, not manufacturer ratings. Use `--max-velocity`
and `--max-acceleration` to set them explicitly. Joint-space interpolation does
not check collisions or guarantee a straight tool path. Keep hands clear during
both phases, including reset, and fully support the arm before final release.
`ik_up_10mm.json` references its starting pose by repository-relative path.

The lift sends targets directly and holds at the lifted pose until release.
Support the arm's full weight before releasing motors. The shoulder-pan test
returns to its starting pan angle automatically before releasing that motor.

Current observations: rough FK agreement; position-only IK demonstrated on the
arm. Unsupported elbow error was about 3.62 degrees at P=16, 2.06 at P=24, and
0.80 at P=32 in slightly different starting poses. FK-estimated rises were 31.27,
39.17, and 45.43 mm for 50 mm targets. These are not precise physical tip accuracy
measurements; calibration offsets and full pose accuracy remain unverified.

Trajectory finding ([recorded trial](results/trajectory_tracking_20261103T013751_121855Z.json)):
we shortened the outward move to 2 s, predicting larger per-command increments
would reduce visible stepping. The user observed noticeably smoother motion at
2 s than during slower motion, especially the 8 s reverse-direction reset.
Elbow RMS error was 1.836 degrees at 2 s versus 1.792 degrees at 4 s, with no
skipped reference samples. Final elbow errors were 2.088 and 2.176 degrees.
Starting offsets differed; reset also reverses direction, so this is not a
controlled proof that duration alone caused the visible difference. The evidence
supports keeping quintic interpolation and choosing a 2 s default for this small
joint move, rather than deliberately coarsening commands. Visible smoothness and
position accuracy are separate outcomes; gravity/holding error remains.
The latest [outward/reset trial](results/trajectory_tracking_20261103T020259_734784Z.json)
completed both phases at 2 s; this validates execution, not precise physical accuracy.
The historical trial used outward/reset/outward for duration comparisons; the
current script uses only outward/reset. Retained historical logs preserve their original data.

Other tracking defaults: requested period 0.02 s (50 Hz), elbow P=32 during the
test, velocity limit 20 degrees/s, acceleration limit 40 degrees/s², and joint
changes `[0, 5, -10, 5, 0]` degrees. Durations are checked against these reference
limits for each requested move; 2 s is not a universal duration for larger moves.

This README keeps runnable commands, settings, and measured findings beside the
experiment code. JSON/CSV logs retain the detail; this document explains what
changed, what we predicted, what we observed, and the remaining uncertainty.

Intermittent elbow startup failure: the run
`results/trajectory_tracking_20261103T014105_156097Z.json` failed enabling motor 3,
then reported release and gain-restoration errors. LeRobot raised RuntimeError
for a servo reply error, rather than ConnectionError for a missing reply. The
installed SDK leaves some numeric error bits unnamed, hence the blank message.
Read-only follow-up (`results/enable_failure_readonly.json`) found elbow position
3130, stored servo maximum 3129, and stored goal 3129. This supports a
position/calibration-boundary explanation for recovery after moving inward, but
does not prove the enable failure's cause: subsequent reads and a same-value
Lock=0 write returned no fault. The failing enable reply code was not captured.

The additional servo-range preflight checks and tracking write-error logging
were subsequently reverted at the user's request. The earlier URDF checks,
trajectory settings, and gain-restoration behavior remain. No servo limits,
protections, or calibration were changed. The startup fault remains unresolved;
the raw-code logging previously present in the Cartesian lift script remains.

Related reports: [SO-101 servo trouble](https://github.com/huggingface/lerobot/issues/2819)
includes torque-write failures but describes communication/overload symptoms, not
a proven identical cause. A [Feetech-based gripper investigation](https://github.com/Transcend-Mechanics/fastgripper-lerobot/blob/main/docs/multiturn-gripper-notes.md)
reports enable-time faults outside a limit window; its multi-turn gripper differs
from our elbow. [Feetech's register reference](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)
documents the actual servo min/max limits and fault registers. These reports are
supporting context, not confirmation of our specific fault.

A manual two-position startup check succeeded at both positions, including raw position 3130 with maximum 3129.
Thus a one-count range violation alone did not reproduce the intermittent fault.
At the user's subsequent request, the elbow maximum was changed from 3129 to
3130 in both the servo and external calibration JSON. A backup was saved beside
that JSON, and a fresh driver confirmed calibration agreement with all motors.
See `results/calibration_change_20261103T015736_431457Z.json` for the audit record.
No movement was commanded. This changes the LeRobot elbow zero reference by
about 0.044 degrees; historical logs retain their original calibration convention.
The change is not a demonstrated fix for the startup fault.

MuJoCo setup: installed `mujoco` 3.17.0 in the `lerobot` environment using
`python -m pip install mujoco`. `mujoco_dynamics.py` imports the existing official
URDF, sets mesh lookup and preserves fixed frames in memory, without editing the
source model. It reports the full six-joint gravity vector and inertia matrix at
zero velocity, with gripper angle zero. `--angles-deg` accepts the five arm angles.
`--viewer` displays a static pose without time integration; its desktop window
must be checked locally. `--sweep` varies elbow angle while keeping other joints
fixed and writes `results/dynamics_elbow_sweep.csv` and `.svg`. Each inspection
overwrites `results/dynamics_pose.json`.

Software checks: MuJoCo tool positions agreed with our FK at two poses; M was
symmetric positive definite; disabling model gravity yielded zero gravity bias.
At all-zero angles, predicted shoulder-lift and elbow compensation torques were
approximately -0.539 and -0.453 N m. These are model predictions, not measured
servo torques. Imported URDF mass properties omit identified servo-controller,
gear, friction, and camera/payload effects; Present_Load is not directly N m.

Results cleanup: removed 21 files comprising two exact duplicate JSON records,
uninformative/repeated startup and pose-check aborts, one superseded partial run,
empty startup CSV/SVG exports, regenerable preview outputs, a superseded
unmeasured FK snapshot, and the old three-move default trial with its exports.
Independent repeatability trials are not duplicates and remain. Kept the saved
IK seed, software IK example, hardware FK check, gain/load comparisons, 4-vs-8 s
and 2-vs-4 s comparisons, latest two-phase execution, calibration audit, and one
representative tracking startup failure. Regenerate the software preview with
the command above when needed.


Final dynamics experiment: `simulate_gravity_compensation.py` holds all six joints
at zero using the same PD gains in two independent five-second runs. Gravity
remains enabled in both; only the added gravity feedforward torque changes.
P gains in joint order are [8, 8, 8, 2, 0.3, 0.3] N m/rad; D gains are computed
as 2*sqrt(Kp*Mii) at the target. These are simulation gains, not Feetech register
values. Gravity feedforward uses a separate zero-velocity data object at the
current position, avoiding accidental Coriolis compensation. The controller
applies ideal torques through qfrc_applied, then mj_step advances physics.

Prediction: PD alone needs a position offset to counter gravity; adding model
compensation reduces that offset. In
[the recorded run](results/gravity_compensation_20261103T032817_287249Z.json),
final target-minus-actual errors without compensation were -4.040 degrees at
shoulder lift, -3.223 at elbow, and -3.309 at wrist flex. With compensation they
were numerically zero. Corresponding CSV contains state and torque samples; the
SVG plots actual angles and errors over time. Re-running creates timestamped
files. No hardware is connected or commanded.

Checks: final velocities and torque-balance residuals were below 1e-5 in their
respective units; halving timestep from 0.5 ms to 0.25 ms reproduced the final
uncompensated angles within 1e-6 rad. Joint limits were never reached. Contacts
are deliberately disabled to isolate gravity; there are no actuator torque
limits, model mismatch, or identified friction/gear/servo-controller dynamics.
The compensated run starts exactly at rest at its target, so near-zero error is
an ideal equilibrium demonstration, not a robustness or real-arm accuracy test.
This supports the qualitative load-error mechanism observed on the arm; it does
not measure Feetech torque or guarantee hardware improvement from feedforward.

