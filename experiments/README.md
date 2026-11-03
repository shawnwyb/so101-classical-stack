Run experiments from the repository root with the `lerobot` environment active:

```bash
# [Hardware, read-only] Read joint angles and calculate FK.
python -m experiments.inspect_fk --save fk_reading.json

# [Hardware, moves arm] Test a small shoulder-pan movement.
python -m experiments.test_shoulder_pan

# [Software, no hardware] Check IK using results/ik_start_pose.json.
python -m experiments.test_cartesian_lift --dry-run --elbow-p 32

# [Hardware, moves arm] Execute a 50 mm lift with temporary elbow P=32.
python -m experiments.test_cartesian_lift --elbow-p 32

# [Software, no hardware] Generate quintic trajectory samples and plots.
python -m experiments.preview_trajectory --duration 4

# [Software, no hardware] Validate the trajectory tracking plan.
python -m experiments.test_trajectory_tracking --dry-run

# [Hardware, moves arm] Execute 2 s outward, then 2 s smooth reset.
python -m experiments.test_trajectory_tracking

# [Hardware, moves arm] Override outward and reset durations.
python -m experiments.test_trajectory_tracking --duration 4 --reset-duration 2

# [Simulation, no hardware] Print model gravity torque and inertia.
python mujoco_dynamics.py

# [Simulation, no hardware] Display the model at a static pose.
python mujoco_dynamics.py --viewer

# [Simulation, no hardware] Plot gravity and inertia versus elbow angle.
python mujoco_dynamics.py --sweep

# [Simulation, no hardware] Compare PD holding with gravity compensation off/on.
python -m experiments.simulate_gravity_compensation
```

**Key findings**

- **FK:** predicted horizontal reach was approximately 21 cm; measured reach was
  21–22 cm. This establishes rough agreement.
- **Elbow gain:** P=16 → 24 → 32 reduced final elbow error from
  3.62° → 2.06° → 0.80°. FK-estimated rises improved from
  31.27 → 39.17 → 45.43 mm for a 50 mm target.
- **Load:** gently supporting the gripper reduced elbow error from 3.62° to
  1.71°, supporting a load-dependent tracking explanation.
- **Trajectory duration:** two-second motion looked smoother. Elbow RMS tracking
  error was similar: 1.836° at two seconds versus 1.792° at four seconds.
  Smooth appearance and position accuracy are different outcomes.
- **Dynamics:** changing elbow angle changed the model's gravity torque and
  inertia. At zero joint angles, predicted shoulder-lift and elbow holding
  torques were approximately −0.539 and −0.453 N·m.
- **Gravity compensation:** simulated PD alone settled with 4.04° shoulder-lift,
  3.22° elbow, and 3.31° wrist-flex error magnitudes. Adding exact-model gravity
  compensation reduced these errors to approximately zero.

Hardware comparisons used slightly different starting conditions. Results show
useful trends rather than precise controlled benchmarks.

**Defaults**

- Kinematics and trajectory math use radians, metres, and seconds.
- LeRobot arm commands use degrees; gripper commands use normalized travel 0–100.
- Trajectory tracking defaults to two seconds outward and two seconds reset,
  with a requested 50 Hz command rate.
- Tracking temporarily sets elbow P=32 and restores the previous value afterward.
- Reference velocity and acceleration limits are 20°/s and 40°/s². These are
  experiment settings, not manufacturer ratings.

**Results**

`results/` contains JSON logs, CSV samples, and SVG plots. Representative records:

- Gain comparison: [P=16](results/ik_step_20261103T010552_254617Z.json),
  [P=24](results/ik_step_20261103T011226_512554Z.json), and
  [P=32](results/ik_step_20261103T011420_641422Z.json).
- [Duration comparison](results/trajectory_tracking_20261103T013751_121855Z.json).
- [Gravity compensation](results/gravity_compensation_20261103T032817_287249Z.json).

Relative `inspect_fk --save` paths resolve inside `results/`. Preview and
model-inspection outputs overwrite their previous files; hardware trials and
compensation runs use timestamped filenames.

**Limitations and operating notes**

- IK currently controls position only. Full physical pose accuracy remains unverified.
- Joint-space trajectories do not guarantee straight tool paths or collision clearance.
- MuJoCo uses ideal torque control and model mass properties; it does not reproduce
  the Feetech controller, friction, or gear play. `Present_Load` is not directly
  torque in N·m. The compensated run starts at its target with exact model knowledge.
- Fully support the arm's weight before releasing motor holding force.
