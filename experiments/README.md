Run experiments from the repository root with the `lerobot` environment active:

```bash
# [Hardware, moves arm/gripper] Pick, lift 20 cm, lower, release, and return.
python pick_primitive.py

# [Hardware, read-only] Read joint angles and calculate FK.
python -m experiments.inspect_fk --save fk_reading.json

# [Cameras only, no arm control] Desktop previews; R or Record all toggles recording.
python -m experiments.view_cameras

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

**Camera viewer**

```bash
python -m experiments.view_cameras
python -m experiments.view_cameras --list
```

Click **Record all** or press **R** to start/stop recording. **Esc** closes the
viewer and finishes the files. Previewing alone saves nothing; no arm commands
are sent.

Settings are saved in [`camera_settings.json`](camera_settings.json) and reapplied
and verified at startup. Closeup uses **automatic exposure**; wide uses **90 ms
manual exposure** (`exposure_absolute=900`). White balance is automatic on both.
Longer exposure brightens the image but can blur motion and reduce FPS.

**What the camera tests showed**

The cameras advertise 30 fps, but actual capture was slower. These measurements
used the current lighting and **automatic exposure**, except where noted:

| Test | Measured FPS |
| --- | --- |
| Wide alone, four capture buffers | About 20 |
| Both on the USB hub, after viewer fixes | Closeup about 6; wide about 15 |
| Closeup with a temporary 10 ms manual exposure | About 23, with a darker image |

A single capture buffer dropped frames. Using four buffers and starting closeup
before wide improved delivery; sequential startup raised wide from about 10 to
15 fps with both connected. Reducing processing overhead cut preview work from
about 23 ms to 4 ms per refresh.

Shorter wide exposure, different resolutions, and MJPEG versus YUYV did not lift
wide above about 20 fps when tested alone. The remaining camera/firmware versus
USB/driver limit is unresolved. These results do not establish the frame rate at
the current 90 ms manual setting. Read the live FPS below each preview; it measures
recent capture, rather than the requested rate.

**Recording and reproducing the setup**

Recordings go to `experiments/camera_recordings/<UTC timestamp>/` (ignored by Git),
with a separate `.avi` and `.settings.json` for each camera. Settings files include
requested/read-back controls, capture format, and frame count/measured FPS after
stopping. Streams have no audio or hardware synchronization. AVI uses nominal FPS,
so slow capture can make playback faster than real time.

On another Linux PC, run `--list` and check the profile device paths and views.
Wide has serial `202404160005`; closeup has no unique serial. Keep camera mounts
and lighting consistent, and apply the same settings in any separate LeRobot
recording/deployment process. The viewer does not configure those processes.

Use `--config PATH` for another profile, `--output-dir PATH` for another recording
folder, and `--width`, `--height`, or `--fps` to override capture requests. For
unconfigured cameras, use `--no-config --camera NAME=/dev/videoN`; this skips saved
controls but does not reset existing device settings. Missing devices and rejected
settings are reported as errors.

Dependencies: OpenCV, Pillow, and Tk (available in the `lerobot` environment).
Headless OpenCV works because Tk displays the images.

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

- Pick-and-lift uses `grasp_pose.json`: open → approach 3 cm above grasp → descend
  → close → lift 20 cm → lower to grasp → open → retreat → return to startup.
  The cycle runs automatically after the start prompt, without a top hold.
- Pick settings: minimum 1 s per segment, 60°/s maximum speed, 60°/s² maximum
  acceleration, and 50 Hz commands. Segments take longer when limits require it;
  each ends at zero planned velocity with no extra settling wait.
- Gripper positions: 70% open and the taught `gripper_percent` from `grasp_pose.json`
  for closing, with a 0.5 s wait after each
  position command. The script does not adjust gripping pressure, detect contact,
  or change the gripper's torque limit or stored overload settings.
- Pick temporarily sets elbow P=32 and restores its previous value afterward.
  Motor faults abort the cycle. Cleanup asks for arm support, releases motors,
  and verifies gripper torque-off even when an alarm affects normal status reads.
  Firmware protection remains enabled; shutdown does not clear faults through motion.
- Kinematics and trajectory math use radians, metres, and seconds.
- IK and trajectory checks require the local `so101_follower` calibration file
  and use its full recorded ranges. MuJoCo retains the URDF model limits.
- LeRobot arm commands use degrees; gripper commands use normalized travel 0–100.
- Trajectory tracking defaults to two seconds outward and two seconds reset,
  with a requested 50 Hz command rate.
- Tracking temporarily sets elbow P=32 and restores the previous value afterward.
- The standalone tracking experiment uses 20°/s and 40°/s² limits.
  All listed motion limits are experiment settings, not manufacturer ratings.

**Results**

`results/` contains JSON logs, CSV samples, and SVG plots. Representative records:

Pick runs save `pick_trials_<timestamp>.json` with tracking, gripper readings,
operator-scored outcomes, and failure reasons. Skipped outcomes remain unscored.

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
- Place the object at its taught location on a surface that supports it after
  release. Actual lift height depends on tracking. Gripper positions and effort
  settings are not measured grip force; adjust them for the object. A previous
  tighter close command caused overload on reopening; firmware protection remains enabled.
  Position commands do not measure or maintain grip force. Grip retention and
  overload avoidance remain unverified; recover an existing fault before retrying.
- MuJoCo uses ideal torque control and model mass properties; it does not reproduce
  the Feetech controller, friction, or gear play. `Present_Load` is not directly
  torque in N·m. The compensated run starts at its target with exact model knowledge.
- Fully support the arm's weight before releasing motor holding force.
