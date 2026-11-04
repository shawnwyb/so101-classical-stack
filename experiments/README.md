Run experiments from the repository root with the `lerobot` environment active:

```bash
# [Hardware, moves arm/gripper] Pick, lift 10 cm, lower, release, and return.
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

`view_cameras` opens a desktop window using the saved Linux camera profiles in
[`camera_settings.json`](camera_settings.json). It applies and verifies the image
controls every time it opens a camera. Use **Record
all** / **Stop recording** (or **R**) to toggle recording; **Esc** or closing the
window finishes the files and exits. Previewing alone saves nothing. Camera errors
appear below each preview.

```bash
python -m experiments.view_cameras --list
python -m experiments.view_cameras --config experiments/camera_settings.json
# Inspect other cameras without applying profiles (does not reset existing controls).
python -m experiments.view_cameras --no-config --camera overhead=/dev/video0 --camera wrist=/dev/video2
```

Profiles use roles (`closeup`, `wide`) and `/dev/v4l/by-id/...-video-index0` device
paths instead of changing `/dev/videoN` numbers. On another Linux PC, run `--list`,
check which view belongs to each camera, and update the profile device paths if
needed. The wide camera has serial `202404160005`; the close-up camera does not
expose a unique serial, so check its mapping if more identical cameras are added.
`--camera ROLE=PATH` can override a configured role's path. With `--no-config`,
discovery prefers by-id paths and otherwise tries `/dev/video*`; select working
devices explicitly if metadata nodes fail. The viewer requests
640×480 at 30 fps; use `--width`, `--height`, and `--fps` to change these settings.
The viewer reports an error if negotiated settings differ from the request.

Both cameras use **automatic exposure** (`exposure_auto=3`), with no fixed exposure
time applied. White balance is also automatic on both. Check moving gripper visibility,
brightness changes, and motion blur before collecting training data. Neither camera exposes
a standard gain control. Other image controls are explicitly saved per camera.

Missing devices and unsupported/rejected controls appear as errors rather than
silently accepting another setup. Capture FPS readback is nominal: observed frame
rate can be lower, and is shown in the preview and logged on recording completion.
Configured controls are read back again when recording starts. Keep other
camera-control applications closed during a run. `--no-config` skips application
of settings; it does not undo previous settings. To use manual exposure in a
profile, set `exposure_auto` to `1` and add `exposure_absolute` in units of 100
microseconds, then restart the viewer. An inactive manual exposure value does
not reveal the actual exposure while automatic mode is enabled.

For repeatable ML experiments, also document mount positions and angles (with setup
photos and measurements), light placement/type, background, and any changes between
sessions. These physical measurements have not been recorded for this setup yet.
Reuse the camera configuration in training-data collection and policy deployment;
this viewer does not configure a separate LeRobot process automatically. Different
camera models may need different control values and may not support this profile.

Each recording gets a UTC timestamp directory under `experiments/camera_recordings/`
(ignored by Git), with one MJPEG `.avi` and one `.settings.json` per camera. Each
settings file records the role/device, configured controls, actual control readback
and supported ranges, requested/negotiated capture format, OpenCV version, and frame
count/observed FPS after a clean stop. Use `--output-dir PATH` to save
elsewhere. Files contain camera frames without preview overlays or audio. Streams
are independent, with no hardware synchronization; AVI uses the negotiated nominal
frame rate, so slow capture or dropped frames can affect playback timing.

The viewer uses OpenCV, Pillow, and Tk; it works with headless OpenCV because Tk
displays the previews. These are available in the current `lerobot` environment.
If needed, install `opencv-python-headless` and `pillow` with pip and `tk` with Conda.

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
  → close → lift 10 cm → lower to grasp → open → retreat → return to startup.
  The cycle runs automatically after the start prompt, without a top hold.
  To rehearse without an object, use the same command and skip scoring with Enter.
- Pick settings: minimum 1 s per segment, 60°/s maximum speed, 60°/s² maximum
  acceleration, and 50 Hz commands. Segments take longer when limits require it;
  each ends at zero planned velocity with no extra settling wait.
- Gripper positions: 70% open, 38.2% closed, with a 0.5 s movement wait.
  Pick temporarily sets elbow P=32 and gripper effort to at most 200/1000,
  then restores previous settings. Edit constants in `pick_primitive.py` for tuning.
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
- MuJoCo uses ideal torque control and model mass properties; it does not reproduce
  the Feetech controller, friction, or gear play. `Present_Load` is not directly
  torque in N·m. The compensated run starts at its target with exact model knowledge.
- Fully support the arm's weight before releasing motor holding force.
