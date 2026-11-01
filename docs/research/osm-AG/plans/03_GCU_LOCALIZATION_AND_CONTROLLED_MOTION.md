# Checkpoint 03 plan: localization and controlled motion

Status: finalized for bounded simulation checks, 2026-11-01. This is the plan
and detailed measurement history. The achievement record is
[Checkpoint 03](../phases/03_GCU_LOCALIZATION_AND_CONTROLLED_MOTION.md). Previous evidence: [phase index](../phases/README.md).

## Current status

The bounded Phase 03 acceptance suite is complete: local uncertain-prior recovery,
a direct simulated-controller command-timeout check and three fresh-spawn short
motion repeats all passed the frozen smoke criteria. The user also reproduced
motion in Foxglove. See the achievement record for the final result table and
reproduction commands. The strict grazing-edge sensor predicate remains a known
qualified limitation; autonomous navigation, contact validation and global recovery
are not claimed. Sections below retain the investigation history, including
previous failures and statements that were current at those earlier stages.

## Gate A: stationary sensor consistency

Run simulator and alignment with known spawn, then the read-only probe:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
python3 /home/ubuntu/edgeosmag_test_ws/check_gcu_stationary.py \
  --seconds 60 --output /home/ubuntu/edgeosmag_test_ws/results/stationary_report.json
```

Use the ROS domain of the running simulation. The automatic isolated test uses
78 and the `gcu_stationary_measurement` Gazebo partition; ordinary Foxglove uses
77. The duration starts once map, simulation clock and timestamped scan/base
transforms are available. It is wall time; elapsed simulation time is reported
separately. NumPy and ROS 2 Python packages are required.

The probe subscribes only: scan, map, images, clock and Twist command topics. It
compares the last scan to first occupied raster cells along each ray at quarter-cell
sampling. Unknown cells are not physical walls. It reports unexpected short returns,
missing expected returns, infinities/NaNs, finite out-of-range values, scan noise,
TF lookup retries and odometry-based pose drift. It does not modify sensor data or
replace infinities with maximum-range obstacle points.

Predeclared diagnostic gates: at least 10 transformed scans, advancing simulation
clock, zero observed nonzero commands, <= 2 cm pose drift and <= 1 degree yaw drift;
at least 100 compared returns, p95 absolute range error <= 3 map cells (~0.176 m),
and no returns more than 3 cells unexpectedly short or expected returns missing
well inside maximum range. These are initial diagnostic thresholds, not paper
performance claims. Known-spawn odometry stability is not localization accuracy.
Camera reception and all invalid-value counts are reported separately.

If the range gate fails, investigate sensor rendering/geometry/TF before AMCL.
The map and world share an occupancy source, so this is internal consistency,
not independent real-building validation.

## Gate B: AMCL while stationary

Keep the map server and wheel EKF; replace (never duplicate) the fixed map-to-odom
publisher with AMCL. Initialize at the known pose, verify frame ownership,
convergence and stationary alignment. Freeze configuration and evaluation
thresholds before measured trials. Ground truth is for evaluation only.

## Gate C: controlled motion and recovery

Validate a clear route and footprint first. Use bounded low-speed motion,
confirm command timeout/stop, collect straight and rotation trials, then a short
return route. Compare localization with Gazebo ground truth. Test a deliberate
small initial-pose error and recovery. Record success, collision observations,
position/heading error, convergence time, simulation-time duration, wall time and
resource usage. Do not enable the planner's unvalidated straight-line fallback.

## Completion record

Once these gates pass, create `phases/03_GCU_LOCALIZATION_AND_CONTROLLED_MOTION.md`
with configuration, commands, data paths, acceptance results, limitations and next
gate. Autonomous osmAG room-to-room navigation is a subsequent checkpoint.

## Investigation log

The first no-motion trial (Ogre 1) measured 60.001 wall seconds / 53.086 simulation
seconds, 531 timestamp-transformed scans and 1,552 camera images. Pose drift was
zero at reported precision, median scan error 0.00963 m, but p95 error 2.1367 m
and 36 unexpectedly short returns. There were 39 positive infinities, no NaNs,
no negative infinities, no finite out-of-range values and no missing expected
returns. Stationary gate passed; range gate failed.

A desktop-connected Ogre 2 comparison now starts successfully, unlike the earlier
attempt without a working display. It also fails the range gate (p95 ~4.08 m,
87 unexpectedly short returns), so changing the renderer alone does not resolve
sensor correctness. The existing Ogre 1 workflow is retained for the controlled
geometry experiment.

Gazebo ground truth for the original model shows ~1.09 degrees nose-down pitch.
Drive-wheel bottoms are 45 mm below base_link; the original caster bottom is
47 mm below it. That 2 mm support mismatch can tilt the scanner into the floor,
producing a line of real floor returns that a planar navigation map cannot explain.
The candidate fix sets the caster joint to -29 mm (its collision adds -1 mm and
15 mm radius) and raises the base_link-to-footprint offset from 38.75 to 45 mm.
A corrected model is tested via `urdf_path` in an isolated workspace, without
changing the previous installed robot workspace or filtering scan returns.
Before/after measurements are stored in
[`phase3_diagnostics`](../../../../simulation/gcu_ground_floor/phase3_diagnostics/).

The corrected Ogre 1 trial measured 60.002 wall seconds / 52.516 simulation seconds,
526 transformed scans and 1,491 camera images. Ground-truth pitch dropped to
approximately 0.0000056 degrees. Median range error was 0.00895 m, p95 0.02811 m,
with no missing expected returns. One short-return outlier remains at index 241
(angle 61.640 degrees): 3.765 m observed versus 4.090 m raster first-hit.
A ray only 0.1 degrees farther around the corner hits at 3.753 m, consistent with
a grazing-edge/render-sampling effect; that explanation is not yet independently
proven. The predeclared strict gate remains failed rather than being relaxed.
The physical floor-intersection prediction for the original tilt is 6.568 m,
matching the original forward scan return (~6.57 m). This, together with the
corrected-model comparison, supports the caster support mismatch as the cause of
the broad floor-return line.

## Reproduce with corrected model in the existing VM

The corrected Xacros are in `/home/ubuntu/edgeosmag_test_ws/robot_profile/`.
Stop the old simulator and alignment launch, then restart the simulator with its
usual domain/partition and this additional argument:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/lidarbot_ws/install/setup.bash
export ROS_DOMAIN_ID=77
export IGN_PARTITION=gcu_foxglove
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_DRI3_DISABLE=1
ros2 launch lidarbot_gz gz_launch.py \
  world:=/home/ubuntu/edgeosmag_test_ws/assets/gcu_ground_floor_ogre.sdf \
  urdf_path:=/home/ubuntu/edgeosmag_test_ws/robot_profile/lidarbot_gz.urdf.xacro \
  x_pose:=-0.05864395131342248 y_pose:=-12.960313240266368 \
  z_pose:=0.15 yaw_pose:=0.0 use_robot_localization:=false
```

Restart the existing alignment launch and leave Foxglove Bridge in domain 77.
The isolated automated trials use domain 78 and stop afterwards. Installed files
in the previous `/home/ubuntu/lidarbot_ws` were not changed. For a fresh checkout,
build the modified `lidarbot_gz` package or supply its source URDF path, retaining
its neighboring `sensors.xacro` and `lidarbot_gz_core.xacro` includes.


Final renderer comparison on the corrected model: Ogre 2 measured 60.001 wall
seconds / 48.361 simulation seconds, p95 range error 0.1710 m and 11 unexpectedly
short returns. It does not improve on corrected Ogre 1 (0.0281 m, one edge outlier).
Retain corrected Ogre 1 as the current profile. All four automated launches stopped
cleanly; zero velocity commands were sent. The strict Gate A is still open for the
one grazing-edge return. AMCL and controlled motion have not been started. Six local
Python tests pass, including synthetic raycast checks and the existing geometry
contract. These changes remain local pending completion/review of this checkpoint.

## Gate B implementation: stationary AMCL

The user visually confirmed the corrected scan and camera on 2026-11-01. We are
proceeding with a bounded, no-motion AMCL test while preserving the strict Gate A
failure and grazing-edge limitation. This is an explicit scope decision: the
bulk scan error is resolved, but the sensor gate is not relabeled as passed and
movement remains deferred.

`scripts/gcu_amcl.launch.py` starts the map server, the same wheel-velocity EKF,
AMCL and an ordered lifecycle manager (map server, then AMCL). It starts **no fixed
map-to-odom publisher**. Configuration is in
`simulation/gcu_ground_floor/amcl.yaml`. The initial pose is a one-shot `/initialpose` prior at
the known spawn, with 0.25 m position and 10 degree heading standard deviations,
not a ground-truth feed. `gcu_amcl_initial_pose.py` waits for active AMCL, odometry
and simulation clock before publishing it. No Nav2 controllers or goals are started.

Predeclared stationary smoke-test criteria: active AMCL, received pose estimates,
working map-to-lidar TF, dynamic map-to-odom messages with no static map-to-odom,
at least 10 scans, images and advancing clock, no observed nonzero commands,
final pose within 0.25 m / 5 degrees of the known stationary spawn. These thresholds
are smoke checks, not moving-localization accuracy requirements. The evaluator also requires nondegenerate pose covariance. The diagnostic
requests three explicit no-motion updates, because the ordinary AMCL filter is
motion-triggered; it does not publish initial poses or commands.

### Run in the existing Ubuntu/Foxglove setup

Leave the corrected simulator and Foxglove Bridge running. Stop the **entire old
alignment launch** using Ctrl-C: it owns the temporary fixed transform, old EKF
and map server. Do not run both localization launches together. Then:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/lidarbot_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch /home/ubuntu/edgeosmag_test_ws/gcu_amcl.launch.py \
  map:=/home/ubuntu/edgeosmag_test_ws/assets/gcu_ground_floor.yaml \
  params:=/home/ubuntu/edgeosmag_test_ws/assets/amcl.yaml
```

Keep fixed/display frames at `map` in Foxglove. Show `/map`, `/scan`, transforms
and camera; add a Raw Messages panel for `/amcl_pose` and optionally the
`/particle_cloud` topic if available. In another sourced domain-77 terminal:

```bash
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map base_footprint
python3 /home/ubuntu/edgeosmag_test_ws/check_gcu_amcl.py \
  --seconds 45 --output /home/ubuntu/edgeosmag_test_ws/results/amcl_user_report.json
```

Expect lifecycle `active`, pose near (-0.059, -12.960), small heading near zero,
and scan/map agreement. A few early scan drops can occur before the first odometry
TF; persistent drops or two map-to-odom publishers are failures to investigate.
The checker emits zero movement commands. Do not test goal publishing yet.

On a fresh checkout, use the repository launch/config/script paths instead of the
isolated VM copies. The corrected simulator profile and installed ROS dependencies
remain prerequisites. Parameter reference:
[Nav2 AMCL](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/).


An initial integration-only trial was rejected because the parameter-only initial
pose produced essentially zero covariance and an estimate identical to the seed.
The launch now disables automatic parameter seeding and publishes one explicit
nonzero-uncertainty `/initialpose` prior. That is separate from the evaluator,
which never sends initial poses. The rejected report is retained for traceability;
it is not counted as localization acceptance.


### Stationary AMCL result (2026-11-01)

The revised nonzero-prior trial passed its stationary smoke criteria in the isolated
UTM workspace. Over 45.000 wall seconds / 36.701 simulation seconds it received
367 scans, 1,055 images and three AMCL pose estimates, after three explicit
no-motion update requests. AMCL lifecycle was active; 370 dynamic map-to-odom
messages and no static map-to-odom were observed. The final estimate was
(-0.05192, -12.93646), yaw -0.003991 rad: 0.02479 m and 0.22868 degrees from the
known stationary spawn. Variances remained nonzero (x 0.03776 m², y 0.03562 m²,
yaw 0.01030 rad²). These uncertainties are not equivalent to the small realized
error of this single seeded trial. No velocity commands were sent or observed.

Report: [`amcl_stationary_report.json`](../../../../simulation/gcu_ground_floor/phase3_diagnostics/amcl_stationary_report.json).
Known-spawn initialization plus stationary updates is not recovery, global
localization, or a moving ground-truth accuracy result. Gate C remains pending;
Gate A's one grazing-edge strict discrepancy remains explicitly open. The current
AMCL user command above is ready for a visual check without moving the robot.

## Gate C implementation: bounded simulated motion

`gcu_evaluation.urdf.xacro` wraps the corrected robot with a Fortress PosePublisher
at 20 Hz. Its model pose is bridged to `/gcu_ground_truth` as TFMessage, **never to
`/tf` or AMCL inputs**. This avoids the missing frame/timestamp information in the
world SceneBroadcaster pose topic. Ground truth is used by the test supervisor to
bound distance and heading and by the evaluator; it is not a deployed navigation
controller or an independent AMCL input.

The first experiment is a 0.4 m forward move at commanded 0.06 m/s, stop, 45 degree
left rotation at commanded 0.15 rad/s, then stop and observe. The planned straight
segment has >3.3 m wall clearance; the executable check also treats unknown map
cells as blocked. The reference radius is 0.18 m and required path clearance is
0.5 m. The script refuses a non-fresh spawn and requires advancing simulation time,
LiDAR, AMCL, ground truth and a command subscriber. It aborts on stale streams,
obstacles within 0.5 m, displacement >0.65 m, missing matched TF or timeouts.
Zero Twist messages are sent at shutdown. It requires explicit `--execute`.

Predeclared acceptance: straight distance 0.4 ±0.05 m; turn 45 ±5 degrees;
post-settle translational drift <=0.02 m; at least 30 timestamp-matched pose pairs;
p95 localization error <=0.25 m and <=5 degrees. These are initial smoke thresholds,
not evidence of route safety. There is no contact sensor in this experiment;
collision-free navigation cannot be inferred solely from a clear planned segment.

The first launch stopped at readiness without starting movement: the checker
started after AMCL's initial stationary pose message. One explicit no-motion update
request is now issued during readiness so it can observe a fresh pose. This
integration rejection is retained separately from measured movement results.

The isolated harness uses ROS domain 78 and Gazebo partition
`gcu_stationary_measurement`, and cleans up afterwards. The user's Foxglove session
in domain 77 is not driven by these experiments.

### First measured movement and simulation wheel calibration

The first measured movement completed 0.40918 m forward and a 46.301 degree left
turn, with zero measured post-settle translation drift. It produced 337
same-timestamp ground-truth/estimated pose pairs. P95 position error was 0.07798 m,
but p95 heading error was 6.232 degrees, failing the predeclared 5 degree gate.
The result is retained as `motion_original_wheelbase_report.json`.

The shared controller file used 0.180 m wheel separation, while the simulated
sphere collision centers are separated by 0.117 m: joint y coordinates +/-0.065 m,
with 0.0065 m inward collision offsets after wheel-frame rotation. A separate
`lidarbot_gz/config/controllers_sim.yaml` now matches that effective contact geometry.
Only the Gazebo URDF uses this simulation config; the hardware/shared file is
unchanged. The isolated profile resolves it to its copied test asset. The same
motion test is repeated without changing acceptance thresholds or AMCL tuning.

### Reproduce the controlled-motion test in Foxglove

This is optional bounded simulation motion, not an autonomous navigation goal.
Stop the old simulator and AMCL launch. Restart the corrected simulation using
the earlier command, but set:

```text
urdf_path:=/home/ubuntu/edgeosmag_test_ws/robot_profile/gcu_evaluation.urdf.xacro
```

Keep `use_robot_localization:=false`, domain 77 and partition `gcu_foxglove`.
Restart `gcu_amcl.launch.py` as above and keep Foxglove Bridge running. The evaluation
wrapper adds ground truth without changing the robot contact geometry. In another
Ubuntu terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
export IGN_PARTITION=gcu_foxglove
ros2 run ros_gz_bridge parameter_bridge \
  '/model/lidarbot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  --ros-args -r /model/lidarbot/pose:=/gcu_ground_truth
```

Then, after a fresh spawn with no other velocity publishers issuing commands:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
python3 /home/ubuntu/edgeosmag_test_ws/check_gcu_motion.py \
  --execute --map /home/ubuntu/edgeosmag_test_ws/assets/gcu_ground_floor.yaml \
  --output /home/ubuntu/edgeosmag_test_ws/results/motion_user_report.json
```

The script moves the robot 0.4 m forward, stops, turns left about 45 degrees and
stops again. Watch map/scan alignment, camera and robot transform in Foxglove.
Do not repeat from the final displaced pose: restart the simulator and AMCL first.
Ctrl-C the test to send zero commands through its cleanup block. The controller
also has its own velocity-command timeout. These scripts are for simulation only.

The wheel-calibrated repeat completed 0.40798 m and 46.128 degrees, with p95
position error 0.07000 m and yaw error 5.277 degrees. It still failed the unchanged
5 degree heading criterion. The subsequent cadence experiment increases max beams
from 60 to 120, reduces linear/angular update triggers from 0.1 to 0.03 m/rad,
and reduces transform postdating from 1.0 to 0.2 seconds. The prior uncertainty,
speed/distance/turn and acceptance thresholds are unchanged. Snapshots preserve the
stationary and motion parameter versions. This is tuning on a small simulation
trial, not a held-out validation or Pi performance benchmark.

The Gazebo package CMake manifest installs its `config` directory. It is now
exempted from the broad repository `*.txt` ignore so the simulation-only controller
profile can be included in a future package build. The existing installed Ubuntu
workspace is left unchanged; isolated Xacros use the copied profile explicitly.


### Latest bounded-motion result

With the unchanged 0.4 m / 45 degree commands and unchanged acceptance thresholds,
the 120-beam/finer-cadence configuration passed. Measured travel was 0.40798 m,
turn 45.847 degrees, and post-settle translation drift zero at reported precision.
There were 375 timestamp-matched pose pairs; p95 localization error was
0.06844 m and 2.656 degrees. Motion/settling took 21.418 simulation seconds;
the checker ran for 29.364 wall seconds including readiness/cleanup.
Ground truth was used only for evaluation and bounded test supervision, not AMCL.
All test launches were stopped afterwards. No domain-77 session was driven.

Results: [`motion_report.json`](../../../../simulation/gcu_ground_floor/phase3_diagnostics/motion_report.json)
and [`motion_summary.json`](../../../../simulation/gcu_ground_floor/phase3_diagnostics/motion_summary.json).
The stationary profile snapshot remains separate from the current motion profile.
Nine Python tests pass, including rejection of unknown, occupied and outside-map
motion segments. No contact sensor, door crossing or autonomous route was tested.
The controller watchdog was not independently challenged by suppressing commands;
this experiment tested explicit stop commands and post-settle stability.

Next is controlled initial-pose-error recovery plus repeated motion trials, then
footprint-feasible short navigation. Do not equate this tuned single-trial pass
with the complete localization/navigation milestone.


### User-run visual and quantitative confirmation (2026-11-01)

The user reproduced the bounded motion sequence in the domain-77 Foxglove setup
and supplied terminal JSON plus an 82.017-second screen recording. The complete
JSON was recovered from the pasted terminal text and the p95 errors recomputed
from all 414 timestamp-monotonic sample pairs; they match the reported metrics.

- Forward travel: 0.40798 m; left turn: 45.847 degrees.
- Post-settle translation drift: zero at reported precision.
- P95 position/heading errors: 0.03862 m / 2.1756 degrees.
- Maximum position error: 0.04649 m.
- Experiment duration: 21.394 simulation seconds; checker wall time: 34.983 seconds.
- All readiness flags true; motion sequence completed; predefined checks passed.

Sampled recording frames show the camera view changing and scan returns broadly
following the visible map walls. This qualitative check complements the logged
model motion; view zoom/panning is not treated as proof of movement. The recording
was inspected by sampled frames, not a continuous frame-by-frame review.

Artifacts: [`motion_user_report.json`](../../../../simulation/gcu_ground_floor/phase3_diagnostics/motion_user_report.json)
and [sampled Foxglove frame](../../../../simulation/gcu_ground_floor/phase3_diagnostics/user_motion_view_72s.png).
The 220 MB original recording remains on the user's Desktop and is not copied
into Git. This adds a successful manual reproduction of the short motion check;
incorrect-initial-pose recovery, independent command-timeout testing and broader
repeated trials remain pending before the full phase is closed.


## Final acceptance suite

`check_gcu_recovery_watchdog.py` declares its limits before running: recovery within
0.15 m / 3 degrees for three consecutive pose updates, within 30 simulation seconds;
watchdog stop within 1 second of silence, coast <=0.05 m, drift <=0.01 m, with no
explicit zero or other command before the observed stop. Recovery injects a fixed
(+0.20 m, -0.15 m, +8 degree) local prior and requests stationary sensor updates.
Watchdog uses the controller topic directly to avoid conflating mux stops with
controller timeout behavior. Cleanup zero commands are sent only after measurement.

`run_gcu_phase3_suite.py` starts fresh simulator/AMCL/ground-truth processes for each
of five cases, freezes the configuration, writes reports and cleans up. The final
suite passed all cases. See the achievement record for results, known exceptions,
portable workspace preparation, exact rerun command and provenance hashes.
Historical failures above remain retained and are not silently counted as passes.
