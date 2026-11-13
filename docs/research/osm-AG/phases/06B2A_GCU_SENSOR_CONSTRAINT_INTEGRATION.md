# 06B2A: stationary GCU sensor and constraint integration

## Quick summary

- **Scope:** Integrate real Gazebo LiDAR, AMCL and both Nav2 costmaps with the execution mask, without sending navigation goals.
- **Verified:** Three fresh stationary controls passed sensor refresh, both filters, footprint-feasible planning and clean shutdown in 15–16 s each. The existing 98 unit tests remain unchanged.
- **Open issue:** An earlier shutdown hang did not recur in the three controls. Its root cause remains unknown; no general shutdown fix is claimed. No test processes remain.
- **Next:** The [06B2B short crossing](06B2B_GCU_FIRST_SEGMENTED_CROSSING.md) subsequently passed. Broader nominal missions, faults and user acceptance remain Phase 06 work.

Recorded November 8, 2026. Predecessor: [06C1](06C1_GCU_CONSTRAINT_MASKS.md). Parent: [Phase 06 plan](../plans/06_GCU_OSMAG_SEGMENTED_NAVIGATION.md). This records stationary integration and a bounded shutdown follow-up, not Phase 06 completion. The initial failed attempt remains frozen; the later controls are separate evidence.

## Changes and scope

The previous native fixture used a synthetic map, no obstacle layer and an identity `map → odom` transform. This checkpoint uses the accepted Phase 05 GCU simulation assets and bridge binaries in a new workspace, `/home/ubuntu/edgeosmag_phase6b2a_ws`. The accepted workspace and `/home/ubuntu/lidarbot_ws` are unchanged.

The test runs headless physics and software-rendered sensors, AMCL/EKF, `planner_server` and `controller_server`. It starts neither `bt_navigator` nor the old metric-click bridge, and sends no `NavigateToPose` or `FollowPath` goals. A `ComputePathToPose` request only generates a plan. The controller publishes one zero-velocity message when deactivated; no nonzero velocity command was observed. Stationarity here is checked using wheel/EKF odometry, not independent truth-based motion acceptance.

New components:

- `scripts/prepare_gcu_sensor_constraint_test.py`: freezes a short-route fixture and prepares a fresh VM workspace from the accepted baseline, with source/asset hashes.
- `scripts/gcu_costmap_observation.py`: checks native grid cells using the stamped planar transform and its inverse for free-space probes. Accepts the original resolution or its exact ROS float32 serialization; arbitrary rescaling is rejected.
- `scripts/check_gcu_sensor_constraints.py`: checks the fresh localized footprint, mask consumption, post-refresh scans, stationary odometry and native footprint-feasible plans. Deactivates both lifecycle servers before teardown.
- `scripts/run_gcu_sensor_constraints.py`: isolates ROS domain 194, refuses an occupied domain, bounds startup/checker waits, and audits all launched processes plus ordered bridge/server shutdown.
- `scripts/verify_gcu_sensor_constraint_evidence.py`: replays archived native arrays, transforms, plans and process results. Its successful verification explicitly preserves `checkpoint_accepted: false`.

The navigation profile retains the 0.18 m radius, 0.02 m padding, rotation-shim/RPP controller, 0.10 m/s cap and 6 × 6 m local window. Test-specific additions are the keepout filter, 5 Hz costmap publication and a 0.5 s expected obstacle-observation update interval. No installed Nav2 plugin was patched.

For each update the robot remains stationary, the test publishes the mask, refreshes both native costmaps, then requires costmap stamps after refresh and at least three new LiDAR messages with latest age ≤ 0.3 simulation seconds. It independently inspects both costmaps, and obtains a native plan whose entire 0.20 m swept circle must remain in the original physical free space and current permitted mask. Planner success also exercises PlannerServer's native costmap-current wait. This does not expose or certify the controller's internal obstacle-buffer state for future motion.

## Attempts and measured results

| Attempt | Result | Interpretation |
| --- | --- | --- |
| 01 | Failed at startup TF lookup; no goals or motion commands. Two unconfigured server wrappers needed SIGTERM at cleanup. | A newly created TF buffer lacked overlapping timestamps. Fixed by waiting for transform availability and launching native server executables directly. |
| 02 | Sensor/planning checks passed; all four parent processes exited zero; total 16.06 s. | Local window contained **zero** forbidden mask cells. Preserved as preliminary evidence, insufficient proof of the local filter. |
| 03 | Sensor/planning checks passed; checker 5.61 s, total run 46.00 s. **Run failed teardown.** | Added a declared off-route test exclusion inside the rolling window. Both grids applied it, but Gazebo did not complete stop and its child was killed with exit −9 during launch cleanup. |

The test exclusion is a 0.5 × 0.5 m logical patch selected in verified free space away from the reference route. It is **fixture-only**, not a new CAD wall, inferred closure or production mission rule. The original occupancy map remains unchanged. Its purpose is to make local-filter consumption observable rather than accepting a vacuous check.

Attempt 03 measurements:

| Measure | Initial mask | Updated mask |
| --- | --- | --- |
| Expected blocked global cells | 804,770 | 804,851 |
| Expected blocked local cells | 0 | **81** |
| Missing expected blocked cells | 0 | 0 |
| New scans after activation/refresh | 4 | 4 |
| Latest scan age at readiness | 0.133 s | 0.216 s |
| Native plan status | Succeeded | Succeeded |
| Full-circle plan validation | Passed, radius 0.20 m | Passed, radius 0.20 m |

The observed local transform included yaw **0.00831566 rad**; it was not replaced with an identity transform. Its translation was approximately `(-0.06266, -12.94109)` m. Both native grids reported the serialized resolution `0.05864395201206207` m/cell.

Both lifecycle servers reached `inactive`. The evaluation bridge stopped correctly. Gazebo acknowledged `/server_control`, but its server remained alive beyond the 20 s exit deadline; a parent exit of zero did not override the logged child failure. The root cause of this intermittent teardown hang has **not** been established. Domain/partition process auditing found zero remaining test processes after cleanup. No further physics trials or motion tests were started in this batch.

## Evidence and replay

Evidence: [`simulation/gcu_ground_floor/phase6_diagnostics/06b2a_sensor_integration/`](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b2a_sensor_integration/).

The directory retains all three attempt archives, readable reports/run results, the source snapshot and hashes, raw grids, stamped transforms, sensor samples, native plan points, actual parameters, package versions, process audit and unit-test output. Failed attempts are retained with their original status. The replay result separates valid evidence from checkpoint acceptance.

From the repository root on macOS:

```bash
PYTHONDONTWRITEBYTECODE=1 \
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/verify_gcu_sensor_constraint_evidence.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06b2a_sensor_integration
```

Expect `verification_passed: true`, `sensor_and_plan_checks_passed: true`, **`checkpoint_accepted: false`**, and `motion_tested: false`. Verification does not rerun physics.

To reproduce the stationary fixture from an Ubuntu checkout with the Phase 05 prerequisites and accepted workspace already available, use new output names:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/lidarbot_ws/install/setup.bash

python3 scripts/prepare_gcu_sensor_constraint_test.py fixture \
  --assets simulation/gcu_ground_floor \
  --output /tmp/gcu_06b2a_fixture_new

python3 scripts/prepare_gcu_sensor_constraint_test.py prepare \
  --baseline /home/ubuntu/edgeosmag_phase5_ws \
  --fixture /tmp/gcu_06b2a_fixture_new \
  --destination /home/ubuntu/edgeosmag_phase6b2a_new_ws

python3 /home/ubuntu/edgeosmag_phase6b2a_new_ws/run_gcu_sensor_constraints.py \
  --workspace /home/ubuntu/edgeosmag_phase6b2a_new_ws \
  --output /home/ubuntu/edgeosmag_phase6b2a_new_ws/results/stationary_01
```

The Ubuntu desktop must stay logged in for software sensor rendering. The intermittent Gazebo teardown issue remains open despite the three clean follow-up controls below. It is not the final Foxglove mission workflow. Existing accepted Phase 05 commands remain unchanged.

## Shutdown follow-up: three fresh controls

A separate diagnostic workspace, `/home/ubuntu/edgeosmag_phase6b2a_shutdown_ws`, copied the exact sensor checker, runner, filters, controller settings, fixture and shutdown helper from the preceding attempt. The only addition was `scripts/run_gcu_sensor_shutdown_probe.py`: if the existing 20 s server-exit deadline fails, it gives a separately launched VM-administrator watcher a bounded opportunity to collect all native thread stacks with GDB before ordinary cleanup. The watcher verifies the test partition, workspace path and loaded Gazebo server library before attaching. It does not change the VM's ptrace setting, installed packages or robot workspace. A diagnostic hold does not turn a timeout into a pass.

| Control | Sensor/filter/plan checks | Actual child/server exits | Total wall time |
| --- | --- | --- | --- |
| 04 | Passed; 81 local exclusion cells | All four parent exits zero; no critical children; server exited | 15.43 s |
| 05 | Passed; 81 local exclusion cells | All four parent exits zero; no critical children; server exited | 15.41 s |
| 06 | Passed; 81 local exclusion cells | All four parent exits zero; no critical children; server exited | 15.84 s |

The bounded three-control diagnostic stopped after these results. The hang did not recur, so no blocked-process backtrace was captured. **The root cause is unresolved, and these results are not evidence of a general fix.** No navigation behavior or teardown policy was changed to obtain the passes. The earlier failed teardown remains part of the record. These controls demonstrate that the stationary integration can satisfy its sensor, planning and clean-exit gates; proceed only to a single isolated crossing with the same strict auditing, rather than an unattended mission matrix.

Follow-up evidence: [`06b2a_shutdown_followup/`](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b2a_shutdown_followup/). All three runs, raw grids, transforms, scans, plans, source snapshot and zero-leftover process audit are retained. Replay also confirms that the runtime checker/helper still match the repository sources:

```bash
PYTHONDONTWRITEBYTECODE=1 \
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/verify_gcu_shutdown_followup.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06b2a_shutdown_followup
```

Expect `stationary_controls_passed: 3`, `hang_reproduced: false`, `root_cause_established: false` and `general_shutdown_fix_claimed: false`. The original evidence replay above still returns `checkpoint_accepted: false` for the original failed batch; it has not been rewritten to attribute later results to that batch.

Upstream [Gazebo server lifecycle code](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/ServerPrivate.cc) and the [ROS control plugin teardown](https://github.com/ros-controls/gz_ros2_control/blob/humble/gz_ros2_control/src/gz_ros2_control_plugin.cpp) were reviewed as possible investigation paths. With no captured failing stack, neither component nor UTM/OpenGL is established as the cause. Installed binaries were not replaced based on that hypothesis.

## Next gate and paper relationship

[06B2B](06B2B_GCU_FIRST_SEGMENTED_CROSSING.md) subsequently added and validated the short-fixture simulation mission owner around the tested contract/state machine, disabled competing goal inputs, froze constraints before dispatch, and performed one short passage crossing with independent truth/crossing/rest evidence. Its short-fixture preemption, handoff, crossing and arrival/rest are now verified; broader route coverage remains pending. Logical closures during motion, multi-passage missions, cancellation and the final acceptance matrix remain later Phase 06 checkpoints.

This work supports the paper-based **osmAG passage route → Nav2 segment execution** integration. It does not yet demonstrate that complete workflow. Rolling raster/proxy goals remain Phase 07, structural localization Phase 08, and Pi/LLM/perception evaluation later. UTM rendering load and this stationary timing are not edge-hardware navigation performance results.
