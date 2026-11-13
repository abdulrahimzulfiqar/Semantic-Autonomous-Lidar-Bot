# 06B2B: first real Nav2 segmented crossing — short fixture passed

## Quick summary

- **Verified:** The real Nav2 mission adapter completed one constrained crossing, early goal handoff, final arrival and independently measured rest.
- **Results:** 8.4 cm final error, zero observed obstacle contacts, 104 unit tests and independent evidence replay passed; all processes stopped.
- **Limits:** One wide virtual corridor interface, not a narrow doorway or multi-room mission. Earlier failures remain archived; shutdown mitigation is not a general fix.
- **Next:** Freeze and test the nominal multi-passage/consecutive-mission group, then faults and user acceptance. Phase 06 is not complete.

Recorded November 8, 2026. [Parent plan](../plans/06_GCU_OSMAG_SEGMENTED_NAVIGATION.md), [stationary predecessor](06B2A_GCU_SENSOR_CONSTRAINT_INTEGRATION.md). Phase 06 remains incomplete. Attempts 01–07 are retained as failures; attempt 08 passes the bounded short-crossing gates. A passage crossing alone is never counted as successful final arrival.

## Implementation and fixture

`scripts/gcu_segmented_executor.py` connects the existing mission contract/state model to real `NavigateToPose` actions. It uses localized TF and wheel/EKF odometry; it never subscribes to ground truth. Dispatch requires consumed constraints, fresh TF/scans, one navigator and no competing old click bridge. Action handles, superseded results, crossing confirmations and cancellation are recorded separately. Callback state is serialized; evaluator callbacks use separate processing. The path/segment/status topics are published in `map`.

`scripts/check_gcu_segmented_crossing.py` owns lifecycle setup and the independent truth/contact evaluation. It retains the constraint publisher throughout execution. `scripts/prepare_gcu_segmented_crossing.py` and `scripts/run_gcu_segmented_crossing.py` create/run an isolated candidate; neither changes the installed robot or accepted workspaces. Execution requires `--execute`, domain 195, localhost-only DDS and the test partition prefix. The existing mock adapter remains mock-only.

The declared initial spawn is `(-0.5, -12.96031324, 0)` m and the final goal is `(1.44135605, -12.96031324, 0)` m. Passage `-93` is a wide logical seam between corridor sectors, not a narrow physical doorway. The previous spawn lay approximately 3 cm from the passage, inside the 10 cm metric arrival tolerance; moving the **initial spawn** west makes a real approach/handoff observable. There is no teleportation during a mission. The AMCL prior is changed consistently for this candidate only.

The controller, speed cap, footprint and strict behavior tree are retained. Mission constraints admit only corridors 14/16 and passage `-93`. A fresh route supplies the refined crossing coordinate; no fixed midpoint is substituted. Global/local grid observation age limits are 0.75/0.5 simulation seconds, scan age 0.3 simulation seconds, and robot TF age 0.15 simulation seconds. Future-dated grids wait for `/clock`; they do not replace an existing valid observation. TF is sampled from existing history at a timestamp no later than the node clock. The separate frozen-clock watchdog remains 2 wall seconds, with bounded action/service and whole-mission wall deadlines.

## Preserved attempts

| Attempt | Outcome |
| --- | --- |
| 01 | No motion: localhost DDS participant-index allocation failed during localization startup. |
| 02 | No motion: the first explicit DDS XML conflicted with the middleware's interface configuration. |
| 03 | No motion: missing contact manifest. The broad baseline-copy exclusion had removed it; the preparer now derives and validates all 13 contact entries from the unchanged instrumented SDF. Required files are checked before physics startup. |
| 04 | Motion and early handoff occurred; stale-scan watchdog cancelled the mission. Clean native exits. |
| 05 | Separate callback processing exposed a clock-ordering error: local grid stamp 8.729 s arrived while the node clock was 8.712 s. It caused cancellation. Sensor shutdown hung; native stacks were captured. |
| 06 | With future-grid deferral, passage `-93` was confirmed. A future TF/clock ordering mismatch later triggered cancellation. Sensor shutdown hung; native stacks were captured. This trial failed. |
| 07 | Clock-consistent TF sampling passed the crossing. A scan only 0.075 simulation seconds old was rejected by a separate 2-second wall scan timer during VM slowdown. Failed mission; pause-before-stop produced clean exits. |
| 08 | Correct time-domain checks completed the crossing and final goal/rest gates. Independent replay passed; all five parent processes and their children exited cleanly. |

The compatible DDS profile keeps localhost-only discovery and explicitly allows more participant indices. A cheap ROS-node preflight validated it before further physics. See [CycloneDDS discovery configuration](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html) for the role of participant indices and unicast peers. These are workspace-local configuration changes, not installed middleware patches.

In attempt 06, sampled truth began near `(-0.5000, -12.9603)` and ended near `(0.8519, -13.0015)` m. Localization confirmed the selected crossing at simulation time 15.336 s. There were no observed wall/unclassified contacts, and all 13 contact publishers were present. **That attempt did not achieve final arrival/rest acceptance.** Its raw samples and original failure status remain retained; the later accepted result below is separate evidence.

The debugger captured the Gazebo main thread joining a stop thread, which was itself waiting in `ignition::gazebo::v6::systems::SensorsPrivate::Stop()`. This locates the observed wait in sensor teardown. It does not establish the initiating race or prove a Mac/UTM/OpenGL cause. The server failed the existing exit deadline and was killed during cleanup; zero parent exit codes did not override the logged child failure.

## Follow-up corrections and accepted short trial

The saved failure was not evidence of old localization alone: the last accepted TF stamp was 23.32 s while the clock in the fault event was still 23.31 s. TF and `/clock` are independent topic streams. The new `coherent_pose_time` check rejects genuinely old TF as before, and queries the existing buffer at `min(latest TF stamp, current ROS clock)` when the latest TF is future-dated. Failed history lookups still fault; no extrapolated pose or truth pose is substituted. Timing records retain the clock, latest stamp and actual query stamp.

Attempt 07 then isolated the scan problem. At clock 20.075 s, scan stamp 20.0 s remained within the 0.3 s simulation-age limit. Its receipt was just over 2 wall seconds earlier because UTM was advancing slowly. The per-scan wall-age predicate incorrectly treated that slowdown as sensor failure. Sensor age now uses ROS/simulation time; the **separate 2-second frozen-clock wall watchdog remains**, as do the action/service wall deadlines and 420-second checker deadline. A real-time ROS clock still expires a missing scan after the same 0.3 seconds. A future scan does not replace a usable past scan. Regression tests cover both future data and genuinely expired data; this is an explicit time-domain policy correction, not a claim that the old per-scan wall predicate is unchanged.

After mission cancellation or lifecycle deactivation, `gcu_quiescent_shutdown.py` requests a paused world, allows a 0.25 s settling interval, then uses the existing server-control exit deadline and child auditing. The pause acknowledgement does not separately prove renderer quiescence. Attempts 07 and 08 exited cleanly with this candidate mitigation. The captured [upstream sensor-stop implementation](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/sensors/Sensors.cc) is consistent with waiting for a render worker, but the initiating race and general effectiveness of this mitigation are not established. Installed Gazebo and robot workspaces were not patched.

| Attempt 08 gate | Result |
| --- | --- |
| Ordered passage | `-93`, confirmed by localization and independently by sampled truth |
| Action handoff | Next goal sent at 10.847 s; previous goal's superseded/aborted result arrived at 10.858 s |
| Handoff rest | 0.0 s observed in the declared interval from 0.5 s before to 2 s after the handoff |
| Final action | Succeeded; no unresolved requests |
| Final position / heading error | 0.0840 m / 0.0074° |
| Independent rest | At least 1 simulation second after terminal action success |
| Localization error p95 | 0.02995 m / 0.8974° |
| Frozen-sample replay coverage | 476/476 timestamp matches; maximum truth gap 0.05 s |
| Physical/topological clearance | Original 0.18 m swept-circle check and ordered truth crossing passed |
| Contacts / command cap | Zero observed obstacle/unclassified contacts; maximum commanded speed 0.10 m/s |
| Ownership | One controller velocity owner and navigator; old click bridge/direct goal publishers absent |
| Teardown | All five parent exits zero, no critical child failures, actual server exit, no remaining test processes |
| Duration | 60.68 s checker; 88.48 s including startup and teardown |

The frozen samples include the short lifecycle-deactivation interval after initial scoring. Therefore the original report's interim coverage is 467/468 (99.79%), while independent replay of the complete frozen arrays matches 476/476. Both satisfy the same 90% gate; the raw report is not rewritten. Actual measured travel was approximately 1.864 m; final arrival permits the retained 0.20 m position tolerance.

## Evidence and reproduction

Original evidence remains in [`06b2b_crossing/`](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b2b_crossing/). Follow-up evidence is in [`06b2b_clock_followup/`](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b2b_clock_followup/): failed attempt 07, accepted attempt 08, raw inputs/results, exact runtime parameters, source snapshot/hashes, 104-test output and cleanup audit. Replay matches the executed source to the runtime manifest, checks the stored geometry digest, independently replans the query, and scores truth/topology/rest/handoff. Native costmap grid consumption was observed online and retained as receipts; this crossing archive is not an independent replay of every live native grid.

From the repository root on macOS:

```bash
PYTHONDONTWRITEBYTECODE=1 \
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/verify_gcu_segmented_crossing.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06b2b_clock_followup \
  --trial 08
```

Expect `verification_passed: true`, `trial_passed: true`, passage `-93`, `handoff_before_terminal: true` and `shutdown_passed: true`. Replaying the original directory without `--trial 08` still reports its failed trial and explicitly matches changed sources to the historical source snapshot. Original manifests and results remain unchanged.

For the same automated short test in the existing Ubuntu workspace, log in to the desktop, leave other Phase 06 tests stopped, then use a new output name:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/lidarbot_ws/install/setup.bash

python3 /home/ubuntu/edgeosmag_phase6b2b_clock_ws/run_gcu_segmented_crossing.py \
  --workspace /home/ubuntu/edgeosmag_phase6b2b_clock_ws \
  --output /home/ubuntu/edgeosmag_phase6b2b_clock_ws/results/user_short_01 \
  --execute
```

The runner starts and stops its own headless physics, sensors, localization and Nav2. It refuses an occupied test domain and does not overwrite results. Expect `report.json` and `run.json` to report success; a fresh failure must be retained rather than ignored. This is an automated single-fixture test, **not** a general Foxglove click-to-mission flow. It does not launch Foxglove or authorize unrelated goals through the test adapter. Existing accepted Phase 05 commands remain unchanged.

## Next gate and project relationship

06B2B is passed for this one short fixture; it does not complete Phase 06. Next freeze exact feasible starts/goals, including enclosed-sector routes through actual doorways, and run the nominal multi-passage and consecutive-mission group in separately bounded batches. Then test cancellation, logical closures, blocked routes and stale/missing-data faults, followed by the final user-facing mission workflow and visual acceptance. Keep the sensor shutdown diagnostic and strict process auditing available.

This is the first measured **paper-based osmAG route → passage-derived Nav2 goal handoff → physical arrival** connection in the Phase 06 stack. Refined crossings and topology constraints remain declared adaptations; no exact author-runtime or globally optimal continuous-trajectory claim is made. The full raster and AMCL remain in use. Rolling maps/proxy goals and structural localization remain later phases; UTM timings are not Pi 4B performance results.
