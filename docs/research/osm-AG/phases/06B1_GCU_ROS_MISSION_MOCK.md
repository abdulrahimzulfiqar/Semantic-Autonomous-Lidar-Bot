# Phase 06B1 — ROS mission adapter with a mock navigator

## Quick summary

- **Goal:** Exercise the mission contract through actual ROS 2 messages, TF and `NavigateToPose` action transport before enabling robot motion.
- **Result:** Ten ROS mock cases passed in 23.4 seconds; all 80 local unit tests passed. Both child processes exited cleanly.
- **Scope:** Synthetic TF and a fabricated navigator on an isolated action name. No Gazebo, controller, real navigation server or velocity publisher was started.
- **Next:** 06C1 verifies topology/closure masks and installed-Humble costmap integration without motion. Full Phase 06 remains incomplete.

Date: 2026-11-06. Previous batch: [06A](06A_GCU_MISSION_CONTRACT.md). Full sequence and acceptance boundaries: [Phase 06 plan](../plans/06_GCU_OSMAG_SEGMENTED_NAVIGATION.md).

## What changed

[The ROS adapter](../../../../scripts/gcu_mission_mock_adapter.py) wraps the existing pure contract/state model. It accepts mission requests, gets the start from TF, computes a fresh route in a worker thread, publishes the mission path and active target, sends segment actions, confirms ordered transitions, handles cancellation, and publishes explicit outcomes. Keeping planning off the callback thread allows TF updates and cancellation to continue during route calculation. If the robot pose changes during planning, the request is rejected rather than executing stale start geometry.

This adapter is deliberately **mock-only**. Its action is `/gcu_phase6b1_NAME/navigate_to_pose`, not the real `/navigate_to_pose`. It requires a matching token, action name and mock-scope response from a `Trigger` identity service before sending a goal. A unit test checks that live or nested namespaces are rejected before ROS import. This is an accidental-use guard, not a ROS network security claim. Production goal topics and costmap readiness remain later integration work.

The adapter publishes test-prefixed `mission_status`, `events`, `mission_path` and `segment_goal`; goal/cancel inputs use the same isolated prefix. It does not publish TF or wheel velocities. A separate fixture publishes synthetic `map -> base_footprint` TF and hosts the mock action server. Its action results do not move any robot; the fixture changes TF independently.

ROS time is checked for pose freshness; a stable offset maps observation timestamps to the state model's monotonic clock. Pending-response deadlines use monotonic wall time. Rest confirmation in this checkpoint comes from synthetic TF samples; it is not evidence of physical braking or odometry quality.

## Cases and retained failure

| ROS interface check | Result |
| --- | --- |
| Unverified mock identity | Rejected before sending an action. |
| Unsupported frame | Rejected without a navigator request. |
| Courtyard goal | Rejected by the map contract. |
| Continuous mock handoff | Next segment dispatched before old completion; one ordered crossing and final synthetic rest confirmed. |
| Busy destination / whole-mission cancellation | Existing mission preserved; pending/accepted actions cancelled. |
| Cancel before delayed acceptance | Late accepted handle cancelled; no mission revival. |
| Navigator rejection | Explicit failed outcome after synthetic rest verification. |
| Acceptance timeout | Stop status stays unknown until the late handle is resolved and fresh synthetic rest is observed. |
| Stale TF | Active action cancelled; terminal confirmation requires fresh observations. |
| Absent navigator | Rejected without an action request. |

The action trace records the old goal's abort result arriving **before** the next goal's acceptance callback. The mission still advances correctly. The short crossing target keeps the refined route's y coordinate instead of returning to the interface midpoint.

The first session passed five cases and then timed out: navigator rejection put the pure model into `cancelling`, but the adapter had not started its cancellation/rest timestamp for that internal transition. The fix initializes that timestamp for both `cancelling` and `failed` states. The fresh session passed all ten cases. The original failed logs, report and exact source remain archived; they were not rewritten as successful results.

## Evidence and limits

[Accepted report](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b1_ros_mock/report.json), [runner/cleanup result](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b1_ros_mock/run.json) and [independent verification](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06b1_ros_mock/verification.json) retain the measured ROS events. A SHA-256 manifest covers 48 archived files across both attempts, including source, map inputs and logs. The verifier checks current accepted source against the snapshot, action-event ordering, terminal outcomes, zero command publishers and process cleanup.

The accepted session observed zero `/cmd_vel` and `/cmd_vel_nav` publishers, zero command messages and zero real `/navigate_to_pose/_action/status` publishers. Both child exits were zero, with no critical child-process errors. A final process check found no checkpoint adapter, fixture or runner left running. Unrelated Ubuntu services and the accepted robot workspaces were preserved.

These results establish ROS wiring and state behavior against a mock. They do **not** establish real Nav2 preemption behavior, physical crossing/clearance, speed limits, stopping distance, localization performance or costmap constraints. Synthetic TF can move faster than the robot's configured speed; no robot-speed result is claimed. Actual constrained navigation remains 06B2 and the later mission groups.

## Reproduce the bounded mock session

In Ubuntu, from the repository root with ROS sourced, use a new directory. No desktop renderer, simulator or robot workspace setup is required:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/prepare_gcu_mission_mock.py \
  --destination /home/ubuntu/edgeosmag_phase6b1_repeat_ws
python3 /home/ubuntu/edgeosmag_phase6b1_repeat_ws/run_gcu_mission_mock_test.py \
  --workspace /home/ubuntu/edgeosmag_phase6b1_repeat_ws \
  --output /home/ubuntu/edgeosmag_phase6b1_repeat_ws/results/check_01 \
  --domain 186 --wall-timeout 180
```

Use a free ROS domain. The runner refuses a domain already in use, starts only its fixture and adapter, audits child exits and tears down its own processes. The prepared accepted workspace is `/home/ubuntu/edgeosmag_phase6b1_mock_v2_ws`; the first-attempt workspace remains historical. The unit/dependency environment is the same Humble/Cyclone DDS plus Python geometry stack used in Phase 05.

On macOS, replay the archived verification without ROS or Ubuntu:

```bash
PYTHONDONTWRITEBYTECODE=1 \
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/verify_gcu_mission_mock_evidence.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06b1_ros_mock
```

No new manual Foxglove test is required yet. Keep using Phase 05 for the accepted preview. The next batch is **06C1 only**: constraint-mask fixtures and plan-only costmap checks before any new real-Nav2 motion test.
