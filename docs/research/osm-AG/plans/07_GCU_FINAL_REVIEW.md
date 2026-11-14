# Phase 07: final Ubuntu and Foxglove review

## Quick summary

- **Purpose:** Confirm the moving planning window and boundary proxy behavior visually after automated acceptance.
- **Environment:** Ubuntu 22.04 UTM runs headless Gazebo and ROS; Foxglove on the Mac displays them. Keep Ubuntu logged in for the sensor renderer.
- **Expected:** The complete mission stays visible while the bounded planning map moves and Nav2 updates its current target/path.
- **Status:** Phase 07 accepted November 14, 2026 after user confirmation and one retained successful clicked-goal session. The commands below remain available for reproduction.

## Accepted review result

The user confirmed successful testing on November 14, 2026. The saved session
records one successful mission, 16 windows, 10 proxies and final rest, with all
six launched processes exiting 0. Manual return/cancel steps below are optional
reproduction checks; the saved user session does not demonstrate those steps.
The [completion record](../phases/07_GCU_ROLLING_RASTER.md) contains the automated
fault/recovery evidence and remaining architectural limitations.

## 1. Start one Ubuntu terminal

Use the **Ubuntu terminal inside UTM**. The prepared workspace is specific to
Phase 07. Leave this command running; it owns the simulator, localization,
Nav2 and Foxglove bridge. No other navigation terminals are needed.

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/lidarbot_ws/install/setup.bash

review_workspace=/home/ubuntu/edgeosmag_phase7_review_final_ws
review_output="$review_workspace/results/manual_$(date +%Y%m%d_%H%M%S)"
echo "Results: $review_output"

python3 "$review_workspace/run_gcu_segmented_crossing.py" \
  --workspace "$review_workspace" \
  --output "$review_output" \
  --fixture "$review_workspace/assets/mission_fixture.json" \
  --interactive --foxglove --session-wall-timeout 900 --execute
```

Wait for localization and `/osmag/session_status` to say `ready` before sending a
goal. The session lasts at most **15 wall minutes total**, including startup and
all goals; rerun the command for a new session if it expires. A failed/timed-out
mission is not an accepted arrival. Ctrl+C requests shutdown and stops the owned
processes. Do not run another launcher while it is shutting down.

If you see `test_domain_already_in_use`, another domain-195 session is still
running. Return to its terminal and stop it with Ctrl+C; wait for shutdown, then
retry. Do not change the domain or launch competing goal publishers to bypass
this check. To find which sessions remain, open another Ubuntu terminal:

```bash
ps -u ubuntu -o pid,etime,args | grep -E 'run_gcu_segmented_crossing|check_gcu_segmented_crossing|ign gazebo|planner_server|controller_server'
```

## 2. Connect Foxglove on the Mac

Open a Foxglove WebSocket connection to `ws://192.168.64.2:8765` (use Ubuntu's
current IP from `hostname -I` if it changed). In the 3D panel, choose `map` as
both fixed and display frame. Set the goal tool to `/goal_pose` in frame `map`.
The review executor receives this topic; native Nav2's direct goal input is
remapped so it cannot bypass the mission/constraint checks.

Enable these topics. Colors are presentation settings, not routing semantics.

| Topic | What to look for |
| --- | --- |
| `/map` | The full-floor localization reference; it stays fixed. |
| `/osmag/planning_window` | Approximately 7.51 × 7.51 m source raster that shifts with the robot. Its occupied border is intentional. |
| `/global_costmap/costmap` | Bounded 128 × 128-cell planning costmap built from the source window, constraints and sensor observations. |
| `/local_costmap/costmap` | The controller's separate approximately 6 × 6 m obstacle map. |
| `/osmag/mission_path` | Complete osmAG reference route to the final goal. |
| `/osmag/segment_goal` | Actual current Nav2 target; it can be a temporary proxy before the original passage/final target. |
| `/plan` | Native metric plan; also includes plan-only preflight responses. It can briefly show a future candidate. |
| `/received_global_plan` | Controller's transformed/pruned path actually being followed; use this to interpret active motion. |
| `/osmag/passage_progress` | Raw messages report sends with `proxy` and `window_revision`, then actual passage crossing events. |
| `/osmag/session_status`, `/osmag/mission_status` | Ready/running/succeeded/cancelled/failed and mission state. |
| `/scan`, `/robot_description`, `/camera/image_raw` | Sensor alignment, robot and camera; camera is optional for this review. |

For window inspection, hide `/map` temporarily so it cannot visually cover the
moving raster, then restore it for alignment. Keep unknown space visually distinct.

## 3. Forward crossing and return

The spawn is approximately **(-0.5, -12.9603)** facing +x. Pick the clear corridor
point **(3.9, -12.9603)** with heading 0. You may click it in Foxglove; for the exact
automated query, paste this into a **second Ubuntu terminal**:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=195
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/edgeosmag_phase7_review_final_ws/assets/cyclonedds_local.xml

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 3.9, y: -12.960313240266368}, orientation: {w: 1.0}}}'
```

Watch for a complete mission route, an initial target within the window, then
window shifts and changed proxy targets as the robot advances. The robot should
stay aligned with scan/map walls and pass around forbidden space. A proxy being
reached must not cause a final `succeeded` status. Expect arrival and confirmed
rest near the **original** destination, within 0.20 m and 10° in the automated
scoring. The 4.4 m run normally takes around 2–4 wall minutes on this VM; loaded
VMs can take longer. Speed remains capped at 0.10 m/s of simulated time.

After `succeeded` and rest, send the return in the same second terminal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: -0.5, y: -12.960313240266368}, orientation: {z: 1.0, w: 0.0}}}'
```

This direction exercises a proxy on the passage segment itself. The original
passage must still be confirmed in order. A very short nearby goal is valid but
may not exercise any window shift or proxy and cannot establish this milestone.

## 4. Cancel, rest and a new goal

While a goal is executing, send a cancellation from the second terminal:

```bash
ros2 topic pub --once /osmag/mission_cancel std_msgs/msg/Empty '{}'
```

Expect `cancelled` and physical rest, with no resumed motion by itself. Then pick
a new clear goal; the review harness requires confirmed rest before it recovers
and admits the new mission. Goal clicks during an active mission are rejected.
Do not publish wheel commands directly. Fault injection and obstacle clearance
already have separate automated evidence; manual source killing is unnecessary.

## 5. Finish and report

Stop the **first terminal** with Ctrl+C after the robot has stopped. Its results
folder contains per-mission `user_mission_NN.json`, window NPZs, raw logs,
`report.json` and `run.json`. The session's final boolean does not substitute for
checking every mission, especially a deliberately cancelled one.

Report whether forward/return succeeded, the planning window moved, proxy updates
preserved the final goal, and cancel/restart worked. Include the results path and
any failed status when repeating the review. Phase 07 is already accepted; the
[Phase 07 record](../phases/07_GCU_ROLLING_RASTER.md) separates automated results
from your visual acceptance.

## Preparing another fresh workspace from the repository

The named review workspace above is already prepared on this Ubuntu VM. For a
later fresh clone, run from the repository root **inside Ubuntu**, with the
accepted Phase 06 baseline still available. Choose a destination that does not
exist; preparation does not launch physics.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/prepare_gcu_rolling_motion.py \
  --baseline /home/ubuntu/edgeosmag_phase6_review_final_ws \
  --source-assets simulation/gcu_ground_floor \
  --fixture simulation/gcu_ground_floor/phase7_fixtures/manual_review.json \
  --destination /home/ubuntu/edgeosmag_phase7_my_fresh_review_ws
```

Then use that new path as `review_workspace` in step 1 and in the optional DDS
configuration path in step 3. The preparer validates pinned runtime sources and
preserves the accepted fused baseline; it does not install ROS/Gazebo or recreate
the Ubuntu environment. Frozen executed archives remain available for independent
evidence replay without this VM baseline.
