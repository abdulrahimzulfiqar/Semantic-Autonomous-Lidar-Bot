# Phase 06: final Foxglove review

## Quick summary

- **Purpose:** Verify the final click-to-mission interface visually after automated acceptance.
- **Status:** Bounded Phase 06 accepted November 13, 2026 after short-distance user review; [completion scope](../phases/06_GCU_SEGMENTED_NAVIGATION.md). These commands remain for reproduction.
- **Expected:** Full osmAG reference, current Nav2 segment, ordered progress, arrival, cancellation and another goal without restarting.
- **Boundary:** This phase uses the static global raster and AMCL. Rolling global maps remain Phase 07.

Test processes are stopped. Restart only when you want a new review.
Each session stops after 15 minutes. Run commands in **Ubuntu inside UTM**,
with its desktop logged in. The
runner owns simulator, localization, Nav2 and Foxglove bridge startup/teardown.
Use a new output directory for every session. Do not combine it with earlier
manual launch terminals.

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/lidarbot_ws/install/setup.bash
unset ROS_DOMAIN_ID
python3 /home/ubuntu/edgeosmag_phase6_review_final_ws/run_gcu_segmented_crossing.py \
  --workspace /home/ubuntu/edgeosmag_phase6_review_final_ws \
  --output /home/ubuntu/edgeosmag_phase6_review_final_ws/results/review_$(date +%s) \
  --fixture /home/ubuntu/edgeosmag_phase6_review_final_ws/assets/mission_fixture.json \
  --interactive --foxglove --session-wall-timeout 900 --execute
```

On the Mac, connect Foxglove to `ws://192.168.64.2:8765` (the verified VM address;
run `hostname -I` inside Ubuntu if it changes). The robot **starts** at
`(-0.5, -12.9603)` in the entrance corridor and waits for a goal; there is no
automatic initial navigation.

In the 3D panel, use fixed/display frame `map`. Enable:

| Topic | What it shows |
| --- | --- |
| `/map`, `/scan`, `/robot_description` | Floor, sensed obstacles and robot |
| `/osmag/mission_path` | Complete high-level reference route; choose a distinct color |
| `/plan` | Current native metric plan; choose another color |
| `/osmag/segment_goal` | Currently executing intermediate/final target |

Add Raw Messages panels for `/osmag/session_status` (request/result) and
`/osmag/mission_status` (active mission progress). The optional Image panel uses
`/camera/image_raw`.

Configure the 3D panel's **Publish → 2D pose** tool to publish on `/goal_pose`.
Click a free destination and set its heading. Foxglove documents this tool as
publishing `PoseStamped`; the bridge enables `clientPublish`.
[Official 3D publishing instructions](https://docs.foxglove.dev/docs/visualization/panels/3d).

Review these behaviors:

1. Choose a nearby free corridor point, initially about 2 m to the robot's right.
   The complete reference appears, the current segment changes as passages are
   reached, and the robot eventually stops with state `succeeded`.
2. While it is moving, publish another goal. The new request is rejected as busy;
   the existing mission continues.
3. Cancel while moving: use a Publish panel on `/osmag/mission_cancel`, type
   `std_msgs/msg/Empty`, message `{}`. Expect `cancelled` and physical rest.
4. After stopping, choose another feasible goal, then a return with a different
   heading. Both should work without restarting the stack.

The reference and executed metric path can differ inside permitted free space.
If a new same-area goal is rejected near a virtual boundary, the footprint would
intersect its proposed mask. The robot remains stopped; choose a goal across the
open passage. The controller does not bypass that guard.

To finish gracefully, publish `{}` as `std_msgs/msg/Empty` on
`/osmag/session_stop`, or press Ctrl+C in the runner terminal. The session is
bounded to 15 minutes and saves its reports under the chosen output directory.

Please report whether the full route, segment changes, arrival/heading,
cancel-to-rest and restart after cancellation look correct. A short recording
or the saved session reports can resolve any discrepancy. The accepted scope is recorded separately; no new checkpoint numbers are introduced.

The 900-second limit is a whole-session wall-clock budget, including waiting
before goals. The simulation can run much slower than real time. Use nearby
goals for manual review; an unfinished goal cancelled at timeout is a failed
mission even if session handling/cleanup reports `passed`.
