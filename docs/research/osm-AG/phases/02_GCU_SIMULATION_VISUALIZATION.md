# Checkpoint 02: GCU simulation and Foxglove visualization

Date: 2026-11-01. Status: initial stationary visualization achieved;
localization and autonomous navigation remain pending.

## Starting point and outcome

[Checkpoint 01](01_GCU_CAD_TO_OSMAG.md) produced accepted reconstructed geometry.
This checkpoint adapts it to the ROS loader, creates an aligned simulation world
and occupancy map, and brings simulated sensors into Foxglove on macOS.

## Changes

- Canonical-name and legacy numeric-reference support in the C++ area loader,
  malformed-reference checks, and a CLI using the actual loader.
- Hole-preserving subdivision into 59 simple traversable sectors plus a floor
  container, with 68 two-node passages; these are not 59 semantic rooms.
- 830 static wall boxes from occupancy, aligned SDF/PGM/YAML and diagnostic assets.
- An Ogre 1 VM profile, Foxglove workflow and stationary alignment launch using
  a temporary known-spawn map-to-odom transform and a wheel-odometry EKF.

## Evidence

Ubuntu 22.04 ARM64 in UTM, ROS 2 Humble and Gazebo Fortress 6.16.0:
C++ build passed, both loader exports passed, SDF validated, and headless world
physics ran for 200 iterations. The stationary automated probe received 225 clock
messages, 3 scans, 47 odometry messages and 7 actual 640×480 RGB images, with zero
velocity commands sent. See [test results](../../../../simulation/gcu_ground_floor/utm_test_results/test_summary.json).

The user then confirmed live Foxglove map/scan overlay and actual camera display.
Screenshots show broad agreement between nearby wall edges and scan returns, with
map frame selector errors cleared. This is qualitative evidence, not a measured
alignment-error result or a 60-second drift measurement.

## Limits

Stock Gazebo shapes also flicker in this VM. Anti-aliasing disable did not fix it;
headless server plus Foxglove on macOS is the working visualization workflow.
39 infinite scan ranges and a line across a right-side opening were observed;
scan-range/renderer behavior and sensor accuracy require further checks.
Known-spawn alignment is not AMCL localization. Eleven inferred interfaces are
narrower than the reference robot-radius clearance at their midpoints. The planner's
straight-line fallback must be removed or collision-validated before autonomous
routes. No completed route or collision-free navigation claim is made.

## Reproduction and next gate

See [GCU_SIMULATION_READINESS.md](../GCU_SIMULATION_READINESS.md) for current
commands, assets, transform ownership and checks. Next: record stationary stability
and scan accuracy, replace the fixed map-to-odom transform with AMCL, verify
controlled motion, validate footprint-feasible routes, then test segmented osmAG
room-to-room execution. Record completion, collisions, localization/goal errors and
latency as the reproducible baseline for later EdgeOsmAG comparisons.
