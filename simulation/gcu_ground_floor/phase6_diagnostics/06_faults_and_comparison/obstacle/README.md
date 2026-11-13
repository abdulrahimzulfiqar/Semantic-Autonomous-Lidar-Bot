# Preserved failed initial obstacle trial

This trial remains failed under its original declared gate. It reached the goal
without contacts; executed body clearance passes the evaluation raster and exact
box centerline distance is 0.4054 m. A plan-only preflight query, never sent to the
controller, failed the full-box oracle (minimum exact box clearance 0.1611 m).
The navigation raster did not contain this unknown box. Initial native sensor
costmaps were not captured, so a specific perception-level cause is not proven.

The test gate conflated an unexecuted plan-only query with executed clearance.
The fresh obstacle_followup trial retains known-map preflight checks, physical
body clearance against the full obstacle and all contact limits; it records the
unknown-box preflight oracle as a diagnostic. Robot, box and controller were not
tuned. This archive and its failed result were not relabelled as passing.

Independent replay tools and actual sources are frozen in the verification and
source archives. See the main Phase 06 record for commands and limitations.
