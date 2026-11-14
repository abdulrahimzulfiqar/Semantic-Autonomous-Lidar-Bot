# Phase 07 native plan-only window integration

Final trial **06** and independent replay pass: five revisions, three window
origins, padded proxy plans, closed-passage rejection and reopening. Global
planning grids stay 128 × 128; actual parameters and graph inspection show no
full-floor `/map` subscription in this isolated fixture. No controller, physics
or motion starts. A second planner owns the rolling local-style costmap; this
is not evidence from an actual moving controller.

`WindowAdmission` binds the window, physical grid, constraint and proxy versions;
fresh native grids cannot conceal an expired source heartbeat. Live atomic
update sequencing, cancellation/rest on source loss and motion remain pending.

Earlier attempts are preserved: 01 import failure; 02 plan checks passed but
invalid teardown logged; 03 standalone costmap crashed during teardown; 04
planner-owned clean follow-up; 05 added admission binding; 06 added source
freshness. Do not treat earlier runner `passed` flags as final acceptance.

`attemptNN.tar.gz` freezes actual executed source, parameters and results.
`verification.json` replays the selected raw grids/paths without ROS.
`cleanup06.json` is the empty final isolated-domain audit. `tests160.log` is the
full local regression result. `manifest.json` binds source and artifacts.

See [the progress record](../../../../docs/research/osm-AG/phases/07_GCU_ROLLING_RASTER.md)
for reproduction and the single remaining integration/acceptance group.
