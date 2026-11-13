# Phase 06 simulation mission fixtures

Each JSON is a versioned query, not a claim of runtime acceptance. Runtime checks
verify its map and execution-source hashes, replan from localized TF, and check
the new route against the padded mission mask before dispatch.

- `doorway.json`: original single physical doorway query. Its accepted old
  workspace/evidence remains separate from the newer multi-passage experiments.
- `two_corridor_seams_map_wire.json`: latest tested two-passage profile, 7.5 m through
  `-91`, `-94`, with the local rolling costmap explicitly in `map` and exact
  float32 mask-resolution interpretation. See the measured phase record for status.
- `two_corridor_seams.json` and `two_corridor_seams_map_frame.json`: historical
  failed diagnostic profiles. Preserve their pinned versions; do not overwrite
  their hashes to make them run with changed sources.
- `offline_candidates.json`: earlier proposals. The corridor-to-room and
  room-to-room queries fail their padded mask checks and must not run.

Reproduce offline admission from the repository root, using a new output path:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/admit_gcu_mission_fixture.py \
  --assets simulation/gcu_ground_floor --name two_corridor_seams_map_wire \
  --spawn -0.5 19.9 0 --goal 7 19.9 0 --minimum-passages 2 \
  --local-costmap-frame map --output /tmp/gcu_two_passages_reproduced.json
```

This command imports no ROS and starts no simulator. Admission does not replace
live path preflights, ordered crossing/contact scoring or shutdown verification.
The frame choice is explicit per fixture; the preparer's default remains `odom`.
This local 6 × 6 m costmap is separate from the paper's rolling global raster,
which remains future Phase 07 work.

`doorway_consecutive.json` and `doorway_consecutive_publish10.json` retain failed out-and-back session profiles. Neither is accepted. Their pinned checker predates the pending fail-fast change; re-admit to a new file before testing changed source. See phase record 06D2.

The capture, sigma075 and imu consecutive fixtures are diagnostic controls, all unaccepted. The sigma setting was rejected. The imu fixture executed before the pending world-plugin repair; freshly re-admit before testing changed source. Continue in phase record 06D2, without adding checkpoint numbers.

Latest verified consecutive profile: `doorway_consecutive_imu_aligned.json` (motion 08). `doorway_imu_stationary.json` and `doorway_consecutive_imu_plugin.json` document verified IMU delivery and the pre-alignment failed control. Keep their actual source pins; re-admit to new files for changed code. All results and remaining gates stay in 06D2.

`corridor_to_room_masked.json` is a newly admitted 5.803 m query through `-91`,
then `-95`, using padded-mask route reconstruction and the verified IMU/aligned
grid profile. It does not overwrite either rejected longer proposal. Admission
uses `--constrain-route --imu-yaw-rate --align-local-grid --local-costmap-frame map`;
see [the 06D record](../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for exact endpoints, bounded live results and reproduction. An admitted fixture
alone is not runtime acceptance; freshly admit to a new file after source changes.

The same frozen execution profile is used by `adjacent_sector_masked.json`
(1.941 m, passage `-93`) and `room_to_room_masked.json` (8.858 m, passages
`-95`, `-94`, `-106`). The latter connects verified room sectors 33 and 34 with
turns; it is a conservative mask-valid reference, not a shortest-path claim.
The [06D record](../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
tracks actual live/replay acceptance separately from offline admission.

`cancel_near_handoff.json` uses the same straight passage `-93`, starting at
x = -1.5 m to allow meaningful motion before a cancel request. Its normal route
is 2.941 m; `--cancel-near-handoff` deliberately cancels before the first handoff
instead of claiming arrival. Use only the fresh admitted source-pinned workspace;
see the existing [fault/comparison group](../phase6_diagnostics/06_faults_and_comparison/).

`close_room_exit_stopped_mask.json` starts in room 33 at `(3.9, 23.3, -π/2)`
and targets corridor 15 at `(3.9, 19.9, -π/2)`, initially through `-95`.
`--close-upcoming-passage` closes that only exit while moving, then requires
stop, revised native mask consumption and explicit graph/native no route.
This is a logical closure, not a physical obstacle. The new source-pinned fixture
was freshly admitted before its live run; earlier accepted fixtures are unchanged.

## Final automated acceptance fixtures

`required_imu_loss_v1.json` tests loss of the executor's forwarded IMU input while
the raw sensor continues. `physical_detour_execution_gate.json` inserts a physical
box without adding it to the navigation raster; executed-body checks use its
complete evaluation geometry. Earlier failed obstacle evidence remains retained.
`final_review_seed_v2.json` supplies the tested profile and entrance spawn for
interactive goals; it does not automatically dispatch its seed goal. Each clicked
goal gets a freshly validated route and mask. Matched comparison fixtures retain
the same controller, localization and permitted-area mask for one-final-goal and
segmented execution. See the consolidated 06D record for results and limits.
