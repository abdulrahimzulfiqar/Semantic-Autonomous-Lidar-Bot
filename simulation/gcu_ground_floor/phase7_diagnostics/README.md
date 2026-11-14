# Phase 07 evidence index

Accepted rolling-map integration evidence, with user confirmation on November 14, 2026.
All motion trials run in isolated UTM Ubuntu/Gazebo simulation. No real robot is
commanded. Actual executed workspaces are archived, including runtime manifests,
sources, parameters and raw reports. Some successful runs use earlier source
snapshots; do not rewrite those archives to match the latest root code.

| Group | Selected trial | Evidence |
| --- | --- | --- |
| Offline OSM geometry | `offline_osm_windows_04` | 56-window reference coverage, four route fixtures, 76 padded pieces |
| Native no-motion integration | `native_windows/attempt06.tar.gz` | Five revisions, three origins, passage closure/reopening, no velocity publishers |
| Forward actual motion | `rolling_motion/attempt03.tar.gz` | Seven windows, two proxies, crossing/arrival/rest |
| Reverse actual motion | `reverse_boundary/attempt02.tar.gz` | Proxy on passage segment zero, seven windows, crossing/arrival/rest |
| Physical obstacle | `obstacle/attempt02.tar.gz` | Eight windows, three proxies, complete-box body sweep, independent clock observer |
| Source loss and guarded recovery | `source_loss/attempt05.tar.gz` | Selected replay verifies raw loss and return reports in the same native session |
| User visual review | `manual_review/attempt01.tar.gz` | One successful clicked-goal mission; 16 windows, 10 proxies, final rest and six clean exits |
| Matched static-map control | `static_baseline/attempt01.tar.gz` | Same query/world/localization/controller with whole-floor planning raster |
| Window timing | `live_window_timing.json` | Wall-time generation/mask work and simulated-time startup/moving preflight commits |
| Comparison | `matched_comparison.json` | Component RSS/CPU samples, cells and route/arrival metrics; single trial per variant |

Failures remain failures: forward 01 lost readiness during a window change;
obstacle 01 tripped the existing clock-stall watchdog; source loss/recovery 02
failed return preflight because the previous mission index skipped a target;
source loss/recovery 03–04 encountered stale rest-time/preflight TF samples. These
fault scenarios stopped without observed contact; inspect raw stop fields
for the exact outcome of each failure rather than assuming a successful cleanup
means successful mission acceptance. Native attempts 01–03 have separate import/
teardown failures described in that group's historical README.

`verification.json` in each live group independently replays selected raw
occupancy, action, topology, freshness and truth evidence. Archive SHA-256 values
and a final root source snapshot are recorded in `manifest.json`; runtime
`crossing_manifest.json` pins the code actually executed. Offline/native groups
retain their earlier immutable manifests and test counts. Final regression
output in `tests_final.log` passes 169 tests; older logs are historical results.
`cleanup_final.json` confirms empty test domains and no active Phase 07 units.

The [Phase 07 record](../../../../docs/research/osm-AG/phases/07_GCU_ROLLING_RASTER.md)
explains limits and adaptation from the paper. The
[manual review guide](../../../../docs/research/osm-AG/plans/07_GCU_FINAL_REVIEW.md)
contains the fresh Phase 07 Ubuntu workspace and Foxglove commands. Human
acceptance is confirmed. `closure_audit_20261114.json` retains the successful
manual-session summary and empty Ubuntu robot-process audit. Manual return and
cancel/restart were not recorded in that user session; automated evidence stays
separate. Unused imports were removed from current source; archived executed
sources remain unchanged. `tests_closure.log` is the final regression rerun.
