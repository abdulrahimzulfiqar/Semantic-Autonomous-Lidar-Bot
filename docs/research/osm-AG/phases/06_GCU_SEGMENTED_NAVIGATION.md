# Phase 06: osmAG-guided segmented navigation

## Quick summary

- **Status:** Complete for bounded GCU simulation scope; accepted November 13, 2026 following automated acceptance and the user's short-distance visual review.
- **Built:** Complete osmAG route display, passage-derived Nav2 goals, continuous handoff, constraint masks, cancellation and guarded recovery.
- **Evidence:** 135 unit tests; six nominal runs, consecutive missions, fault controls, physical detour, matched execution comparisons and ten final interface checks. Detailed raw evidence remains linked below.
- **Next:** Phase 07 adds a rolling planning raster and boundary proxy goals. Full author-runtime reproduction, structural localization and hardware performance remain unclaimed.

## What is accepted

The paper-based planner supplies the mission route; the executor dispatches
successive passage/final targets through `NavigateToPose`. Nav2 generates each
metric segment and controls movement using sensor-updated costmaps. Ordered
crossing, early handoff, arrival/rest, footprint and contact checks have independent
simulation evidence. Cancellation, required executor IMU-input loss, logical
closure/no-route handling and an unannounced physical obstacle were tested.

The user confirmed short-distance visual behavior, then explicitly requested
phase closure and merge. Their review does **not** establish a manually observed
long multi-passage journey. Automated retained trials establish the longer bounded
mission results. A November 12 manual 14.17 m reference journey ended with
`crossing_wall_timeout` after about 7.50 m; 75.44 simulated seconds took 394.40
wall seconds during motion. That attempt remains failed, not relabelled as arrival.
The review speed cap is 0.10 m/s; observed simulation slowdown is separate from
the configured speed. Session `passed` measures harness/session handling and is
not proof that every requested mission arrived; inspect per-mission outcomes.

## Evidence and reproduction

- [Nominal matrix and final acceptance details](06D_GCU_NOMINAL_MISSIONS.md).
- [Consecutive mission evidence and localization follow-up](06D2_GCU_CONSECUTIVE_MISSIONS.md).
- [Grouped fault, comparison and interface evidence](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06_faults_and_comparison/README.md).
- [Restart and visual review instructions](../plans/06_GCU_FINAL_REVIEW.md).
- [Original plan and component checkpoints](../plans/06_GCU_OSMAG_SEGMENTED_NAVIGATION.md).

Run local regressions from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python -m unittest discover -s tests
```

Actual runtime sources, configurations, failed trials and independent verifier
snapshots remain frozen in the evidence folders. Diagnostic tools are retained
because they reproduce those results. Disposable caches are excluded; source
archives are not unused application code. Ubuntu test processes were stopped
and a fresh empty-process audit recorded at closure.

## Limits and relationship to osmAG-Nav

This closes the segmented-execution milestone described in the existing paper
audit and roadmap. Refined crossing targets and topology masks are explicit
adaptations in our labelled paper-based implementation. The current global
planning raster still covers the floor, the local costmap is a 6 × 6 m rolling
window, and AMCL still consumes the full-floor map.

Two matched comparisons use the same permitted-area mask and motion profile;
they establish execution functionality, not a statistical speed or memory advantage.
The closure case establishes safe stop/no-route, not general alternate-route
execution. Known teardown warnings/hangs in historical attempts remain documented.
No Pi 4B, camera/LLM fusion, multi-floor execution or complete author-runtime
equivalence is claimed by Phase 06.

The [Phase 07 plan](../plans/07_GCU_ROLLING_RASTER_AND_BOUNDARY_GOALS.md)
removes the full-floor raster dependency from Nav2 planning while retaining AMCL
as an explicit baseline. Structural localization follows separately in Phase 08.
