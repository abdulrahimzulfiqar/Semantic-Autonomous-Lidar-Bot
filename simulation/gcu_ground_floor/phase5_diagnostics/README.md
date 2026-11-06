# Phase 05 evidence index

The accepted checkpoint is described in the [Phase 05 record](../../../docs/research/osm-AG/phases/05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md).

- **Current preview:** `preview_refinement_v2/` contains the 10 offline cases,
  15 live ROS cases, 47-test log, before/after figure, independent verification,
  process cleanup and separately labelled user visual acceptance.
- **Metric execution:** `metric_results.json` summarizes the frozen 16-case
  matrix, three two-goal trials and five shutdown controls. These measurements
  used the original Phase 05 runtime; preview refinement did not change the
  executor or repeat the motion matrix.
- **Historical preview and diagnosis:** `preview_acceptance_final/`,
  `live_preview_report.json`, `route_quality_audit/` and `route_previews.png`
  preserve v1 results and explain the fixed-crossing detour.
- **Source provenance:** `source_audit.json` records the paper-based
  implementation and intentional differences from the publications.

`runtime_evidence.tar.gz` and its SHA-256 manifest retain raw runtime history,
including failed attempts. `preview_v1_source.tar.gz` pins changed preview source
so historical measurements are not attributed to v2. The v2 subdirectory has its
own smaller runtime archive and manifest. Readable summaries duplicate selected
archived reports for inspection; the verifiers check their agreement. These
archives are retained research evidence, not temporary build products.

From the repository root, using Python with the documented pipeline dependencies:

```bash
python3 scripts/verify_gcu_phase5_evidence.py \
  --evidence simulation/gcu_ground_floor/phase5_diagnostics
python3 scripts/verify_gcu_preview_refinement.py \
  --evidence simulation/gcu_ground_floor/phase5_diagnostics/preview_refinement_v2
```

The first command writes `independent_verification_v1_replay.json` when archived
v1 source is needed. The second writes `preview_refinement_v2/verification.json`.
Neither command starts ROS, Gazebo or robot motion.
