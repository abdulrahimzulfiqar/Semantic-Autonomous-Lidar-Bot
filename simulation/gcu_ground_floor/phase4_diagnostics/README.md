# Phase 04 metric-navigation evidence

- `robot_collision_model.urdf`, `footprint_audit.json`: expanded collision input,
  fingerprint and conservative nominal radius audit. The URDF is an audit input,
  not a portable launch file.
- `plan_only_report.json`, `plan_only_run.json`: ordered-startup stationary
  costmap/preview test; 7/7 cases passed, no commands/drift, clean lifecycle shutdown.
- `rejected_concurrent_start_*`: readiness failure from concurrent initialization;
  retained as a rejected portability test, not counted as a path-planning failure.
- `plan_preview_examples.png`: static rendering of the three measured preview
  paths and original occupancy map. This is no-motion metric planning, not osmAG
  topometric routing or autonomous navigation acceptance.

See the [Phase 04 plan](../../../docs/research/osm-AG/plans/04_GCU_AUTONOMOUS_METRIC_NAVIGATION.md)
for inputs, commands, known limitations and implementation history.

- `interface_audit.json`: all 68 interfaces screened for sampled local straight
  crossings; 58 found, 10 unresolved. This does not certify all routes.
- `autonomous_acceptance/`: complete 16-case final matrix, losslessly compressed
  raw reports, development/interrupt history, software versions and input hashes.
- `autonomous_trajectories.png`: initial planned paths and measured trajectories.

The final runner uses Cyclone DDS. Interrupted middleware/navigation attempts are retained
inside the autonomous evidence package, including the unsuccessful UDP-only profile.
Final reports and selected logs remain directly readable in
`autonomous_acceptance/trials/`. Development/interrupted reports and earlier-suite
metadata are bundled in `autonomous_acceptance/development_history.tar.gz`.
Its manifest verifies all 165 original members; historical report paths in
`results.json` retain their original names inside that archive. No failed outcome
was removed from the results summary.
See [Checkpoint 04](../../../docs/research/osm-AG/phases/04_GCU_AUTONOMOUS_METRIC_NAVIGATION.md)
for current acceptance and complete manual commands.

This is a qualified functional checkpoint. The consecutive-goal heading regression
failed, and two evaluation bridges segfaulted during shutdown after their tests.
See `autonomous_acceptance/teardown_review.json`; launch-group exits alone did
not expose these child-process failures. The final process audit is empty.

To verify the archived measurements from the repository root:

```bash
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/verify_gcu_navigation_evidence.py
```

Runtime snapshots include fingerprint-matched scripts, configuration and the
collision URDF. `snapshot_manifest.json` lists older scratch launchers that were
hashed in the VM but are not present in these snapshots; they are not dependencies
of the documented Phase 04 runner.

To inspect the development history without adding files to the checkout:

```bash
mkdir -p /tmp/gcu_phase4_history
tar -xzf simulation/gcu_ground_floor/phase4_diagnostics/autonomous_acceptance/development_history.tar.gz \
  -C /tmp/gcu_phase4_history
```

To compact a newly packaged evidence directory, use
`python3 scripts/compact_gcu_evidence.py --evidence PATH`, or pass
`--compact-history` to `summarize_gcu_navigation.py`. The compactor verifies
every byte before removing the individual historical copies.
