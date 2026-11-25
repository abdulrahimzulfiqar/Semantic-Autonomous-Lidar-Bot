# Phase 08 initial shared-input tracking evidence

Phase 08 closed as a preliminary feasibility study by user decision on November
14, 2026. Only the first work group was executed; further structural integration
and controlled comparisons are deferred, not passed. Native AMCL controls
both missions, while the structural reference is replayed offline in Ubuntu.

- `source_audit/`: attached-paper fingerprint and pinned author-project tree.
- `clean_tracking/attempt01.tar.gz`: actual frozen clean native workspace, raw
  paired capture and structural estimate; `paired_scores.json` is independent replay.
- `clutter_tracking/attempt01.tar.gz`: same treatment with the physical box absent
  from the map. This is one occluder, not a crowd/dynamic-clutter benchmark.
- `candidate_source_snapshot.tar.gz`: exact executed tracker, config, worker and
  recorder sources. It excludes evaluator truth/AMCL inputs.
- `profiling.json`: preliminary native-live versus candidate-offline samples;
  deliberately labelled uncontrolled, not a RAM/CPU win or Pi benchmark.
- `tracking_comparison.png`: fixed-frame trajectories and absolute errors.
- `tests_initial.log`: 183 passing local tests, including 14 new implementation/
  scoring checks. Synthetic unit scenes are not performance evidence.

The candidate's small ideal-simulation errors do not establish real-robot accuracy,
global recovery or closed-loop navigation. Odometry-only is a separate diagnostic.
Source and artifacts are bound by `manifest.json`; final process state is in
`shutdown_audit.json`. See the Phase 08 closure record and plan for the reduced scope and two deferred
groups. `closure_decision.json` records why AMCL was retained and the move to
actual Pi baseline profiling.
