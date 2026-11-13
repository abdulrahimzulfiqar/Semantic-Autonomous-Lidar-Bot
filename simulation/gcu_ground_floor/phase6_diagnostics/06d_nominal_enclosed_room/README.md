# 06D: two frozen corridor-to-room trials

Both fresh trials pass live scoring and independent replay. This is **two of
six nominal runs**, not full Phase 06 acceptance. See the
[phase record](../../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for the route repair, measured results and remaining gates.

`attempt01.tar.gz` and `attempt02.tar.gz` retain raw reports, native logs,
constraint arrays, actual executed Python sources and prepared world/parameters.
The same source, fixture and assets were used in both runs. The prepared IMU
launch differs from the root launch; `source_snapshot.tar.gz` preserves it and
the verifier reports that difference explicitly. `admission_source.tar.gz`
retains the source used to prepare the admitted fixture. No failed older trial
was overwritten or relabelled.

`manifest.json` binds source and artifact hashes; `verification_tools.tar.gz`
pins the replay tools. `verification.json` contains both independent results.
`cleanup.json` records zero remaining isolated test processes/units. All five
native parents exit zero in each run; both checker logs retain the known Python
`Destroyable` teardown warning. The root 118-test log and route comparison plot
with its plotting source are retained.

From the repository root, replay either raw trial (substitute `02` for `01`):

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_segmented_crossing.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06d_nominal_enclosed_room \
  --trial 01 --case-prefix nominal
```

Because the prepared launch is archived, this command writes
`historical_verification_replay.json`; it does not replace the saved two-trial
result. That derived replay output is intentionally outside the artifact hash set.
