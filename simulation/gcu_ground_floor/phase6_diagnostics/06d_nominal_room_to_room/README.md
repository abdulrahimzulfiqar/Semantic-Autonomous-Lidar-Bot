# 06D: room-to-room nominal pair

The frozen 8.858 m reference connects room 33 to room 34 through `-95`, `-94`,
`-106`, using the same padded execution-route, measured-IMU and aligned-grid
profile as the other nominal pairs. This is a conservative mask-valid reference,
not a continuous shortest-path claim. See the
[phase record](../../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for measured results, current acceptance status and remaining Phase 06 gates.

Each `attemptNN.tar.gz` retains the raw report, native logs, constraint arrays,
actual executed Python sources and prepared assets. `verification.json` records
independent replay for completed trials. `frozen_inputs.json`, when the pair is
complete, verifies identical source/assets/fixture bytes between repetitions.
`manifest.json` binds artifact/source hashes and `verification_tools.tar.gz`
freezes the replay tools. The actual prepared IMU launch is archived separately
from the root launch and reported as a historical-source difference by replay.

`cleanup.json` is the pair's final isolated-process/unit audit. Both runs
pass live scoring and independent replay, with all five native exits zero, zero
observed contacts and all three handoffs continuous. Both checkers retain the
known Python `Destroyable` warning.
VM timing warnings in native logs remain preserved. `local_tests118.log` retains
the passing suite for unchanged worker code; it was not rerun for every trial.

From the repository root, replay a completed trial (`01` or `02`):

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_segmented_crossing.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06d_nominal_room_to_room \
  --trial 01 --case-prefix nominal
```

The derived `historical_verification_replay.json` is outside the artifact hash
set and does not overwrite the saved pair result. No older failed trial or
rejected proposal was overwritten. Completing this nominal pair does not close
fault, baseline-comparison or final Foxglove acceptance gates.
