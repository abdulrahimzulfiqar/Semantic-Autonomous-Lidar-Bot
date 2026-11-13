# 06D: frozen adjacent-sector pair

Two fresh isolated trials on one frozen profile cross passage `-93` from
corridor 14 to corridor 16. The 1.941 m query uses the same padded route,
measured-IMU yaw-rate fusion, aligned map-frame local raster, controller and
limits as the accepted corridor-to-room pair. This group is part of the
six-run nominal matrix, not full Phase 06 acceptance.

`attempt01.tar.gz` and `attempt02.tar.gz` retain raw reports, constraint arrays,
native logs, actual sources and prepared assets. `frozen_inputs.json` verifies
identical source/asset/fixture bytes in both. Both native runs pass; independent
replay results are stored in `verification.json`. Final errors are 0.05742 m /
1.1674° and 0.08836 m / 1.0899°. The checker logs are empty in these two runs;
that does not resolve the intermittent warning recorded in earlier trials.

The prepared IMU launch differs from the root launch and is preserved in the
source snapshot. `manifest.json` binds artifacts and sources, and
`verification_tools.tar.gz` freezes the replay tools. `local_tests118.log` is the
already-passing unit log for the unchanged worker code, retained from admission;
it is not a claim that the suite was rerun for each live trial. `cleanup.json`
records zero isolated test processes/units at the end of the pair.

From the repository root, independently replay either trial (`01` or `02`):

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_segmented_crossing.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06d_nominal_adjacent_sector \
  --trial 01 --case-prefix nominal
```

This writes a derived `historical_verification_replay.json` because the actual
prepared launch is archived; it leaves the saved pair result unchanged.
See [the phase record](../../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for remaining gates and Ubuntu live reproduction.
