# Phase 07 offline window geometry

Offline geometry gate passes; native motion remains unadmitted. The final
report covers 56 tiles/all 844,000 reference cells and four padded-route fixtures.
`window_01` through `window_03` save the three demonstration snapshots, not the
whole-floor evaluation grid. The full PGM is used only by the evaluator.

The blocked edge intentionally moves; overlap consistency applies to interiors.
Raw window occupancy is 16 KiB, not the memory of a Nav2 costmap or whole stack.
Local timings do not establish VM/Pi or online update performance.

146 tests pass. `source_snapshot.tar.gz` and `manifest.json` pin implementation
and artifacts. Reproduction and the remaining two work groups are in
[the progress record](../../../../docs/research/osm-AG/phases/07_GCU_ROLLING_RASTER.md).
