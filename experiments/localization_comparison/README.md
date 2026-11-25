# Separate localization variants for EdgeOsmAG

Phase 08 closed as a preliminary feasibility study on November 14, 2026. AMCL
remains the production localizer. The structural reference is retained for
optional research; further recovery/TF/closed-loop integration is deferred.
The experiment establishes no controlled CPU/RAM advantage or Pi benefit.


- `baseline_amcl/`: frozen inventory of the accepted Phase 07 AMCL profile. No estimator source here is a replacement for Nav2 AMCL.
- `structural_2d/`: our experimental, paper-inspired 2D reference. It is **not** released AGLoc or AGLoc++ and does not implement their complete initialization/fusion pipeline.
- `common/`: read-only live capture and evaluator-only scoring. Candidate observations exclude truth and AMCL outputs.

The accepted Phase 07 launch/executor files are not modified. Native AMCL remains
the sole `map -> odom` owner during shadow evaluation. The structural reference
publishes no TF or commands. Its worker accepts only `observations.npz`, OSM
geometry and a declared configuration; `evaluation.npz` is read only by scoring.

Keep the same scans, fused wheel/IMU odometry, timestamps, initial-pose rule,
metric frame, world and navigation profile across variants. Label sensor
adaptations and do not claim our one-ring scan reproduces vertical 3D clutter
filtering. Evaluate every recorded frame, including rejected/weak/lost frames.
No truth-based pose alignment or per-case tuning is allowed in the fixed batch.

The first experiment is open-loop tracking on an AMCL-controlled trajectory.
Synthetic estimator unit tests are not performance evidence. Shadow results do
not establish closed-loop candidate navigation, global recovery, scaling or Pi
resource performance. Those are explicit later gates in the Phase 08 plan.

After a read-only live capture, replay the candidate with the project Python:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 experiments/localization_comparison/structural_2d/replay.py \
  --observations /path/to/capture/observations.npz \
  --osm simulation/gcu_ground_floor/gcu_ground_floor.osm \
  --output /tmp/structural_estimate.json

PYTHONDONTWRITEBYTECODE=1 python3 experiments/localization_comparison/common/score.py \
  --capture /path/to/capture \
  --estimate /tmp/structural_estimate.json \
  --output /tmp/paired_scores.json
```

Linux component samples and Mac development timings must remain separate. The
3D paper's published speed/accuracy figures are not measurements of this code.
