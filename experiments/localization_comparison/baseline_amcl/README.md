# Variant A: accepted native AMCL baseline

This folder inventories the baseline rather than reimplementing AMCL.
`amcl_profile.yaml` is the exact resolved profile from the first clean workspace;
`baseline.json` records its provenance and accepted EKF sensor policy. The two
fixtures preserve the same query and execution-source pins, differing by the
physical box used for the occlusion condition. The accepted Phase 07 files stay
unchanged.

To prepare another clean workspace from a repository checkout inside Ubuntu,
use a new destination and the already accepted Phase 06 template:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/prepare_gcu_rolling_motion.py \
  --baseline /home/ubuntu/edgeosmag_phase6_review_final_ws \
  --source-assets simulation/gcu_ground_floor \
  --fixture experiments/localization_comparison/baseline_amcl/fixtures/clean.json \
  --destination /home/ubuntu/edgeosmag_phase8_my_clean_ws
```

For the box condition use `fixtures/clutter.json` and another new workspace.
Run the existing `run_gcu_segmented_crossing.py` from that workspace with its
fixture and explicit `--execute`. The existing simulation guard, runtime pins,
TF/goal ownership checks, arrival/rest scorer and teardown remain authoritative.

`common/capture.py` is a read-only observer, not a launcher. Start it only after
that runner creates `simulator.log` and has passed the domain-195 guard. It needs
ROS sourced, the runner's Cyclone DDS localhost config, `ROS_DOMAIN_ID=195` and
`ROS_LOCALHOST_ONLY=1`. Give it separate capture output, the eventual native
`run.json` path and the declared configured start. It automatically finishes
after the native run ends, with a bounded wall timeout. Do not start it before
the runner or bypass an occupied-domain error.

The first two raw captures are already frozen and can be replayed without
starting Gazebo or ROS; use `common/verify.py` as shown in the Phase 08 record.
