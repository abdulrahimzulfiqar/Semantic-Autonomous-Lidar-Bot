# Phase 03 evidence

- `final_acceptance/`: frozen five-case suite, complete motion/recovery/watchdog
  traces, summary, software versions and input hashes. All five cases passed.
- `motion_user_report.json` and `user_motion_view_72s.png`: user-run reproduction
  with full pose pairs and one sampled recording frame. The original movie remains
  outside the repository.
- `motion_summary.json` and preliminary motion reports: tuning comparisons.
  Redundant exploratory sample arrays were removed; aggregate results remain.
  Complete measured motion samples are retained in the final and user reports.
- Stationary, robot-pose and rejected-initialization reports: diagnostic evidence
  for caster leveling, renderer selection and nonzero-uncertainty initialization.
- Parameter snapshots distinguish the original stationary profile from the final
  motion profile. Main runtime configuration is `../amcl.yaml`.

See the [Phase 03 record](../../../docs/research/osm-AG/phases/03_GCU_LOCALIZATION_AND_CONTROLLED_MOTION.md)
for acceptance criteria, reproduction and the retained grazing-edge limitation.
