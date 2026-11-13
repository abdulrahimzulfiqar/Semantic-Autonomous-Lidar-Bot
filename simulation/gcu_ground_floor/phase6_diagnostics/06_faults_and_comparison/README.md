# Phase 06: fault/comparison and automated acceptance

- **Status:** All declared automated gates pass; **135 tests pass**. Bounded acceptance recorded November 13, 2026; user review covered short-distance behavior.
- **Next:** [Phase 07 plan](../../../../docs/research/osm-AG/plans/07_GCU_ROLLING_RASTER_AND_BOUNDARY_GOALS.md). [Phase 06 completion scope](../../../../docs/research/osm-AG/phases/06_GCU_SEGMENTED_NAVIGATION.md).
- **Scope:** Static global raster/AMCL; limited fixture evidence, not full author-runtime reproduction or a performance benchmark.

| Gate | Evidence |
| --- | --- |
| Live cancel | `cancel/`: 0.283 simulated seconds to rest onset |
| Passage closure | `closure/`: 0.297 seconds, revised native mask and explicit graph/native no-route |
| Required executor IMU input loss | `imu_loss/`: 0.432 seconds; raw sensor remains live |
| Physical unknown obstacle | Failed `obstacle/` retained; strict executed-body follow-up passes in `obstacle_followup/` |
| Matched execution comparisons | `baseline_adjacent/`, `baseline_room/`, `matched_comparison.json`; one baseline per query, same mask/profile |
| Fresh ROS mock controls | `mock_regression/`: ten pass, zero commands, no physics/controller |
| Final live interface | `interactive_review/`: ten interface checks and three independently replayed arrivals pass |

Each motion group retains raw trials, sources/assets and independent replay.
`status.json` binds the final reports. Earlier nominal and consecutive evidence
remains frozen separately. Automated stacks were cleaned before a separate,
bounded human-review session; that intentional session is not a leftover.
See [the phase record](../../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for failed-trial explanations and limits. General alternative-route execution,
rolling global maps and edge-hardware performance are not established here.
