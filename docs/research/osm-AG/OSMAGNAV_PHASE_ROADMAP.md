# osmAG-Nav reproduction and EdgeOsmAG phase roadmap

## Quick summary

- **Current position:** Phases 01–07 retain their accepted bounded scopes. Phase 08 is closed as a preliminary localization study (183 tests, 1,260 paired frames); AMCL remains in production.
- **Current:** [Phase 09](phases/09_PI4B_NAVIGATION_BASELINE.md) closed for the measured corridor baseline on the Pi 4B. Phase 10 camera perception is next, followed by language and combined-load experiments.
- **Paper relationship:** osmAG routing, rolling maps and segmented execution are retained; author AGLoc/AGLoc++ integration is explicitly deferred. Exact full-stack reproduction is not claimed.
- **Research focus:** Measure resource-budgeted navigation, perception and language on constrained hardware. CPU/RAM benefits, novelty and robustness remain claims to establish experimentally.

Originally planned on 2026-11-04; reprioritized by user decision on 2026-11-14. This is the current forward roadmap for the numbered GCU
checkpoints. It complements the measured [phase records](phases/README.md).
The older master roadmap and “Phase 2” integration design preserve earlier work;
their numbering and direct `FollowPath` design are historical.

## Architectural target

The reference is [osmAG-Nav, arXiv:2603.28271v1](https://arxiv.org/abs/2603.28271v1).
Its building-wide planner operates on the hierarchical passage graph. It displays
the full route and sends intermediate goals through `NavigateToPose`; Nav2 plans
and controls each segment using a bounded raster generated from osmAG geometry.
The paper also integrates structure-based LiDAR localization.

The moving raster in Section 5.1 is **50 × 50 m at 10 Hz** in the authors'
deployment. This is separate from a controller's local costmap. Our Phase 04
controller uses a **6 × 6 m** local costmap and a full-floor occupancy map for
global planning/localization. Changing that local costmap's dimensions alone
would not reproduce the authors' rolling-map architecture.

## Provisional remaining phases

| Phase | Planned work and expected result | Relationship to the paper | Completion gate |
| --- | --- | --- | --- |
| **05 — Reliable execution and osmAG route preview** | Repair the consecutive-goal/shutdown gaps, record source availability and the explicitly approved paper-based routing implementation, validate frames and feasible passages, and display an osmAG route from a clicked destination. | Prepare the planner and interfaces in §§4, 5.2–5.3 and 7. | Reliable metric regressions plus correct, motion-free osmAG route previews and explicit failure results. [Measured Phase 05 record](phases/05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md); bounded [quality/fidelity follow-up](GCU_ROUTING_PAPER_AUDIT.md) passed (47 unit / 10 offline / 15 live preview cases), with user visual acceptance. |
| **06 — Segmented osmAG navigation** | Dispatch passage-derived `NavigateToPose` goals with continuous handoff, enforce/monitor transitions, and show the full route alongside the active Nav2 segment. Retain the static map. [Detailed plan](plans/06_GCU_OSMAG_SEGMENTED_NAVIGATION.md). | Implement §5.4 and §4.3; refined crossing targets and topology constraints are explicit adaptations. | Repeated missions confirm ordered crossings, final arrival, cancellation/closure handling and clean teardown; user visual acceptance. |
| **07 — Rolling raster and boundary goals** | Rasterize nearby osmAG geometry online, supply it to Nav2 and project segment goals inside the available map. | Reproduce §5.1, Figure 4 and §5.4.1. | Missions cross window boundaries, excluded regions stay blocked, and Nav2 planning no longer consumes a full-floor raster. Measure the memory of each remaining component. |
| **08 — Preliminary localization study** | [Closed record](phases/08_GCU_LOCALIZATION_COMPARISON.md): separate AMCL and 2D-reference variants, source/sensor audit and two paired shadow captures retained. AMCL selected for continued deployment. | Preliminary study related to §6; author AGLoc/AGLoc++ and production structural integration remain unreproduced. | Closed by user decision November 14, 2026. Further structural recovery, TF handoff and candidate closed-loop/resource comparisons are deferred, not passed. |
| **09 — Pi 4B navigation baseline** | [Completed for the measured corridor](phases/09_PI4B_NAVIGATION_BASELINE.md). Navigation ran on the Pi with AMCL; Gazebo and scoring ran in Ubuntu. | Establish the navigation-only baseline for the EdgeOsmAG hardware extension. | Three consecutive passes at 0.669–0.717× measured simulator rate with 2.19–2.21 average Pi CPU cores and about 370 MiB peak sampled PSS. |
| **10 — Camera perception and semantic observations** | Add a measured object detector and associate detections with poses/areas, confidence and timestamps. | EdgeOsmAG extension beyond core navigation reproduction. | Useful detections/area associations with measured accuracy, latency, resources and navigation impact; camera display alone is insufficient. |
| **11 — Language-guided tasks and map updates** | Convert bounded language requests into validated room/object goals or passage-status updates; compare feasible local and offboard inference explicitly. | EdgeOsmAG reasoning extension, informed by related osmAG/LLM work. | Supported tasks succeed, ambiguous requests have explicit outcomes, and language decisions do not directly command wheel velocities. |
| **12 — Combined edge evaluation and paper** | Compare navigation alone, +camera, +language and +both under matched tasks/seeds/hardware; collect controlled ablations and limitations. | Establish the actual capability/resource tradeoff of EdgeOsmAG. | Repeatable reliability and task-quality results with CPU/RAM, deadline/thermal metrics and claims matching the evidence. |

The original provisional Phase 09 integration freeze is folded into the new Pi
baseline scope. Former Phase 10 Pi work becomes Phase 09; former Phases 11–13
become 10–12. Completed phase numbers 01–08 are preserved. This reordering does
not report any hardware/perception/language milestone as already achieved.

Phase numbers are working checkpoints, not fixed deadlines. A phase may be split,
combined or reordered when a dependency is discovered. Calendar estimates should
follow the Phase 05 source/sensor audit; the source port, rolling-map integration
and localization compatibility are the largest current uncertainties.

## Important dependencies and scope decisions

1. **Preserve the Phase 04 baseline and Phase 05 repairs.** A return-goal heading
   regression failed, and two evaluation-only ground-truth bridges crashed during
   teardown. Propagate child-process failures into test status. Preserve the
   [Phase 04 evidence](phases/04_GCU_AUTONOMOUS_METRIC_NAVIGATION.md). Phase 05 validates a separate repaired profile with three two-goal runs, five cleanup controls and the original 16-case matrix; earlier failures remain archived.
2. **Reuse verified author components first.** The paper describes ROS2 packages
   `area_graph_data_parser` and `localization_using_area_graph`. Audit and pin the
   matching release before deciding what to port. The local older wrapper's name
   and existing graph search do not establish equivalence to the paper's full
   hierarchical pipeline. The 2026 project endpoint was audited in Phase 05 and
   contains a website rather than runtime packages. The user approved our
   explicitly labelled paper-based routing implementation. Exact author-runtime
   equivalence is not claimed. Phase 08 audited source/sensor compatibility and closed with AMCL retained;
   the author localization component has not been reproduced and its integration
   is deferred.
3. **Keep intermediate baselines honest.** Phase 06 is osmAG-guided execution with
   a static raster. Phase 07 can bound Nav2's planning map while AMCL still uses a
   full-floor localization map. Until that dependency is addressed, report it;
   do not claim that the whole stack has building-size-independent memory.
4. **Check localization sensor requirements early.** Existing simulated scan and
   odometry interfaces may differ from the author's point-cloud/localization
   inputs. If a compatible sensor or a 2D adaptation is needed, record that choice
   if structural localization is reopened. Retaining AMCL does not reproduce
   the authors' structure-aware localization module.
5. **Keep multi-floor work an explicit scope choice.** Add hierarchical cross-floor
   queries and elevator simulation after the single-floor baseline if needed.
   Respect wheelchair access constraints. Topological floor connectivity is not
   proof of an executed elevator mission. A single-floor reproduction must be
   labelled as such.

## What we should see at each major milestone

- **After 05:** Clicking a room shows the osmAG passage sequence and global route;
  the route-preview mode sends no motion goal.
- **After 06:** The robot follows that mission through ordered passages, while
  Foxglove shows the high-level route and currently executing metric segment.
- **After 07:** The served metric map follows the robot; distant segments advance
  through validated proxy goals and window updates.
- **After 08:** A bounded localization feasibility study and recorded decision to
  retain AMCL; original AGLoc/AGLoc++ parity remains unclaimed.
- **After 09:** Measured navigation compute/reliability on actual Pi 4B, with
  simulator and evaluator costs separated.
- **After 10–12:** Evaluate whether perception/reasoning add useful capability
  within that measured navigation budget.

## Research evidence to collect throughout

Save source revisions, map/parameter hashes, accepted and failed attempts, and
separate startup, planning, execution and shutdown results. Measure route success,
observed contacts, goal/localization errors, path length, planning time and resource
use. Distinguish offline graph preprocessing from online query latency.

For later comparisons, keep robot footprint, permitted passages, maps, goals,
hardware, middleware and timeouts consistent across the grid baseline and osmAG
variants. Add scaling maps because one shallow GCU hierarchy cannot establish the
paper's campus-scale planning advantage. Preserve the conventional metric baseline
and evaluate rolling maps, localization, camera and LLM additions separately.

Integration milestones can provide useful engineering evidence. Novelty, edge
resource savings and generalization remain hypotheses until comparisons support
them. Each completed phase gets a new achievement record with a four-bullet
summary, results, limitations and reproduction commands.

[06B2B record](phases/06B2B_GCU_FIRST_SEGMENTED_CROSSING.md): one constrained mission now passes ordered crossing, early handoff, final arrival/rest, independent replay and clean exits, with 104 unit tests. Earlier failures and the limited shutdown-mitigation claim are retained. This is a short-fixture result, not full Phase 06 acceptance.

[Phase 06 is complete for bounded simulation scope](phases/06_GCU_SEGMENTED_NAVIGATION.md),
accepted November 13, 2026. All 135 tests and retained automated mission/fault
checks pass. User visual acceptance covers short-distance behavior; the longer
manual journey remains a recorded wall-time timeout. Test processes are stopped.
Phase 07 is [complete for bounded simulation scope](phases/07_GCU_ROLLING_RASTER.md).
[Phase 08 closed its preliminary study](phases/08_GCU_LOCALIZATION_COMPARISON.md)
with AMCL retained. [Phase 09](phases/09_PI4B_NAVIGATION_BASELINE.md) then established the measured Pi corridor baseline.
The full-floor planning raster and AMCL remain in Phase 06; paper-runtime
reproduction and edge resource advantages are not claimed.
