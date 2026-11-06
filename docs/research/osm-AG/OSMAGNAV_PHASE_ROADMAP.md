# osmAG-Nav reproduction and EdgeOsmAG phase roadmap

## Quick summary

- **Current position:** Phases 01–05 established the map, simulation, localization, metric navigation and tested paper-based osmAG route preview.
- **Next:** Phase 05 automated and user visual checks are complete. Execute ordered passage goals in Phase 06; the accepted [Phase 05 preview](phases/05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md) stays motion-free.
- **Authors' workflow:** Phases 06–09 add segmented execution, rolling raster maps, structural localization and integrated evaluation.
- **Our research:** Phases 10–13 evaluate Pi 4B deployment, camera perception, language reasoning and their combined resource/accuracy tradeoffs. This sequence is provisional.

Planned on 2026-11-04. This is the current forward roadmap for the numbered GCU
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
| **06 — Segmented osmAG navigation** | Dispatch passage-derived `NavigateToPose` goals, monitor area transitions and show the complete osmAG route alongside the active Nav2 segment. Initially retain the static floor map. | Implement the execution pattern in §5.4 and the action boundary in §4.3. | Repeated room-to-room missions cross the expected interfaces; cancellation, rejection and blocked-passage handling work without unsafe fallback. |
| **07 — Rolling raster and boundary goals** | Rasterize nearby osmAG geometry online, supply it to Nav2 and project segment goals inside the available map. | Reproduce §5.1, Figure 4 and §5.4.1. | Missions cross window boundaries, excluded regions stay blocked, and Nav2 planning no longer consumes a full-floor raster. Measure the memory of each remaining component. |
| **08 — Structure-based localization** | Audit sensor compatibility, then integrate/evaluate the authors' AGLoc estimator and Nav2 pose/TF adapter against our AMCL baseline. | Reproduce the relevant capabilities in §6 and the localization interfaces in §7. | Measured tracking, clutter and recovery trials pass with one TF owner. Report any sensor-driven adaptation explicitly. |
| **09 — Integrated paper-aligned baseline in simulation** | Freeze the combined stack and evaluate end-to-end routes, obstacle/closure handling, localization, planning latency and map scaling. | Evaluate the assembled architecture in §§4–7 using experiments motivated by §8. | Reproducible single-floor simulation report, repeat trials, versioned inputs and a clear list of reproduced/omitted paper capabilities. |
| **10 — Pi 4B navigation baseline** | Run the navigation workload on the actual Pi, with Gazebo on a separate machine; measure RAM, CPU, latency, temperature and reliability. | Begin the EdgeOsmAG extension; constrained-hardware results are ours to establish. | Same frozen navigation tasks run within an explicitly measured resource budget; middleware/network costs are reported separately. |
| **11 — Camera perception and semantic observations** | Add a measured object detector and associate observations with poses/areas, confidence and timestamps. | Extend the semantic representation toward our project goals; this is beyond the core navigation workflow being reproduced. | Useful detections/area associations with measured accuracy, latency and resource overhead. Camera display alone does not meet this gate. |
| **12 — Language-guided tasks and map updates** | Turn bounded language requests into validated room/object goals or passage-status updates; compare feasible local-model and offboard configurations. | Our reasoning extension, informed by the related osmAG/LLM work rather than a claim about this navigation paper. | Supported tasks succeed, ambiguous/unsupported requests produce explicit outcomes, and model decisions never directly publish wheel velocities. |
| **13 — EdgeOsmAG evaluation and paper** | Evaluate navigation, perception and reasoning together; run controlled ablations and document limitations. | Establish our contribution relative to the reproduced authors' baseline and conventional navigation. | Repeatable results showing the actual capability/resource tradeoffs; paper claims match the measured evidence. |

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
   equivalence is not claimed. AGLoc source/sensor compatibility remains a
   separate dependency for Phase 08; that component has not been reproduced.
3. **Keep intermediate baselines honest.** Phase 06 is osmAG-guided execution with
   a static raster. Phase 07 can bound Nav2's planning map while AMCL still uses a
   full-floor localization map. Until that dependency is addressed, report it;
   do not claim that the whole stack has building-size-independent memory.
4. **Check localization sensor requirements early.** Existing simulated scan and
   odometry interfaces may differ from the author's point-cloud/localization
   inputs. If a compatible sensor or a 2D adaptation is needed, record that choice
   before Phase 08. Retaining AMCL remains a useful variant, but does not reproduce
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
- **After 08–09:** The chosen localization module and combined stack have measured
  evidence. This is the checkpoint for assessing single-floor architectural
  reproduction of osmAG-Nav in simulation.
- **After 10–13:** We can evaluate whether our edge implementation and added
  perception/reasoning offer useful tradeoffs on Pi 4B.

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
