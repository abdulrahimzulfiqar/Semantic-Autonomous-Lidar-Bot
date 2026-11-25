# EdgeOsmAG phase records

Each completed checkpoint gets a numbered record here. Keep detailed technical
references at their existing paths and link them from the record. Future records
should state the goal, changes, evidence, limitations, reproduction instructions
and next acceptance gate. Plans describe intended work; records describe evidence.

Start every new phase record and plan with a **Quick summary** immediately below
its title. Keep it to four short bullets (about 100 words maximum): goal/scope,
result or current status, open issues, and the next checkpoint. Keep measured
results distinct from planned work, and put detailed evidence and commands below.


| Checkpoint | Status | Record |
| --- | --- | --- |
| 01 — GCU CAD-to-osmAG reconstruction | Accepted ground-floor output | [Phase 01](01_GCU_CAD_TO_OSMAG.md) |
| 02 — GCU simulation and Foxglove visualization | Initial stationary visual alignment and camera confirmed; navigation pending | [Phase 02](02_GCU_SIMULATION_VISUALIZATION.md) |
| 03 — Localization and controlled motion | Complete for bounded simulation checks; known sensor limitation retained | [Phase 03](03_GCU_LOCALIZATION_AND_CONTROLLED_MOTION.md) |
| 04 — Footprint-feasible autonomous navigation | Original 16-case matrix passed; bounded follow-up repairs validated in Phase 05 | [Phase 04](04_GCU_AUTONOMOUS_METRIC_NAVIGATION.md) |
| 05 — Reliable execution and osmAG route preview | Complete for bounded scope: 47 unit / 10 offline / 15 live preview cases passed; user visual acceptance confirmed | [Phase 05](05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md) |
| 06 — osmAG-guided segmented navigation | Complete for bounded simulation scope; 135 tests, retained automated motion/fault evidence and limited short-distance user visual acceptance | [Phase 06](06_GCU_SEGMENTED_NAVIGATION.md) |
| 07 — Rolling raster and boundary goals | Complete for bounded simulation scope; 169 tests, automated motion/fault evidence and successful user clicked-goal review; accepted November 14, 2026 | [Phase 07](07_GCU_ROLLING_RASTER.md) |
| 08 — Localization comparison | Closed preliminary study; 183 tests and 1,260 paired frames; AMCL retained, structural production integration deferred | [Phase 08](08_GCU_LOCALIZATION_COMPARISON.md) |
| 09 — Pi navigation baseline | Complete for the measured corridor scope Nov 25. Three consecutive scored passes at 0.669–0.717×, with two separate fixed-rate stationary replays. | [Phase 09](09_PI4B_NAVIGATION_BASELINE.md), [hotspot and replay diagnostics](09E_PI4B_FULL_STACK_REPLAY.md), [plan](../plans/09_PI4B_NAVIGATION_BASELINE.md) |

Phase 06 component records retain the development history and failed attempts.
The consolidated completion record above defines the accepted scope. Long manual
navigation timed out on the slow VM; automated longer mission evidence remains
separate from the user's short-distance review. Full osmAG-Nav simulation remains
in progress; see the [forward roadmap](../OSMAGNAV_PHASE_ROADMAP.md).

[Phase 07 completion record](07_GCU_ROLLING_RASTER.md): bounded OSM windows feed
the live Nav2 planner/controller. Automated motion, fault/recovery, matched
static-map comparison and user visual acceptance pass. The user session records
16 windows and 10 proxies; Ubuntu has no remaining robot processes. Full-map
AMCL remains explicit. Phase 08 subsequently closed its preliminary localization
study with AMCL retained.

[Phase 08 closure record](08_GCU_LOCALIZATION_COMPARISON.md): AMCL and the
paper-inspired structural reference remain separate. The study captures
and scores 1,260 common scan frames; further structural integration/evaluation
is deferred by user decision. Next is actual Pi 4B navigation baseline profiling.
