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

Full osmAG-Nav simulation remains in progress. The current forward sequence is
the [osmAG-Nav reproduction and EdgeOsmAG roadmap](../OSMAGNAV_PHASE_ROADMAP.md),
covering proposed phases 05–13 and their relationship to the authors' paper.
The earlier [integration design](../PHASE_2_GAZEBO_ROS2_INTEGRATION_PLAN.md)
is retained as implementation history.
