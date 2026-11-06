# Phase 05 plan: reliable execution and osmAG route preview

## Quick summary

- **Goal:** Make the execution handoff reliable and verify that osmAG can select and display valid GCU routes.
- **Current status:** Complete for the bounded Phase 05 scope: 47 unit tests, 10 offline and 15 live preview cases passed; user visual acceptance is confirmed. Initial metric acceptance is preserved. See the [measured record](../phases/05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md).
- **Deliverable:** A pinned planner implementation, validated frame/passage contract and click-to-preview osmAG routes; new osmAG-driven motion stays disabled here.
- **Next:** Phase 06 will execute ordered passage goals using the static floor map, followed by rolling-map integration in Phase 07.

Date: planned 2026-11-04. Predecessor:
[Phase 04](../phases/04_GCU_AUTONOMOUS_METRIC_NAVIGATION.md).
Overall sequence: [osmAG-Nav and EdgeOsmAG roadmap](../OSMAGNAV_PHASE_ROADMAP.md).
The sequence below specifies the acceptance gates. Current progress and the
approved source-availability adjustment are recorded at the end; no Phase 05
completion is claimed until runtime gates pass.

## Objective and paper alignment

Prepare the building-wide routing layer described in §§4 and 5.2–5.3 of
[osmAG-Nav v1](https://arxiv.org/abs/2603.28271v1). A selected destination should
produce an ordered passage route and a geometric route, with explicit validation
and failure reasons. The output must identify the next area transitions that
Phase 06 will turn into `NavigateToPose` requests.

The existing GCU executor uses NavFn A* on a whole-floor occupancy map. The older
`lidarbot_osmag` wrapper sends a complete route directly to `FollowPath` and
contains a missing-TF `(0,0)` start fallback. Audit its other fallback paths before
reuse. Do not enable that legacy executor alongside the Phase 04 stack.

The GCU adapter has 59 geometric leaf sectors and 68 exported interfaces; these
are not 59 named rooms. The Phase 04 search found local straight crossings for
58 interfaces and left 10 unresolved. Neither result certifies an entire mission.

## Work sequence and bounded gates

### 05A — Close the execution regressions

- Reproduce the saved consecutive-goal failure: short corridor goal, then return
  to the original position with the requested final heading, using the same live
  stack. Inspect the interaction between position tolerance, heading correction
  and the progress checker; record the cause and the chosen repair.
- Investigate the evaluation ground-truth bridge's SIGINT teardown crashes.
  Evaluate shutdown ordering and the bridge behavior; do not assume a cause from
  the existing logs alone. Check child-process exits, not only launch-group exits.
- Preserve Phase 04's commit and frozen evidence. Put new trials in a new Phase 05
  results directory and identify changed inputs; never rewrite old failed runs as
  successes.

**Proposed gate:** Three fresh repetitions of the same two-goal sequence reach
both requested poses with the Phase 04 final-error/rest limits. Five short
start/stop cycles have no critical child-process failures, and a failing child is
reported as a harness failure. Cancellation/rejection still work. If the runtime
controller/profile changes, repeat the original 16-case matrix once with frozen
settings after targeted checks pass. Diagnose the first new failure before reruns;
do not begin an open-ended parameter sweep.

### 05B — Pin the author source and identify compatibility gaps

- Locate the implementation matching the paper's `area_graph_data_parser`
  planner/map services and record repository URL, revision, license, ROS/dependency
  requirements and entrypoints. Prefer reuse/porting of verified author code.
- Compare it with our local parser and engine. Check passage-level graph
  construction, raster-derived edge costs, virtual start/goal attachment, cached
  hierarchy and the attach → lift → common-parent search → expand pipeline.
- Produce a component table: reusable as-is, needs an adapter, or absent/unverified.
  Identify the source and sensor requirements of the localization module now,
  while deferring its runtime integration to Phase 08.
- Report the GCU map's actual hierarchy depth. If it is too shallow to exercise
  higher-level routing, use declared hierarchical fixtures for algorithm tests;
  do not invent room semantics or claim a scaling result from the flat map.

**Gate:** Exact source versions and an honest paper-to-code compatibility report
exist. Verified supported fixtures produce valid graph routes, with cost
agreement against a reference shortest-path search on the same graph. No timing
or hierarchical-equivalence claim is accepted on the package name alone.

### 05C — Establish coordinates and a feasible passage contract

- Record the mapping between osmAG geometry, the occupancy map and simulation
  coordinates: units, origin, scale, axis directions and floor identifier.
  Validate at least three non-collinear landmarks; proposed positional tolerance
  is one occupancy cell (about 0.0586 m). Explain any systematic offset.
- Document `AGmap`, `map`, `odom` and robot-frame relationships. Keep a single
  owner of `map -> odom`. An identity transform is acceptable only after measured
  agreement; missing/stale TF must reject a request rather than substitute a pose.
- For each interface, check its incident sectors, bounds, footprint clearance and
  candidate approach/crossing/exit poses. Confirm the directed transition actually
  enters the destination sector and respects excluded holes.
- Keep unresolved or non-traversable interfaces out of executable route selection
  until validated. A failed straight-crossing search does not prove every curved
  crossing impossible. Do not widen real geometry or reduce the audited robot
  footprint to make a route pass.
- Define rejected routing searches as no-route results. A Euclidean connection
  across a failed raster search may not become executable without independent
  footprint validation. Preserve raw author behavior separately if needed for
  comparison and label the execution policy difference.

**Gate:** A versioned coordinate report and per-interface feasibility report are
available, and every passage selected in accepted preview routes is validated.
Disconnected or unresolved transitions produce an explicit reason or a valid
alternate route through the remaining feasible graph.

### 05D — Click-to-preview osmAG routes in Foxglove

- Accept a destination, obtain the robot start from TF, and locate both in valid
  leaf sectors. Compute and publish the passage sequence and geometric route.
- Display area/passages, full route, selected goal and rejection reason. Show the
  Phase 04 Nav2 metric route in a separate color when performing comparisons.
- Include a route ID, map/version hash, frame, ordered area/passage IDs, graph cost,
  preprocessing/query timing and validation status in saved output.
- Validate the route's swept footprint against the reference geometry. Distinguish
  a valid graph result from an executable path and record both outcomes.
- Route preview must not send navigation actions or publish velocity commands.
  Motion in 05A is a separate, explicitly enabled metric regression profile.

Proposed interface names below are a design contract to confirm during source
integration, not interfaces already implemented:

| Interface | Intended role |
| --- | --- |
| `/osmag/preview_goal` (`PoseStamped`) | Motion-free goal input |
| `/osmag/global_path` (`nav_msgs/Path`) | Complete high-level geometric route in its declared frame |
| `/osmag/route_status` | Route ID, ordered transitions, feasibility state and failure reason |
| `/osmag/map_markers` | Sector/passage and feasibility visualization |

## Proposed preview acceptance cases

Freeze coordinates, map hashes, expected outcomes and timeouts after source/frame
validation and before running the acceptance set. Use deterministic fixtures for
failure cases that the real floor does not naturally provide.

| Case | Expected evidence |
| --- | --- |
| Same-sector free goal | Valid within-sector route, no unnecessary passage transition |
| Adjacent-sector goal | Correct incident passage and directed sector transition |
| Room/corridor goal across multiple interfaces | Ordered feasible passage sequence and a footprint-valid route |
| Courtyard, occupied or outside-map goal | Explicit rejection and cleared stale route |
| Free but disconnected goal | No-route result; no fabricated straight-line substitute |
| Disabled/unresolved passage | Alternate feasible graph route or explicit no-route result |
| Missing/stale TF or unsupported frame | Rejection; no synthetic start pose |
| Repeated preview request | New route/status replaces the old one; no navigation action or motion command |

For each accepted path, compare the selected passage sequence with the map and
independent graph/geometry checks. When equal-cost alternatives exist, compare
cost and validity rather than requiring one arbitrary tie-breaking sequence.
Cold-start preprocessing time and warmed query time must be separate.

## Deliverables and stop point

- Source/component audit and versioned coordinate/passage reports.
- Isolated preview launch, explicit metric-regression launch and reproducible
  commands. The old direct-execution launch remains disabled in these tests.
- Visual examples showing a complete osmAG route and at least one rejection.
- Frozen acceptance summary, raw evidence and all failed/interrupted attempts.
- A new `phases/05_...md` achievement record only after the gates pass, beginning
  with a short summary and distinguishing repaired, validated and deferred items.

Phase 05 ends with a trusted route/transition contract and reliable metric goal
handling. Phase 06 adds passage-by-passage motion using the static map. Rolling
raster serving, boundary proxy goals, structural localization and Pi/LLM/camera
claims have their own later gates in the roadmap.

## Potential research evidence

The interface-feasibility audit, unsafe-fallback handling, explicit author-port
differences and repeatable consecutive-goal failures can support an engineering
analysis. Reusing the authors' routing algorithm is a reproduction result;
novelty, planning speedups and edge resource savings require later comparisons.

## Execution update (2026-11-05)

The official [project page](https://jiajiezhang7.github.io/osmAG-Nav/) labels code
availability “Upon Acceptance.” Its website repository at revision
`9f4fa7e6cff57f460b773b7a79f64122658f920b` does not contain the ROS2 runtime
packages described in the paper. The user explicitly approved a clearly labelled
paper-based implementation. Accordingly, 05B now evaluates our reference
implementation and compatibility report; an exact author-runtime port remains
unavailable and is not claimed.

The new preview reuses the GCU schema, coordinate convention and footprint-check
helpers, with a Python cached-child-trace/LCA search and raster-derived costs.
The older direct `FollowPath` wrapper remains disabled. Offline tests check costs
against SciPy's independent graph search and exercise a deeper hierarchy fixture;
the real GCU hierarchy remains shallow. Reference-raster dependence is retained
until the later rolling-map phase.

The installed Nav2 rotation shim now handles final heading over the existing RPP profile. The final frozen stack passed three fresh two-goal sequences, five cleanup controls and the unchanged original 16-case matrix. The evaluation-only pose bridge matched native payloads at 80 timestamps. Config-file bridges use the installed conversions with ordered transport/context teardown; Gazebo shutdown waits for actual server exit after its stop RPC. Failed attempts remain in the archived evidence.

Offline routing passed eight cases; live preview passed thirteen, including TF guards, closures and explicit rejection. Preview published no velocity commands or navigation actions, measured zero robot drift, and observed one publisher per required frame edge. Independent verification checked 642 archived files, 21 measured navigation traces and 49 matching repository source/config files. The source-availability adjustment above remains in effect: these results do not establish exact author-runtime equivalence.

Complete commands, route examples, measured limits and the interactive Foxglove check are in the [Phase 05 record](../phases/05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md). New osmAG-driven motion remains a Phase 06 milestone.

## Follow-up review (2026-11-06)

The [paper audit](../GCU_ROUTING_PAPER_AUDIT.md) reproduced fixed-crossing detours in clear space and documented algorithmic differences from the publications. The original checks establish bounded safety, process reliability and agreement with the constructed graph; they do not certify continuous-space shortest paths or exact Algorithm 2 equivalence. Add route-quality cases and resolve the documented fidelity boundary before advancing to passage-driven motion. No historical accepted result was rewritten by this review.

## Follow-up completion (2026-11-06)

The approved bounded repair is complete for the preview contract: topology/closure-preserving string pulling, separate graph/refined outputs, duplicate-interface filtering, recursive cached compact/lift propagation and a validated global-graph fallback. New evidence passes 47 unit tests, 10 offline and 15 live motion-free cases, including direct clear routes, real obstacles, narrow portals, ordered crossings, disabled interfaces and hierarchy fallback. No executor change or repeat motion matrix was needed. The v2 workspace, current commands, before/after figure and retained v1 evidence are in the [Phase 05 record](../phases/05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md). The user confirmed the updated v2 Foxglove preview on 2026-11-06, closing the interactive gate before Phase 06. Exact author runtime, global continuous optimality, multi-floor policy, rolling maps and structural localization remain outside this checkpoint.
