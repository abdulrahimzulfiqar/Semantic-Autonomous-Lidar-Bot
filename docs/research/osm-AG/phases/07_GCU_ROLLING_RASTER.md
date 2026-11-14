# Phase 07: rolling raster and boundary goals — complete

## Quick summary

- **Status:** Complete for bounded GCU simulation scope; user acceptance confirmed November 14, 2026.
- **Built:** OSM-only 128 × 128 planning windows, validated boundary proxy goals, native consumption/freshness gates and passage-aware execution.
- **Results:** 169 tests pass. Forward/reverse crossings, an unseen physical obstacle and source-loss/recovery pass independent replay; matched static/rolling runs bound the planning grid.
- **Next:** Phase 08 evaluates structure-based localization. Full-floor AMCL and route preprocessing remain explicit dependencies.

## User acceptance and closure — November 14, 2026

The user confirmed that the Phase 07 review works and requested finalization.
The retained interactive session `manual_20261114_072600` records one successful
clicked-goal mission to approximately `(5.3944, -3.2883, 1.5395 rad)`. It traverses
**16 planning windows**, sends **10 proxy goals**, confirms passage `-93` and
finishes with **0.1068 m position error and 0.1382° heading error**. The session
lasts 815.48 wall seconds, confirms final rest and has six clean process exits
(the sixth process is Foxglove). Raw evidence and independent replay are in
`phase7_diagnostics/manual_review/`.

This is acceptance of that observed mission and the user's overall visual
confirmation. The saved session does not record a manual return or cancel/restart;
those remain automated evidence from the bounded fault/recovery checks above.
It does not establish arbitrary-route, hardware or exact author-runtime parity.
The closure audit finds **no remaining robot/navigation processes in Ubuntu**;
there was nothing left to terminate. The VM desktop is left running.

## Offline geometry group — November 13, 2026

`osmag_rolling_raster.py` reads OSM area and passage geometry using the accepted
polyconic map frame. Parent floor/container polygons do not become free space.
Leaf geometry supplies free space, area boundaries supply barriers, and named
passages open those barriers. Polygon holes and outside geometry stay nonfree.
Conservative cell coverage blocks boundary-touching cells; each window also has
an occupied one-cell border. Existing mission constraints can close a passage
on that same bounded grid, verified by a synthetic integration test.

The renderer does **not** read the full-floor PGM. Its fixed grid is 128 × 128
at the accepted 0.0586439513 m resolution, approximately 7.51 × 7.51 m. This
deliberately small offline fixture exercises shifts; it is not the paper's
50 × 50 m configuration and is not the 6 × 6 m controller costmap. Origin shifts
use integer indices on one common lattice. Overlapping interiors retain the
same values; moving safety borders deliberately differ. Snapshots carry source,
origin, resolution, revision, timestamp and content hashes.

## Measured checks

The [retained audit](../../../../simulation/gcu_ground_floor/phase7_diagnostics/offline_osm_windows_04/report.json)
compares the candidate against the accepted CAD-derived PGM **for evaluation
only**. All 844,000 reference cells are covered by 56 overlapping interior windows;
none is marked free where the reference is occupied or unknown. The generator is
conservative, not pixel-identical: the three saved demonstration windows block
557, 624 and 810 cells that the reference marks free, including window borders.
Over-conservatism can restrict paths and must still be checked in native planning.

The four retained fixtures cover adjacent sectors, corridor-to-room, room-to-room
and a physical doorway. Their paths are split into pieces at most 0.25 m long;
each piece passes the existing swept-path checker with the unchanged **0.20 m**
padded radius on its centered raster window. There are 76 pieces total. This is
offline geometry evidence, not evidence of actual motion or native consumption.

Raw occupancy storage is **16,384 bytes per window**. Full-floor sweep render
times in the retained local audit have p50 about 4.43 ms and p95 about 7.00 ms,
excluding approximately 61 ms vector preprocessing. These are local measurements,
not a VM/Pi benchmark or proof of online 10 Hz operation. Vector geometry remains
resident and scales with the building. Existing route preprocessing and AMCL
still use full-floor inputs; whole-stack memory savings are not established.

## Reproduction

From the repository root, choose a new output directory:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python -m unittest discover -s tests
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/check_gcu_rolling_raster.py --output /tmp/gcu_phase7_windows_replay
```

The audit saves three candidate/reference arrays, conservative/error counts,
full-floor coverage and route-clearance results. Read `offline_geometry_gate_passed`
for the combined gate. `native_motion_admitted` intentionally remains false.
The retained evidence contains a frozen source snapshot and SHA-256 manifest.
Three superseded exploratory outputs were removed; only the completed audit remains.

## Native plan-only group — November 13, 2026

The isolated Ubuntu workspace `/home/ubuntu/edgeosmag_phase7_native_ws_06`
contains OSM geometry and a frozen graph route, but no full-floor PGM. Both native
costmaps subscribe to `/osmag/planning_window`. Five source/constraint revisions
cover three different origins, closing passage `-93`, then reopening it. All
four open cases produce native A* plans with the unchanged 0.20 m padded-path
check; the closed case rejects proxy admission and returns an empty aborted
native plan. The global planning costmap remains **128 × 128** cells throughout.
Runtime parameter and graph checks find **zero `/map` subscribers** in this
isolated fixture. AMCL is not running in this no-motion test; it remains a
full-map dependency in the live setup below.

`osmag_window_goal.py` clips the *first* route exit from a padded window and does
not skip to a later re-entry. Targets inside the usable window retain their final
heading. Blocked/unknown prefixes are rejected. `WindowAdmission` requires matching
physical content, window/constraint revisions, exact global-grid geometry, both
fresh consumed costmaps and a fresh source-publication heartbeat. Fresh costmap
publication alone cannot conceal a stale raster source. These checks produce
admission receipts, not wheel commands or navigation dispatch.

The native fixture uses declared **synthetic TF poses** to shift the window.
There is no robot, simulator, controller or `NavigateToPose` execution. A second
planner owns the 6 m rolling, local-style costmap and its lifecycle/thread; this
proves native costmap consumption, not the real controller's moving behavior.
Physical handoff, source-loss cancellation/rest and overlap/update sequencing
are checked separately in the live integration group below. Native costmaps do not echo our source
hashes: observed geometry/cells and freshness are evidence, not an atomic
multi-topic revision acknowledgement.

The retained final trial 06 takes about **2.03 wall seconds**, passes independent
raw-grid/path replay, receives zero velocity commands and observes no velocity
publishers. Both planner/costmap owners reach `unconfigured`; all three processes
exit 0. The post-run isolated-domain audit is empty.

Earlier trials remain in the [native evidence folder](../../../../simulation/gcu_ground_floor/phase7_diagnostics/native_windows/README.md):
01 failed an unavailable Humble Python helper import; 02 passed plan checks but
logged invalid lifecycle teardown; 03 crashed the standalone costmap after
explicit cleanup. Trials 04–06 use planner-owned costmaps, with progressively
stronger admission/source-freshness checks. The last passing run is selected for
acceptance; the earlier failures are not relabelled. The owner change is a tested
harness adaptation, not a diagnosis or general repair of the standalone crash.

The installed [Humble StaticLayer source](https://github.com/ros-navigation/navigation2/blob/1.1.19/nav2_costmap_2d/plugins/static_layer.cpp)
supports resizing a non-rolling master when a new full map changes its origin;
the retained native observations verify that behavior for this fixture.

Replay on the Mac from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_rolling_native.py --evidence simulation/gcu_ground_floor/phase7_diagnostics/native_windows
```

To repeat the no-motion native run in Ubuntu, use a fresh output directory and
no other domain-197 session:

```bash
source /opt/ros/humble/setup.bash
python3 /home/ubuntu/edgeosmag_phase7_native_ws_06/run_gcu_rolling_native.py --workspace /home/ubuntu/edgeosmag_phase7_native_ws_06 --output /home/ubuntu/edgeosmag_phase7_native_ws_06/results/my_native_review_01
```

This launches only plan/costmap nodes. The runtime runner bounds the checker to
180 seconds and owns teardown. This is not the Phase 06 visual-review command.

## Live integration and bounded motion — November 13, 2026

The live manager now supplies `/osmag/planning_window` to the static layers of
**both actual Nav2 costmaps**. The global planning grid stays 128 × 128; the
controller retains its 102 × 102-cell, approximately 6 × 6 m rolling costmap.
AMCL continues to use `/map`. Ground truth evaluates motion independently and
never supplies the manager's pose or window decisions.

The original osmAG mission, passage sequence and crossing monitor remain intact.
A target outside the usable window becomes a padded route-prefix proxy. Reaching
a proxy does not complete the original passage or final destination. New windows
trigger native preflight before proxy replacement. Ordinary passage handoff still
requires the original target and observed crossing progress.

Windows shift after approximately **0.65 m** of localized travel. Decisions run
at most **25 Hz of wall time**. The last source window and constraint mask are
republished at **10 Hz of simulation time**, while both native costmaps update
and publish at 5 Hz. Heartbeat publication is not raster regeneration: this is
an adaptation of the paper's rolling-map idea, not a reproduction of its
50 × 50 m raster regenerated at 10 Hz.

Before an asynchronous update, the manager requires recent native acknowledgements
and checks the actually followed remaining controller path in both old and new
windows at the unchanged **0.20 m padded radius**. The old lease expires after
0.5 simulated seconds; the new source becomes usable only after native consumption
and feasibility checks. Observed cells/geometry and freshness support this gate;
Nav2 does not provide an atomic multi-topic source-revision acknowledgement.
Source heartbeat expiry cannot be hidden by fresh native costmap messages.

### Selected motion evidence

All selected runs use actual Gazebo physics, measured IMU yaw-rate fusion, wheel
translation and a 0.10 m/s speed cap. The independent scorer checks topology,
body sweep, localization coverage, ownership, arrival/rest and all 13 collision
sensor publishers. Successful runs have five native process exits of 0 and
separate truth-bridge and simulator shutdown checks.

| Scenario | Selected archive | Result |
| --- | --- | --- |
| Forward 4.4 m crossing | `rolling_motion/attempt03.tar.gz` | 7 windows, 2 proxy sends; 0.0973 m / 1.112° final error; no handoff rest; 129.90 wall seconds |
| Reverse crossing, initial passage target outside window | `reverse_boundary/attempt02.tar.gz` | 7 windows, 1 proxy; 0.1178 m / 0.966° final error; proxy on original passage segment; 123.43 wall seconds |
| Unseen physical box on the route | `obstacle/attempt02.tar.gz` | 8 windows, 3 proxies; 0.1231 m / 0.988° final error; body sweep clears the complete physical box; 204.08 wall seconds |
| Source interruption | `source_loss/attempt01.tar.gz` | Source expiry cancels navigation; physical rest confirmed 0.763 simulated seconds after freezing; held at rest with no new send |
| Same-session source restoration and return | `source_loss/attempt05.tar.gz` | Rest 0.785 simulated seconds after source freeze; guarded return error 0.1624 m / 2.264°; both legs pass independent replay; 135.04 wall seconds |

Archives and independent replay outputs live in
[phase7_diagnostics](../../../../simulation/gcu_ground_floor/phase7_diagnostics/README.md).
They freeze the sources actually executed; earlier runs need not match the latest
root implementation. Read each selected verification file for exact metrics.
These short crossings do not establish arbitrary-building or long-route reliability.

The selected forward trial's seven window generations have a **7.51 ms median**
raster time (10.17 ms maximum), plus **12.63 ms median** mask/overlap work
(20.34 ms maximum), measured in wall time on UTM. Publication through native
consumption and sequential preflight commit takes 1.363 simulated seconds at
startup, before dispatch. Its six moving updates take at most **0.284 simulated
seconds**. These intervals measure different work in different clocks; they do
not establish continuous 10 Hz raster regeneration. Raw values are retained in
`live_window_timing.json` and each window report.

### Retained failures and fixes

- Forward attempt 01 lost native readiness during a window change. The manager
  now begins replacement with fresh native receipts, leaving 0.35 s of its
  unchanged 0.5 s lease available for overlap. Failed arrays and first-fault
  information are retained; the failure is not counted as a successful mission.
- Obstacle attempt 01 hit the existing clock-stall watchdog and stopped. Contact
  callbacks had triggered 2,688 TF queries in one second. Window decisions now
  have a 25 Hz wall-time cap, covered by a contact-burst regression. Attempt 02
  passes with a separate clock observer. The original stall's root cause is
  not proven, and the clock watchdog was not relaxed.
- Source-loss/recovery attempt 02 stopped safely on loss, then failed to dispatch
  its return because preflight used the previous mission's segment index.
  Preflight now scopes that index to the same mission ID and route revision;
  the regression checks a new mission starts preflight at segment zero.
- Source-loss/recovery attempts 03–04 retained the safe stop but encountered stale
  TF during restored-mission setup/preflight. Setup and preflight now wait up to
  two wall seconds for a genuinely fresh pose; the 0.15 s TF age limit is unchanged.
- The occasional pre-existing Python teardown `Destroyable` warning remains
  visible in raw checker logs. Passing process/shutdown checks do not constitute
  a general repair of that warning.

### Matched static-map comparison

The [comparison](../../../../simulation/gcu_ground_floor/phase7_diagnostics/matched_comparison.json)
uses a newly executed **Phase 06-style static control** with the same 4.4 m query,
world, sensors, AMCL/IMU configuration, speed, padding and costmap update rates.
It is not a relabelled historical Phase 06 run. Automated checks compare pinned
runtime inputs and normalized parameters. Rasterization and proxy dispatch differ
by design between the variants.

| Measurement | Static control | Rolling OSM window |
| --- | ---: | ---: |
| Global planning cells | 844,000 | 16,384 |
| Controller grid cells | 10,404 | 10,404 |
| Planner RSS after motion | 116,580 KiB | 27,444 KiB |
| Planner CPU during sampled interval | 7.48 s | 4.36 s |
| Arrival position error | 0.09758 m | 0.09732 m |
| Dispatch to confirmed rest | 47.743 simulated s | 47.453 simulated s |
| Whole-run wall duration | 129.887 s | 129.902 s |

The planning grid has **98.06% fewer cells**. RSS is two per-process samples,
not a peak, PSS or shared-memory-deduplicated measurement. CPU covers dispatch
through post-score rest; it excludes preprocessing and other processes. The
comparison includes no statistical speed claim. AMCL, the map server and Python
route preprocessing/evaluation still retain full-floor data, and vector geometry
scales with the building. This is a useful research measurement of bounded metric
planning, not proof of constant whole-stack RAM or Pi 4B performance.

## Replaying accepted evidence

From the repository root, using the existing Python environment:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python -m unittest discover -s tests
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_rolling_motion.py --evidence simulation/gcu_ground_floor/phase7_diagnostics/rolling_motion --trial 03
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_rolling_motion.py --evidence simulation/gcu_ground_floor/phase7_diagnostics/reverse_boundary --trial 02 --case-prefix reverse_boundary
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_rolling_motion.py --evidence simulation/gcu_ground_floor/phase7_diagnostics/obstacle --trial 02 --case-prefix obstacle
```

Replay source loss/recovery and the independently scored matched comparison:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_rolling_motion.py --evidence simulation/gcu_ground_floor/phase7_diagnostics/source_loss --trial 05 --case-prefix source_loss
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/compare_gcu_rolling_resources.py --rolling simulation/gcu_ground_floor/phase7_diagnostics/rolling_motion/attempt03.tar.gz --baseline simulation/gcu_ground_floor/phase7_diagnostics/static_baseline/attempt01.tar.gz --output /tmp/gcu_phase7_matched_comparison.json
```

The local regression suite passes **169 tests**; the closure rerun is saved in
`tests_closure.log`. The retained
`cleanup_final.json` finds no remaining domain-195/197 test processes or active
Phase 07 units. The fresh review workspace's runtime pins are validated and it
had not been launched at that earlier audit. The November 14
`closure_audit_20261114.json` records the subsequent successful user session and
confirms no robot processes remain.

Use the [final review guide](../plans/07_GCU_FINAL_REVIEW.md) for Ubuntu/Foxglove.
Do not reuse a Phase 06 workspace: its native planner still consumes the full
map. The [plan](../plans/07_GCU_ROLLING_RASTER_AND_BOUNDARY_GOALS.md) includes the
Nav2/controller explanation and architecture diagram. Phase 07 is accepted for the bounded scope above. Phase 08 evaluates
structure-based localization while preserving this tested navigation baseline.

Replay the accepted user mission independently on the Mac:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_rolling_motion.py --evidence simulation/gcu_ground_floor/phase7_diagnostics/manual_review --trial 01 --results-prefix manual_20261114_072600 --report-file user_mission_01.json
```
