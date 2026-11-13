# Upcoming room-exit closure: one independently replayed native trial

`attempt01.tar.gz` retains the actual runtime sources/assets and complete
`results/closure_01/` evidence. The robot starts inside room 33. While it is
moving, the harness publishes revision 1 closing its only exit, `-95`, based on
fresh localized distance. The executor cancels its action, confirms rest from
odometry and reports `no_route` on `/osmag/closure_status`. The harness then
publishes mask revision 2, verifies native consumption and requests a native
plan to the original goal. Nav2 aborts with an empty path. Ground truth does
not trigger the closure, choose the new route or gate revised mask publication.

Independent replay measures **0.297 simulated seconds** from request to rest
onset, with native cancellation, no later segment, continued rest and zero
observed contacts. Captured global/local grids have zero missing required cells
among 834,090 / 4,956 checked cells. Both the graph query and native query have
no route. The mission remains failed/stopped for `passage_closed`; the fault
test passes because this is the intended outcome, not arrival.

`constraint.npz` and `closure_constraint.npz` inside the raw trial preserve the
initial and revised masks. Raw published and native service costmaps and recent
scans are retained in the same archive. `source_snapshot.tar.gz` freezes actual
executed code, including the prepared IMU launch. `admission_source.tar.gz`
freezes the source used to prepare the freshly admitted final fixture.
`verification_tools.tar.gz` pins independent tools and closure/stop helpers.
`manifest.json` binds all sources/artifacts. `verification.json` is the saved
independent acceptance result; replay writes a separate derived
`historical_verification_replay.json` because the actual IMU launch is archived.

All five native parents and audited children exit cleanly, the simulator exits,
and `cleanup.json` contains no leftover isolated processes/units. The known
Python `Destroyable` teardown warning is retained. `local_tests129.log` is the
final local suite, and `cancel_replay_control.log` confirms compatibility with
the separately frozen cancellation evidence. `closure_stop.png` and
`plot_stop.py` preserve the reference/trajectory/velocity figure and its source.

From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_closure.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06_faults_and_comparison/closure --trial 01
```

See [the phase record](../../../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for measurements, Ubuntu reproduction and remaining gates. This is one logical
closure with no route; physical-obstacle handling, resumed alternative routes
and final interactive acceptance are not established by this case.
