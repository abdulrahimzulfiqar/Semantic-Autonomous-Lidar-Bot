# One live cancel-topic test near a passage handoff

`attempt01.tar.gz` contains the actual executed sources/assets, raw
`results/cancel_01/` report, native logs and physical/logical constraint arrays.
The query starts at x = -1.5 m so the robot moves meaningfully before cancellation.
The evaluator publishes `/osmag/mission_cancel` from fresh localized distance,
not ground truth. The native action returns cancelled; measured rest begins
0.283 simulated seconds after the request, followed by one second of confirmation
and 1.20 further seconds of recorded rest. No next segment is sent. No passage
crossing or final arrival is claimed.

All five native parents and audited children exit cleanly, the simulator exits,
and `cleanup.json` records zero isolated test processes/units. The known Python
`Destroyable` teardown warning remains preserved. The state-model component's
historical mock label does not describe this enclosing native simulation trial.

`manifest.json` binds runtime sources and artifacts. `source_snapshot.tar.gz`
retains actual prepared code, including the IMU launch that differs from the root
launch. `admission_source.tar.gz` freezes the preparer/admission source and fixture.
`verification_tools.tar.gz` pins both the default verifier and cancellation scorer.
The 124-test log includes six new cancellation regressions; the nominal replay
control confirms compatibility of the default verification interface. These
controls do not substitute for the remaining live fault and comparison gates.

Independently replay from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python scripts/verify_gcu_cancel.py \
  --evidence simulation/gcu_ground_floor/phase6_diagnostics/06_faults_and_comparison/cancel --trial 01
```

The verifier checks frozen hashes, native teardown, rebuilt mask/reference and
preflight geometry, sampled body clearance, ownership, contacts, native action
events, motion before cancel, stop latency, continued rest and lack of subsequent
dispatch. It reports the archived launch difference explicitly and writes a
derived `historical_verification_replay.json`, outside the artifact hash set.

See [the measured phase record](../../../../../docs/research/osm-AG/phases/06D_GCU_NOMINAL_MISSIONS.md)
for exact commands, measurements and remaining gates. This is one bounded case,
not general cancellation robustness or final Foxglove acceptance.
