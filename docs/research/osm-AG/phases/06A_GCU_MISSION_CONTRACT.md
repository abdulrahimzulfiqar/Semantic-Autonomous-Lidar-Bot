# Phase 06A — Offline mission and crossing contract

## Quick summary

- **Goal:** Establish passage goals, ordered crossing confirmation and asynchronous mission-state behavior before connecting them to ROS/Nav2.
- **Result:** 32 new targeted tests and all 79 unit tests passed. Eight offline GCU cases passed in about 11.4 seconds.
- **Scope:** Pure geometry and fabricated action/pose events. No ROS, UTM, Gazebo, costmap integration or physical motion was tested.
- **Next:** 06B1 adds a ROS adapter with a mock navigator; constraint integration and actual motion remain later checkpoints. Full Phase 06 is incomplete.

Date: 2026-11-06. Parent checkpoint: accepted [Phase 05](05_GCU_OSMAG_ROUTE_PREVIEW_AND_RELIABLE_HANDOFF.md), commit `253ce3c`. Full plan and short-batch sequence: [Phase 06](../plans/06_GCU_OSMAG_SEGMENTED_NAVIGATION.md).

## What is implemented

[Mission geometry](../../../../scripts/osmag_mission_contract.py) queries the planner afresh from a timestamped start, freezes route/map/settings identities, and generates intermediate passage goals plus the final requested pose. Refined targets use the actual crossing locations rather than the broad-interface midpoint. A separately labelled `paper_center` comparison reconstructs validated local connections through passage centers. Passage headings point toward the final destination; the terminal goal retains its requested yaw.

Targets on shared boundaries are validated against the incident-area corridor and unchanged footprint checker. They do not go through the final-goal helper's strict single-area containment test. Incorrect incident areas, corrupted crossing provenance and physically blocked routes are rejected.

The crossing monitor requires an observed crossing of the expected interface in the correct direction, followed by three fresh observations at least 0.06 m inside the destination area. Boundary jitter before confirmation resets the evidence; it does not count an extra crossing. Stale/duplicate timestamps, gaps over 0.15 s, observation jumps over 0.25 m, wrong interfaces and reversals after confirmation produce explicit errors. These are initial development settings, not author-reported values.

[The pure mission-state model](../../../../scripts/osmag_mission_state.py) produces send/cancel **data intents** for a future adapter. It separates dispatched goals from confirmed crossings, permits only one goal ahead of an unconfirmed transition, and handles cancellation during pending dispatch and delayed acknowledgements. The old goal can terminate before the new acceptance callback; that supersession must not fail the new segment. Final action success still requires all crossings, valid final pose and externally verified rest before mission success.

No ROS action client, velocity publisher, execution launch or VM workspace was added in this batch. The accepted Phase 05 implementation and Ubuntu workspaces remain unchanged. Phase 06A work is on `feat/gcu-phase06-segmented-navigation`; it has not been committed, pushed or merged as a completed phase.

## Evidence and practical limits

[Checkpoint metadata](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06a_contract/checkpoint.json) pins source and artifact hashes and retains test logs, including the first failed test attempt. That failure was an exact floating-point assertion differing by about 1.6e-13 m; the test now uses a 1e-9 m tolerance. No runtime behavior was altered to conceal it.

The [GCU report](../../../../simulation/gcu_ground_floor/phase6_diagnostics/06a_contract/results.json) covers same-sector, short adjacent-sector, diagonal, enclosed-sector, closed virtual-interface alternate and passage-center comparison routes, plus courtyard and occupied-goal rejection. Six valid reference geometries were resampled as synthetic pose observations and replayed with fabricated action results. That checks the internal contract; it does not establish actual Nav2 handoff, localization, motion speed, stop behavior or trajectory tracking.

The closed `-93` alternate remains a logical center-path test. It does not certify that a physical keepout mask can safely be activated at the original near-seam start, whose footprint straddles that interface. Constraint/initial-footprint conflicts must be checked in 06C1; they cannot be waived by this offline pass.

The initial confirmation policy rejects a terminal goal within 0.06 m of its area's boundary after a passage transition, since this model cannot certify its destination-band confirmation. Very thin sectors and closely spaced portals may also require a separately validated policy before execution. The state model cannot detect a sensor stream that stops calling it, or independently measure physical rest; those are mandatory runtime adapter/watchdog/evaluator responsibilities. A reported ACK timeout leaves stop status unknown and blocks a new mission while requests remain unresolved.

## Reproduce this checkpoint on macOS

From the repository root, use the existing pipeline virtual environment. These commands do not require Ubuntu to be running:

```bash
PYTHONDONTWRITEBYTECODE=1 \
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  -m unittest discover -s tests -p 'test_osmag_mission*.py' -v

PYTHONDONTWRITEBYTECODE=1 \
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/check_gcu_mission_contract.py --output /tmp/gcu_phase6a_repeat_01
```

The output directory must be new. Ubuntu can use `python3` with the documented NumPy/SciPy/OpenCV/Pillow/PyYAML/Shapely/pyproj dependencies. No new Foxglove test is needed for 06A; the current Phase 05 commands still run only the accepted preview.

## Resume point

Next is **06B1 only**: build the ROS-facing adapter and test against an isolated mock `NavigateToPose` server with synthetic TF. Freeze message/action ownership and prove late-acceptance/cancellation behavior through actual ROS interfaces. Stop and record that checkpoint before starting constraint integration or Gazebo motion. Keep the full Phase 06 completion record separate until real constrained missions and user acceptance pass.
