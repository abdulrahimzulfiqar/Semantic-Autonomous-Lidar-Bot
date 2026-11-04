# Checkpoint 01: accepted GCU CAD-to-osmAG reconstruction

## Quick summary

- **Goal:** Convert the GCU ground-floor CAD drawing into usable osmAG geometry.
- **Result:** Accepted reconstruction with 34 regions and 35 interfaces; side rooms separated and the central enclosure excluded.
- **Limit:** Geometry acceptance does not establish robot traversability or generalize to every CAD drawing.
- **Next checkpoint:** Adapt the map for ROS, simulation and sensor visualization.

## Outcome

The user accepted the reconstructed ground-floor trial-12 result. The pipeline
retains 34 reconstructed regions and 35 interfaces, separates the side rooms and
excludes the central enclosure while preserving the surrounding corridor.

## Work and evidence

Scale/alpha tuning improved initial room detection. Occupancy-seeded geometry
reconstruction corrected incomplete room polygons and preserved excluded holes;
a correct Voronoi skeleton alone did not guarantee correct final area geometry.
The accepted configuration, before/after images, diagnosis and reproduction steps
are maintained in [GCU_TUNING.md](../osm_cad_pipeline/GCU_TUNING.md) and
[commands_simplified](../osm_cad_pipeline/commands_simplified).

Accepted artifacts:
[`gcu_config_parameters_testing_12`](../osm_cad_pipeline/osmAG-from-cad/runs/local-cad/gcu_config_parameters_testing_12/).
Source implementation history: `1063565`; run/image publication: `037a75d`.

## Limits and handoff

Acceptance applies to this ground-floor drawing. Other floors may require CAD
cleanup and parameter/semantic checks. The reconstructed export contains area
relations and polyline interfaces; adapting it to the existing ROS loader is the
next checkpoint. Geometry acceptance is not proof of robot traversability.

Continue with [Checkpoint 02](02_GCU_SIMULATION_VISUALIZATION.md).
