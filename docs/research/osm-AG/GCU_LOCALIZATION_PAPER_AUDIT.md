# Localization paper and source compatibility audit

## Quick summary

- **Paper:** The attached nine-page PDF is the published AGLoc article, including a publisher cover page; DOI `10.1109/LRA.2023.3334158`.
- **Compatibility:** The original method uses 64-line 3D data to create a clutter-reduced 2D set. Our one-ring 2D scan cannot reproduce that preprocessing.
- **Source:** The audited osmAG-Nav endpoint at `9f4fa7e6cff57f460b773b7a79f64122658f920b` is a project website, without ROS packages or estimator source. Its link still says “Code (Upon Acceptance)”. This does not prove no other implementation exists elsewhere.
- **Decision:** Preserve native AMCL and evaluate a separate, explicitly labelled 2D structural reference; do not call it author AGLoc/AGLoc++.

## What was read and checked

The audit uses [Robust Lifelong Indoor LiDAR Localization using the Area Graph](https://arxiv.org/abs/2308.05593v3).
Relevant full pages were extracted and visually inspected, including the weight
formula, passage treatment, experiment conditions and comparison discussion.
Source and input fingerprints are recorded in
`simulation/gcu_ground_floor/phase8_diagnostics/source_audit/` from the repository root.

| Paper component | Original method | Current 2D reference |
| --- | --- | --- |
| III-A loading | Floor/region prior, leaf polygons and passages | Same accepted GCU vector geometry; full current floor remains resident |
| III-B clutter removal | Furthest horizontal return per vertical 3D column | Unavailable with one ring; explicitly omitted |
| III-C associations | Ray/polygon intersections; special passage/transparent boundaries | Ray/physical-segment associations; near-passage return may match a closed door; glass semantics omitted |
| III-D initialization | Candidate search around WiFi/barometer prior | Supplied declared pose only; global localization omitted |
| III-E Eq.8 | Asymmetric signed-distance weights, cutoffs -1 m and +3 m | Implemented with those constants |
| III-F Eq.9 | Weighted point-to-line tracking, previous pose prediction | Weighted local optimization plus shared wheel/IMU odometry prediction; adaptation |
| III-G Eq.10 | Downsample dominant line orientation using corridorness | Deterministic inverse-rate weighting; approximation, not exact sampling |
| osmAG-Nav §6 AGLoc++ | Enhanced rescue, filtering, corridor treatment and adaptive fusion | Not implemented or claimed by the current reference |

The current prototype preserves odometry prediction in geometrically unobservable
directions rather than claiming ICP determines a featureless corridor's lengthwise
position. It reports weak/rejected/lost frames and publishes no TF or commands.
There is no global rescue or authoritative Nav2 adapter yet.

## Experimental cautions that matter to the paper

The original study uses 3D point clouds, WiFi/barometer search priors and different
odometry conditions across its baselines. It explicitly reports detailed-grid
AMCL performing slightly better in some cases, and Area-Graph-grid AMCL performing
worse than AGLoc. Those results do not establish a winner on our hardware.

Our A baseline uses the accepted CAD-derived grid, not the original study's
freshly mapped clutter-rich occupancy map. B uses geometry from the same CAD
pipeline. The initial simulation is favorable to structural matching: map and
Gazebo architecture share a construction source, the start is known, and odometry
is very accurate. Keep an odometry-only diagnostic to show how much matching
contributes. Millimetre-level simulator errors must not be described as expected
physical-robot accuracy.

The prototype's Python/vectorized implementation, preprocessing, all-floor ray
association and per-scan update policy differ from native C++ AMCL. Native AMCL
updates at configured motion thresholds and uses 120 selected beams; the prototype
processes every scan's valid returns. Live native resource samples and standalone
replay samples have different co-resident processes and scheduling. They are
profiling evidence, not a controlled RAM/CPU win or Pi result.

## Primary references

- [Original article](https://arxiv.org/abs/2308.05593), especially III-B–G and IV-C.
- [osmAG-Nav](https://arxiv.org/html/2603.28271v1#S6), enhanced localization pipeline.
- [Author project page](https://jiajiezhang7.github.io/osmAG-Nav/) and [audited repository](https://github.com/jiajiezhang7/osmAG-Nav).
- [Nav2 AMCL source](https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl).
