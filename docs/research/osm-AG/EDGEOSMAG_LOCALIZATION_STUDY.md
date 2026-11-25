# EdgeOsmAG localization comparison: research protocol

## Quick summary

- **Question:** Does structural localization provide enough robustness or memory benefit to justify its cost on constrained hardware?
- **Disposition:** Preliminary Phase 08 study closed November 14, 2026. AMCL remains in production; the structural reference and evidence stay separate.
- **Claim today:** A reproducible comparison setup and preliminary simulation tracking results, not a new localization algorithm or a demonstrated Pi advantage.
- **Priority:** Profile actual Pi navigation, then add camera/language workloads. Further structural-localization integration and comparisons are deferred.

## Decision and scope for the paper

The user chose to retain AMCL and defer further custom AGLoc implementation.
Keep the localization experiment as preliminary/optional supporting material;
do not present it as a completed author-method comparison or proof that AGLoc
cannot help on a Pi. No structural CPU/RAM advantage was established by these
uncontrolled resource samples.

The main paper direction is **resource-budgeted osmAG-guided navigation with
perception and language on constrained hardware**. Start from the working
navigation baseline and measure the additions separately: navigation alone,
+camera, +language, and +both. Assess navigation reliability, task usefulness,
latency, CPU/RAM, deadline behavior and thermal/energy effects where measured.
Local versus offboard language inference must be explicit. Merely adding these
modules is not, by itself, evidence of novelty or a publication guarantee.

The hypotheses below are retained for a possible reopened localization study.
They are not an active commitment to complete structural integration now.

## Optional localization research contribution

Evaluate the localization design tradeoff in an osmAG-guided edge navigation
system under a restricted sensor and compute budget. The contribution can be an
engineering/evaluation result: where compact structural maps help, what is lost
with inexpensive 2D sensing, and how localization competes with camera/language
workloads. Merely running an existing estimator is not algorithmic novelty.

Use three declared hypotheses:

1. Structural geometry may reduce sensitivity to moved objects relative to the
   same CAD-derived occupancy-map baseline.
2. Removing a dense localization grid may reduce map storage, but implementation
   overhead, ray association and process memory may erase that benefit.
3. Sharing constrained CPU/RAM with perception/language may change which backend
   offers the best reliability and deadline behavior, even if accuracy is similar.

Hypotheses may be rejected. Report failures, distributions and useful negative
results; select the deployed estimator after measurement.

## Measurements and comparisons

- Main variants: accepted native AMCL + wheel/IMU EKF, and a separately versioned
  structural 2D reference with the same odometry. Add actual author AGLoc/AGLoc++
  as another explicitly identified variant only when compatible source and
  sensor conditions are available.
- Diagnostic: odometry only, using the same supplied start. This exposes whether
  a simple trajectory really tests scan-to-map correction.
- Cases: clean route, physical occluder, corridor ambiguity, changed furniture,
  wrong initial pose, dropout and controlled recovery. Separate development,
  holdout and repeated trials; pair seeds and input streams.
- Accuracy: no truth-based trajectory alignment; RMSE, p95/max position/yaw,
  out-of-tolerance/lost-frame fractions, coverage, recovery time and end-to-end
  contacts/arrival/failures. Show every condition, not only favorable cases.
- Resources: map bytes and preprocessing, component/whole-stack RSS/PSS, CPU
  seconds, latency/deadline misses and energy/thermal behavior where measured.
  Match replay rate, hardware and co-resident workload for controlled comparisons.
- Hardware: Pi 4B runs the estimator/navigation workload; simulation and truth
  evaluation stay on another machine. Repeat with camera and language work, and
  report offloading/model quantization explicitly.

## Reporting boundaries

The initial replay follows an AMCL-controlled trajectory and supplies a known
start. It cannot establish structural closed-loop success, kidnapping recovery
or hardware efficiency. Ideal Gazebo architecture closely matches its source
map; test map inaccuracies and real robot data before generalizing. The current
2D reference omits the original 3D vertical clutter filter and is not AGLoc++.

The [Phase 08 plan](plans/08_GCU_LOCALIZATION_COMPARISON.md) records one completed
group and two deferred groups; the preliminary study is closed. The [paper audit](GCU_LOCALIZATION_PAPER_AUDIT.md) lists
method differences so the final paper can describe them accurately.
