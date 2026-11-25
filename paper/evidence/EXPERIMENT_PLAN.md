# Experiment framework for the paper

## Quick summary

This framework defines the planned comparison. The navigation-only Pi measurements ran with GCU simulation on the laptop and established the short-corridor baseline. The next experiment adds camera perception. Language-task definition follows. [Phase 09 results](../../docs/research/osm-AG/phases/09_PI4B_NAVIGATION_BASELINE.md)

## First experiment: navigation only

The Pi runs the existing osmAG routing/execution, rolling planning-window generation, AMCL/EKF and Nav2 workload. The laptop's Ubuntu environment runs Gazebo physics/sensors and independent scoring; Foxglove visualizes from the host. Record the bridge, transport and preprocessing placement explicitly. Use direct goals and no LLM or camera-processing workload. Keep the accepted navigation settings fixed until baseline measurements identify a reason to change them.

First establish connectivity and one short working mission. Then prepare a small pilot set covering an open route, a passage/room crossing and an obstacle response, using retained fixtures where possible. Freeze repetitions, rate/time conditions and scoring before collecting the reporting batch. Pilot measurements guide the final protocol and are labelled separately from confirmatory trials.

Measure success/failures, arrival and localization errors, planning/controller latency, input/TF freshness, CPU, memory, temperature/throttling and network overhead. Report both simulated and wall time. This establishes what the existing stack costs on the Pi and how much headroom remains; it does not yet establish a language/perception benefit or physical-robot performance.

## Candidate research question

Can an osmAG-guided navigation system share a Pi 4B with perception and language tasks while preserving declared navigation reliability and timing budgets, and what design choices make the difference?

We still need a focused, literature-supported contribution. A scheduling/resource policy is one possibility, not an implemented result. If measurements show no useful improvement, revise the research question and report what the experiments actually support.

## Planned comparison ladder

| Group | Comparison | What it can tell us |
| --- | --- | --- |
| Navigation baseline | Matched static-map Nav2 configuration versus osmAG-guided bounded planning, where supported | Effect of the declared navigation architecture changes; hold localizer/controller/tasks constant |
| Workload additions | Navigation alone, +camera, +language, +both | Cost and interference of each workload, with its output quality measured |
| Initial language task (proposed in Review 01) | Same room-navigation missions with/without onboard inference; a simple parser versus LLM on a declared command set | Separate compute interference from interpretation quality; keep the camera optional until resource headroom is measured |
| Proposed policy, if developed | Same workloads with a declared simple policy versus proposed resource policy | Whether our policy causes a benefit rather than merely changing model size or input rate |
| Physical transfer | A smaller house map with the same declared stack and a physical robot | Behavior with real sensors and motion; does not establish campus-scale physical performance |

Only include variants that are actually implemented and measured. Treat custom structural localization as optional supplementary work; AMCL remains the production baseline. Add closest relevant literature baselines after the reading stage.

## Before each measurement batch

Fill a protocol in this folder with:

1. **Question and variants:** what changes, what stays fixed, expected effect and what would count against it.
2. **Execution placement:** which processes run on Pi, simulator host or another machine; distinguish local and offboard inference and include network effects.
3. **Configuration:** hardware/RAM, OS/ROS, code revision, model/quantization, cooling/power settings, maps, input rates, route/goal fixtures and initialization.
4. **Metrics and units:** mission successes over all attempts, goal error, path length, wall/sim duration, command/controller latency and missed deadlines, CPU definition, RSS/PSS sampling, temperature/throttling. Add power only if measured with a suitable method. Camera needs task-specific accuracy and throughput; language needs command success, ambiguity handling and response latency.
5. **Trial design:** declare independent mission repetitions, order/seeds, warm-up, timeout, exclusions and stopping rules. Use pilot runs to choose a feasible repeat count and precision target, then freeze confirmatory settings. Do not choose the count or thresholds after inspecting which result wins.
6. **Scoring and evidence:** evaluator-only truth/reference, expected artifacts, derivation scripts, raw timestamps and all failure logs. Separate tuning routes from held-out evaluation routes.
7. **Acceptance criteria:** choose explicit task/resource/timing budgets before confirmatory trials; record why they are appropriate. Do not infer real-world safety from simulation acceptance.

## Timing and interpretation

With Gazebo elsewhere and navigation on a real Pi, report this as a **processor-in-loop simulation**. Slow simulation can give the Pi extra wall time between sensor messages. Measure wall-time input cadence and repeat representative workloads using fixed-rate replay when assessing compute capacity; replay alone does not prove closed-loop navigation reliability.

Report per-trial outcomes and uncertainty appropriate to the metric. Do not treat thousands of correlated scan frames as thousands of independent missions. Distinguish planner-process memory, navigation-stack memory and total Pi memory; a bounded grid does not imply bounded whole-system memory.

## Phase 09 reporting sequence

1. Establish that the full navigation stack completes the 1× Pi-local replay with fresh inputs and emits controller commands into the recording sink. Use this run to diagnose computation and scheduling on the Pi.
2. With a frozen configuration, run three consecutive independent closed-loop trials of the same GCU corridor fixture at the intended operating rate. Record the achieved rate and apply the existing passage, final pose, rest, contact and input checks. Keep failed attempts and their logs.
3. Report wall and simulated durations, sampled CPU and PSS, temperature and throttling, including which Pi processes were sampled. Attribute the results to the tested route and configuration. The earlier slow trials provide functional evidence at their measured rates.

The [Phase 09 plan](../../docs/research/osm-AG/plans/09_PI4B_NAVIGATION_BASELINE.md) records the Pi inventory and explains how the closed-loop setup is split across hosts. Later workload comparisons remain future work.
