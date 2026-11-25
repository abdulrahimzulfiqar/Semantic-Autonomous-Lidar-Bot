# Phase 09 Raspberry Pi 4B navigation baseline

## Quick summary

- **Completed November 25, 2026** for the short GCU ground-floor corridor in processor-in-loop simulation. The Pi ran navigation and the Ubuntu VM ran Gazebo and the independent scorer.
- **Results:** Three consecutive scored passes at 0.669–0.717× measured simulation rate. Every run crossed the required passage, matched every source LiDAR scan and recorded zero contacts.
- **Pi load:** 2.19–2.21 average CPU cores and about 370 MiB peak sampled concurrent proportional set size (PSS) during motion.
- **Next:** Phase 10 adds camera workloads against this measured baseline. The full-rate moving and physical-robot cases remain open research measurements.

## Configuration and acceptance

Trials 76–78 used the same short corridor fixture, AMCL/EKF, osmAG route,
rolling raster, and Nav2 planner and controller. The 4 GB Pi 4B used the
`pixel` hotspot on 2.4 GHz (2,437 MHz) with Wi-Fi power saving off. The Pi's
global costmap updated and published at 10 Hz. The local costmap remained at
5 Hz with the same footprint, navigation limits and sensor topics. The Gazebo
target was 1.0×. Achieved rate is the source scan timestamp span divided by
the corresponding Ubuntu receipt wall-time span. The 150 ms pose freshness
limit and 500 ms window acknowledgement lease were retained.

The independent scorer checked ordered passage completion, a valid swept
footprint, final position within 0.20 m and heading within 10°, localization
coverage and error, command bounds, contacts and a final rest period. These
tests establish repeatability for the measured route and rates.
[Independent scorer](../../../../experiments/pi_baseline/score.py) ·
[Stage and runner instructions](../../../../experiments/pi_baseline/README.md)

## Results

| Trial | Measured simulator rate | Endpoint error | AMCL p95 position error | Source scans matched | Window changes, simulated ms | Mean Pi CPU cores | Peak sampled Pi PSS |
| --- | ---: | ---: | ---: | ---: | --- | ---: | ---: |
| [76](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_76/independent_score.json) | 0.717× | 0.072 m | 0.046 m | 188/188 | 297, 344 | 2.21 | 369.8 MiB |
| [77](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_77/independent_score.json) | 0.669× | 0.113 m | 0.052 m | 180/180 | 307, 279 | 2.19 | 369.8 MiB |
| [78](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_78/independent_score.json) | 0.709× | 0.070 m | 0.051 m | 190/190 | 347, 349 | 2.20 | 369.5 MiB |

Window change time is measured from the publication event to the final commit
event, after native grid observation and plan validation. The Pi workload
intervals from first dispatch to final Nav2 success lasted 24.5–25.2 wall
seconds, including both sensor relays. Cold graph preprocessing took 54.7–58.8
wall seconds and is measured separately. Maximum board temperature across
the runs was 59.9°C. The firmware samples reported no active undervoltage or
throttling.
[Trial 76 resources](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_76/resource_interval_summary.json) ·
[Trial 77 resources](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_77/resource_interval_summary.json) ·
[Trial 78 resources](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_78/resource_interval_summary.json)

In the first 10.99 wall seconds after dispatch, the sampled workload used
2.22–2.24 CPU cores across the 10 Hz runs and 2.24 in the 5 Hz pilot. The
planner server used 11.1–12.5% of one core across the 10 Hz runs and 11.3% in
the 5 Hz pilot. The 5 Hz pilot cancelled during its first window change. These
short sampled intervals give a preliminary cost comparison with a single
5 Hz attempt. The two stationary 1× replays remain the evidence for the
test stack's performance at a fixed real-time input rate.
[5 Hz pilot record](09E_PI4B_FULL_STACK_REPLAY.md#pixel-hotspot-follow-up) ·
[1× replay evidence](09E_PI4B_FULL_STACK_REPLAY.md)

The preparer flag `--global-costmap-hz 10` generates the trial configuration
without changing the defaults for other stages. The retained source archives,
role manifests, window observations, firmware logs and independent scores
allow trial-by-trial verification. The local regression suite passed 223 tests,
and the final process audit found no Phase 09 ROS processes on either host.
[Phase 9 diagnostics](../../../../simulation/gcu_ground_floor/phase9_diagnostics/)
