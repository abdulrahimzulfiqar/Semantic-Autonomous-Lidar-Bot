# Pi full-stack replay and power fault

## Quick summary

- The full-stack replay now feeds recorded LiDAR, IMU and odometry to navigation on the Pi through loopback.
- A power bank cleared the undervoltage. The Pi now dispatches and produces controller output in the 1× loopback replay.
- Replays 19 and 20 each completed the eight-second 1× action interval after scheduling and costmap validation changes.
- Early hotspot trials passed individually and recorded input freshness failures. Trials 76–78 later established the short-corridor baseline with 10 Hz global costmap updates. [Phase 09 results](09_PI4B_NAVIGATION_BASELINE.md)

## November 20, 2026 attempts

| Attempt | Outcome |
| --- | --- |
| Higher-rate pilot 17 | The target physics factor was 1.0. The Pi saw scans and the complete transform chain. The AMCL lifecycle service logged a timed-out response, and the readiness client waited on that response until its overall deadline. The mission actor never started. |
| Pi-local stack replay 01 | AMCL/EKF, rolling costmaps, planner, controller and navigator started on the Pi. The actor passed native preflight and rejected a 0.472 s old scan at admission. No navigation goal or nonzero velocity command was sent. |

The readiness client now discards unanswered lifecycle requests after 1.5 wall
seconds and retries within the existing overall deadline. An optional dedicated
TF callback thread serves the existing mission buffer. Its effect on moving
navigation still needs validation. The existing TF, scan, pose-gap and controller
limits determine admission and runtime behavior.

## Replay input and isolation

An Ubuntu subscriber recorded the source LiDAR scans, raw IMU, wheel odometry
and static robot transforms from pilot 17. The selected 5-second clip contains
50 scans, 500 IMU messages and 250 odometry messages. Independent simulator
truth verifies that the robot stayed stationary throughout the clip. The Pi
receives sensor messages and robot extrinsics. The replay publisher shifts
each repeat by a fixed offset onto a continuously advancing 1× ROS clock and
preserves the recorded inter-message intervals.

The launch uses the same Pi navigation processes as the live experiment.
ROS uses localhost and Nav2 publishes velocity commands on a dedicated topic
connected to a recording sink. The diagnostic ends after an eight-second
action interval and verifies cancellation if the action remains healthy.
Replay exercises the navigation workload with a stationary pose, so the
independent moving mission score still comes from Gazebo.

## Power fault and evidence

During the local replay, at 17:01 PKT, firmware reported `0x50005` and the board
was at 50.1°C. The firmware measured an ARM clock of 600,169,920 Hz at that
instant. A simultaneous kernel frequency field read 1,800,000 kHz. The firmware
flags report active and historical undervoltage and throttling. A single sample
does not establish the time spent throttled or identify the failing part of the
power path. An idle check at 17:05 PKT still reported `0x50005` at 44.8°C.
[Raspberry Pi voltage monitoring](https://www.raspberrypi.com/documentation/computers/config_txt.html#monitoring-voltage)

Cold route preprocessing took 178.4 wall seconds in this attempt. The replay
publisher logged as much as 260 ms of scan publication lateness. Treat these as
power fault observations. The physical setup previously recorded a 30 W MacBook
adapter and USB-C cable. The specified input for the Pi 4 is a good-quality
5 V, 3 A USB-C supply. [Raspberry Pi 4 Model B datasheet](https://pip-assets.raspberrypi.com/categories/545-raspberry-pi-4-model-b/documents/RP-008341-DS-1-raspberry-pi-4-datasheet.pdf)

The initial stop signal interrupted cleanup because the owner had already
entered its shutdown handler. Scoped cleanup then stopped the remaining
processes and the audit found both hosts clear. The supervisor now finishes
bounded cleanup even if another stop signal arrives. Future Pi resource sampling
also records the firmware throttling flags alongside CPU, PSS and temperature.
The repository's 213 tests pass. The revised replay stage is built and its
manifest and Python sources have been checked. Its hardware run awaits the
power change.

- [Raw capture and startup-failure archive](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_17/)
- [Replay result and supplementary logs](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_01/)
- [Replay power fault record](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_01/power_fault.json)
- [Pi and Ubuntu process audit](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_process_audit.json)

## November 21 power bank retest

The Pi held 1.8 GHz for a 25-second CPU load test. All 24 readings reported
`throttled=0x0`, and peak sampled temperature was 64.2°C. Wi-Fi power saving
was restored to off for the live trials after reboot.
[Power bank qualification](../../../../simulation/gcu_ground_floor/phase9_diagnostics/power_bank_qualification_20261121.json)

Five traced Pi-local replay attempts reached action dispatch with clear
firmware readings. One used the previous stage, three shared an unchanged
message-reuse variant and one added a dedicated sensor thread and serialized
heartbeat cache. All continued with the same recorded clip and the same
freshness thresholds.

| Replay | Time from dispatch to failure (simulated s) | Guard |
| --- | ---: | --- |
| 02 | 0.390 | Pose observation gap |
| 03 | 4.820 | IMU freshness |
| 04 | 0.312 | Pose observation gap |
| 05 | 5.878 | Pose observation gap |
| 09 | 0.419 | Pose observation gap |
| 12 | 2.699 | Pose observation gap with tracing off |

Replay 03 showed the independent receiver processing IMU readings after the
mission's IMU callback had stopped updating. Replay 09 served IMU, scan and TF
on a dedicated executor. Its mission timer still ran for 187 ms on one
invocation before the following sample exceeded the pose gap cap.
The traced loopback failures establish a local deadline problem independent of
the two-host network. Replay 12 removed the detailed callback trace and
arrival probe and produced 26 nonzero controller commands before the pose gap.
The stationary input publisher runs on the Pi in every replay.

The heartbeat now caches a window's occupancy payload and verifies its use of
the installed ROS serializer against a second header timestamp. Native
costmap validation checks new grids and preserves the original validity period
for previously verified grids. A bounded admission retry now waits for actual
fresh scan and TF input before the model owns an action. The pilot harness
keeps the running watchdog and clearance gates.

| Pilot | Gazebo target | Achieved source rate over run | Outcome |
| --- | ---: | ---: | --- |
| 22 | 1.0× | — | Simulator startup aborted by a stale self-reference in the rebuilt manifest. The preparer now excludes the manifest from its own file list. |
| 26 | 1.0× | 0.711× | Scan age was 342 ms at admission. No goal sent. |
| 35 | 1.0× | 0.608× | Goal accepted and cancelled on stale TF before movement. |
| 32 | 0.5× | 0.479× | Goal accepted and cancelled on stale TF. |
| 33 | 0.5× | 0.427× | Goal accepted and cancelled on stale TF. |
| 34 | 0.5× | 0.445× | Goal accepted and cancelled on stale scan. |

Achieved rates use the simulator observer's scan timestamp span divided by its
wall receipt span. The three matched half-rate runs all retained the same
navigation settings and sampled Pi resources. Independent truth recorded
sampled rest following cancellation and zero classified wall contacts in each.
[Frozen payload comparison](../../../../simulation/gcu_ground_floor/phase9_diagnostics/half_rate_frozen_source_comparison.json)

Pilot 36 repeated the corridor at a 0.2× target with the power bank, the
current callback and heartbeat code, and detailed tracing disabled. It passed
independent scoring at a measured 0.196×. Final position error was 0.074 m and
heading error was 1.63°. The observer confirmed the passage crossing, sampled
rest, swept clearance and zero classified wall contacts. All 185 scans
received by the Pi matched the simulator's scan bytes.
[Independent score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_36/independent_score.json)
[Scan link analysis](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_36/scan_link_analysis.json)

Pilot 36's motion interval averaged 1.68 CPU cores across the sampled ROS
process set, which includes the Pi scan relay. Its maximum concurrent sampled
PSS was 341 MiB. There were 81 samples over 86.6 wall seconds in the motion
interval. All 177 firmware samples over the broader run were clear. Cold route
preprocessing took 60.2 wall seconds and was measured separately.
[Sampled resource interval](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_36/resource_interval_summary.json)

Two more runs used the same payload and settings as pilot 36. Archive checks
confirmed only the deployment paths, session identifiers and regenerated
manifests differed.

| Pilot | Independent outcome | Final position error | Motion CPU and maximum sampled PSS |
| --- | --- | ---: | --- |
| 36 | Pass, 185/185 scans matched | 0.074 m | 1.68 cores, 341 MiB |
| 37 | Pass, 193/193 scans matched | 0.059 m | 1.68 cores, 340 MiB |
| 38 | Stale scan after crossing, 93/93 scans eventually matched | — | Failed motion interval |

The two passing runs sustained measured simulator rates of 0.196× and 0.198×.
The CPU and PSS measurements describe that slow corridor and include the
Pi-side scan relay. The batch has two passes in three trials, enough to
reproduce the functional result and expose a residual fault on the same profile.

Pilot 38 crossed the passage and moved approximately 0.925 m before the
running watchdog cancelled the goal. Its newest actor scan was stamped 24.500
simulated seconds when the actor clock reached 24.801. The next scan reached
the independent Pi relay 0.494 wall seconds after the fault timestamp. The
observer confirmed rest and zero classified wall contacts.
[Failed trial score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_38/independent_score.json)
[Scan delivery check](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_38/scan_link_analysis.json)

The power bank had clear flags in all 1,438 firmware samples across the six
local runs and eight live attempts with an actor report. The maximum sampled
temperature across these runs was 70.1°C. The process audit found zero Phase 09
jobs on either host. The full regression suite passed 216 tests.
[Consolidated data](../../../../simulation/gcu_ground_floor/phase9_diagnostics/power_bank_navigation_summary.json)
[Process audit](../../../../simulation/gcu_ground_floor/phase9_diagnostics/power_bank_process_audit.json)

## Dedicated timer and clock comparison

Replay 15 gave the 40 ms mission timer its own ROS executor. After dispatch,
the actor's clock remained at 75.771 s as newer TF timestamps arrived, then
advanced by 161 ms. The pose observation guard cancelled the goal.
[Replay 15 actor log](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_15/results/actor/worker.json)

Replay 16 also moved the actor clock subscription to the TF and sensor
executor. The sampled clock and TF timestamps continued to advance through
the next fault. The monitor still stopped on a 200 ms gap between its last
accepted pose and its next attempted sample. The recorded sink received
nonzero velocity commands in both runs. All 156 combined firmware samples
reported `0x0`.
[Replay 16 actor log](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_16/results/actor/worker.json)

Replay 17 used callback tracing without a second arrival-probe process. The
global costmap receipt was 509 ms old at the dispatch check, exceeding the
500 ms lease. The admission code now waits up to its existing five-second
limit for a fresh costmap receipt when no action or mission has started.
[Replay 17 actor log](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_17/results/actor/worker.json)

Replay 18 dispatched with fresh inputs. One mission timer callback took
195 ms wall time, including 193 ms inside the window observation check.
The overlapping heartbeat took 169 ms. The pose monitor faulted after its
next attempted sample was 170 ms newer than the last accepted pose. The
window check includes lock waits, so that duration cannot be attributed to
grid arithmetic alone. The gate now caches the fixed mask-to-grid mapping
for each source and checks the contents and age of every new costmap.
[Replay 18 callback trace](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_18/results/actor/callback_trace.json)

### Passing 1× replays

Replay 19 completed its eight-second action interval and confirmed action
cancellation at the end. Its stationary command sink recorded 80 nonzero
linear commands during the interval, with a maximum of 0.10 m/s. The
passage monitor recorded 196 poses with a maximum consecutive timestamp gap
of 100 ms. Eight process samples give 2.39 Pi CPU cores on average over their
sampling span and 329.5 MiB maximum concurrent sampled PSS. Across the full
run, all 83 firmware readings were `0x0` and peak temperature was 58.4°C.
These measurements include the replay publisher and apply to this single
recorded-input run. [Replay 19 orchestration](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_19/orchestration.json)
[Resource interval](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_19/resource_interval_summary.json)

Replay 20 reproduced the result with the same profile and source clip. It
recorded 80 nonzero linear commands, 191 poses and a maximum consecutive
pose timestamp gap of 70 ms. Seven process samples give 2.39 CPU cores on
average and 329.2 MiB maximum concurrent sampled PSS. All 82 firmware
readings were `0x0` and peak temperature was 58.4°C.
[Replay 20 orchestration](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_20/orchestration.json)
[Resource interval](../../../../simulation/gcu_ground_floor/phase9_diagnostics/stack_replay_20/resource_interval_summary.json)

### Closed-loop retest

Pilot 43 used the replay's scheduling and costmap configuration with Gazebo
on Ubuntu and the navigation stack on the Pi. The simulator target was 1.0×.
The source scan receipts span approximately 0.65 simulated seconds per wall
second during the attempted motion interval. Nav2 accepted the goal. The
actor cancelled it on stale robot TF 0.916 simulated seconds after dispatch.
The simulator observer recorded no movement during that interval. The source
relay recorded 11 scans in the scoring window, and the Pi relay recorded six
with matching contents. All 83 Pi firmware readings were `0x0`.
[Independent score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_43/independent_score.json)
[Scan delivery](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_43/scan_link_analysis.json)

With the trial shut down, the Pi sent 70 ICMP echoes to each host on the
same Wi-Fi. All replies arrived. Peak round-trip times reached 2.030 s to the
Mac and 2.254 s to Ubuntu. Delays on the Mac path also occur without using the
guest. This measures local round-trip latency. The individual ROS delivery
times and the source of the network spikes remain unmeasured.
[Post-trial link measurements](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_43/post_trial_link_rtt.json)

The regression suite passed 216 tests. The final audit found no Phase 09 jobs
left on the Pi or Ubuntu.
[Process audit](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_43/post_retest_process_audit.json)

## November 23 local link recheck

The Pi received all 70 ICMP replies from each of the router, Mac and Ubuntu
guest. Median round-trip times were 7.3, 15.6 and 17.8 ms, with maxima of
16.7, 30.1 and 88.2 ms. Six concurrent ROS echo streams per host each
completed all 60 messages. The largest observed round trip across the 720
echoes was 53.8 ms. These are short idle-link and no-motion message probes.
The closed-loop trial uses the same motion limits and scores as pilot 43.
[ICMP](../../../../simulation/gcu_ground_floor/phase9_diagnostics/link_recheck_20261123/icmp.json) ·
[Pi ROS echo](../../../../simulation/gcu_ground_floor/phase9_diagnostics/link_recheck_20261123/pi_dds.json) ·
[Ubuntu ROS echo](../../../../simulation/gcu_ground_floor/phase9_diagnostics/link_recheck_20261123/simulator_dds.json)

### Closed-loop diagnostics after the link recheck

Pilots 44 and 47 achieved approximately 0.96× simulator rate over their
attempted motion intervals. The Pi scan relay recorded all 55 and 30 source
scans, respectively, with matching payloads. Both trials cancelled on pose
observation gaps. The simulator recorded no contacts. [Pilot 44 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_44/independent_score.json) ·
[Pilot 47 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_47/independent_score.json)

Pilot 47 traced a 226 ms mission timer invocation containing a 169 ms
second-goal dispatch callback. New clock and TF data reached the input
executor during the invocation. The dispatch timing includes connector
validation, goal admission and action publication, along with their lock and
scheduling delays. The swept-footprint checker now avoids per-pose grid
queries when the entire clearance envelope contains free cells. It preserves
the original sample positions, search distance, boundary clearance and
blocked-cell checks. The full regression suite passes 217 tests.
[Pilot 47 trace](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_47/pi_results/actor/callback_trace.json)

Pilot 48 stopped during first-action admission. The most recent local
costmap stamp was 71.582 s and the failure clock was 72.125 s, a 543 ms
age against the 500 ms expiry. No goal was sent. First-action admission now
waits for the same costmap and source freshness reserve already used for
window replacements. The running checks retain their original thresholds.
[Pilot 48 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_48/independent_score.json)

Pilot 51 sent and received acknowledgements for both goals. It recorded all
42 source scans, then cancelled when the running local costmap reached 530 ms
of age. Its last source heartbeat was 11 ms old. The native controller also
reported a missed control-loop cycle. The next variant sends the unchanged
transient-local keepout mask once per window revision while continuing the
raster heartbeat and checking both native costmaps. [Pilot 51 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_51/independent_score.json) ·
[Scan delivery](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_51/scan_link_analysis.json)

Pilot 54 stopped before creating the mission actor. Cyclone DDS reported that
the Pi's configured IP was absent from its interfaces. The Pi returned on the
same address with Wi-Fi power saving on and a different channel. This run
records a link interruption during setup. It contains no motion observations
for the revision-only mask change.
[Actor log](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_54/pi_results/actor.log)

Pilot 55 restored the matched power-save setting and repeated the mask
variant. All 16 preflight pings to Ubuntu returned within 26 ms. The actor
sent the first goal and received its acknowledgement. It then cancelled on
stale robot TF, 0.523 simulated seconds after dispatch by the failure clock.
All eight source scans were eventually recorded by the Pi relay. The
controller missed a cycle and recorded no repeated-mask updates. The source
scan cadence was approximately 0.70× over the attempted motion interval.
[Pilot 55 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_55/independent_score.json) ·
[Scan delivery](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_55/scan_link_analysis.json)

None of the six November 23 diagnostics passed independent mission scoring.
The configuration changed during diagnosis. All sampled firmware flags
were `0x0`. The six results, including the setup interruption, remain in the
record. All 218 tests pass. The Pi and Ubuntu process audit found no remaining
Phase 09 jobs after the final attempt.
[Diagnostic summary](../../../../simulation/gcu_ground_floor/phase9_diagnostics/link_recheck_20261123/retest_summary.json) ·
[Process audit](../../../../simulation/gcu_ground_floor/phase9_diagnostics/link_recheck_20261123/process_audit.json)

## November 23 odometry, TF and costmap trace

Pilot 58 exposed a probe compatibility issue in the installed ROS Humble
executors. They supplied only the message payload to Python callbacks, so
the probes exited when their callbacks required DDS metadata. The mission
cancelled on stale TF and the Pi scan relay recorded six of nine source scans.
The preflight round-trip time peaked at 2.06 s.
[Pilot 58 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_58/independent_score.json)

The probe now accepts an optional DDS metadata argument. Both read-only
probes completed in pilot 59. Its final 12 preflight pings returned within
38.1 ms. The live run cancelled on stale TF 0.246 simulated seconds after
dispatch. The simulator recorded no movement or contacts. The Pi scan relay
recorded three of eight source scans, and the motion and teardown interval
achieved approximately 0.71× simulator rate.
[Pilot 59 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_59/independent_score.json) ·
[Scan coverage](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_59/scan_link_analysis.json)

| Stream in the two simulated seconds before cancellation | Ubuntu stamps | Matching stamps seen by the Pi probe | Longest gap between stamps on Ubuntu and on the Pi |
| --- | ---: | ---: | --- |
| Wheel odometry | 100 | 69 | 20 ms and 180 ms |
| IMU | 200 | 104 | 10 ms and 170 ms |

Three matched stamps in each stream appeared at the Pi probe after the
fault. At cancellation, the last `odom -> base_footprint` observed by both
the independent probe and the mission was stamped 65.400 s against a mission
clock at 65.671 s. The latest identical global and local costmap stamps were
296 ms and 391 ms old, within their 500 ms lease at this fault. The installed
Python callbacks exposed no DDS source times. The comparisons therefore
identify a gap between the simulator observations and Pi callback receipts.
They cannot distinguish packets missed over the air from local middleware
queueing or probe callback loss, or establish exact native publish latency.
[Pipeline comparison](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_59/pipeline_timing_analysis.json)

## Sensor-rate Wi-Fi load checks

The Ubuntu probe replayed 20 seconds of recorded sensor-sized traffic,
including wheel odometry at 50 Hz, IMU at 100 Hz, LiDAR at 10 Hz and a
100 Hz clock. Source payloads totalled approximately 0.79 Mbit/s before
protocol overhead. Both hosts used isolated test topics. Gazebo, Nav2 and
localization were inactive during the probes.

| Check | Matched clock, IMU, LiDAR and odometry stamps | Longest Pi callback gap, IMU and odometry | Peak concurrent ICMP RTT |
| --- | --- | --- | ---: |
| Original profile | 1,687/2,000, 1,708/2,000, 200/200, 972/1,000 | 455 ms and 223 ms | 355 ms |
| Reliable IMU and odometry | 1,999/2,000, 2,000/2,000, 200/200, 1,000/1,000 | 93 ms and 120 ms | 96 ms |
| Original profile repeated | 2,000/2,000, 2,000/2,000, 200/200, 1,000/1,000 | 53 ms and 78 ms | 58 ms |

The first check reproduced late or missing callbacks without any navigation
workload. The largest LiDAR callback gap was 375 ms even though all 200
scans eventually arrived. The concurrent link conditions improved across
the three runs, and both QoS settings recorded complete sensor delivery in
at least one run. The no-motion measurements do not determine a QoS effect
or predict the sustained closed-loop result. The corridor follow-up retains
the original delivery settings.
[Original probe](../../../../simulation/gcu_ground_floor/phase9_diagnostics/sensor_link_load_01/summary.json) ·
[Reliable input probe](../../../../simulation/gcu_ground_floor/phase9_diagnostics/sensor_link_load_02/summary.json) ·
[Original profile repeat](../../../../simulation/gcu_ground_floor/phase9_diagnostics/sensor_link_load_03/summary.json)

The first uninstrumented corridor retest after the perfect load check
cancelled on stale TF 0.392 simulated seconds after dispatch. All six
source scans were recorded by the Pi relay. The achieved simulator rate
was approximately 0.55× over the attempted interval. The matched batch
stopped at that failure.
[Pilot 60 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_60/independent_score.json)

A Gazebo-only audit with the Pi's localizer running confirmed that
`/diff_controller/odom` and `/imu_broadcaster/imu` both offer reliable DDS.
The observed Pi subscriptions used best effort. The staged diagnostic
adds a Pi relay that requests reliable input and forwards original bytes
to separate localization and mission IMU topics. It keeps the existing
clock transport, scan relay and motion watchdog thresholds.
[Live endpoint audit](../../../../simulation/gcu_ground_floor/phase9_diagnostics/qos_audit_01/endpoints.json)

Pilot 63 ran with the reliable Pi input relay. At the stale-scan fault,
the relay's newest in-time wheel odometry stamp was 16 ms old and the
newest IMU stamp was 46 ms old. The actor's latest scan stamp was 316 ms
old against its 300 ms cutoff. The next scan reached the Pi relay 686 ms
after the fault by the same host's monotonic time. The robot recorded no
contacts. Both relay processes exited cleanly. The receiver preserved the
odometry and IMU payloads, and the independent motion score failed on
freshness. The variant remains optional.
[Pilot 63 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_63/independent_score.json) ·
[Reliable input timing](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_63/reliable_input_timing_summary.json)

## November 25 hotspot qualification

The Mac and Ubuntu guest reached the Pi over the hotspot. The Pi reached
Ubuntu on all eight pings, with a maximum round trip of 11.4 ms before the
load test. With Pi power saving off, the 30-second no-motion replay delivered
all 1,500 wheel odometry, 3,000 IMU and 300 LiDAR stamps. The best-effort
clock delivered 2,968 of 3,000 stamps. The largest callback gaps were
180 ms for odometry, 181 ms for IMU and 252 ms for LiDAR. All 200 concurrent
pings returned, with an 11.8 ms mean and 191.5 ms maximum round trip. The
load result justified a bounded live retest using the reliable sensor relay.
[Hotspot load summary](../../../../simulation/gcu_ground_floor/phase9_diagnostics/hotspot_20261125/summary.json)

### First hotspot corridor batch

Both runs used the reliable scan, odometry and IMU relays with the same 1.0×
Gazebo target. The independent scorer accepted pilot 64. The robot crossed the
required passage, recorded zero contacts and stopped 0.077 m from the goal.
AMCL's p95 position error against simulator truth was 0.048 m. Gazebo ran at
0.695× real time over the measured scan interval. The Pi used an average of
2.18 CPU cores and a maximum of 372 MiB sampled concurrent PSS during motion.
[Independent score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_64/independent_score.json) ·
[Resource interval](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_64/resource_interval_summary.json)

Pilot 65 crossed the passage and received all 127 source scans, then stopped
when the rolling-window update tried to use a stale pose. Pilot 66 was staged
and the batch stopped before it launched. The window update now defers if its
pose lookup reports stale TF. The mission watchdog retains its original 150 ms
limit, and pose lookup logs now retain the clock and latest TF timestamp when a
lookup rejects stale data. The first two trials belong to the previous revision.
[Pilot 65 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_65/independent_score.json) ·
[Scan coverage](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_65/scan_link_analysis.json)

### Matched retest after changing the window lookup

The retest uses the same short corridor, 1.0× physics target, original
freshness limits and scoring rules. The source comparison normalized only
session names and workspace paths before comparing the staged files. Each
host's manifest retains hashes of its original staged files.

| Trial | Independent result | Measured simulator rate | Goal error | Scan payloads matched | Mean Pi CPU cores during motion | Maximum sampled concurrent Pi PSS |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 67 | Passed, one passage and zero contacts | 0.633× | 0.081 m | 186/186 | 2.14 | 372 MiB |
| 68 | Timed out at Nav2 window preflight before goal dispatch | — | — | — | — | — |
| 69 | Staged, batch stopped before launch | — | — | — | — | — |

Trial 67 had 100% matched localization coverage and 0.042 m p95 localization
position error. The sampled workload reports CPU for the motion interval and
includes the sensor relays. Peak temperature across the run was 56.5°C, with
no firmware undervoltage or throttling flags in the recorded samples.
[Pilot 67 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_67/independent_score.json) ·
[Pilot 67 resources](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_67/resource_interval_summary.json) ·
[Pilot 67 scans](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_67/scan_link_analysis.json)

Pilot 68's 16 pre-launch pings all returned within 9.9 ms. From initial
window publication to preflight timeout, Ubuntu recorded 40 scan stamps with a
maximum 192 ms gap. The Pi relay recorded only 16 stamps in that same source
timestamp range across its full log, with one gap of 2.04 wall seconds. The
latest TF in the Pi lookup was 365 ms behind the actor's clock at timeout.
Ubuntu's clock transport continued publishing across this interval. The scan
gaps are measured on each host's monotonic clock and describe delivery to
application callbacks. The measurements leave open the relative contribution of
the hotspot, DDS and receiver scheduling. No goal was sent in this trial.
[Pilot 68 score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_68/independent_score.json) ·
[Ubuntu scan sender](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_68/simulator_results/results/scan_sender.json) ·
[Pi scan receiver](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_68/pi_results/scan_receiver.json) ·
[Pi actor](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_68/pi_results/actor/worker.json)

All 223 local tests passed, including a regression test for the window
lookup. After trial 68, a process audit found no remaining Phase 09 ROS or
Gazebo processes on the Pi or in Ubuntu.

### Pixel hotspot follow-up

The Pi's Wi-Fi scan saw `pixel` on channels 1 and 161. The Pi stayed on
channel 1 (2,412 MHz) before and after the 30-second stationary 1× replay,
with Wi-Fi power saving enabled. The Pi recorded all 300 LiDAR, 3,000 IMU
and 1,500 wheel odometry stamps and 2,997 of 3,000 best-effort clock stamps.
Maximum callback gaps were 138 ms for LiDAR, 75 ms for IMU and 81 ms for
wheel odometry. All 200 concurrent pings returned, with an 11.4 ms mean
and 98.4 ms maximum RTT. The 5 GHz broadcast is visible at 5,805 MHz.
The measurements cover the 2.4 GHz application callbacks with navigation
inactive. A comparison would hold the probe workload and power-saving
setting fixed and recheck both hosts' IPs after changing bands.
[Sensor probe summary](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pixel_20261125_2ghz/summary.json) ·
[Concurrent pings](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pixel_20261125_2ghz/ping_console.txt)

The 5 GHz profile and explicit access point selection both activated, but the
Pi associated at 2,437 MHz. Pilot 70 used this 2.4 GHz connection with power
saving enabled and the same navigation profile as the revised window retest.
The robot crossed the required passage and recorded zero contacts. The mission
watchdog cancelled 15.18 simulated seconds after dispatch. At that point
the mission clock was at 64.920 s and the latest usable transform was stamped
64.740 s, 180 ms behind the clock and past the 150 ms limit. All 153 source
scans were recorded in the corresponding interval. At the time of the stop,
the Pi relays had also logged scan, wheel odometry and IMU stamps within 30 ms
of the actor clock. Some recent inputs arrived in bursts shortly before the
fault, so the trace leaves open the contribution of delivery and local
processing to the late transform. The measured simulator rate to the
fault was approximately 0.710×. Both host roles collected their logs and shut
down. Pilots 71 and 72 were staged and did not launch.
[Independent score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_70/independent_score.json) ·
[Scan coverage](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_70/scan_link_analysis.json) ·
[Actor clock and transform observations](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_70/pi_results/actor/worker.json)

The matched power-saving-off retest stayed on 2,437 MHz, with a clear firmware
power status before and after launch. Pilot 73 crossed the required passage
with zero contacts and matched all 79 source scans. At the stop, 7.785 simulated
seconds after dispatch, the last acknowledged revision-1 global grid was
540 ms old against the 500 ms limit. The incoming global grid still had the
old origin, so it could not acknowledge revision 2. The actor recorded the new
origin on the next global update and committed revision 2 after cancellation
began. The observed simulator rate to the fault was 0.710×. This attempt shows
a gap between native global-costmap acknowledgements during a window change,
with both software phases and the timing of their inputs relevant. One run per
power-saving setting does not estimate an effect of that setting. The matched
batch stopped before launching the remaining two repeats.
[Pilot 73 independent score](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_73/independent_score.json) ·
[Costmap readiness observations](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_73/pi_results/actor/worker.json) ·
[Scan coverage](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_73/scan_link_analysis.json)

## Phase 09 completion

Trials 76–78 passed consecutively with the same Pixel link and 10 Hz global
costmap update and publication. The local costmap and watchdog limits matched
trial 73. The measured simulator rates were 0.669–0.717×. The
[completion record](09_PI4B_NAVIGATION_BASELINE.md) contains the accepted
short-corridor scope, per-run scores, handoff times and resource comparison.

The [November 17 checkpoint](09D_PI4B_CALLBACK_AND_SCAN_DELIVERY.md) records
the earlier successful slow mission and its resource measurements. The
[experiment harness](../../../../experiments/pi_baseline/README.md) documents
how to reproduce the local stage.
