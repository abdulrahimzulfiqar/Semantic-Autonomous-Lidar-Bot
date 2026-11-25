# Actual Pi processor-in-loop experiments

## Quick summary

This separate harness keeps Gazebo, simulated hardware drivers and ground-truth
recording in Ubuntu. The Pi runs AMCL/EKF, osmAG preprocessing/routing/window
management and native Nav2. Initial trials are bounded setup pilots. They do not
complete Phase 09 or establish real-time capacity from a slow simulator.

## Scope and isolation

- Dedicated ROS domain **218**; only `/phase9/sim_cmd_vel` feeds the simulated
  twist relay. No physical robot driver is started on the Pi.
- An explicit pinned JSON/DDS profile authorizes the generated simulation actor.
  The original loopback-only execution guard and frozen Phase 07/08 evidence stay intact.
- The preparer derives a private executor copy by replacing only the guard
  definition with that profile import. It verifies that the remaining Python AST
  is identical. Sensor/TF/constraint freshness, action handling and speed limits
  are preserved.
- The current window manager revalidates received native grids before each
  preflight admission, including after a slow query. Phase 09 exposed stale
  cached receipts even when fresh grids had arrived; the regression test checks
  that missing native updates still fail. No freshness limit is widened.
- Ground truth and contact recording exist only on the simulator side. The Pi
  actor has no ground-truth subscription or ground-truth input file. Independent
  scoring later reads the frozen Pi inputs and simulator observations.
- Pilot-only changes: two-host DDS profile, simulated command-topic remap and
  disabled camera sensor/image bridge on the simulator. No LLM or perception
  processing runs. Keep cooling/power/network/software version differences explicit.

The optional `--clock-transport-hz 100` profile remaps the simulator bridge's raw
clock to `/phase9/raw_clock` and coalesces its original samples into `/clock`.
This is a measured bandwidth experiment: a no-robot diagnostic observed 1,000
clock messages per simulated second. The transport does not extrapolate time or
advance a paused source. Its counters and per-second buckets are retained; the
first capped pilot still cancelled on scan age, so a reliability benefit is not
established. Apply the same declared clock/network profile to matched variants.

`--physics-real-time-factor 0.2` is an optional **slow functional diagnostic**;
the default is 1.0. It gives the Pi more wall time per simulated second and
therefore cannot demonstrate real-time processing capacity. Physics step size,
velocity and freshness limits remain unchanged. Pilot 09 still stopped on a
pose-observation gap; do not present that slower trial as an accepted baseline.

The user reports a MacBook 30 W USB-C adapter/cable and a small aluminum CPU
heatsink. Adapter rating is not Pi power consumption; fan presence and ambient
temperature are unreported. Wi-Fi power saving is verified off before pilots
08/09; both fail timing guards. See `physical_setup.json` and the per-trial
`network_condition.json` records. Do not infer that power saving caused the
earlier failures from this small sequential diagnostic series.

## Initial pilot protocol

Start approximately `(-0.5, -12.9603, 0)` and navigate to
`(1.0, -12.9603, 0)`, crossing passage `-93`. The Pi actor has a 260 s wall budget
and 90 s simulated motion budget. Process-owner and host watchdogs allow bounded
setup/cleanup around that interval. Failures are retained and never converted to
success by relaxing the established freshness or geometry checks.

Correctness gates inherit the Phase 06/07 policy: at most 0.20 m / 10° final error,
0.25 m / 5° p95 localization error, at least 90% truth/TF matching, maximum 0.15 s
truth gap, independent sampled rest, physical swept clearance, no classified wall
contacts and an unchanged 0.1 m/s linear command limit. This short pilot does not
require three planning windows or a proxy; longer-window behavior was covered by
the earlier native fixture and remains a future split-host test.

The sampler records per-process cumulative CPU seconds and RSS/PSS approximately
once per wall second, plus host memory, temperature and frequency. Its process
scope is domain-218 children; the recorder thread/owner is outside that ROS domain.
CPU percentages, peaks between samples and whole-device power are not inferred.
Startup, preprocessing and movement intervals must be separated when reporting.

Pilot 14 is the first independently scored successful mission, under a declared
0.2× physics target after the reported closer-router move. It is not a full-rate
capacity result. Higher-rate pilots 15/16 fail TF admission without sending a
navigation goal. Current harness setup allows up to five wall seconds to acquire
fresh TF and scans while the model is pristine and idle. Active missions use
the existing input-age thresholds and fail-closed watchdogs. The archived successful pilot 14
predates that setup change, and its exact runtime snapshot remains retained.

## Files

`prepare.py` builds separate role trees from an accepted baseline export.
`run_pilot.py` deploys them and supervises one trial through SSH and UTM's guest
agent. `run_role.py` owns local processes. `worker.py` is the Pi actor;
`observer.py` is the laptop-only recorder; `resources.py` samples the Pi.
`score.py` checks motion independently afterward. Source manifests and archives
pin the exact executed code/configuration and preserve failed attempts.

`summarize_pilots.py --evidence <phase9_diagnostics>` derives the latest failed
trial summary from retained actor/observer/resource records. It reports sampled
rest, displacement and whole-attempt sampled PSS/temperature with input hashes;
it does not replace independent mission scoring or estimate a sustained budget.

`analyze_pilot_resources.py --evidence <attempt-directory>` derives sampled
process costs from actual dispatch to the last native success/fault response.
It reports the actual sample interval and sensor receipt rates, labels diagnostic
overhead and rejects complete CPU accounting when the observed process set
changes. Pre-dispatch failures cannot produce a motion-resource summary. Its
first successful-trial output remains an instrumented slow-run profile, not a
sustained or uninstrumented capacity budget.

`link_group.py` checks six independent processes per host without motion.
`sensor_link_probe.py` replays the retained 10–15 s sensor capture at its
original rates, adding a 100 Hz clock stream. Run the `source` on Ubuntu
and the `receiver` on the Pi with the same `--session` and `--duration`,
domain 217, the matching DDS peer files, and `ROS_LOCALHOST_ONLY=0`.
Pass `--recording` on the source. Both processes must use the same
`--reliable-inputs` setting. The topics are isolated under `/phase9_link_load/`
and carry no motion commands. Match header stamps to compare delivery, and
use each receiver's monotonic time for gaps between callbacks.

`probe_clock_rate.py` measures the source clock without spawning a robot.
`clock_transport.py` implements the optional clock-rate cap with unit tests for
unchanged timestamps, paused sources and backward jumps.

Use new staging, remote workspace and evidence paths for every attempt. Do not
reuse or modify the accepted Phase 07/08 workspaces. The plan and current results
belong in `docs/research/osm-AG/plans/09_PI4B_NAVIGATION_BASELINE.md` and
`simulation/gcu_ground_floor/phase9_diagnostics/`.

## Optional full-stack timing variants

The split-host preparer accepts optional timing flags, **off by default**:

- `--timing-trace`: bounded actor callback entry/return traces plus a separate
  read-only Pi receipt probe. Timestamps identify callback processing, not wire
  arrival; instrumentation adds measurable overhead.
- `--separate-mission-timer`: recreate the mission timer in its own mutually
  exclusive group, preserving the original callback/state lock, steady clock and
  40 ms period. This is a diagnostic variant, not a proven reliability fix.
- `--reliable-scan-link`: a reliable depth-5 simulation scan hop, with original
  serialized scan hashes/stamps recorded at both ends. The Ubuntu observer reads
  the original local scan source. Reliable QoS did not establish timely or
  complete callback receipt in pilot 13; do not present it as an accepted fix.
- `--dedicated-tf --dedicated-mission-timer --dedicated-actor-clock`: service
  sensors, transforms and the mission clock on a dedicated input executor,
  and schedule the original 40 ms timer on its own executor.
- `--mask-on-revision`: send the transient-local keepout mask once for each
  revision while continuing the planning-raster heartbeat. The mission still
  checks the contents and timestamps of both native costmaps.
- `--global-costmap-hz 10`: set the Pi global costmap update and publish rates
  for a window handoff timing experiment. The local costmap and admission
  limits come from the baseline profile.
- `--pipeline-trace`: add read-only wheel odometry and IMU probes on Ubuntu
  and the Pi, plus independent filtered odometry, TF and native costmap
  receipts on the Pi. This enables the actor callback trace as well. Run
  `analyze_pipeline.py --evidence <attempt>` after collection. Match source
  stamps across hosts and compare wall-time receipts on the Pi. DDS source
  metadata is optional and missing values leave those timings unset.
- `--reliable-localizer-inputs`: request reliable DDS for the existing
  Gazebo odometry and IMU publishers on the Pi. A relay republishes the
  original CDR payloads to the staged localizer and the actor's IMU check on
  separate topics. The actor uses the same freshness checks, and the relay
  records input and forwarding timestamps in `localizer_input_relay.json`.
  This is a split-host diagnostic option. It is inactive in loopback replay.

The original production executor and frozen baselines are not edited by these
options. `analyze_callbacks.py --evidence <attempt>` correlates same-Pi traces;
`analyze_scan_link.py --evidence <attempt>` verifies scan coverage/content;
`plot_callbacks.py` renders the failure timeline. Source manifests pin each
variant. Read [checkpoint 09D](../../docs/research/osm-AG/phases/09D_PI4B_CALLBACK_AND_SCAN_DELIVERY.md)
for retained successes/failures within the diagnostics and the remaining gates.

## Pi-local replay diagnostic

For full-stack stationary replay, use `prepare_stack_replay.py` with a frozen
Pi stage from `prepare.py` and a raw simulation capture. The `--capture-inputs`
pilot flag saves the input capture on the simulator side. The stage pins the
selected clip and runs AMCL/EKF, rolling costmaps and the normal Nav2 servers
over localhost. Its isolated command recorder collects velocities with the
stationary recording held independently of control output.

```bash
mkdir /tmp/edgeosmag_stack_replay_template
tar -xzf simulation/gcu_ground_floor/phase9_diagnostics/pilot_17/pi_sources.tar.gz \
  -C /tmp/edgeosmag_stack_replay_template
docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  experiments/pi_baseline/prepare_stack_replay.py \
  --template /tmp/edgeosmag_stack_replay_template \
  --destination /tmp/edgeosmag_stack_replay_NEW \
  --remote-root /home/pi-project/edgeosmag_stack_replay_NEW \
  --recording simulation/gcu_ground_floor/phase9_diagnostics/pilot_17/simulator_results/results/sensor_recording.json.gz \
  --start-ns 10000000000 --end-ns 15000000000
```

Run the resulting stage via the Mac-side `run_local_replay.py` command below
with `--full-stack` and the new stage, output and remote-root paths. An eligible
template includes the Phase 09 executor, configured behavior trees and trace
probe. `analyze_pilot_resources.py` reads a completed action interval from the
local role results. Verify firmware power status and measured input cadence
before using a run in the baseline. See the [full-stack replay record](../../docs/research/osm-AG/phases/09E_PI4B_FULL_STACK_REPLAY.md).

To reproduce the scheduling comparison, stage the split-host template with
`--dedicated-tf --dedicated-mission-timer`. Add `--dedicated-actor-clock`
to service the existing mission ROS clock on the same input executor as TF,
LiDAR and IMU. The timer still uses its original 40 ms steady-clock period.
The replay stage carries these settings forward, preserving the recorded
sensor stamps and the existing timing limits.
Add `--trace-actor` to `prepare_stack_replay.py` for a diagnostic run that
logs callback durations without starting the separate arrival probe process.

### Localization-only replay

This separate, command-free path runs AMCL/map server plus a replay publisher
and observer on localhost. It excludes EKF and the navigation planner/controller.
It preserves original Phase 08 scan stamps/ranges, interpolates recorded planar
odometry without extrapolation, and generates a declared 1× replay clock. A
one-second recorded-input warmup precedes the 55.1-second scored interval. The
publisher and observer overhead are measured separately from AMCL.

Use a new suffix for each local/remote/evidence path; for example:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  experiments/pi_baseline/prepare_replay.py \
  --capture simulation/gcu_ground_floor/phase8_diagnostics/clean_tracking/attempt01.tar.gz \
  --destination /tmp/edgeosmag_phase9_local_replay_NEW

PYTHONDONTWRITEBYTECODE=1 python3 experiments/pi_baseline/run_local_replay.py \
  --stage /tmp/edgeosmag_phase9_local_replay_NEW \
  --output simulation/gcu_ground_floor/phase9_diagnostics/local_replay_NEW \
  --remote-root /home/pi-project/edgeosmag_phase9_local_replay_NEW \
  --key /Users/teamincredibles/.ssh/id_rsa_localhost_run \
  --host pi-project@192.168.100.129
```

`analyze_local_replay.py --evidence <attempt-directory>` independently checks
frozen inputs and timing, and derives sampled per-process CPU/PSS costs.
The source/receipt clock trace uses same-host monotonic time; it does not isolate
middleware from callback scheduling. Read
[the diagnostic record](../../docs/research/osm-AG/phases/09C_PI4B_LOCAL_REPLAY.md)
before interpreting the numbers: one complete local replay also exceeds the
observation-gap guard despite no Wi-Fi sensor path. Phase 09 remains open.
