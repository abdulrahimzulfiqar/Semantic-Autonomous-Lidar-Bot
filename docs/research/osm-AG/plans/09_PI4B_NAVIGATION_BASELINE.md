# Phase 09: actual Pi 4B navigation baseline

## Quick summary

- **Goal:** Measure the accepted osmAG + AMCL + Nav2 workload on actual Pi hardware before adding camera or language inference.
- **Status:** The measured corridor baseline is complete. Trials 76–78 passed consecutively at 0.669–0.717× measured simulator rate. The Pi also completed two eight-second 1× stationary replays. [Phase 09 results](../phases/09_PI4B_NAVIGATION_BASELINE.md)
- **Setup:** Pi runs navigation compute; another machine runs Gazebo/evaluation. Keep the working AMCL/EKF and separate structural prototype unchanged.
- **Finish when:** Repeated frozen tasks establish reliability, latency and a measured CPU/RAM/network/thermal budget, with source and hardware versions recorded.

## Why this is next

Phase 08 established no controlled CPU/RAM advantage for structural localization.
The user chose to retain AMCL and prioritize the core EdgeOsmAG workload. Measure
what actually limits the Pi before optimizing or replacing a working component.
This phase also freezes the implemented single-floor simulation stack; it does
not claim complete reproduction of the authors' localization architecture.

## Current checkpoint — November 16, 2026

The actual 4 GB Pi 4B runs Ubuntu 22.04.5/aarch64 and ROS 2 Humble. Missing planner
libraries and Cyclone DDS were installed by the user because SSH did not have
passwordless administrator access. Read-only inventories pin pre/post versions.
The retained five-case native plan-only test passes on the Pi and independently
replays; this is not a robot-motion result.

Ubuntu now uses a reversible UTM bridge on `en0` and has LAN address
`192.168.100.137`; Pi address is `192.168.100.129`. A dedicated domain-217 probe
exchanges 60/60 messages in each direction after a discovery-profile adjustment.
All attempts are retained. Wi-Fi echo-time tails remain hundreds of milliseconds;
power saving is enabled. The single-participant diagnostic profile cannot be used
unchanged for a multi-process navigation stack. A wired connection is requested
before interpreting performance results; Wi-Fi-only trials would need explicit
network/rate characterization.

**Remaining work stays within groups 2–3 below:** adapt and verify an explicitly
scoped two-host simulation launcher/executor profile, run one short closed-loop
mission, then freeze and repeat the benchmark tasks. The existing accepted
loopback-only execution guard and frozen Phase 07/08 sources have not been changed.

## Resumed investigation — November 17, 2026

Both hosts are still on the same LAN addresses, Ubuntu is logged in, and a
six-process-per-host probe discovers all peers with 719/720 echoes. This short
probe does not establish reliable sensor/control delivery under navigation load.

Pilot 05 exposes a concrete preflight bug: cached consumption receipts are over
1.2 simulated seconds old although the latest received grids are under 0.1 s old.
The window manager now revalidates actual grids before each admission and after
native planning. A controlled regression reproduces the old failure; genuinely
stale input remains rejected. Pilot 06 gets through preflight and dispatches a
goal, then cancels on a stale scan. The fix addresses admission bookkeeping; it
does not establish a successful mission.

A clock-only diagnostic measures 1,000 messages per simulated second. Optional
simulator-side transport caps clock delivery at 100 updates per wall second,
preserving real source stamps and pauses. Pilot 07 reduces clock traffic but
still cancels on scan age. All 191 tests pass, including the receipt regression
and clock transport tests. Failed runs and independent sampled-rest observations
remain evidence; they are not successful navigation benchmarks.

The user subsequently disabled Pi Wi-Fi power saving; `iw` confirms it is off.
Pilot 08 still stops on stale TF. Pilot 09 declares a 0.2 target physics real-time
factor, with the same simulated-time freshness thresholds and 0.1 m/s velocity
limit. It travels approximately 0.525 m before the crossing monitor rejects a
0.157 s gap between pose observations (limit 0.15 s). Both are failed diagnostic
pilots, not nominal navigation results. Independent truth records sampled rest
after both failures and no classified wall contacts, with all 13 contact
publishers present. Slower simulation cannot demonstrate full-rate Pi capacity.

Do not attribute these failures solely to Internet service or claim the clock
cap/power-saving change fixed the link. Simulator-local scans arrive promptly;
the remaining cross-host/receiver timing problem still needs isolation. The next
bounded diagnostic is fixed-rate sensor replay locally on the Pi, recording
actual input delivery and callback/TF timing. Replay can isolate compute from
Wi-Fi delivery but cannot establish closed-loop navigation success. A reporting
batch remains gated on successful closed-loop tasks and a frozen protocol.

The user reports a MacBook 30 W USB-C adapter/cable and a small aluminum CPU
heatsink. Fan presence and ambient temperature are unreported. The adapter rating
is not measured Pi power draw. The post-trial firmware check reports
`throttled=0x0`; this does not replace continuous sampling. Hardware details,
failed trial artifacts, and the exact summary derivation are linked in
[checkpoint 09B](../phases/09B_PI4B_SPLIT_HOST_DIAGNOSTICS.md). No domain-217/218
experiment processes remained on either host after cleanup.

### Local replay follow-up — November 17, 2026

Three complete Pi-local AMCL replays each receive all 552 scored scans at 1×
recorded time. Two pass timing gates; the middle attempt exceeds the pose gap
(171 ms) and clock receipt gap (186 ms) limits. These evolving diagnostic runs
are not independent reliability repetitions. This failure occurs without the
Wi-Fi sensor path, so that path cannot be the sole explanation. AMCL averages
9–11% of one core in these short recordings; the publisher and Python observer
costs are separate. This is not a full-stack or sustained performance budget.

A traced final run passes and records clock publications continuing during its
largest receipt gap. The trace does not explain the earlier failure or isolate
middleware from callback scheduling. See [checkpoint 09C](../phases/09C_PI4B_LOCAL_REPLAY.md)
for all outcomes, inputs, thresholds, source hashes and reproduction. Next is
full-stack clock/callback tracing before another bounded closed-loop task.
No new Gazebo simulation was needed for this local diagnostic.

### Full-stack trace follow-up — November 17, 2026

Pilot 10 records 39 independent Pi clock callbacks during a 0.987 s gap in the
actor clock callback while the control timer keeps running. A separate timer
group preserves the same callback, state lock, period and clock; pilot 11 then
records a passage crossing, but stops on stale TF after scans present in Ubuntu
are absent from the independent Pi probe. A subsequent reliable scan-hop pilot
also stops on stale scan input and fails scan coverage verification. No threshold
is loosened and neither optional variant is declared an accepted reliability fix.

[Checkpoint 09D](../phases/09D_PI4B_CALLBACK_AND_SCAN_DELIVERY.md) retains all
attempts, including an observer CLI startup failure, plots and byte-hash checks.
There are no remaining owned experiment processes on either host. Next improve
or isolate the delivery path before a controlled retest; a physical/network
change can help distinguish wireless conditions from DDS/receiver behavior.
The nominal mission and repeated budget remain pending.

### Closer-router and higher-rate follow-up — November 17, 2026

The user selects moving the Pi close to the same router; measured signal is
−44 dBm. A reboot resets power saving to on, and the user restores off before
testing. The staged payload matches pilot 13 after path/session substitutions.
Pilot 14 passes independent correctness scoring: 0.0881 m final position error,
one confirmed passage, observed rest, no classified wall contacts, and all
191 source scans received with matching content. Reboot/session/temperature
changes prevent attributing the result solely to distance. This is one slow
functional mission, not a controlled reliability estimate or sustained budget.

Changing only the physics target to 1.0 causes pilot 15 to fail TF admission
before any goal. A bounded fresh-TF acquisition helper in 16 also times out while
the model stays idle. All 35 scans in that admission interval are eventually
received; completeness does not establish timely TF availability. Running
watchdogs, speed, path checks and action deadlines remain unchanged. The next
step is full-stack local input replay/profiling to separate local processing from
cross-host delay, then higher-rate admission/runtime validation and frozen
repetitions. [Checkpoint 09D](../phases/09D_PI4B_CALLBACK_AND_SCAN_DELIVERY.md)
now consolidates these outcomes and the preliminary instrumented resource sample.

### Saved Wi-Fi profiles

`netplan-wlan0-WIFI` and `iPhone 6` are already saved with autoconnect enabled and
equal priority 0. NetworkManager can select an available saved profile when it
needs a connection; equal priority generally favors the most recently used one.
It does not replace a working active connection merely because Internet service
is slow. These are metadata checks only: no credential was exported and no
intentional failover was induced during experiments. Actual backup credential
validity is not established by inspecting the profile. See
[NetworkManager connection settings](https://www.networkmanager.dev/docs/api/latest/settings-connection.html).

## Bounded sequence

1. **Inventory and compatibility.** Record Pi model/RAM, OS/architecture, ROS/Nav2
   versions, cooling/power, storage and network. Verify supported packages and
   freeze launch/config/source versions. Record any unavoidable difference from
   the accepted Humble setup. Establish a separately versioned ROS network
   profile: the current localhost-only Cyclone DDS configuration cannot connect
   the Pi to the external simulator. Measure network/middleware overhead.
2. **Navigation-only profiling.** Run existing short accepted tasks with Gazebo
   and independent truth evaluation off the Pi. Preserve map/footprint, AMCL/EKF,
   velocity limits, controller settings and goal semantics. Distinguish startup,
   preprocessing, online planning, localization/control and idle resource use.
   Use bounded trials with explicit wall budgets; retain failed/aborted attempts.
3. **Repeat and set the budget.** Repeat declared tasks and report distributions
   of arrival, localization errors, contacts/failures, timing and resource use.
   Identify actual bottlenecks before changing parameters. Freeze an acceptable
   baseline and reserve measured headroom for later perception/language tests.

## Required measurements

- Per-process and whole navigation-workload CPU time/utilization and RAM, with
  RSS/PSS definitions and sampling intervals stated.
- Planning/action latency, controller update/deadline behavior, scan/TF freshness,
  task completion/arrival, localization errors and failure/shutdown outcomes.
- Network throughput/latency and clock/time policy; distinguish simulated time
  from wall time and avoid attributing slow external simulation to Pi performance.
- Temperature, throttling and sustained-load behavior; power/energy only where
  an appropriate measurement method is available.
- No simulator, ground-truth evaluator, Foxglove UI or unrelated host compute
  included as Pi navigation cost. Record any offboard preprocessing explicitly.

No CPU percentage or RAM target is invented before the actual Pi inventory and
baseline measurement. A successful run alone is insufficient to establish a
sustained resource budget. If a required task fails, record the limitation and
make a bounded change followed by matched verification.

## Following experiments

Phase 10 adds camera perception and checks useful area/object associations plus
navigation impact. Phase 11 adds asynchronous, validated language-to-task logic,
with local/offboard inference separated. Phase 12 compares navigation alone,
+camera, +language and +both under matched tasks and hardware. The research
contribution is the measured architecture/tradeoff, not merely the presence of
those components.

Reopen structural localization only if Pi profiling identifies a material AMCL
bottleneck, required cases expose localization failures, or a compatible author
implementation becomes available. Preserve its existing experimental code and
inputs for that optional comparison.
