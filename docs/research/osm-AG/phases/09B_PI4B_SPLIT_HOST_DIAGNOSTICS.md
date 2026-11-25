# Phase 09 diagnostic checkpoint: Pi execution and cross-host timing

## Quick summary

- **Scope:** Actual Pi navigation compute with Gazebo/sensors/evaluation in Ubuntu; camera and LLM workloads excluded.
- **Result:** Native preflight repaired; goals dispatch and motion occurs. All 192 tests pass. Latest two missions fail existing timing guards.
- **Open:** No successful complete Pi mission, sustained resource budget or real-time capacity result. Wi-Fi power saving off did not resolve the failures.
- **Next:** Fixed-rate sensor replay locally on the Pi to isolate processing from transport, then a bounded closed-loop retest. Phase 09 remains open.

## What changed

Phase 09 revealed a stale receipt cache in the rolling-window preflight: the
admission check used old native-grid receipts even after newer grids arrived.
The manager now revalidates actual received grids before each admission and
after native planning. A regression reproduces the old failure and confirms
that genuinely stale input still fails. No freshness or geometry threshold was
relaxed. The original localhost-only execution guard and frozen Phase 07/08
evidence remain intact; a separately pinned profile permits the two-host pilot.

A robotless diagnostic measured 1,000 clock samples per simulated second.
An optional transport coalesces original timestamps to at most 100 publications
per wall second, without extrapolating or advancing paused time. This reduces
clock traffic but has not established a reliability improvement. The latest
slow-simulation profile also preserves the original timestamps and thresholds.

## Declared setup

The actual 4 GB Pi 4B runs AMCL/EKF, osmAG preprocessing/routing, rolling-window
management and native Nav2 in domain 218. Ubuntu runs headless Gazebo, simulated
sensors and an independent observer. The Pi receives no ground-truth input.
Only `/phase9/sim_cmd_vel` controls the simulator; no physical robot driver runs.

The user reports a **MacBook 30 W USB-C adapter and USB-C cable**, plus a **small
aluminum CPU heatsink**. Fan presence and ambient temperature are unreported.
Negotiated voltage/current and Pi power consumption have not been measured.
Wi-Fi power saving was on in pilots 02–07 and verified off before 08–09.
Software versions differ slightly between hosts; this is not a matched
Pi-versus-VM hardware comparison.

## Latest bounded trials

The fixture requests a 1.5 m route across passage `-93`, with the unchanged
0.1 m/s command limit. Both trials below use the 100 Hz wall-clock transport cap
and power saving off. Target real-time factor is a configuration, not a measured
claim that the simulator maintains that rate.

| Trial | Target physics RTF | Outcome | Independent recorded displacement |
| --- | --- | --- | --- |
| 08 | 1.0 | Goal accepted; stopped on stale robot TF | 0.013 m |
| 09 | 0.2 | Goals accepted; stopped on pose-observation gap | 0.525 m |

Trial 09 last admitted a pose at simulated time 19.703; the next recorded TF
candidate was 19.860. Their **0.157 s observation gap** exceeds the 0.15 s crossing
monitor limit. This is a gap between observations, distinct from TF age relative
to the current clock. The admission/crossing guards were retained rather than
weakened to obtain a pass. Independent mission scoring rejects both trials.

The observer records sampled rest after both stops, with no classified wall
contacts and all 13 contact publishers present. These observations do not turn
failed missions into successful ones. Simulator-local scan delivery was prompt
in the earlier retained comparisons; the cross-host delivery/receiver scheduling
contribution has not been isolated. Disabling power saving and slowing physics
were sequential diagnostics, not randomized evidence of causality.

Pi preprocessing takes 51.59 s in trial 08 and 53.26 s in trial 09; this is full
planner startup/preprocessing, not online route-query latency. Maximum sampled
domain-process PSS across the **entire attempts**, including startup and
preprocessing, is 315,231 and 318,996 KiB respectively. Maximum sampled CPU
temperature is 55.504°C and 65.244°C. Approximately one-second sampling can miss
peaks, and failed trials do not establish a successful-motion or sustained
resource budget. The post-trial firmware query reports `throttled=0x0`.

## Evidence and verification

- [Physical setup](../../../../simulation/gcu_ground_floor/phase9_diagnostics/physical_setup.json)
- [Derived failed-trial summary with input hashes](../../../../simulation/gcu_ground_floor/phase9_diagnostics/latest_pilot_summary.json)
- [Trial 08](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_08/)
- [Trial 09](../../../../simulation/gcu_ground_floor/phase9_diagnostics/pilot_09/)
- [Receipt regression](../../../../simulation/gcu_ground_floor/phase9_diagnostics/preflight_receipt_regression.json)
- [Investigation state](../../../../simulation/gcu_ground_floor/phase9_diagnostics/investigation_20261117.json)
- [Harness and scope](../../../../experiments/pi_baseline/README.md)

Source/configuration archives and per-host manifests pin the executed trials.
The derivation is reproducible from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  experiments/pi_baseline/summarize_pilots.py \
  --evidence simulation/gcu_ground_floor/phase9_diagnostics
```

The full unittest suite passes **192 tests**. Both role owners completed cleanup;
a subsequent process audit found no domain-217/218 experiment processes on
either host. Ubuntu's desktop and the Pi remain available. No extra live retry
is left running.

## What remains together in Phase 09

1. Isolate local compute timing from Wi-Fi by fixed-rate local replay, then
   obtain an independently scored complete closed-loop mission under a declared
   time/network profile. Replay is not a substitute for that mission.
2. Freeze representative tasks, repetitions and input-rate conditions; measure
   reliability, CPU/PSS, latency, thermal behavior and network overhead before
   defining camera/language headroom. Do not infer capacity from 0.2× simulation.

This checkpoint documents progress and negative evidence. It does not close
Phase 09 or justify a Pi real-time performance claim in the paper.
