# Phase 09 diagnostic checkpoint: Pi-local AMCL replay

## Quick summary

- **Scope:** Recorded scans/odometry replayed locally on Pi at 1×; AMCL/TF only.
- **Result:** All 552 scored scans arrive in three complete runs; two pass timing, one fails. Source versions differ.
- **Finding:** Timing gaps occur without Wi-Fi sensor delivery. AMCL averages 9–11% of one core in these short runs.
- **Remaining:** Trace full-stack clock/callback timing, then complete a closed-loop Pi mission and repeated measurements. Phase 09 stays open.

## Purpose and input isolation

The previous [split-host checkpoint](09B_PI4B_SPLIT_HOST_DIAGNOSTICS.md) leaves
transport and Pi scheduling entangled. This diagnostic removes the external
simulator and Wi-Fi sensor path. The Pi runs only AMCL, its map server/lifecycle
manager, a local replay publisher and a separate timing observer. No goals,
velocity messages, physical drivers or Nav2 planning/control servers are started.
Domain 218 is restricted to localhost, using the repository's existing local DDS
profile pattern. No network credentials or operating-system settings change.

The source is the verified Phase 08 clean recording inside
`phase8_diagnostics/clean_tracking/attempt01.tar.gz`. The preparer extracts only
`observations.npz`, AMCL configuration and map files. Evaluator truth and old AMCL
poses are not extracted. The input recording's SHA-256 is
`e9754ca4675dcf7944ba426ff54ee5041841ed80b4fc3d7997f9221cc19615e9`.

This is an **SE(2) reconstruction**, not replay of every original ROS message:
original scan ranges/timestamps are preserved; recorded odometry is interpolated
without extrapolation at nominal 50 simulated Hz. A generated replay clock
advances at 1× wall time, published at nominal 100 simulated Hz. The planar laser
extrinsic and known spawn prior are supplied. The first 1.0 simulated second is
an explicit input warmup, allowing AMCL's future-dated map transform and odometry
history to compose. The remaining **55.1 simulated seconds / 552 scans** are
scored. No source observation is relabelled with a fresh timestamp.

## Retained attempts

| Local replay | Change / outcome |
| --- | --- |
| 01 | Harness startup error: explicit loopback interface duplicates `ROS_LOCALHOST_ONLY` configuration. All native nodes abort before a usable replay. Corrected using the existing local profile pattern. |
| 02 | Harness startup timeout: clock held at the first frame cannot establish the complete initial TF chain. Replaced with the declared recorded-input warmup. |
| 03 | Complete 1× replay passes timing gates. Replay publisher still repeatedly decompresses NPZ arrays. |
| 04 | Arrays cached once; complete 1× replay still has an intermittent clock/pose-observation gap. Fails timing gates. |
| 05 | Same cached input plus exact clock publication tracing; complete 1× replay passes. The previous intermittent failure is not declared fixed. |

| Measured during scored interval | 03 | 04 | 05 |
| --- | ---: | ---: | ---: |
| Expected / received unique scans | 552 / 552 | 552 / 552 | 552 / 552 |
| TF queries / failed queries | 1,378 / 0 | 1,375 / 0 | 1,377 / 0 |
| Maximum TF age, simulated ms | 19.98 | 45.38 | 23.21 |
| Maximum pose-observation gap, simulated ms | 104.28 | **171.19** | 68.60 |
| Maximum clock receipt gap, wall ms | 79.91 | **185.80** | 52.01 |
| Maximum scan publication lateness, wall ms | 90.65 | 10.32 | 13.65 |
| AMCL sampled CPU, % of one core | 9.11 | 10.80 | 10.26 |
| AMCL maximum sampled PSS, KiB | 32,141 | 32,220 | 32,172 |

The existing pose-observation/TF-age threshold is 150 ms; replay publication
lateness is bounded at 100 wall ms. Independent analysis also checks that clock
receipts cover the recording, do not reverse, and have no gap over 150 wall ms
at 1×. All three completed runs preserve the declared duration and deliver all
expected scans. Each reports 112 AMCL pose updates. No localization-accuracy or
mission-success result is inferred from these timing checks.

The optimized replay generator itself consumes about 51% of one core, and the
Python timing observer about 58–59%; these are listed separately from AMCL.
They are diagnostic overhead, not physical robot sensor drivers or the full
navigation workload. Process CPU is cumulative CPU-time delta divided by the
actual first-to-last in-interval sample duration, with **100% meaning one core**.
PSS/RSS are approximately one-second samples and may miss peaks. These short,
sequential, evolving diagnostics do not establish sustained headroom for an LLM
or camera workload or a production reliability rate.

## What the clock trace establishes

Run 05 records 4,965 clock publications; 4,955 original timestamp values are
observed in the explicit clock callback after replay begins. Depth-one,
best-effort subscriptions may coalesce/skip samples: these counts alone are not
evidence of network packet loss.

The largest receipt gap is 52.01 wall ms. During it, four source publication
calls return; the largest source publication-return gap over the run is 18.38 ms.
This local trace distinguishes an ongoing source from a stopped source during
that particular gap. It does **not** separate middleware delivery from Python
callback scheduling, explain run 04's larger gap retroactively, or establish
wire latency. The runs give a reason to instrument the actual executor's clock,
sensor and callback path before further network tuning.

## Verification and reproduction

All **198 tests pass**, including rejection of time reversal/extrapolation,
moving extrinsics, lost scans, missing/stale TF, observation gaps, incorrect
replay pacing and a frozen clock masquerading as fresh historical TF. The
independent analyzer validates the executed input/source hashes and derives the
expected scan interval from the frozen recording, rather than trusting only the
publisher's reported counts.

Raw evidence and pinned source archives are retained in
[local_replay_03](../../../../simulation/gcu_ground_floor/phase9_diagnostics/local_replay_03/),
[local_replay_04](../../../../simulation/gcu_ground_floor/phase9_diagnostics/local_replay_04/) and
[local_replay_05](../../../../simulation/gcu_ground_floor/phase9_diagnostics/local_replay_05/).
Failed startup attempts 01/02 are retained alongside them. Recompute each summary
from the repository root, changing only the attempt directory:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  experiments/pi_baseline/analyze_local_replay.py \
  --evidence simulation/gcu_ground_floor/phase9_diagnostics/local_replay_05
```

[Harness instructions](../../../../experiments/pi_baseline/README.md) describe
preparing a fresh stage and running another bounded local replay. Reusing an
existing evidence or remote workspace path is rejected. The experiment owner
stops its children and retains exit codes for every attempt.

The final [process audit](../../../../simulation/gcu_ground_floor/phase9_diagnostics/local_replay_process_audit.json)
finds no experiment processes on either host. Pi Wi-Fi power saving remains off;
the post-test firmware query reports `throttled=0x0`. The desktop and operating
systems remain running; this diagnostic did not start Gazebo.

## Remaining Phase 09 work

1. Instrument full-stack clock receipt and executor callback timing, then obtain
   an independently scored complete closed-loop Pi mission under a declared
   network/time profile. Keep existing input and geometry guards intact.
2. Freeze repeated tasks and measure the actual full-stack reliability/resource
   budget before camera/language additions. Local AMCL replay is a diagnostic,
   not a substitute for this acceptance gate.
