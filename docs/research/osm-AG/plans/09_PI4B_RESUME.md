# Phase 09 resume point — November 16, 2026

**Phase 09 is complete for the measured corridor baseline.** Trials 76–78
passed consecutively with 10 Hz global costmap updates and publication. The
[completion record](../phases/09_PI4B_NAVIGATION_BASELINE.md) lists the measured
rates, acceptance results and workload for Phase 10. The notes below retain
the earlier diagnostic checkpoints.

**November 25 diagnostic checkpoint.** The Pi completed two 1× local replays.
On the hotspot (`192.168.43.194`), the first higher-rate corridor run passed
independent scoring at 0.695× measured simulation rate. The second stopped on
a stale pose lookup during a window update. After the update was changed to
defer when TF was stale, pilot 67 passed at 0.633×. Pilot 68 failed before
dispatch and pilot 69 stayed staged. On `pixel` (`10.76.115.30`), pilot 70
crossed the passage and then cancelled when TF was 180 ms old. With power
saving off on the same 2.4 GHz channel, pilot 73 crossed the passage and then
cancelled when the old native global-costmap acknowledgement exceeded its
500 ms lease before the new revision was acknowledged. For the next retest,
measure the native grid update and publication cadence during a window shift
while preserving the existing scoring limits. See the
[latest test record](../phases/09E_PI4B_FULL_STACK_REPLAY.md#pixel-hotspot-follow-up).
The record below describes the November 16 checkpoint.

## Quick summary

- **Paused by the user:** all experiment jobs stopped; Ubuntu VM shut down. Pi remains powered on with SSH available.
- **Verified:** actual Pi inventory, offline osmAG route, five native plan-only cases, and a small bidirectional ROS probe. 185 tests pass.
- **Not achieved:** a successful split-host navigation mission or a repeatable Pi performance baseline. No pilot dispatched a navigation goal.
- **Next:** establish reliable cross-host delivery, then retry one short pilot with unchanged freshness limits before collecting benchmark trials.

## Saved work

The actual Pi is a 4 GB Pi 4B with Ubuntu 22.04/Humble. Its last SSH address is
`pi-project@192.168.100.129`. The Ubuntu VM is
`8AC561B9-3616-43CC-A7DF-2BE570294415`; it now uses an `en0` LAN bridge and its last
address is `192.168.100.137`. Both addresses may change on another Wi-Fi network.

The separate implementation is in [experiments/pi_baseline](../../../../experiments/pi_baseline/README.md).
Accepted Phase 07/08 workspaces and the original loopback-only executor remain
unchanged. The split harness derives a private guard, uses domain 218 and only
`/phase9/sim_cmd_vel`, and leaves ground-truth scoring on the laptop.

All attempts are retained under [phase9_diagnostics](../../../../simulation/gcu_ground_floor/phase9_diagnostics/):

| Attempt | Recorded outcome |
| --- | --- |
| Readiness/native | Five plan-only cases pass and independently replay; no velocity publishers |
| Offline route | Intended passage `-93` found on Pi; initial preprocessing approximately 50.44 s, not online query latency |
| Pilot 02 | Ubuntu at GDM; renderer never started. Pi correctly times out with no clock/scans. Early host cleanup bug repaired; raw results retrieved separately |
| Pilot 03 | Localization initially ready; graph preprocessing 52.72 s; native preflight then times out before dispatch. Logs show stale scans/timestamp drops. All native processes stop; complete data recollected after SFTP timeout |
| Pilot 04 | Simulator records 365 scans locally; Pi receives zero clock/scans within 45 s. No actor or navigation goal starts; shutdown completes |

The two failed live pilots do not isolate a single cause: network conditions,
Wi-Fi power saving, DDS discovery/delivery and scheduling still need investigation.
Wi-Fi as a method is not ruled out, and no claim is made that the Pi cannot run
the stack. Do not loosen freshness or geometry gates to obtain a pass.

The scorer was checked using an explicitly adapted **old Phase 07 reference in a
temporary test fixture**, including injected-contact rejection. That self-check is
not a new Pi navigation result. Pilot resource samples are retained but are not a
sustained navigation benchmark or a confirmed headroom estimate.

## Resume tomorrow

1. Start Ubuntu in UTM, log into its desktop and leave it open. Confirm Pi/VM
   addresses and the chosen network. The user offered another Wi-Fi network;
   switch both hosts locally if desired and provide the new Pi SSH address.
2. Recheck discovery plus clock/scan delivery under the actual multi-process
   profile. A small two-node ping probe is insufficient. Compare reception on
   both hosts and separate transport delays from Pi preprocessing/compute.
3. Use a **new** stage, remote workspace pair and evidence path for the next
   bounded pilot. Never overwrite `pilot_02`, `pilot_03` or `pilot_04`. The preparer
   can re-export the untouched VM baseline at
   `/home/ubuntu/edgeosmag_phase8_amcl_baseline_ws_01` if local `/tmp` stages vanish.
4. Only after independent motion scoring passes, freeze repeated trials and
   report CPU/PSS/RSS, thermal, timing, success and failure distributions with
   simulation rate and network conditions explicit.

All source/configuration and evidence are saved locally in the repository. No
commit, push, merge or Phase 09 completion has been performed for this checkpoint.
