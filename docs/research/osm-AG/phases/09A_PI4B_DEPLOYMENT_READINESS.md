# Phase 09 readiness checkpoint: actual Pi and ROS link

## Quick summary

- **Verified:** 4 GB Pi 4B inventory, planner dependencies, five native planning cases and bidirectional ROS messages with Ubuntu.
- **Evidence:** All native processes exit 0; offline replay passes; final link probe echoes 60/60 messages per host. Earlier failed attempts are retained.
- **Open:** Wi-Fi latency spikes, unverified cooling/power details, and two-host motion integration. No navigation performance or physical-motion result yet.
- **Next:** Establish the benchmark network profile, then one short split-host mission and repeated Phase 09 measurements. Phase 09 remains in progress.

## Setup and changes — November 16, 2026

The Pi at `pi-project@192.168.100.129` is a Raspberry Pi 4 Model B Rev 1.4 with
approximately 4 GB RAM, Ubuntu 22.04.5/aarch64, Python 3.10.12 and ROS 2 Humble.
Its current connection is Wi-Fi; Ethernet initially has no carrier. At inspection
there were no running robot workloads. Power supply and physical cooling details
have not been verified.

The user enabled SSH public-key login and installed the requested Cyclone DDS,
SciPy, Shapely and pyproj packages with administrator authentication. Automation
can log in as the user, but it does not know the administrator password. The
existing `/home/pi-project/lidarbot_ws` was not modified. Test files live in
`/home/pi-project/edgeosmag_phase9_readiness_ws`.

The simulator remains in the Mac's Ubuntu UTM VM. Its original shared-network
address `192.168.64.2` received no ping replies from the Pi. The VM was shut down
cleanly, its existing adapter changed to bridged mode on `en0`, and restarted.
Its new address is `192.168.100.137`; three subsequent pings from the Pi receive
replies. RAM, CPU, disks and graphics settings were not changed. The original
adapter settings and applied change are saved in the evidence directory.

Pi Nav2 planner/controller packages are **1.1.20**, while the VM's are **1.1.19**.
Both have Humble and Cyclone DDS RMW 1.3.5. Preserve these inventories: this setup
does not constitute a controlled Pi-versus-VM hardware speed comparison. Later
same-Pi variants must use a matched software profile.

## Native planning on the Pi

The existing `prepare_gcu_rolling_native.py` fixture was generated locally and
copied to the isolated Pi workspace. It supplies OSM geometry and synthetic TF;
no Gazebo, AMCL, hardware driver or motion controller runs in this fixture.

The retained trial passes five window/constraint revisions, including three
distinct initial origins, passage closure rejection and successful reopening.
The global grid remains 128 × 128. Two native costmaps consume the window source;
there are zero full-map subscribers and zero velocity-command publishers. All
three test processes exit 0 and both lifecycle nodes return to `unconfigured`.
The existing offline verifier independently rechecks source hashes, recorded
arrays, mask consumption and swept-path clearance.

This establishes native package/geometry compatibility in that fixture. It does
not establish localization, motion, sustained controller timing or resource use
under the complete navigation workload.

## Cross-host ROS probe

`scripts/check_gcu_pi_link.py` publishes only namespaced String ping/echo topics
in domain **217**, with no goals, TF or velocity messages. Each host sends 60
messages at a requested 10 Hz with a 2,048-character payload. Round-trip duration
uses the sender's monotonic clock, including application scheduling and the echo
callback. It is not a one-way network-delay measurement or a sensor-rate benchmark.

| Attempt | Outcome | Pi echo RTT median / reported p95 | VM echo RTT median / reported p95 |
| --- | --- | ---: | ---: |
| 01 | VM ROS logging initialization fails; Pi times out discovering peer | No samples | No samples |
| 02 | Both hosts echo 59/60 messages; strict delivery gate fails | 334.4 / 1,262.1 ms | 338.7 / 1,186.1 ms |
| 03 | Both hosts echo 60/60 messages and receive all 60 peer messages | 41.4 / 733.9 ms | 41.7 / 588.1 ms |

Attempt 01 is corrected by an explicit `ROS_LOG_DIR` for the guest-agent process,
without overriding `HOME`. Attempt 02 uses automatic participant indices up to
100. Attempt 03 uses one fixed participant per host and explicit discovery ports,
reducing discovery fan-out for this two-node diagnostic. These are short diagnostic
attempts, not repetitions sufficient to attribute the latency difference to the
profile change. Wi-Fi conditions may differ between attempts.

The probe's historical p95 field uses sorted RTT rank
`max(1, floor(0.95 * sample_count))`, retained unchanged in its frozen source.
Wi-Fi power saving was on and the observed link was 2.4 GHz. These observations
identify possible contributors, not a demonstrated cause. Successful message
delivery does not establish acceptable navigation timing. Ethernet is preferred
for the next characterization, or explicitly characterize the Wi-Fi limitation.

The fixed-index profile is **link-probe only**: multiple navigation processes on
one machine require distinct participant indices/ports. The accepted motion runner
also enforces loopback isolation. Neither profile nor guard has been bypassed to
claim a two-host mission.

## Evidence and verification

Evidence is under [phase9_diagnostics](../../../../simulation/gcu_ground_floor/phase9_diagnostics/):
pre/post Pi inventories, simulator inventory, XML profiles, all probe results,
bootstrap notes, Wi-Fi observations, exact source archive, native logs/arrays and
the native fixture archive. `readiness_verification.json` explicitly states
`native_motion_tested: false` and `phase09_complete: false`.

Final scoped audits find no remaining test processes in domains 197/217 on
either host. No simulator physics or physical-robot motion was launched in this
checkpoint. The Ubuntu desktop is left running for subsequent work.

Recheck native evidence from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 docs/research/osm-AG/osm_cad_pipeline/osmAG-from-cad/venv/bin/python \
  scripts/verify_gcu_rolling_native.py \
  --evidence simulation/gcu_ground_floor/phase9_diagnostics/native_plan_01 --trial 01
```

The collector is read-only:

```bash
ssh -i ~/.ssh/id_rsa_localhost_run pi-project@192.168.100.129 \
  'python3 /tmp/edgeosmag_phase9_inventory.py'
```

For a new link probe, use the saved per-host XML with current addresses,
`ROS_DOMAIN_ID=217`, `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
and `CYCLONEDDS_URI=file://<absolute-profile-path>`. Start both hosts with the same
new `--session` and separate new `--output` paths. Run
`check_gcu_pi_link.py --role pi` on the Pi and `--role simulator` in Ubuntu.
The source has a bounded discovery/stream/echo interval and exits automatically.

To restore UTM's original network, gracefully stop Ubuntu, change adapter index 0
from bridged/en0 to shared mode with an empty host interface, then restart. The
guest address will change; update profiles and Foxglove accordingly. Network mode
definitions and the stopped-VM configuration API are documented by
[UTM](https://docs.getutm.app/settings-qemu/devices/network/network/) and its
[scripting guide](https://docs.getutm.app/scripting/cheat-sheet/).

## What remains

1. Finish the network/timing profile and separate simulator/evaluator processes
   from the Pi's actual navigation workload, retaining explicit simulation guards.
2. Run and independently score one short closed-loop GCU mission, then freeze and
   repeat declared tasks while measuring resources, timing and thermal behavior.

These are the remaining work in the existing Phase 09 plan, not new project phases.
For the paper, this checkpoint supports deployment-readiness statements only.
