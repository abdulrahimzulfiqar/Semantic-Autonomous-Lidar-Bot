# Final live interface acceptance

**Automated pass; human visual review pending.** `attempt05.tar.gz` contains the
passing final trial: ten interface checks, three verified arrivals, safe cancel,
inactive-navigator rejection/recovery, no automatic dispatch, invalid/busy goal
rejection and all-passages-closed no-route rejection. All six native groups exit
0; the checker log is empty. `verification.json` independently checks raw motion,
source hashes, cancellations, zero-command rejection and cleanup. Intentional
cancel/rejected sub-missions are expected fault cases, not hidden arrival failures.

Earlier failed attempts remain: `attempt01_02.tar.gz` holds their actual older
sources; `attempt03.tar.gz` also includes copied historical 01/02 outputs, so use
the first archive for those older source versions. `attempt04.tar.gz` retains a
driver assertion typo failure despite a clean runtime. Final trial 05 is the
complete acceptance run after the documented fixes. The source and verifier
snapshots are separate. `cleanup.json` records the empty automated environment
before starting the intentional bounded review session.

[Review instructions](../../../../../docs/research/osm-AG/plans/06_GCU_FINAL_REVIEW.md).

The AMCL launch file is generated from the repository template with admitted
IMU yaw-rate fusion parameters. The manifest records both template and executed
workspace hashes; runtime-source verification checks the executed version.
