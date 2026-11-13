# Required executor IMU input loss

One native trial passes independent replay. A ROS relay forwards unchanged real
IMU messages to the executor, then stops forwarding while the robot is moving.
The EKF keeps its original real IMU stream. The executor's unchanged 0.15 s
freshness limit triggers a watchdog cancellation; physical rest begins 0.432
simulated seconds after the injected loss. No next segment is dispatched.
This tests the executor input contract, not simultaneous physical sensor/EKF loss.

The raw archive retains executed sources/assets and input/relay samples. The
independent verifier checks real source continuation, forwarded/accepted sample
stamps, stale age, native cancellation, rest and geometry. Both publishers remain
present. Native exits and cleanup pass. See verification.json and cleanup.json.

Independent replay tools and actual sources are frozen in the verification and
source archives. See the main Phase 06 record for commands and limitations.
