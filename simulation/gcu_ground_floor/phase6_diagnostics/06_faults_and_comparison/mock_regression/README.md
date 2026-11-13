# Fresh final ROS mock controls

Ten controls pass with zero velocity commands, no simulator/controller and no
real navigation status. Raw trial/source hashes are independently checked in
`verification.json`; `cleanup.json` confirms no isolated test processes remain.
These controls cover state/transport rejection handling, not physical fault
response. The live interface and input-loss groups provide separate motion proof.
