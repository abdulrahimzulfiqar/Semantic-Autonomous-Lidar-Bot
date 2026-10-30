# Source provenance

This directory vendors the source of [osmAG-from-cad](https://github.com/jiajiezhang7/osmAG-from-cad)
at upstream commit `df6902a288e94df815b8d9f661ce1b97ef919ba0`, with local changes for the CAD-to-osmAG workflow.
It is stored as source files in this repository, not as a Git submodule.

Local changes include modern build compatibility, reproducible configuration,
graph-cut memory/order fixes, intermediate diagnostics, occupancy-seeded region
reconstruction, hole-aware export and regression tests. See ../GCU_TUNING.md.

Original license notices remain in their source files and component directories:
`area_graph_segment/LICENSE` (GNU GPL v3) and `cad2osm/gui/LICENSE` (MIT).
This provenance note does not replace those licenses.

Private CAD inputs, derived maps, Python environments, build products and local
run artifacts are excluded. GCU integration tests skip when the local input is
unavailable; synthetic geometry/component tests remain runnable.
