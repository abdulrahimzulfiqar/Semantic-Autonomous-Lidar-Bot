# Matched fixed-mask single-goal comparator: adjacent sectors

One native query passes independent replay with exactly one final Nav2 goal.
Endpoints, original raster, selected permitted-area mask, controller, body,
localization and sensor configuration match the frozen segmented reference.
See ../matched_comparison.json for actual metrics and profile equality checks.

This is an execution-only comparator: both variants use the same osmAG-derived
scenario setup. It is not an end-to-end grid-only planning or resource benchmark.
One baseline run per query does not establish a statistical performance benefit.
Native exits and cleanup pass; actual source/parameter snapshots remain frozen.

Independent replay tools and actual sources are frozen in the verification and
source archives. See the main Phase 06 record for commands and limitations.
