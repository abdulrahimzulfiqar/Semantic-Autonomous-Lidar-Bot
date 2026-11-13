# Matched fixed-mask single-goal comparator: corridor to room

One native query passes independent replay with exactly one final Nav2 goal,
the same endpoints, selected scenario mask and native profile as the frozen
three-goal segmented reference. Arrival/rest, topology and contacts pass.
See ../matched_comparison.json. The native request count differs; the source
raster and controller/localization/sensor parameters do not.

This is an execution-only comparator with shared osmAG-derived scenario setup,
not a pure grid-planning/resource benchmark or statistical speed claim.
Native exits and cleanup pass. Actual executed source and assets are archived.

Independent replay tools and actual sources are frozen in the verification and
source archives. See the main Phase 06 record for commands and limitations.
