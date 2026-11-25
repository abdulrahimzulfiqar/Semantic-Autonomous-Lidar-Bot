# Phase 09 experiment evidence

The [Phase 09 completion record](../../../docs/research/osm-AG/phases/09_PI4B_NAVIGATION_BASELINE.md)
summarizes the accepted Raspberry Pi 4B corridor baseline. Trials `pilot_76`
through `pilot_78` contain the independent scores, scan comparisons, window
timing and resource samples for the three consecutive passes.

The numbered pilots and local replays preserve earlier results, including
failed and incomplete trials. The [diagnostic record](../../../docs/research/osm-AG/phases/09E_PI4B_FULL_STACK_REPLAY.md)
describes the fixed-rate replay and the main timing failures. Follow its links
to find the evidence for individual claims.

Extracted result files are the retained copies used by the scorers and cited
in the paper. A duplicate result archive was removed only after every file
inside matched its extracted copy. Historical checkpoint manifests list those
archives under `retired_result_archives` with the original checksum and the
directory containing the retained results. Paths use the same base as each
manifest's `files_sha256` entries. Empty files with a recorded checksum are
kept with their checkpoint. Source archives preserve the exact staged code and
configuration, and compressed sensor captures preserve the replay inputs.
See the [experiment harness](../../../experiments/pi_baseline/README.md) for
staging and scoring commands.
