# Writing review log

## Quick summary

The research brief established the navigation-first evaluation. Codex now maintains the manuscript as a reference draft for the authors, with measured and planned work identified in the text.

| Date | Draft/version | Outcome | Evidence checked | Changes and unresolved issues | Manuscript destination |
| --- | --- | --- | --- | --- | --- |
| 2026-11-16 | Research brief, original answers; see Review 01 | Needs revision | Current localization scope; osmAG-Nav §8.1 and related-work passages; primary LLM-paper records | Correct prior-work claim; proposed comparison and shorter brief supplied; author to define one initial LLM task | None — planning note only |
| 2026-11-16 | User's baseline-first clarification; Review 01 scope update | Working direction recorded; previous LLM assignment deferred | Existing Phase 09 plan and experiment framework | Start with Pi inventory and navigation-only protocol; keep language/camera future and unmeasured | None — planning decision only |
| 2026-11-21 | First consolidated manuscript | Working draft | Primary papers, matched rolling/static comparison, localization records, Pi replay and closed-loop pilot records | Drafted current system and measured findings. Update the baseline after the higher-rate reporting batch and revise the manuscript as later phases produce data. | [Working manuscript](../manuscript/EDGEOSMAG_DRAFT.md) |
| 2026-11-23 | Pi link recheck and higher-rate diagnostics | Recorded negative results and revised methods | Saved ROS probes, independent mission scores, scan hashes, pose and costmap traces, VM and host inventory | Reported complete scan coverage alongside pose and costmap deadline failures. Added the startup interruption and stale-TF retry. The matched reporting batch is still pending. | [Working manuscript](../manuscript/EDGEOSMAG_DRAFT.md) |

Full feedback and the suggested revision are in [Review 01](01_research_brief_review.md). The original-answer text remains in the draft; its status/footer were updated after review.

Reviewed input SHA-256 before those metadata edits: `f643d9878667e13d98b06e8750d1eee615ea2f9b216f4d709be99f5c30dd9f4a`.

## Template for each completed review

Copy this block when reviewing a specific draft:

```text
Date:
Draft path and version/commit (or dated snapshot):
Purpose of this piece:
Implementation/evidence/source sections checked:
What is clear and supported:
Corrections needed:
Outcome: Needs revision / Evidence pending / Ready for working manuscript
Changes made and issues still open:
Manuscript destination, if ready:
One next action:
```

Log a new entry if a previously reviewed claim changes. Preserve the reason for the change so later wording does not quietly lose its evidence boundary.
