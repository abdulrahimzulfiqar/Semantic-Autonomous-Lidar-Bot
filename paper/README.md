# EdgeOsmAG paper-writing workspace

## Quick summary

Codex maintains the [working manuscript](manuscript/EDGEOSMAG_DRAFT.md) alongside the engineering work. At the end of the project, the authors can use it as a complete reference when writing and reviewing their own paper. This folder contains the manuscript, its references, research notes and experiment protocols. Engineering records and raw experiments are linked from their existing locations.

**The Phase 09 corridor baseline is complete.** The Pi passed three consecutive measured closed-loop missions at 0.669–0.717× and completed two eight-second 1× stationary-input replays. Phase 10 adds the camera workload against this baseline. [Phase 09 results](../docs/research/osm-AG/phases/09_PI4B_NAVIGATION_BASELINE.md)

## Manuscript status

The [Phase 09 readiness checkpoint](../docs/research/osm-AG/phases/09A_PI4B_DEPLOYMENT_READINESS.md) records the Pi inventory, a passing plan-only test and cross-host ROS connectivity. The slow corridor provides functional and sampled resource evidence. Gazebo and independent evaluation run on the laptop, and navigation computation runs on the Pi.

The current draft includes an abstract, related work, the architecture, experimental methods, the measured navigation results and their limitations. Update the Pi results after the independently scored closed-loop batch. Add camera and language methods and results as those experiments are completed, then revise the abstract and conclusion around the full evaluation. [Writing roadmap](ROADMAP.md) · [Review workflow](REVIEW_WORKFLOW.md)

## Where everything goes

| File or folder | Purpose |
| --- | --- |
| [ROADMAP.md](ROADMAP.md) | Writing stages, outputs and completion conditions |
| [REVIEW_WORKFLOW.md](REVIEW_WORKFLOW.md) | How a draft becomes part of the working manuscript |
| [drafts/](drafts/01_research_brief.md) | Research brief and early notes |
| [manuscript/EDGEOSMAG_DRAFT.md](manuscript/EDGEOSMAG_DRAFT.md) | Editable full-paper draft |
| [manuscript/OUTLINE.md](manuscript/OUTLINE.md) | Section status and remaining evidence |
| [reviews/REVIEW_LOG.md](reviews/REVIEW_LOG.md) | Review decisions, changes and unresolved questions |
| [evidence/CLAIM_LEDGER.md](evidence/CLAIM_LEDGER.md) | What our existing evidence supports |
| [evidence/EXPERIMENT_PLAN.md](evidence/EXPERIMENT_PLAN.md) | Framework for collecting the evidence we still need |
| [references/SOURCE_REGISTER.md](references/SOURCE_REGISTER.md) | Primary sources and a reading-card template |

Create additional drafts, reference notes and figures in this folder as needed, keeping original experiment outputs in their engineering locations. We will choose a journal/conference template after the scope is clearer.

## Where the project stands

- **Implemented and tested within a bounded simulation scope:** CAD-to-osmAG processing, routing, segmented navigation and rolling planning windows. See the [phase index](../docs/research/osm-AG/phases/README.md).
- **Current localization decision:** retain AMCL; the separate structural 2D prototype is a preliminary study. See [Phase 08](../docs/research/osm-AG/phases/08_GCU_LOCALIZATION_COMPARISON.md).
- **Next engineering milestone:** Camera perception and semantic observations, followed by language and combined workloads. [Project roadmap](../docs/research/osm-AG/OSMAGNAV_PHASE_ROADMAP.md)
- **Paper direction to investigate:** whether a resource-budgeted osmAG-guided system can preserve navigation reliability while sharing constrained hardware with perception and language tasks. This is a candidate research question, not an established novelty or result.

GCU simulation, computation on a real Pi, and physical robot tests in a house will be reported separately. Physical GCU access can support a later study; we will not describe simulation as a deployment in that building.

Last initialized: **2026-11-16**. Update the current step and review log when a writing step is completed.
