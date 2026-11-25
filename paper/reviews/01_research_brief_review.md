# Review 01: research brief

## Quick summary

**Current outcome: working research direction recorded; navigation-only Pi baseline comes first.** The user clarified this priority after the initial review. The earlier LLM task-definition exercise is deferred and does not block experimentation. Your original answers remain in the [draft](../drafts/01_research_brief.md). No section is being promoted into the manuscript yet.

Reviewed: **2026-11-16**. This review checks the planning brief, the current project scope and the primary-source passages linked below; it is not a new experimental evaluation.

## What your answers already establish

You want to investigate navigation and onboard language processing on a Pi 4B with 2D LiDAR. You distinguish a physical house experiment from GCU simulation with computation on real Pi hardware. Camera features depend on available capacity. These are useful scope decisions and are retained in the suggested revision.

## Corrections and suggestions

| Your point | Review and proposed correction |
| --- | --- |
| The authors used an Intel NUC and 3D LiDAR | Correct for the platform described in osmAG-Nav §8.1: Core i7-1165G7 and Hesai PandarQT 64-line LiDAR. Attribute this to that experiment rather than saying these are the authors' only tested platforms. [Source](https://arxiv.org/html/2603.28271v1#S8.SS1) |
| The core navigation experiment was separate from LLM use | This distinction does not establish a literature gap by itself. Related work includes [LLM as Copilot](https://arxiv.org/abs/2409.08493) and [osmAG-LLM object navigation](https://arxiv.org/abs/2507.12753). Their existence rules out “adding an LLM to osmAG” as a sufficient novelty claim. Their edge-compute implications still require full-text review. |
| We will run osmAG-Nav with 2D LiDAR | Our current system is an osmAG-guided adaptation using AMCL. Switching localization is a substantive difference from the paper's structural framework. Describe the implemented system precisely; do not call it the identical full stack with a cheaper sensor. See the [localization decision](../../docs/research/osm-AG/phases/08_GCU_LOCALIZATION_COMPARISON.md). |
| Room commands, blocked corridors, object search and door checks | These are several tasks with different evaluation needs. Start with named-room language commands. Then consider user-reported blockage handling. Keep camera/object search conditional. Check a door before attempting to cross it; checking after entering the room cannot establish that entry was possible. A door-state task needs its own perception evidence. |
| Comparison is undecided | First compare the same navigation stack and matched missions with no language inference versus onboard language inference. Then compare the LLM against a simple command parser on a declared language test set. This separates inference overhead from language-task value. Include optional camera variants later. |
| Success is that the whole stack runs and the LLM is fast enough | Feasibility is a useful first milestone. A paper also needs repeated task success, command interpretation/validity, response latency, controller deadline behavior, CPU, memory and thermal measurements. Define acceptable budgets before confirmatory trials. Do not select speed or accuracy thresholds merely because a tested model happens to meet them. |

The phone/PC can be an input interface while inference runs on the Pi. If inference instead runs on the phone, PC or a service, report it as offboard. “All onboard” remains the intended hypothesis until measured. The LLM can propose a goal or map constraint; deterministic validation and the navigation planner should determine admissible goals/routes. A claimed blockage does not create an alternate route if the map has none.

## Suggested revised brief

This is a proposed planning revision based on your answers, not a measured result or a finalized contribution statement:

> We plan to investigate whether a Raspberry Pi 4B can run osmAG-guided navigation with 2D LiDAR while supporting an onboard language model. Our current navigation system uses AMCL, segmented execution and rolling planning windows. The initial task will be navigating to a named room from a natural-language command. We propose comparing matched navigation missions with and without language inference, and evaluating command interpretation against a simple parser. We will measure task success, response latency, control deadlines, CPU use, memory and thermal behavior. Evaluation is planned in GCU simulation with navigation computation on the Pi, followed by physical house trials. Camera-based tasks will be considered if the measured resource budget permits. The research contribution will depend on literature review and whether a proposed design produces a measurable benefit.

The comparison is my suggested starting design; its configurations, repetitions and thresholds are still to be specified. Blockage reasoning remains a possible second language task. Object search and door-state detection remain possible camera extensions, not conditions for completing this first brief.

## Original revision exercise — now deferred

Write **one sentence** completing:

**“For our first experiment, the LLM will ___ from the user's command, and the navigation system will ___.”**

For example, name the information it should produce and who checks/executes it. If named-room commands are not the task you want first, use your preferred single task instead. Send just that sentence or add it below; no full rewrite or new experiments are needed now.

**Your sentence:** _Write here._

This exercise is retained for later language-task planning. It is no longer the current assignment.

## Scope update: navigation baseline first

On 2026-11-16 the user proposed testing the established navigation system on the Pi while running GCU simulation on the laptop. This is the agreed immediate direction, consistent with the existing Phase 09 plan. It gives us a resource and reliability baseline before introducing additional workloads.

The first research question is: **Under declared GCU simulation tasks and sensor rates, what navigation reliability, timing and resource use does the existing osmAG-guided stack achieve on the Pi 4B?** This is a baseline characterization question, not yet a claim of superiority or a complete paper contribution.

Use direct goals initially. Keep AMCL, EKF, routing, rolling planning windows and Nav2 behavior fixed. Record offboard simulator, bridge and evaluator placement, network effects, and any offboard preprocessing. After compatibility and pilot runs, freeze the repeated trial protocol. Language and optional camera experiments follow the measured baseline.

**Next action:** confirm the actual Pi's hardware/software inventory. Writing continues with setup notes and protocol preparation. Neither LLM task definition nor a completed introduction is required to proceed.
