# Step 1: explain the research idea

## Quick summary

**Status: reviewed on 2026-11-16; user clarified navigation-only Pi experiments come first.** Your original answers are preserved below. See the [review and scope update](../reviews/01_research_brief_review.md). This is a working planning note, not a manuscript section. LLM task definition is deferred until the baseline is understood.

You do not need citations, polished academic English or a claim of novelty yet. Use “we plan to” for work we have not tested.

## Context you can use

We have bounded GCU simulation navigation evidence. The current system uses osmAG-guided routing, segmented execution, rolling planning windows and AMCL localization. Actual Pi resource measurements and the integrated camera/language evaluation are still ahead. The separate structural-localization prototype is preliminary and is not the deployed localizer. Physical GCU experiments are unavailable for now; a house experiment is an option.

## Your five answers

### 1. What problem do you want to solve?

Explain what becomes difficult when a small robot must navigate while also running perception and language tasks on limited hardware.

**Your answer:**

_The thing we want to solve is test and run osmagnav + llm (and camera if pi handled llm and osmag nav correctly and effeciently) for autonomous navigation on low contrained hardware like pi 4b and 2d lidar instead of 3d. Previously author has only tested osmagnav separate on bigger processor like Intel NUC (Core i7-1165G7) and without llm and used 3d lidar._

### 2. What system and task will you study?

Name the intended robot/computer and one concrete task, such as going to a named room. Separate what exists from what you plan to add.

**Your answer:**

_We will be studying pi 4b and task will be going to different rooms of house and (gcu in simulation but on pi hardware)._

### 3. What approach will you investigate?

Describe how osmAG-guided navigation, camera processing and language assistance might work together. It is fine if the precise new method is undecided.

**Your answer:**

_Approach is not yet decided but will be something like you can ask the llm from your phone or pc to go to a particular room and it will go to that room and you can tell it like this corridor is blocked and it can then reason and detour from another corridor to destination. And if we add camera we can ask it to find something and then it can reason and try to find that thing. and when we ask it to go to a room it after reaching there turns on camera and check if door is open , if not alarm or mission abort. and snd message._

### 4. What comparison would make the study convincing?

Suggest what to hold constant and what to change. For example, compare navigation alone with added workloads on the same Pi and tasks. A reviewer will also need a relevant navigation or scheduling baseline; we will define it later.

**Your answer:**

_Comparsion are undecided , i am open to suggestion._

### 5. What would count as success, and where will you test it?

Name observable outcomes, such as successful arrivals and acceptable control latency within a resource budget. State the intended simulation/Pi/physical setting. Exact thresholds come after baseline measurement and before confirmatory trials.

**Your answer:**

_Succcess will be running the whole stack and llm on the pi with multiple successful initial pose to goal, with llm speed and reasoning enough to perform a task._

## When you finish

The user chose to prioritize navigation-only Pi experiments after this review. Start with the [Phase 09 hardware inventory](../../docs/research/osm-AG/plans/09_PI4B_NAVIGATION_BASELINE.md). The earlier LLM sentence exercise is deferred; it does not block the baseline.
