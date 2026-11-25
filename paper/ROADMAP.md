# Writing roadmap

## Quick summary

Codex writes and updates the [working manuscript](manuscript/EDGEOSMAG_DRAFT.md) as the experiments advance. The authors can use the complete draft as a reference at the end of the project. The current draft covers the implemented navigation system, its experimental setup and measured outcomes. Update the Pi baseline after the reporting runs, then revise the results, discussion and abstract after evaluating the added camera and language workloads.

The article's final order and our writing order are different. Starting with a clear research question and methods makes the introduction and abstract easier to write later. The eventual article outline follows the usual research components described by the [IEEE Author Center](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/create-the-text-of-your-article/structure-your-article/); the small-step workflow below is our working plan.

| Step | Small deliverable | Ready to move on when… | Status |
| --- | --- | --- | --- |
| 1. Explain the idea | Five short answers in the [research brief](drafts/01_research_brief.md) | Problem, intended system, comparison and success are understandable; planned work is labelled | Working direction recorded; refine after baseline; LLM task deferred |
| 2. Understand the foundations | Verify relevant primary-source sections | We can explain what each source contributes and how our system differs, with page/section references | Navigation and language foundations cited in the draft |
| 3. Define the study | One research question and testable hypotheses | A literature-supported gap and fair baseline are identified; candidate contributions have measurable tests | Baseline question is drafted. Update the integrated question once workloads are fixed. |
| 4. Describe the system | Architecture and methods | Each component is labelled reused, adapted or proposed; another researcher could understand the data flow | Navigation methods drafted |
| 5. Plan validation | Experiment protocol | Inputs, comparisons, metrics, repeats, time bases and failure rules are fixed before the experiment | Pi setup and scoring drafted. Camera and language protocols follow. |
| 6. Write results | Measured tables and interpretation | Values trace to saved runs; all attempts, uncertainty and limitations are accounted for | Pi pilot and replay results drafted. Closed-loop reporting batch is pending. |
| 7. Build the argument | Introduction and related work | Claims match our actual contribution and cited work; closest competing approaches are covered | Initial sections drafted |
| 8. Interpret the findings | Discussion, limitations and conclusion | We explain what results mean, where they do not generalize and what remains future work | Navigation discussion drafted. Combined-workload discussion follows evaluation. |
| 9. Summarize the finished study | Final title and abstract | Every outcome mentioned is supported in the manuscript | Working version drafted. Revise after integrated evaluation. |
| 10. Prepare submission | Whole-paper review and venue formatting | Technical consistency, references, reproducibility, authorship and current venue requirements have been checked with coauthors/advisor | Pending |

Update the manuscript after each engineering checkpoint. Check changed claims against primary sources and retained measurements, and log material revisions. The writing and engineering stages can proceed in parallel. [Review workflow](REVIEW_WORKFLOW.md)

## Write alongside these engineering milestones

| Engineering milestone | Writing we can do alongside it | Evidence needed before its result claims |
| --- | --- | --- |
| Existing GCU simulation | System architecture, routing/map representation, implementation boundaries | Frozen configurations and retained acceptance records; broader comparisons require new controlled trials |
| Phase 09: actual Pi baseline | Hardware/setup and navigation benchmark protocol | Actual Pi resource, timing, thermal and mission measurements; controlled comparison |
| Phase 10: camera perception | Perception method and task-specific evaluation protocol | Detection quality, workload and effect on navigation |
| Phase 11: language-guided tasks | Command interpretation, validation and scheduling method | Task success, latency, invalid/ambiguous-command handling and navigation impact; local/offboard compute stated |
| Phase 12: combined evaluation | Main results, ablations, discussion and final scope | Repeated comparisons under matched workloads and physical tests appropriate to the claims |

An **ablation** changes one component at a time to find which part causes an observed benefit. For example: navigation alone, then with camera, with language, and with both. If we propose a scheduling policy, compare it with a declared simpler policy on the same hardware and tasks.

We can start without access to physical GCU. Plan GCU simulation with Pi computation first, and a smaller physical house experiment when feasible. Neither substitutes for the other: simulated sensors do not establish performance with real sensor noise, and a house experiment does not establish campus-scale deployment.

Do not promise a submission date or venue until the research question, contribution and validation workload are clear. A mixed or negative result can change the paper's story; we will revise the scope rather than invent an improvement.
