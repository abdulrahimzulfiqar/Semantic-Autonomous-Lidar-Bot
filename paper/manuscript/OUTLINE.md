# Working manuscript outline

## Quick summary

The [working manuscript](EDGEOSMAG_DRAFT.md) contains the current full-paper draft. The navigation system, methods and measured results are drafted. Revise the title, argument and abstract once the integrated experiments are complete.

| Article section | Question it answers | Evidence or preparation needed | Status |
| --- | --- | --- | --- |
| Abstract | What problem, method, measured findings and implications does this paper present? | Completed study | Navigation draft |
| 1. Introduction | Why does the problem matter, what gap remains and what do we contribute? | Supported integrated contribution statement | Navigation draft |
| 2. Related work | How does this compare with osmAG-Nav, edge navigation and relevant perception/language systems? | Expand the comparison after selecting the perception and language workloads | Navigation and osmAG foundations drafted |
| 3. System design | How do map preparation, route planning and execution work? | Add the perception and language interfaces once implemented | Navigation architecture and timing drafted |
| 4. Experimental setup | How could someone repeat the study and assess its fairness? | Complete the planned workload protocols | Pi hardware and scoring drafted |
| 5. Results | What did the recorded trials show? | Higher-rate closed-loop baseline and combined-load measurements | Simulation, replay and pilot outcomes drafted |
| 6. Discussion and limitations | Why did it behave this way, and where do the findings apply? | Interpret the completed comparison | Navigation findings drafted |
| 7. Perception and language | How do the extensions affect task quality and navigation? | Implement and measure both workloads | Planned evaluation |
| 8. Conclusion | What did we learn, within the tested scope? | Results and contribution aligned with the introduction | Navigation draft |
| References and acknowledgments | Whose work and support did we use? | Complete bibliographic entries and agree authorship and acknowledgments | Primary references linked |

Keep the sections together in the working manuscript. Link figures and evidence from their source records. Apply venue formatting when the manuscript is ready for submission.

## Contribution boundary to preserve

osmAG and osmAG-Nav supply foundational ideas; Nav2 and AMCL are reused components. Our routing is a labelled paper-based implementation, and our deployed localization differs from the authors' structural-localization approach. The separate 2D prototype is not a complete AGLoc reproduction.

Possible contributions to investigate include a measured resource-management method, a controlled characterization of concurrent workloads, or a reproducible edge-navigation evaluation. These are candidates. “Runs on a Pi,” “uses an LLM,” and “combines existing tools” alone do not establish a literature-supported contribution. We must determine what is new, useful and measured before presenting it as such.
