# How we review a piece of writing

## Quick summary

Codex leads the drafting, technical checks and manuscript integration in `manuscript/`. The working standard is a journal-ready manuscript whose methods, numbers and claims can be checked against the retained evidence. The human research team can use that manuscript as a reference for its own paper and remains responsible for authorship, approval and any submission.

## The repeatable process

1. Draft the section from the current implementation and retained experiment records.
2. Check technical claims, measurements, comparisons and attribution.
3. Update the manuscript with supported findings and describe planned experiments in future tense.
4. Record material revisions and their evidence in the [review log](reviews/REVIEW_LOG.md).

Review outcomes:

- **Needs revision:** wording, reasoning, source interpretation or method description needs correction.
- **Evidence pending:** a statement requires a measurement or source we do not yet have. It stays as a labelled draft or planned-method statement.
- **Ready for working manuscript:** the reviewed version is supported for its stated scope. It may still change during whole-paper/coauthor review.

## What gets checked

| Check | Practical question |
| --- | --- |
| Meaning | Can we explain every technical term and why this paragraph belongs? |
| Attribution | Which parts come from osmAG/osmAG-Nav, Nav2, AMCL or other work, and which are ours? |
| Evidence | Does every substantive factual or numerical claim have a source or traceable run? Are units, setting and time base clear? |
| Fairness | Are hardware, workloads, tuning, initialization and measurement boundaries comparable? Are failures and repeats included? |
| Scope | Does the text distinguish simulation, real Pi computation and physical robot experiments? Does it avoid claiming untested generality? |
| Reproducibility | Could another researcher identify the code/configuration, inputs and evaluation procedure? |
| Writing | Is the paragraph clear, concise and written in our own words? Are limitations stated where they affect interpretation? |

We will use [claim IDs](evidence/CLAIM_LEDGER.md) in working notes, and proper citations in the final manuscript. A linked paper is not enough to support a detailed algorithm claim: check the relevant section/equation before using it. Do not invent references or copy source prose into the draft.

Correctness tests help establish implementation behavior. They do not by themselves establish research novelty, a resource advantage or real-world reliability.

A review applies to a particular version and its evidence. If implementation, data or conclusions change, reopen affected sections and log the change. Planned experiments use future tense; completed methods and results describe what actually ran. Unknowns remain explicit until resolved.

Before submission we will also verify the chosen venue's current format, authorship, data/code and AI-assistance disclosure requirements. No venue has been selected yet.
