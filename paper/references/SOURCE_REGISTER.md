# Source register and reading cards

## Quick summary

These primary references support the current navigation draft. Expand the review of constrained-compute robotics, concurrent inference and resource scheduling as the workload comparison takes shape. Detailed method claims should cite the relevant full-text sections.

## Scientific foundations

| ID | Reference and primary link | Why we need it | Reading status for the writing workflow |
| --- | --- | --- | --- |
| S01 | Yongqi Zhang, Jiajie Zhang, Chengqian Li, Fujing Xie and Sören Schwertfeger. *osmAG-Nav: A Hierarchical Semantic Topometric Navigation Stack for Robust Lifelong Indoor Autonomy*. arXiv:2603.28271v1, 2026. [Record](https://arxiv.org/abs/2603.28271v1) | Main navigation architecture and closest system relationship | Draft cites §5. The [routing audit](../../docs/research/osm-AG/GCU_ROUTING_PAPER_AUDIT.md) records implementation differences. |
| S02 | Delin Feng, Chengqian Li, Yongqi Zhang, Chen Yu and Soeren Schwertfeger. *osmAG: Hierarchical Semantic Topometric Area Graph Maps in the OSM Format for Mobile Robotics*. arXiv:2309.04791v1, 2023. [Record](https://arxiv.org/abs/2309.04791v1) | Map representation and planning foundations | Draft cites §§III–IV. |
| S03 | Fujing Xie and Sören Schwertfeger. *Robust Lifelong Indoor LiDAR Localization using the Area Graph*. arXiv:2308.05593, initially 2023; v3 revised 2024. [Versioned record](https://arxiv.org/abs/2308.05593v3), [journal DOI](https://doi.org/10.1109/LRA.2023.3334158) | Structural localization and limits of our separate 2D prototype | Existing [localization audit](../../docs/research/osm-AG/GCU_LOCALIZATION_PAPER_AUDIT.md); card must identify which PDF/version is read |
| S04 | Fujing Xie, Jiajie Zhang and Sören Schwertfeger. *Intelligent LiDAR Navigation: Leveraging External Information and Semantic Maps with LLM as Copilot*. arXiv:2409.08493, initially 2024. [Record](https://arxiv.org/abs/2409.08493) | Closely related language/context-guided navigation; must compare before claiming novelty for blockage reports or LLM assistance | Primary record/abstract checked during Review 01; full-text reading card pending |
| S05 | Fujing Xie, Sören Schwertfeger and Hermann Blum. *osmAG-LLM: Zero-Shot Open-Vocabulary Object Navigation via Semantic Maps and Large Language Models Reasoning*. arXiv:2507.12753, initially 2025; journal publication 2026. [Record](https://arxiv.org/abs/2507.12753), [journal DOI](https://doi.org/10.1109/LRA.2026.3653280) | Closely related object-goal navigation; relevant if camera/object-search scope is adopted | Primary record/abstract checked during Review 01; full-text reading card pending |
| S06 | Steve Macenski, Francisco Martín, Ruffin White and Jonatan Ginés Clavero. *The Marathon 2: A Navigation System*. 2020. [Primary paper](https://arxiv.org/abs/2003.00368) | Native Nav2 navigation framework | Cited for navigation orchestration. |
| S07 | Steve Macenski, Shrijit Singh, Francisco Martín and Jonatan Ginés Clavero. *Regulated Pure Pursuit for Robot Path Tracking*. [Primary paper](https://arxiv.org/abs/2305.20026) | Native path-tracking controller | Cited for the controller reused in the navigation stack. |

The seed list does not establish a literature gap. At the reading stage, add primary research on constrained navigation, concurrent perception/language workloads and relevant resource policies. Record contradictory and stronger competing results too. Verify publication metadata and the actual version used before building the final bibliography.

## Reading-card template

Create one Markdown note per source here, starting with S01 when Step 2 begins:

```text
Source ID, title and exact version read:
Problem the authors address (in my words):
Their main method and contribution:
Inputs, hardware and environment:
Evaluation/baselines and important measured result (with units):
Limitations relevant to our study:
What our system reuses, adapts or does differently:
Exact pages/sections/equations supporting these notes:
Open questions and claims to verify:
```

The title/abstract is enough to register a source, but not to validate equations, runtime behavior or performance claims. Use the attached/local version explicitly when it differs from the current online version. Cite the paper for scientific claims and the pinned repository version separately for source-code behavior.

## Writing guidance

- [IEEE Author Center: Structure Your Article](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/create-the-text-of-your-article/structure-your-article/) — general article organization; supports our outline, not an EdgeOsmAG result.
- [IEEE Author Center: Checklist for Creating Your Article](https://journals.ieeeauthorcenter.ieee.org/create-your-ieee-journal-article/authoring-tools-and-templates/checklist-for-creating-your-article/) — consult again when preparing a submission.

Venue-specific rules and templates will be checked after we choose a target. No venue or publication outcome is assumed.
