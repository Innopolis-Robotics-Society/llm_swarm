# Changes made in response to review

Revision of 12 October 2026. The basis is the current Springer version of the
manuscript. The goal was to apply the editorial comments without new
experimental runs and without strengthening any claim beyond the available
data. No venue has been chosen yet.

## What changed

| Comment | Change | What remains a limitation |
|---|---|---|
| M1. Scientific contribution | Title, abstract and introduction are centred on correctness boundaries. RQ1–RQ3 were added, along with a connected discussion that answers them and a narrative comparison of the mechanisms in the closest related work | No new general hierarchy principle or universal theory of checking is claimed. The review is not a comparative benchmark |
| M2. Effectiveness of guards | Duplicate-carrier assignment is separated from the shared-drop-off conflict. The generalising title about defect absorption was removed. The uncontrolled before/after is kept and its causal reading is explicitly bounded | No controlled ablation. 8/9 → 9/9 is not attributed to a single guard and does not estimate an overall reliability gain |
| M3. Scale and load | Twenty robots are described as the deployed population. The number 20 was removed from the main title. Overview, abstract and limitations now separate deployment, concurrency and scaling | Simultaneous activity and the number of agents in joint requests are not measured in the summaries presented |
| M4. Meaning of task completion | A section on the operational meaning of task completion was added. A carry requires a dwell at the pickup, then a dwell at the drop-off. Visiting the drop-off before the pickup does not complete the task | Grasp, physical transport of the load, collision safety and full equivalence to operator intent are not checked. The numeric dwell threshold is not stated in the original manuscript |
| M5. Provenance | A table of evidence provenance was added. The main series, the decision journals, the before/after and the development observations are not added into one denominator | No session-ID mapping. Separating the series makes the account more precise but does not restore the missing register |
| Coverage of checks | A table of implemented coverage, observed evidence and limits was added | This describes coverage, it is not a formal safety verification |
| Failure cases | The carry cases, the formation geometry error and the local statuses were moved ahead of the aggregate scores | Qualitative substrate observations were not turned into failure statistics |
| Reproducibility | Unsupported claims that every number had been checked against committed scripts were removed, as was the promise of a release as proof | The manuscript package does not include the simulation code, rosbags, decision journals, raw responses or analysis scripts. Whether the authors hold them is unknown |

## New and corrected sources

HiP-CL, Symbolic Wrappers, a review of LLMs for multi-robot systems and
ProgPrompt were added. Their descriptions were checked against the papers and
the authors' project pages. ProgPrompt is not described as a deterministic
guard architecture: its assertion checks use an LLM.

Corrected: the surname of the first author of CommandSwarm — Mohammed Majid —
the exact title of that work, its author list and the name of LaMMA-P, the
author names of SMART-LLM, and the title and metadata of the paper on direct
MAPF through an LLM. LaMMA-P is tied to the verified author link
arXiv:2409.20560; the incorrect arXiv:2403.06940 from older notes is not used.

Main sources used for verification:

- https://link.springer.com/article/10.1007/s44230-025-00130-5
- https://link.springer.com/article/10.1007/s44430-026-00023-7
- https://link.springer.com/article/10.1007/s10514-026-10257-4
- https://progprompt.github.io/
- https://arxiv.org/abs/2605.07764
- https://arxiv.org/abs/2309.10062
- https://lamma-p.github.io/
- https://arxiv.org/abs/2401.03630

The current manuscript cites 19 entries in total. That does not mean every
inherited entry received a fresh audit: the sources listed above were checked
in this revision, the rest were carried over from the previous source.

## Checks

- Clean build `pdflatex → bibtex → pdflatex → pdflatex` without pre-existing
  auxiliary files.
- Visual check of every page and of the new tables.
- The values of the two original experimental tables were preserved.
- No run results, new error rates or significance estimates were added.
- The stock Springer Nature class and bibliography style were not modified.

The work remains a draft: the agreed author list, the declarations and an
actual audit of the original experimental records still require material from
the authors.
