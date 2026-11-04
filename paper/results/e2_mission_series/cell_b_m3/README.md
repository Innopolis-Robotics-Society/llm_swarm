# Cell B-M3: composition, providers, and the limits of its metrics

An operator decision, written down so it does not have to be reconstructed
later. State as of **2026-11-03**.

## What the cell contains

Three runs, all with a channel-3 journal, all 9/9:

| run | rep | providers in routing | result |
|---|---:|---|---|
| `20260818_111548` | 1 | Parasail, AtlasCloud, GMICloud, Chutes | 9/9 |
| `20260818_113030` | 2 | Parasail, AtlasCloud, GMICloud, Chutes | 9/9 |
| `20260818_114445` | 3 | Parasail, AtlasCloud, GMICloud, Chutes | 9/9 |

The spec (§4.6) requires re-running a cell in full when the provider changes.
Chutes dropped out on 2026-08-18 in the middle of collection, and the cell will
**not** be re-run: the rule costs extra runs, and the variance it was meant to
catch is not visible in the data that exists (all three are 9/9). This is a
weakening of the design, and the article states it in the limitations rather
than leaving it implicit.

## Before/after the shared-drop-off guard

The two pre-guard runs live separately, in
`paper/results/m3_guard_comparison/pre_guard/`:

| run | rep | provider | result | makespan |
|---|---:|---|---|---|
| `20260815_153035` | 1 | Chutes | 8/9 | 445.6 s |
| `20260815_154541` | 2 | Chutes | 8/9 | 496.4 s |

Both were recovered from the operator archive `scenarios_6_7_12.zip` (complete
bags), because their bags were never tracked in git. The archive also records
why the runs are irregular: `153035` never logged the operator command, and
`154541` got `MapfPlan: FAILURE — no agents planned` from the planner. On tasks
both give 8/9; both contain a `MAPF goal SUCCEEDED having planned nobody`.

Both have an empty `llm_chat/`: the mission never reached the point where the
final file is written, and per-phase checkpoints only appeared in `1b9521b`,
i.e. after these runs. The archive does not contain those files either, so
there is nothing to restore.

What that makes readable and what it does not:

| metrics | source | these two runs |
|---|---|---|
| 1 task fraction, 2 full missions, 3 makespan, 8 distance travelled | `/tasks/state` and `/robot_*/odom` in the bag | **readable** |
| 4 model calls and time, 5 guard firings, 6 guard firings, 7 first plan without intervention, 9 refusals, 10 transport failures | channel-3 JSONL | **absent** |

Practical consequence: the cell's **n differs by metric** — three runs for the
mechanism counters, five for the task fraction if the pre-guard cases are
counted with the post-guard ones. Ablation cells are defined through metric 5,
so the pre-guard runs are not part of any ablation.
`analyze_sessions.py` flags them as
`no llm_chat/*.jsonl -- ... this run cannot score an ablation cell`, and the
`mechanism activity` block counts firings only over runs that have a journal,
printing that count as `N/M`.
