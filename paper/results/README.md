# `paper/results` — what is here and which article claim it supports

This directory holds every artefact behind the numbers in
`paper/llm_swarm_narrative/main.tex`. The layout is claim-oriented: each
subdirectory is one evaluation, and inside it the rosbags sit next to the
derived files computed from them, with the analysis scripts one level above.

```
paper/results/
├── README.md                     ← this file
├── scripts/                      ← every analysis script, no data
├── e1_plan_accuracy/             ← E1/E2 offline plan benchmark
│   ├── local_tools/              ← summaries for the local (ollama) models
│   ├── hosted_tools/             ← summaries for the hosted (OpenRouter) models
│   ├── schema_grounding/         ← schema-constrained decoding experiments
│   └── raw/                      ← per-condition raw JSON, driver scripts, logs
├── e2_mission_series/            ← the 18-run simulation campaign (E2 table)
│   ├── no_llm/                   ← 5 runs, no language model
│   ├── model_9b/                 ← 5 runs, qwen3.5-9b, no guards
│   ├── cell_b/                   ← 5 runs, guards on
│   ├── cell_b_m3/                ← 3 runs, guards on, mission M3
│   └── README_code_versions.md   ← which run was recorded on which code
├── m3_guard_comparison/          ← the M3 before/after shared-drop-off account
│   └── pre_guard/                ← 2 runs recorded before the guard existed
└── qualitative_cases/            ← M4 runs cited qualitatively, not in the table
```

Runs that the article does not use are **not** here. They live in
`llm_swarm_rejected/paper_results/` at the repository root, with a README
explaining why each was withdrawn.

## Rosbags are not in the repository

Every `*.db3` under `paper/results/` is gitignored (see `.gitignore`). A clone
therefore contains the text half of the evidence — `session.json`,
`launch.log`, `bag/metadata.yaml`, `llm_chat/*.jsonl`, the raw benchmark JSON,
the scripts — and **not** the recordings.

To keep the numbers checkable without the bags, every run directory carries
`derived/metrics.json`: the full `analyse_run()` record for that run, written
by `scripts/write_derived_metrics.py`. That file is tracked. It is the
per-run evidence a reader can inspect in a clone; the bag is what a reader
needs to recompute it.

| what | in a clone | needs the bag |
| --- | --- | --- |
| task completion, makespan, distance travelled | `derived/metrics.json` | yes, to recompute |
| LLM call counts, guard firings, tool calls | `llm_chat/*.jsonl` | no |
| plan text, duplicate-carrier cases | `llm_chat/*.jsonl` | no |
| BT error counts, MAPF goal outcomes | `derived/metrics.json` | yes, to recompute |
| formation metrics (M4) | `derived/metrics.json` | yes, to recompute |

## Claim → file → script

### E2 mission series (Table `tab:e2`, 18 runs)

| cell | runs | mean makespan | mean in-model time | directory |
| --- | --- | --- | --- | --- |
| no-LLM | 5 | 203.5 s ± 10.6 | 0 s | `e2_mission_series/no_llm/` |
| model-9b | 5 | 252.2 s ± 26.6 | 9.5 s | `e2_mission_series/model_9b/` |
| B | 5 | 260.0 s ± 16.5 | 31.6 s | `e2_mission_series/cell_b/` |
| B-M3 | 3 | 587.7 s ± 67.0 | 51.2 s | `e2_mission_series/cell_b_m3/` |

Reproduce the whole table:

```bash
python3 paper/results/scripts/analyze_sessions.py
```

One cell, or one run in full detail:

```bash
python3 paper/results/scripts/analyze_sessions.py --cell B-M3
python3 paper/results/scripts/analyze_sessions.py --detail 20260818_111548
```

Regenerate the per-run `derived/metrics.json` files (needs the bags):

```bash
python3 paper/results/scripts/write_derived_metrics.py
```

The `--sessions` default is `paper/results/e2_mission_series`; the script walks
one level down, so the cell grouping is transparent to it.

### M3 before/after the shared-drop-off guard

| series | runs | result | directory |
| --- | --- | --- | --- |
| before the guard | 2 | 8/9, 8/9 (445.6 s, 496.4 s) | `m3_guard_comparison/pre_guard/` |
| after the guard | 3 | 9/9, 9/9, 9/9 | `e2_mission_series/cell_b_m3/` |

```bash
python3 paper/results/scripts/analyze_sessions.py \
    --sessions paper/results/m3_guard_comparison/pre_guard
```

The two pre-guard runs were recovered from the operator archive
`scenarios_6_7_12.zip`; their bags were never tracked. Both have an empty
`llm_chat/`, so metrics 4–7, 9 and 10 are undefined for them and they cannot
score an ablation cell. See `m3_guard_comparison/README.md`.

### E1 / E2 offline plan benchmark

| directory | contents | script |
| --- | --- | --- |
| `e1_plan_accuracy/local_tools/` | `e1_summary.json`, `e1b_summary.json` and their manifests — local ollama models, schema off | `scripts/rebuild_e1_summaries.py` |
| `e1_plan_accuracy/hosted_tools/` | the same for the hosted OpenRouter models | `scripts/build_hosted_summary.py` |
| `e1_plan_accuracy/schema_grounding/` | `e2_summary.json`, `e2_ladder_summary.json`, node distribution, key-order probe, mechanism analysis | `scripts/build_e2_summary.py`, `scripts/build_e2_ladder.py`, `scripts/analyze_e2_nodes.py` |
| `e1_plan_accuracy/raw/` | per-condition raw JSON, driver `.sh`, probe `.py`, sweep logs | input to all of the above |

```bash
python3 paper/results/scripts/build_e2_summary.py --check   # verify, write nothing
python3 paper/results/scripts/build_e2_summary.py           # verify and write
```

`build_e2_summary.py` refuses to write while the E1b baseline it reuses does
not match on prompt fingerprint — that check is deliberate, not a nuisance.

### Qualitative M4 cases

`qualitative_cases/` holds the two M4 runs the article cites: the leader
ejection that exposed the substrate defect chain, and the interactive session
that produced the only formation metrics. Neither is a valid repeat, so
neither is in the 18-run table. See `qualitative_cases/README.md`.

## Conventions

- A run directory is named by its session stamp (`YYYYMMDD_HHMMSS`), except in
  `qualitative_cases/`, where the descriptive prefix comes first and the stamp
  is kept as the suffix.
- Every run directory has the same shape: `session.json`, `launch.log`,
  `bag/`, `llm_chat/`, `derived/metrics.json`.
- `derived/metrics.json` is generated, never hand-edited. If a number in the
  article disagrees with it, the script is wrong, not the file.
- Scripts take their paths from their own location, so they work from any
  working directory. They are not installed; run them with `python3` from the
  repository root.
