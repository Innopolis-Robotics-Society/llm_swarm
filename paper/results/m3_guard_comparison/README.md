# M3 guard comparison: the two runs recorded before the shared-drop-off guard

This directory holds the "before" half of the article's M3 before/after
account. The "after" half is the three B-M3 runs in
`paper/results/e2_mission_series/cell_b_m3/`; they are not duplicated here.

## `pre_guard/` — 2 runs before the shared-drop-off guard

`20260815_153035` and `20260815_154541`, both on Chutes, both 8/9, makespan
445.6 s and 496.4 s. Source: `scenarios_6_7_12.zip` from the operator archive;
their bags were never tracked in git. `llm_chat/` is empty for both, so metrics
4–7, 9 and 10 are undefined for them; 1–3 and 8 are read from the bag.

This is the evidence for the article's claim "before the shared-drop-off guard
… 8/9". After the guard, the three B-M3 runs give 9/9.

Each run directory has the usual shape — `bag/` next to `derived/metrics.json`,
plus `session.json` and `launch.log`. `analysis.json` is the combined summary
over both runs.

Recompute:

```bash
python3 paper/results/scripts/analyze_sessions.py \
    --sessions paper/results/m3_guard_comparison/pre_guard
```

## Limitation

Bags are not tracked (`*.db3` in `.gitignore`), so a clone keeps only the text
and the summaries. `derived/metrics.json` in each run is the same record as the
corresponding entry in `analysis.json`, split per run.
