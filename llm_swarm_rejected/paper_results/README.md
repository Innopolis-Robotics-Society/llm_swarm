# Runs withdrawn from the article's set

Kept here rather than deleted. Nothing in `paper/results/` points at this
directory, and `analyze_sessions.py` does not scan it — its default root is
`paper/results/e2_mission_series`.

| run | cell | why it was withdrawn |
|---|---|---|
| `20260815_160042` | B-M3 rep 3 | the plan was rejected by the executor: one parallel branch sent `robot_10` and `robot_11` to two different goals, and the whole branch was discarded — hence 0/9 with a working planner. The defect was fixed in `ec688ec`; the code that produced this result no longer exists in HEAD |
| `20260818_135527` | M4 rep 1 | the pilot run: the stack died mid-run and `session.json` has no `finished_at`; the recorder then wrote an empty stream for an hour and a half and was stopped by hand. It is one of the three M4 sessions the article excludes as non-valid repeats |

The other two M4 exclusions are **not** here, because the article cites them as
evidence: `20260818_140918` (the substrate defect chain) and `20260819_073009`
(the only source of formation metrics) are in
`paper/results/qualitative_cases/`.

The two pre-guard B-M3 runs `20260815_153035` and `20260815_154541` were once
withdrawn and have been **returned to the set**: they are the "before" half of
the article's M3 guard comparison and live in
`paper/results/m3_guard_comparison/pre_guard/`.

Bags here are gitignored like everywhere else; `session.json`, `launch.log`,
`bag/metadata.yaml` and `llm_chat/*.jsonl` are tracked.
