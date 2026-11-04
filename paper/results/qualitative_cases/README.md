# Qualitative M4 cases

The two M4 runs the article cites. Neither is a valid repeat, so neither is in
the 18-run E2 table; both are used as evidence for a specific mechanism.

| directory | stamp | what it is |
|---|---|---|
| `m4_leader_ejection_20260818_140918/` | `20260818_140918` | the leader was dropped from the plan; the run exposed the substrate defect chain described in `../e2_mission_series/README_code_versions.md` |
| `m4_formation_interactive_20260819_073009/` | `20260819_073009` | an interactive session: 7 operator messages during the mission instead of one scripted command |

The other two recorded M4 runs are not here. `20260818_135527` (the pilot, the
stack died mid-run) is in `llm_swarm_rejected/paper_results/`; `20260819_072231`
(rep 3) was deleted on 2026-08-19 — 42 s, nothing moved, zero channel-3 events,
nothing to record.

## Why `073009` is kept even though it is not a valid repeat

It is the most informative run of the campaign, and several things happened in
it for the first time:

- **The partial-delivery report fired in anger.** `partial: robot_0 did not
  reach the requested goal` travelled from the planner through the BT to the
  chat in 0.33 s. Exactly the case that `140918` silently closed as
  `All agents arrived, planned=4`.
- **The formation assembled for the first time** —
  `Formation 'cyan_column' registered and activated`. Metrics 11–14 finally
  have data: `t_stable` 6.5 s, `eps_ss` 0.035 m, `eps_peak` 0.409 m,
  `f_degraded` 0.022, `f_broken` 0. The narrow section was never traversed, so
  `eps_narrow` is empty.
- **33 tool calls** against zero in every M2 cell. The `constrained` cell
  finally has something to measure, and it has to be measured on M4.
- **A planning error worth quoting.** The model emitted
  `offsets_x: [0,0,0]`, `offsets_y: [1.5,-1.5,3.0]` — a 4.5 m line abreast,
  which it called a column. Standing still it assembles to within 3.5 cm, but
  it cannot move at all: with the formation active the group's volume moves
  into the leader's inflated footprint, and `no static path ... footprint
  won't fit through` — the widest passage on the map is 2.85 m. In `140918`
  the same model emitted the correct `offsets_x: [-1.5,-3.0,-4.5]`.
- **A provider-dependent client failure**, which is why the run ended: the
  tool-call history was sent in the native Ollama format to an
  OpenAI-compatible OpenRouter endpoint. Fixed; the analysis is in the header
  of `common/tool_definitions.py`.

The tool-call wire format was fixed **after** this run, so every M4 recording
carries that defect. For future repeats this means: if a run died on `HTTP 400`
or `LLM timeout` while continuing supervision, it was recorded before the fix
and does not show a property of the model.
