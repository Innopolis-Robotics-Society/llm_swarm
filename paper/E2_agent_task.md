# Task: Implement and run E2 — Grounding and Constraint Ablation

> **Scope revision (2026-07-27).** The tool-calling factor (originally §1.4) is
> **struck from E2** and deferred to E3. Rationale is recorded in
> "Scope decisions" below. E2 now covers exactly two factors:
> **schema-constrained decoding on/off** and **symbolic-vs-raw grounding
> (three variants)**. Live pose context, goal auto-spread, and context budget
> are also out of scope for this pass — see "Scope decisions".

## Context (read first)

Repo: `Innopolis-Robotics-Society/llm_swarm`, branch **`paper`**.

This is one experiment (E2) inside a larger evaluation for an academic paper.
Before writing any code, read:

- `paper/main.tex`, section **"Experimental Design" → "E2: Grounding and
  constraint ablation"** (`\label{sec:e2}`) — authoritative spec for what E2
  is and why. Also read "Research questions" (RQ2) above it, and "Principle:
  reuse existing instrumentation" — the paper commits to adding as little new
  measurement code as possible.
- `paper/main.tex`, section **"Results" → "E1: Planning accuracy"** — the only
  experiment currently done, and the template for rigor and tone your E2
  results subsection must match (pass@1/pass@5, per-category breakdown,
  failure-mode table, explicit caution paragraph about sample size).
- `iros_llm_orchestrator/benchmark_ch3.py` — the existing offline harness
  (~1000 lines) used for E1. 44 test cases, each a prompt + synthetic runtime
  context + programmatic validator (`chk_*` helpers). You are extending this,
  not replacing it.
- `paper/results/e1b_manifest.json`, `e1b_summary.json`, `raw/e1b_*.json` —
  the output convention to match exactly. Read the `notes` array carefully:
  it documents real bugs found mid-sweep and is the model for how to report
  deviations.
- `iros_llm_orchestrator/iros_llm_orchestrator/common/user_prompt.py` —
  `build_user_prompt()` (line ~351) and `build_map_context()` (line ~81).
- `iros_llm_orchestrator/iros_llm_orchestrator/common/plan_schema.py` —
  exports `PLAN_RESPONSE_SCHEMA`.
- `iros_llm_orchestrator/config/orchestrator.yaml`.

**Ground truth is the code, not this document.** Where this brief names a
file, line, or parameter, verify it. If it's wrong, follow the code and
record the discrepancy in your manifest `notes`.

## Goal

1. Extend the offline harness with the two ablation factors E2 needs.
2. Run the ablation, output in the project's existing results format.
3. Draft the E2 results subsection for `main.tex`, replacing the placeholder
   `\pending{E2 --- ablation: effect of constrained decoding, tool calling,
   pose context, context budget.}`

## Models

Primary models for this experiment: **`qwen3.5:4b`** and
**`qwen3.5-9b:latest`**.

Rationale: same family, ~3x size difference (3.4 GB vs 10 GB), and both
already have E1b baselines to ablate against (4b: 68.2% pass@1 / 88.6%
pass@5; 9b: 77.3% / 90.9%, the best of the five models swept). Their E1
failure profiles differ qualitatively — 4b's errors look like sampling noise
(largest pass@1→pass@5 recovery in the table), 9b's are fewer and more
systematic — so a factor that helps one and not the other is itself a result.

Note the exact Ollama tag is `qwen3.5-9b:latest`, **not** `qwen3.5:9b` — the
latter does not exist locally and returns instant HTTP 404. This already
burned one sweep slot during E1b (see `e1b_manifest.json` notes); a sanity
check that per-call latency is seconds, not ~0.0 s, catches it immediately.

## Part 1 — Code

### 1.1 Schema-constrained decoding on/off — NOT currently in the harness

Verified: `_call_llm()` (line ~745) calls `llm.stream(messages)` with no
`response_format`, and `OllamaClient.stream()` defaults it to `None`. **E1 and
E1b therefore ran fully unconstrained.** This is good news — E1b is already a
clean "constrained decoding OFF" baseline — but it means the ON arm is new
code, and it means the current sentence in `main.tex` §E2 claiming the
harness "exercises only the plain and schema-constrained paths" is inaccurate
about the second one. Flag that prose error in your report; it needs fixing.

Add a harness flag that passes `PLAN_RESPONSE_SCHEMA` (from
`common/plan_schema.py`) through as `response_format`. Verify it reaches
Ollama's `format` field.

Watch for this: `e1_manifest.json` documents a bug where `build_user_prompt`
emitted two leading system-role messages, and Ollama's `format=json_schema`
template compiler **for the qwen3.5 chat template specifically** statically
rejects any non-first system message. It was fixed by merging runtime context
into the single leading system message (the current code does this). E2's
constrained arm is the first actual use of `json_schema` in this harness, and
the primary models are qwen3.5 — so this fix is load-bearing here. Confirm it
still holds before the sweep, or every constrained call fails.

### 1.2 Symbolic vs. raw grounding

The map block is assembled in **`common/user_prompt.py:build_map_context()`**
(line ~81) — a single clean function, not the `context/` modules. It emits, in
order: map name + description, bounds, named locations (with aliases), robot
groups (ids / home / spawn positions), formation zones ("Good spots for
formations"), and a heuristics block from map config.

That structure maps directly onto the three required variants:

1. **full** — everything, current default, must be byte-identical when the
   flag is unset so E1b stays reproducible
2. **locations-only** — drop the heuristics block (and state explicitly in
   your manifest whether you also dropped formation zones; either choice is
   defensible, but the paper text must match what you actually ran)
3. **coords-only** — bare coordinates, no aliases, no zones, no heuristics

### 1.3 Output format

Match `e1b_manifest.json` / `e1b_summary.json` exactly:
`paper/results/e2_manifest.json`, `e2_summary.json`, and per-condition raw
files under `paper/results/raw/` named `e2_<factor>_<value>_<model>.json`
(e.g. `e2_schema_on_qwen3.5-4b.json`,
`e2_grounding_coords_only_qwen3.5-9b.json`).

## Part 2 — Data collection

- **Protocol:** identical to E1b except the one factor under test — 44 cases ×
  N=5 repeats × temperature 0.1 × `--llm-max-tokens 2048` × `--llm-num-ctx
  32768`. Read the `e1b_manifest.json` notes for why each of these values was
  chosen before deviating from any of them.
- **One-at-a-time, not factorial.** Hold everything at the E1b default and
  toggle one factor. The paper's design calls for this explicitly; a full
  cross is neither required nor affordable.
- **Both primary models for every factor.** The 4b-vs-9b contrast is a
  deliberate part of this experiment, not a stretch goal — a factor that
  rescues 4b but does nothing for 9b is one of the more interesting outcomes
  available here.
- **Grounding factor:** three conditions (full / locations-only / coords-only).
- **Baseline reuse:** the E1b runs for both models *are* the baseline arm for
  the factors where E1b's settings equal "off". Don't re-run them; reference
  them, and verify the git commit still matches
  (`b31fdbd48d454b2ae185cd3bfac174b5f1128f93` in `e1b_manifest.json`) — if
  your code changes shift the default prompt at all, the baseline must be
  re-run instead. This is the single easiest way to produce a silently invalid
  comparison, so check it explicitly.

### Conditions to run

Baseline is (schema=off, grounding=full) — that *is* the E1b configuration.
Five labelled conditions, of which two coincide with the baseline cell:

| # | Condition label | schema | grounding | New run needed? |
|---|---|---|---|---|
| 1 | `baseline`                 | off | full           | No — reuse E1b |
| 2 | `schema_on`                | on  | full           | **Yes** |
| 3 | `grounding_full`           | off | full           | No — same cell as #1 |
| 4 | `grounding_locations_only` | off | locations_only | **Yes** |
| 5 | `grounding_coords_only`    | off | coords_only    | **Yes** |

New runs: 3 conditions × 2 models = 6 runs × 220 calls = 1,320 calls.

## Part 3 — Deliverables

1. Code changes as a reviewable diff against `paper`.
2. `paper/results/e2_manifest.json`, `e2_summary.json`,
   `paper/results/raw/e2_*.json`.
3. Drafted LaTeX subsection replacing the `\pending{E2 ...}` line, in the
   voice of the existing "E1: Planning accuracy" results subsection: specific
   numbers, which factors moved which failure modes (cross-reference E1's
   failure-mode table), per-category reading where the global rate misleads,
   and an explicit caution paragraph wherever N or case count is too small for
   a strong claim. Do not overstate what the data supports. The subsection
   must also state plainly that tool calling, pose context, auto-spread, and
   context budget were **not** measured in E2, and where each went.
4. A short list of discrepancies found between `main.tex` and the code. Two
   are already known and expected — the "schema-constrained path" claim in
   §E2 (§1.1 above), and `orchestrator.yaml` currently shipping both
   `tool_calling_enabled: true` and `structured_output_enabled: true` despite
   its own comment stating they are mutually exclusive and that running both
   silently breaks tool invocation. Report any others you hit.

## Scope decisions

Recorded here so the paper text and the run stay in sync.

### Struck: tool calling (was §1.4)

**Deferred to E3.** Investigation before implementation found that a
tool-calling arm cannot be made faithful inside an offline harness:

- `check_occupancy` subscribes to a live `/robot_{id}/scan` `LaserScan`
  (`common/tool_executor.py:478-499`) and ray-classifies it. It requires
  Stage. There is no honest offline substitute.
- `verify_plan_execution_state` is post-execution verification. Nothing is
  executed in an offline planning benchmark, so it is vacuous here.
- `list_tasks` / `reset_task` are irrelevant — no case among the 44 concerns
  tasks.
- `get_robot_position` and `get_positions` return, respectively, the pose
  snapshot that the harness *already injects into the prompt* and static map
  YAML. Offline they add no information the model does not already have.

Only `find_free_group_goals_in_room` and `find_group_placement_in_room`
(both pure functions over `map_cfg` + a pose snapshot, per the module
docstring in `context/group_placement.py`) would carry real signal. Running
the factor on those two alone would measure the cost of the tool path
against a fraction of its benefit — a biased comparison that produces a
number looking like a finding but which is an artifact.

Standing up a fake-odom publisher + map server to feed the tools was
considered and rejected: that plumbing would only transport poses the
harness already holds as a Python dict, so the tool implementations — and
therefore the measurement — would be identical either way. It adds
apparatus without adding fidelity.

**Consequence for `main.tex`:** §E2 lines 658–665 currently assert *"requires
either extending the harness to route through the chat server, or measuring
that factor only in E3. We take the first option, as the second confounds
planning mode with execution noise."* That sentence must be rewritten — the
project takes the second option. The stated objection to it (execution noise
confounds planning mode) is real and should be acknowledged rather than
dropped.

**Model-capability note, still worth recording:** contrary to an earlier
assumption in `e1_manifest.json`, a live `/api/tags` check shows
`qwen3.5:4b` *does* advertise the `tools` capability (`vision, completion,
tools, thinking`). `qwen3.5-9b:latest` does not (`completion, vision`).
`qwen2.5:14b` does. So the blocker for E2 was never model capability — it
was the tools' dependence on live ROS state.

### Struck: live pose context on/off

Deferred. Cheap to implement (`build_user_prompt` already skips the runtime
block when `runtime_context.get('source') == 'none'`, line ~363), but the
interpretation is not cheap: several cases exist *because* of their context
(the missing-robot-12 escalation cases), so they legitimately cannot pass
with pose context off, and the factor only means something read per-category.
Out of scope for this pass.

### Struck: goal auto-spread

Out of scope, deliberately. `goal_spread_enabled` is a ROS parameter on
`chat_server` / `execute_server` (default `False` in `orchestrator.yaml`),
applied by `_postprocess_plan()` *after* the model returns. It is not in the
harness path at all, so E1b neither used it nor could have. Applying
`_postprocess_plan()` inside the harness would change what the
goal-distinctness validators see, so the two arms would not be scoring
identical objects. The few-shot examples were also written assuming it is off
(see the `_cluster()` docstring in `user_prompt.py`). The paper must say this
was not measured rather than implying it was.

### Struck: context budget

Out of scope for this pass. Three different things could be meant and they
live in different places: `--llm-num-ctx` (already a harness flag, 32768 in
E1b); `llm_context_window_tokens` / `llm_context_margin_tokens` (16384 / 512
in `orchestrator.yaml`, chat_server path only); and `context_max_chars`
(6000, runtime-context truncation, chat_server path only). Only the first is
coherently sweepable from the harness. If revived, the interesting region is
near and below the ≈9,700-token system prompt (see `sec:e1`).
