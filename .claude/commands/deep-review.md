---
description: Run all reviewer agents in parallel against the current diff and synthesize the verdict.
argument-hint: [optional: branch / SHA / file paths; default = git diff main...HEAD or staged changes]
---

You are running a multi-perspective code review.

## Scope
Determine what to review (priority order):
1. If `$ARGUMENTS` is provided, use it as the scope (branch, SHA, file glob).
2. If on a feature branch, `git diff main...HEAD`.
3. If on main with staged changes, `git diff --staged`.
4. Otherwise, `git diff HEAD~1`.

Print the scope you chose to the user.

## Phase 1 — Parallel reviewers
Launch in parallel (single tool block, separate sub-agents):
- `architecture-reviewer`
- `ros2-reviewer`
- `cpp-reviewer` (only if any *.cpp / *.hpp / *.h is in scope)
- `python-reviewer` (only if any *.py is in scope)
- `performance-reviewer`
- `security-reviewer`
- `dataflow-reviewer` (only if the diff touches `*.launch.*`, `package.xml`, `*.msg`/`*.srv`/`*.action`, or adds/renames topic/service/action literals — i.e., anything that could change the inter-node graph)

Each receives the diff + the file paths it needs to read for context.

## Phase 2 — Synthesis
Pass all reports to `synthesizer`.

## Output
Print only the synthesizer's final report. Do not echo the individual reviewer reports unless the user asks.
