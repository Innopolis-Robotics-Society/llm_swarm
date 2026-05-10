---
description: Full research → plan → confirm → implement → multi-agent review → tests → final synthesis cycle for a ROS 2 task.
argument-hint: <one-line task description, e.g. "add a lifecycle node that fuses /scan and /odom into /pose_filtered using EKF">
---

You are orchestrating a full development cycle for a ROS 2 (Humble/Jazzy) task.

The task: $ARGUMENTS

Run the following pipeline. Use parallel sub-agents wherever the doc says "in parallel". Pause where the doc says "PAUSE".

## Phase 1 — Research (sequential)
Invoke `researcher` with the task description plus the relevant package globs. Read its report into context.

## Phase 2 — Plan (sequential)
Invoke `planner` with the task and the researcher's report. Read its plan into context.

## Phase 3 — PAUSE for user confirmation
Print the plan to the user verbatim and ask: *"Approve plan? (yes / revise / abort)"*.
- If `yes`, continue.
- If `revise`, take the user's revision notes and re-invoke `planner`.
- If `abort`, stop.

## Phase 4 — Implement (sequential)
Invoke `ros2-implementer` with the approved plan. Confirm `colcon build` passes (surface the first error if not, and re-invoke implementer with the error context).

## Phase 5 — Deep review (PARALLEL)
Invoke these reviewers in parallel against the diff (`git diff main...HEAD` or staged):
- `architecture-reviewer`
- `ros2-reviewer`
- `cpp-reviewer` (skip if no .cpp/.hpp changed)
- `python-reviewer` (skip if no .py changed)
- `performance-reviewer`
- `security-reviewer`
- `dataflow-reviewer` (only if the diff changed launch files, package.xml, message/service/action defs, or renamed topics/services — i.e., anything that could shift the inter-node graph)

Collect all reports into context.

## Phase 6 — Synthesis (sequential)
Invoke `synthesizer` with all reviewer reports. Read its verdict.
- If verdict is 🔴 NEEDS WORK: send the criticals back to `ros2-implementer` and loop to Phase 5 (max 2 iterations; then escalate to user).
- If 🟡 NEEDS ATTENTION: ask the user whether to fix now or proceed.
- If ✅ READY TO MERGE: continue.

## Phase 7 — Tests (sequential)
Invoke `test-writer` with the plan's acceptance criteria + the criticals fixed during synthesis.

## Phase 8 — Test review (sequential)
Invoke `test-reviewer`. If 🔴 → send back to `test-writer` (max 2 iterations).

## Phase 9 — Final report
Print to the user:
- Files changed
- Final verdict from synthesis
- Tests added and `colcon test` result
- The `ros2 launch` command to verify manually
- Any remaining 🟡/🟢 deferred items

Use parallel sub-agents in Phase 5. Be terse in the orchestrator prose; the agents do the work.
