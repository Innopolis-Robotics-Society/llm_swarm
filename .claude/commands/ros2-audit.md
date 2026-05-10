---
description: Focused deep audit of ROS 2 specific concerns across an entire package or workspace. Includes graph-level dataflow analysis.
argument-hint: <package name or path, e.g. "src/my_robot_perception", or "all" for the whole workspace>
---

You are running a focused ROS 2 audit on $ARGUMENTS.

This is broader than a diff review — audit the whole package(s) against ROS 2 best practices AND check that the inter-node graph makes sense.

## Phase 1 — Map the package(s)
Invoke `researcher` with: "Map every node, lifecycle node, executor, callback group, topic, service, action, parameter, TF frame, and pluginlib export in $ARGUMENTS. Also list QoS choices and any custom executor configurations. Resolve launch-file remappings."

## Phase 2 — Quadruple review (parallel)
Launch in parallel:
- `ros2-reviewer` — full checklist against the whole package, not just a diff. Pay extra attention to: lifecycle correctness, QoS dependency rules, callback-group / executor mismatches, real-time paths.
- `dataflow-reviewer` — graph-level analysis: orphan publishers/subscribers, name/type/QoS mismatches across nodes, dead-end pipelines, useless relays, TF chain integrity, composition opportunities. **This is the main reason to run /ros2-audit instead of /deep-review.**
- `performance-reviewer` — hot paths, allocations, complexity, intra-process composition opportunities.
- `architecture-reviewer` — package boundaries, pluginlib hygiene, dependency direction.

## Phase 3 — Synthesis
Invoke `synthesizer`. Final verdict format:

```
# ROS 2 audit: $ARGUMENTS

## Inventory (from researcher + dataflow-reviewer graph summary)
- Nodes: ...
- Lifecycle nodes: ...
- Topics: ... (with QoS, producer count, consumer count)
- Services / actions: ...
- Parameters: ...
- TF frames: ...
- Graph holes (unresolved): ...

## Findings
🔴 / 🟡 / 🟢 with file:line for both ends of each edge problem.

## Verdict
PASS / NEEDS WORK / FAIL
```
