---
description: Graph-level analysis of inter-node and inter-package data flow. Builds a producer/consumer graph and flags orphans, mismatches, dead-ends, useless relays, cycles, and composition opportunities.
argument-hint: [optional: package list / path glob; default = all of src/]
---

You are running a focused dataflow audit. Unlike `/ros2-audit` (which mixes dataflow with general ROS 2 review), this command targets ONLY the inter-node graph.

## Scope
- If `$ARGUMENTS` is provided, restrict the graph to those packages plus their direct dependencies (so cross-package edges are still visible).
- Otherwise, audit the entire workspace under `src/`.

Print the resolved scope to the user before proceeding.

## Phase 1 — Researcher pre-pass (sequential, fast)
Invoke `researcher` with: "List all ROS 2 packages in scope. For each, list the launch files, source files (Python and C++), and message/service/action definitions. Resolve namespaces and remappings if statically determinable. Note any launch substitutions that are dynamic (OpaqueFunction, computed LaunchConfiguration) — these will be 'graph holes'."

## Phase 2 — dataflow-reviewer (sequential)
Invoke `dataflow-reviewer` with the researcher's report and the in-scope file paths. Ask for:
- Full graph summary (node × topic × type × QoS).
- Connectivity matrix.
- All findings against the 13-section checklist in the agent.

## Phase 3 — Targeted follow-up (conditional, parallel)
Based on dataflow-reviewer's findings, launch focused parallel sub-agents:
- If `dataflow-reviewer` flagged a QoS mismatch → also invoke `ros2-reviewer` on those specific files to confirm intent.
- If it flagged a composition opportunity → also invoke `performance-reviewer` to estimate bandwidth savings.
- If it flagged a cross-package coupling smell → also invoke `architecture-reviewer` on the offending boundary.

Skip Phase 3 entirely if dataflow-reviewer's verdict is `DATAFLOW OK`.

## Phase 4 — Synthesis (sequential)
Invoke `synthesizer` with all reports. Final output:

```
# Dataflow audit: <scope>

## Graph
- Nodes: ...
- Topics: ... (orphan publishers: K, orphan subscribers: M)
- Services / actions: ... (caller without server: K)
- TF chain status: COMPLETE / BROKEN (link X→Y missing)
- Graph holes: ...

## Findings (deduped, ranked)
🔴 / 🟡 / 🟢

## Quick wins (one-line fixes)
- "Add `("/scan", "/laser_front")` remapping in launch file Y" — fixes 3 findings.

## Verdict
GRAPH OK / GRAPH HAS GAPS / GRAPH BROKEN
```

## Optional runtime mode
If the user added `--live` to `$ARGUMENTS` AND a ROS 2 system is currently running, instruct `dataflow-reviewer` to also run `ros2 node list`, `ros2 topic list -t`, `ros2 topic info <topic>`, `ros2 service list -t`, `ros2 action list`, and compare against the static graph. Drift between code and runtime is a finding category in itself.
