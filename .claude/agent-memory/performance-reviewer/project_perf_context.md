---
name: llm_swarm perf audit context
description: Key performance findings, hot-path budgets, and architecture notes from the dev-branch audit (2026-05-09)
type: project
---

ROS 2 Humble 20-robot swarm, CycloneDDS, dev branch HEAD e2055bc.

**Committed hot-path budgets (from CLAUDE.md):**
- No allocation in update()/read()/write() or any >100 Hz timer/callback.
- No time.sleep / sleep_for in callbacks.
- Replan check at 2 Hz (check_schedule timer).

**Key performance findings from audit:**

1. ConflictDetector::find_first (pbs_solver.hpp:~57) allocates a
   grace[N][N] vector<vector<size_t>> = 20x20 on every PBS expansion. O(N^2) per call.

2. PBSNode copy in open.top(); open.pop() (pbs_solver.hpp:382) — each PBS node
   contains std::vector<Path> paths (N paths of length up to 600), copied on every pop.

3. pick_one() in destroy_operators.cpp:17 allocates std::vector<AgentId> on every call
   from inside the LNS2 repair loop. This is inside a hot iteration loop.

4. Bottleneck destroy operator (destroy_operators.cpp:163) builds a full
   unordered_map<CellIdx, unordered_set<AgentId>> traffic map over all paths on every
   select() call. O(N * T) allocation per iteration.

5. DecisionLogger::log (logger.py:24) opens/closes a file on every LLM decision.
   Not line-buffered, not fsync'd. Benign for the orchestrator since LLM calls are rare.

6. ResettingObstacleLayer::updateBounds calls resetMaps() before every costmap update.
   resetMaps() is O(map_size). At 20 robots x Nav2 default 10 Hz = 200 resetMaps/s total.

7. RViz panel publishArrowMarkers (llm_panel.cpp:1244) called from onMarkerTick at 30 Hz:
   allocates std::string "robot_N/base_link" and MarkerArray on every tick.

8. check_schedule (mapf_planner_node.cpp:785) allocates std::vector<uint32_t> deviated_ids
   and std::string ids_str inside the 2 Hz timer. Minor (2 Hz), but violates the rule.

9. on_feedback in swarm_bt_nodes.cpp:364 uses std::ostringstream (heap alloc) inside
   the ROS executor feedback callback — not on the BT tick thread, so not a BT hot path,
   but it fires at the replan_check_hz rate (2 Hz) for every robot's feedback message.

10. LNS2 soft_astar.cpp:138 uses std::unordered_map<CT, Cost, CTHash> best_g — fresh
    allocation per A* call. With params.max_expansions=900k and neighborhood repair loops
    this dominates allocator pressure.

**LNS2 refactor note:** User is planning full refactor of iros_llm_swarm_mapf_lns.
Focus LNS2 findings on algorithmic issues that survive refactor, not micro-optimizations.

**Why:** Per user request this was a full multi-hot-path audit at dev HEAD e2055bc.
**How to apply:** In future reviews, reference these findings to avoid re-flagging known
issues; focus on regressions against them.
