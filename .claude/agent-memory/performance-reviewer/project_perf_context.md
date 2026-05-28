---
name: llm_swarm perf audit context
description: Key performance findings, hot-path budgets, and architecture notes from the dev-branch audit (latest: 2026-05-27, branch dev HEAD 26e59fe)
metadata:
  type: project
---

ROS 2 Humble 20-robot swarm, CycloneDDS, dev branch HEAD 26e59fe.

**Committed hot-path budgets (from CLAUDE.md):**
- No allocation in update()/read()/write() or any >100 Hz timer/callback.
- No time.sleep / sleep_for in callbacks.
- Replan check at 2 Hz (check_schedule timer).

**Key performance findings (full audit 2026-05-27):**

## PBS Planner (iros_llm_swarm_mapf)

1. pbs_solver.hpp:49 — ConflictDetector::find_first allocates vector<vector<size_t>>
   grace(N, vector<size_t>(N, 0)) on EVERY PBS expansion. At 5000 expansions × N=20:
   200 000 heap allocations per solve. Fix: precompute grace matrix once in PBSSolver::solve().

2. pbs_solver.hpp:382 — `PBSNode node = open.top(); open.pop()` deep-copies
   vector<Path> paths. At N=20 agents × 600 steps × 16 bytes = 192 kB per pop.
   Up to 5000 pops per solve = 960 MB allocator churn. Fix: switch priority_queue
   to vector+push_heap/pop_heap and std::move out.

3. pbs_solver.hpp:508-511 — NodeKey::lens vector allocated per node expansion.
   10 000 allocs per solve. Fix: reserve(n.paths.size()) before loop.

4. mapf_planner_node.cpp:708 — ros_path.poses.push_back in make_ros_path without
   reserve. Fix: ros_path.poses.reserve(pbs_path.size()).

5. mapf_planner_node.cpp:927,950 — check_schedule at 2 Hz allocates
   vector<uint32_t> deviated_ids and string ids_str. Violates no-alloc rule.

6. mapf_planner_node.cpp:839 — expected_position O(T=600) linear scan per robot
   per 2 Hz tick. Fix: monotonic cursor per robot.

7. mapf_types.hpp / pbs_solver.hpp — duplicate inflate_gradient: called in
   validate_agent AND in PBSSolver::solve for same footprint radius.

## LNS2 Planner (iros_llm_swarm_mapf_lns) — user planning full refactor

8. soft_astar.cpp:138 — unordered_map<CT, Cost, CTHash> best_g allocated fresh
   per A* call. 4000 A* calls per solve × up to 200k expansions each.
   HIGHEST PRIORITY: port EuclideanAStarPlanner's generation-counter flat-array
   trick to soft_astar.

9. destroy_operators.cpp:17-20 — pick_one() allocates vector<AgentId> on every call
   from inside the LNS2 repair loop (~500 calls per solve).

10. destroy_operators.cpp:163-213 — Bottleneck::select builds unordered_map
    <CellIdx, unordered_set<AgentId>> traffic over ALL paths every iteration.
    O(N*T) = O(20 * 300) = 6000 insertions, called ~500 times per solve = 3M ops.
    Fix: query CollisionTable.top_cells_by_occupancy() directly.

11. collision_table.cpp:156 — build_tail_counts allocates vector<size_t>(horizon+2)
    per A* call. 4000 allocs per solve.

12. collision_table.cpp:198 — sample_random_collision allocates vector<AgentId>
    candidates from unordered_set on each call.

13. mapf_lns2_node.cpp:1043 — string concatenation under state_mutex_ at 2 Hz.

## Motion Controllers (iros_llm_swarm_robot)

14. pbs_motion_controller.cpp:454 — pd_step is allocation-free. Sound.

15. pbs_motion_controller.cpp:358-375 — send_next_chunk builds nav_msgs::Path chunk
    without reserve(). Rare path, low priority.

16. Architecture: 20 separate controller processes (40 for both variants).
    Intra-process composition with ComponentManager would eliminate 800
    DDS serialize+deserialize cycles/s for cmd_vel/follow_path.

## ResettingObstacleLayer (iros_llm_swarm_costmap_plugins)

17. resetting_obstacle_layer.cpp:12 — resetMaps() = memset 6400 bytes called
    at 10 Hz × 20 robots = 200 times/s = 1.28 MB/s cache-thrashing writes.
    Fix: dirty-cell tracking, reset only written cells. 10-30x reduction.

## Dynamic Obstacle Manager (iros_llm_swarm_obstacles)

18. dynamic_obstacle_manager.cpp:155-161 — mutex_ held across map_pub_->publish().
    DDS publish under mutex = priority inversion risk. Fix: snapshot map, release
    mutex, then publish.

19. dynamic_obstacle_manager.cpp:152 — full OccupancyGrid copy (up to 1 MB) on every
    obstacle mutation. Fix: maintain pre-built merged_map_ member, patch only delta cells.

## Formation Monitor (iros_llm_swarm_formation)

20. formation_monitor_node.py:259 — _on_odom does O(F * K) string scan per odom msg
    at 50 Hz × 20 robots = 19 000 string comparisons/s. Fix: precompute dict
    {ns: {fid: slot}} at setup.

21. formation_monitor_node.py:298 — list(fs.config.follower_ns), follower_errors_m
    list comprehension, valid_errors list comprehension = 3 new Python lists per tick
    at 10 Hz. Fix: numpy vectorize (K=19 > 16 threshold).

22. formation_monitor_node.py:295 — get_clock().now() called per formation inside loop.
    Call once at top of _publish_status.

## QoS / DDS issues

23. mapf_planner_node.cpp:210 — /robot_N/mapf_path published with KeepLast(10)
    Reliable. 20 robots × 50 kB path × 10 history = 10 MB DDS buffer. Fix:
    KeepLast(1), consider BEST_EFFORT for path topic.

**Top-5 fixes ranked by leverage:**
1. soft_astar best_g → flat generation array (LNS2 alloc pressure, 2-5x speedup)
2. PBSNode move from priority queue (PBS, 960 MB alloc churn reduction)
3. Bottleneck::select → CollisionTable query (3M insertions per solve removed)
4. DDS publish outside mutex in obstacle manager (priority inversion fix)
5. ResettingObstacleLayer dirty-cell tracking (1.28 MB/s → ~50 kB/s)

**LNS2 refactor note:** User is planning full refactor of iros_llm_swarm_mapf_lns.
Focus LNS2 findings on algorithmic issues that survive refactor, not micro-opts.

**Why:** Full multi-hot-path audit at dev HEAD 26e59fe on 2026-05-27.
**How to apply:** In future reviews, reference these findings to avoid re-flagging
known issues; focus on regressions against them. Check file:line citations
before recommending since code evolves.
