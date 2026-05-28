# ROS 2 audit: llm_swarm workspace

## Inventory (condensed summary from researcher + dataflow graph)

- **Graph holes**: `mapf_lns2.launch.py` does not declare `num_robots` / `use_sim_time`, so overrides from `swarm_lns*` / `swarm_full_demo` are silently dropped; `/mapf_grid` debug topic published with no subscribers; `bt_runner` does not propagate `use_sim_time`; `passive_observer` constructs both an `ActionClient(/llm/command)` and a `BTLeafSender(/llm/execute_plan)` — one may be dead code, the other architecturally wrong.

- **`iros_llm_swarm_bringup/package.xml:10-11` + `iros_llm_swarm_local_nav/package.xml:1-19` + `iros_llm_orchestrator/.../user_prompt.py:44-51`** — Missing `exec_depend` declarations: bringup launches six packages it never declares; local_nav declares zero runtime deps; orchestrator reads `iros_llm_swarm_simulation_lite` share files with no declared dep. `rosdep install` silently misses these; clean overlay deployments fail at runtime. [arch] (Architecture Critical bullets 1, 2, 4)

- **`iros_llm_swarm_bringup/launch/swarm_mapf.launch.py:77-100`** — Inline `Node(...)` block with hardcoded PBS parameters duplicates `iros_llm_swarm_mapf/launch/mapf.launch.py`. Two divergent parameter sets for the same node. Use `IncludeLaunchDescription` + `launch_arguments`. [arch] (Architecture Warning bullet)

- **`iros_llm_swarm_bringup/launch/swarm_full_demo.launch.py:256-271`** — Two `Node(...)` declarations (`bt_runner_proxy`, `bt_runner_direct`) for the same logical node. Diverge on parameter edits. Use single Node with conditional remappings. [arch] (Architecture Warning bullet)

- **`iros_llm_swarm_bringup/CMakeLists.txt` + `iros_llm_swarm_mapf/CMakeLists.txt:34` + `iros_llm_swarm_robot/CMakeLists.txt:23,35`** — `ament_target_dependencies` deprecated since Humble. Use `target_link_libraries(... PUBLIC rclcpp::rclcpp ...)`. [ros2] (F-BUP-2)

- **`iros_llm_swarm_bt/src/bt_runner.cpp:91`** — Misleading comment ("send via `ros2 action send_goal /llm/command`") — actual sender is `passive_observer`. [dataflow] (S-4)

**Conflict 3: `swarm_full_demo.launch.py` motion_controllers cross-planner subscribers (dataflow C-4)**
- **dataflow-reviewer**: Self-corrected from 🔴 to "cleared" within the same finding (re-analysis shows mutual exclusion is correct).
- **Decision**: Accept dataflow-reviewer's self-correction. **Not listed.**
## Findings


### 🔴 Critical (block production / 20-robot scale)

- **`iros_llm_orchestrator/iros_llm_orchestrator/decision_server.py:83-87` + `execute_server.py:63-66`** — `fut.result()` blocks the rclpy executor thread for the entire LLM inference (`timeout_sec=10s` for decision, `120s` for execute). With `MultiThreadedExecutor(4)` and unconditional `GoalResponse.ACCEPT`, 4 concurrent goals fully starve `/bt/state` and `/llm/events` subscriptions. [ros2] (F-ORC-1, F-ORC-2)

- **`iros_llm_swarm_bringup/launch/swarm_lns.launch.py:83-86` + `swarm_lns_formation.launch.py:85-89` + `swarm_full_demo.launch.py:166-176`** — `num_robots` and `use_sim_time` passed as `launch_arguments` to `mapf_lns2.launch.py`, which declares neither. ROS 2 silently drops them; the YAML hardcodes both (`use_sim_time: true`). Any LNS2 deployment with `num_robots != 20` plans for 20 phantom robots and subscribes to 10 silent odom topics. Real-hardware runs cannot disable sim-time without editing YAML. [dataflow] (C-1, W-1)

- **`iros_llm_swarm_mapf/src/mapf_planner_node.cpp:210` + `iros_llm_swarm_mapf_lns/.../mapf_lns2_node.cpp` (mapf_path/plan pubs)** — `/robot_N/mapf_path` and `/robot_N/mapf_plan` publish with VOLATILE durability. A motion-controller restart mid-mission silently sits IDLE for up to `replan_cooldown_sec=15s` until the next replan fires. [dataflow] (C-3)

- **`iros_llm_swarm_mapf_lns/src/lns2/soft_astar.cpp:138`** — `std::unordered_map<CT, Cost, CTHash> best_g` freshly allocated on every A* call (~4000 calls per LNS2 solve, each up to 200k expansions). Dominant allocator-pressure point in the system. PBS planner already solved this with generation-counter flat arrays (`euclidean_astar.hpp:167-172`); LNS2 must adopt the same pattern. Estimated 2-5× LNS2 repair-loop speedup. [perf] (perf 2.3, top-5 rank 1)


- **`iros_llm_swarm_mapf_lns/src/lns2/destroy_operators.cpp:166-213`** — `Bottleneck::select` rebuilds a full `unordered_map<CellIdx, unordered_set<AgentId>>` traffic map (~6000 entries) on every LNS2 iteration → ~3M hash inserts per solve. `CollisionTable::vertex_occ_` already maintains this data; expose `top_cells_by_occupancy(K)`. [perf] (perf 2.2, top-5 rank 3)

- **`iros_llm_swarm_costmap_plugins/src/resetting_obstacle_layer.cpp:8-14`** — `resetMaps()` zeros the full per-robot costmap on every `updateBounds` (10 Hz × 20 robots = 200 memsets/s = ~1.28 MB/s of cache-trashing writes). At N=20 the dirty-cell mark-and-sweep is required. Promoted to 🔴 because perf flags it at scale and ros2-reviewer flags it as the only hot path touching every Nav2 update cycle. [perf] [ros2] (perf 4.1, F-CST-1)

- **`iros_llm_swarm_bringup/launch/swarm_full_demo.launch.py:256-270`** — `bt_runner_proxy` and `bt_runner_direct` lack `parameters=[{'use_sim_time': use_sim_time}]`. `/bt/state` timestamps run wall-clock; `passive_observer` cooldown compares sim-time `now()` against wall-time message stamps — cooldown is always exceeded, so every WARN/ERROR triggers an immediate LLM decision. [dataflow] (C-4)
















