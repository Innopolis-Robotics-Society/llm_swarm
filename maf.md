🔴 Critical

  - iros_llm_swarm_bringup/launch/swarm_mapf.launch.py:119 & swarm_mapf_formation.launch.py:96 — spawn lns_motion_controller (different topic + type) under a PBS planner; every robot is dead-on-arrival. Two reviewers confirmed.
  - /llm/command has no server outside swarm_full_demo.launch.py — 4 clients hang; RViz "STOP ALL" silently broken.
  - iros_llm_swarm_robot/src/pbs_motion_controller.cpp:337 — 2 s wait_for_action_server inside subscription callback on a single-threaded executor.
  - iros_llm_swarm_mapf/src/mapf_planner_node.cpp:631,649,334-343,980-1009 and iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:577-579,888-894 — state_mutex_ held across DDS publish().
  - iros_llm_orchestrator/.../decision_server.py:84-85 & chat_server.py:183-186 — fut.result() blocks MTE thread for full 10 s LLM timeout; 4 concurrent escalations starve all subscriptions.
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  class.

  🟡 Significant

  PBS detaches its planner thread (use-after-free at shutdown); zone_map_server spun via rclcpp::spin instead of lifecycle-aware entry; orchestrator asyncio loops not stopped before destroy_node(); ~44 parameters across PBS/orchestrator/obstacle
  nodes without descriptors (CLAUDE.md hard rule); chat_server.py has a dead synchronous _get_obstacle_context; user_chat + chat_server both fire LLM on every WARN — double calls; passive_observer opens subs even when enabled=false; /mapf_grid
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - /llm/command has no server outside swarm_full_demo.launch.py — 4 clients hang; RViz "STOP ALL" silently broken.
  - iros_llm_swarm_robot/src/pbs_motion_controller.cpp:337 — 2 s wait_for_action_server inside subscription callback on a single-threaded executor.
  - iros_llm_swarm_mapf/src/mapf_planner_node.cpp:631,649,334-343,980-1009 and iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:577-579,888-894 — state_mutex_ held across DDS publish().
  - iros_llm_orchestrator/.../decision_server.py:84-85 & chat_server.py:183-186 — fut.result() blocks MTE thread for full 10 s LLM timeout; 4 concurrent escalations starve all subscriptions.
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  class.

  - iros_llm_swarm_robot/src/pbs_motion_controller.cpp:337 — 2 s wait_for_action_server inside subscription callback on a single-threaded executor.
  - iros_llm_swarm_mapf/src/mapf_planner_node.cpp:631,649,334-343,980-1009 and iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:577-579,888-894 — state_mutex_ held across DDS publish().
  - iros_llm_orchestrator/.../decision_server.py:84-85 & chat_server.py:183-186 — fut.result() blocks MTE thread for full 10 s LLM timeout; 4 concurrent escalations starve all subscriptions.
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  class.

  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  class.

  🟡 Significant
  - Lifecycle nodes: ZoneMapServer + 20× Nav2 stacks. AMCL intentionally disabled.
  - Topics: ~110. /bt/state is RELIABLE (memory entry was wrong — corrected). Per-robot odom uniformly SensorDataQoS. /mapf_grid published with zero subscribers.
  - Actions/Services: /swarm/set_goals (PBS XOR LNS2), /llm/decision, /llm/command, /llm/chat, /llm/execute_plan; 14 services under /formation/*, /obstacles/*, /doors/*.
  - Graph holes: /llm/command has no server outside full demo; /mapf_grid orphan; /goal_markers orphan; PBS bringup spawns LNS controllers; swarm_mapf.launch.py has no /map publisher in its own graph.

  Findings (top critical only — see synthesis for full list)

  🔴 Critical
  - Topics: ~110. /bt/state is RELIABLE (memory entry was wrong — corrected). Per-robot odom uniformly SensorDataQoS. /mapf_grid published with zero subscribers.
  - Actions/Services: /swarm/set_goals (PBS XOR LNS2), /llm/decision, /llm/command, /llm/chat, /llm/execute_plan; 14 services under /formation/*, /obstacles/*, /doors/*.
  - Graph holes: /llm/command has no server outside full demo; /mapf_grid orphan; /goal_markers orphan; PBS bringup spawns LNS controllers; swarm_mapf.launch.py has no /map publisher in its own graph.

  Findings (top critical only — see synthesis for full list)

  🔴 Critical

  Findings (top critical only — see synthesis for full list)

  🔴 Critical

  - iros_llm_swarm_bringup/launch/swarm_mapf.launch.py:119 & swarm_mapf_formation.launch.py:96 — spawn lns_motion_controller (different topic + type) under a PBS planner; every robot is dead-on-arrival. Two reviewers confirmed.
  - /llm/command has no server outside swarm_full_demo.launch.py — 4 clients hang; RViz "STOP ALL" silently broken.
  - iros_llm_swarm_robot/src/pbs_motion_controller.cpp:337 — 2 s wait_for_action_server inside subscription callback on a single-threaded executor.
  - iros_llm_swarm_mapf/src/mapf_planner_node.cpp:631,649,334-343,980-1009 and iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:577-579,888-894 — state_mutex_ held across DDS publish().
  - iros_llm_orchestrator/.../decision_server.py:84-85 & chat_server.py:183-186 — fut.result() blocks MTE thread for full 10 s LLM timeout; 4 concurrent escalations starve all subscriptions.
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug

  - iros_llm_swarm_bringup/launch/swarm_mapf.launch.py:119 & swarm_mapf_formation.launch.py:96 — spawn lns_motion_controller (different topic + type) under a PBS planner; every robot is dead-on-arrival. Two reviewers confirmed.
  - /llm/command has no server outside swarm_full_demo.launch.py — 4 clients hang; RViz "STOP ALL" silently broken.
  - iros_llm_swarm_robot/src/pbs_motion_controller.cpp:337 — 2 s wait_for_action_server inside subscription callback on a single-threaded executor.
  - iros_llm_swarm_mapf/src/mapf_planner_node.cpp:631,649,334-343,980-1009 and iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:577-579,888-894 — state_mutex_ held across DDS publish().
  - iros_llm_orchestrator/.../decision_server.py:84-85 & chat_server.py:183-186 — fut.result() blocks MTE thread for full 10 s LLM timeout; 4 concurrent escalations starve all subscriptions.
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  class.

  🟡 Significant

  🔴 Critical

  - iros_llm_swarm_bringup/launch/swarm_mapf.launch.py:119 & swarm_mapf_formation.launch.py:96 — spawn lns_motion_controller (different topic + type) under a PBS planner; every robot is dead-on-arrival. Two reviewers confirmed.
  - /llm/command has no server outside swarm_full_demo.launch.py — 4 clients hang; RViz "STOP ALL" silently broken.
  - iros_llm_swarm_robot/src/pbs_motion_controller.cpp:337 — 2 s wait_for_action_server inside subscription callback on a single-threaded executor.
  - iros_llm_swarm_mapf/src/mapf_planner_node.cpp:631,649,334-343,980-1009 and iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:577-579,888-894 — state_mutex_ held across DDS publish().
  - iros_llm_orchestrator/.../decision_server.py:84-85 & chat_server.py:183-186 — fut.result() blocks MTE thread for full 10 s LLM timeout; 4 concurrent escalations starve all subscriptions.
  - iros_llm_swarm_mapf_lns/src/lns2/lns2_solver.cpp:38, :87 — h_cache_.clear() unconditional on every solve; ~1.25 M extra BFS cell visits per replan. Highest-ROI single-line fix in the workspace — flagged as the top item for your in-flight LNS2
  refactor.
  - pbs_solver.hpp:49-55 + :382 — O(N²) heap-alloc per expansion + deep-copy of full path-vectors per priority-queue pop (~96 KB/pop × 5000 pops).
  - Architecture root cause: PBS emits nav_msgs/Path on /mapf_path; LNS2 emits MAPFPlan on /mapf_plan. Action contract is polymorphic; per-robot wire format is not. Fixing this collapses the two motion controllers and removes the launch-wiring bug
  class.

  🟡 Significant

  PBS detaches its planner thread (use-after-free at shutdown); zone_map_server spun via rclcpp::spin instead of lifecycle-aware entry; orchestrator asyncio loops not stopped before destroy_node(); ~44 parameters across PBS/orchestrator/obstacle
  nodes without descriptors (CLAUDE.md hard rule); chat_server.py has a dead synchronous _get_obstacle_context; user_chat + chat_server both fire LLM on every WARN — double calls; passive_observer opens subs even when enabled=false; /mapf_grid
  default-on with no subscribers (triple-confirmed); full OccupancyGrid copy on every obstacle event; ResettingObstacleLayer clears full layer every cycle × 20; PBS+LNS2 grid value-copies per solve. zone_map_server mispackaged in costmap_plugins;
  costmap_plugins/package.xml missing <export> for the plugin file.

  🟢 Backlog

  Motion-controller duplication (~250 lines), launch-file DRY violations + silent world-file divergence, PBS/LNS2 Cell int-type drift, 11 duplicated LLM params, iros_llm_swarm_simulation should COLCON_IGNORE, stress_test.cpp/smoke_test.cpp outside
  BUILD_TESTING, test_bt_runner/fleet_cmd.py shipped as production binaries, ament_target_dependencies deprecated, /goal_markers orphan, composable-node opportunity for orchestrator.

  Conflicts

  None — four reviewers converged on every overlap. Prior memory claiming /bt/state is BEST_EFFORT is wrong (publisher is RELIABLE); subscribers are compatible mixes.

  Coverage holes (out of scope, run separately)

  security-reviewer (LLM egress, prompt-injection on /bt/state-derived prompts), test-reviewer (CLAUDE.md mandates launch_testing for e-stop/watchdog), cpp-reviewer (project-wide RAII / noexcept sweep), python-reviewer (mypy strict, bare except).

  Verdict

  🔴 FAIL — swarm_mapf.launch.py ships with the wrong motion controller; the entire PBS stack is non-functional end-to-end. Add the held-mutex-across-publish in both planners, the 2 s blocking wait in a sub callback, the LLM fut.result() thread
  starvation, and the missing /llm/command server outside the full demo, and there are five independent ways the system fails to complete a single goal cycle. Fix the criticals, re-run /dataflow-audit + ros2-reviewer, then a focused performance
  pass before merge.