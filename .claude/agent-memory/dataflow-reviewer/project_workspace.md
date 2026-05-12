---
name: llm_swarm workspace overview
description: Core architecture facts for the llm_swarm ROS2 workspace — used to avoid re-deriving graph structure on every audit
type: project
---

20-robot swarm, Stage 2D sim (one_tf_tree:true, enforce_prefixes:true). All frames robot_N/... in one TF tree.

**Two planners, mutually exclusive:**
- PBS: `iros_llm_swarm_mapf/mapf_planner_node` → `/swarm/set_goals` + `/robot_N/mapf_path`
- LNS2: `iros_llm_swarm_mapf_lns/mapf_lns2_node` → `/swarm/set_goals` + `/robot_N/mapf_plan`

**Two motion controllers, matched to planner:**
- pbs_motion_controller: reads `/robot_N/mapf_path` (nav_msgs/Path)
- lns_motion_controller: reads `/robot_N/mapf_plan` (MAPFPlan msg), publishes `/robot_N/follower_status`

**Central action contracts:**
- `/swarm/set_goals` (SetGoals.action) — MAPF planner server, BT MapfPlan client
- `/llm/decision` (LlmDecision.action) — decision_server serves, BT MapfPlan/SetFormation/DisableFormation clients
- `/llm/command` (LlmCommand.action) — LlmCommandReceiver (BT node) serves, passive_observer + RViz panel + tool clients
- `/llm/chat` (LlmChat.action) — chat_server serves, RViz panel client
- `/llm/execute_plan` (LlmExecutePlan.action) — execute_server serves, RViz panel client
- `/robot_N/follow_path` (FollowPath.action) — Nav2 controller_server serves, motion controllers client

**Why:** AMCL disabled, static map→robot_N/odom TF only. 20-robot DDS ceiling, CycloneDDS required.
