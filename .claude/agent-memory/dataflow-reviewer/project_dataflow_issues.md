---
name: known-dataflow-issues
description: Confirmed bugs and accepted/known patterns from full dataflow audits — updated 2026-05-27
metadata:
  type: project
---

## Confirmed bugs (as of 2026-05-27 audit)

### Critical

1. **`mapf_lns2.launch.py` drops `num_robots` / `use_sim_time`**: launch file declares only `params_file` and `log_level`. Parent launches (`swarm_lns.launch.py`, `swarm_lns_formation.launch.py`, `swarm_full_demo.launch.py`) pass `num_robots` and `use_sim_time` as `launch_arguments` — they are silently dropped. Node reads from YAML (hardcoded 20 / true). Fix: declare args in `mapf_lns2.launch.py`, override in Node params.

2. **Dual `/robot_N/cmd_vel` publishers in LNS2 stack**: `lns_motion_controller.cpp:240` and `plan_publisher.cpp:19` both publish `/robot_N/cmd_vel`. The planner publishes a zero-Twist on action cancel; the controller publishes its FORMATION PD step concurrently. Race condition on emergency stop. Fix: remove `cmd_vel_pubs_` from `PlanPublisher`; let the empty MAPFPlan cancel signal the controller to stop.

3. **VOLATILE QoS on `/robot_N/mapf_path` and `/robot_N/mapf_plan`**: Published once per plan, VOLATILE. A late-joining or restarted controller misses the already-published path and sits idle until next replan (up to replan_cooldown_sec). Fix: use TRANSIENT_LOCAL depth 1 on both publisher and subscriber.

4. **`bt_runner` missing `use_sim_time` parameter in launch**: `swarm_full_demo.launch.py` does not pass `use_sim_time` to `bt_runner_proxy` or `bt_runner_direct`. `/bt/state` timestamps are wall-clock, breaking `passive_observer` cooldown in sim. Fix: add `parameters=[{'use_sim_time': use_sim_time}]` to both bt_runner nodes.

### Warnings

1. **`/mapf_grid` orphan publisher**: `mapf_lns2_node.cpp:381` publishes OccupancyGrid on `/mapf_grid` with TRANSIENT_LOCAL/RELIABLE, `publish_debug_grid` defaults to `true`. No subscriber exists in workspace. ~40KB per replan, kept latched in memory.

2. **`PassiveObserver` constructs `BTLeafSender` (→ `/llm/execute_plan`) but should only use `/llm/command` ActionClient for channel 2**. Needs inspection of `_trigger_llm` body to confirm whether both paths fire.

## Resolved issues (fixed since 2026-05-10 audit)

- **PBS odom QoS mismatch (C-1 from prior audit)**: `pbs_motion_controller.cpp:119` now uses `rclcpp::SensorDataQoS()` — VERIFIED at line 119: `own_odom_sub_ = create_subscription<Odometry>("/" + ns_ + "/odom", rclcpp::SensorDataQoS(), ...)`. Fixed.
- **formation_manager + formation_monitor odom QoS**: Both now use `BEST_EFFORT` profiles (formation_manager_node.py:209, formation_monitor_node.py:248). Fixed.
- **PBS planner odom QoS**: `mapf_planner_node.cpp:163` uses `rclcpp::SensorDataQoS()`. Fixed.
- **LlmCommandReceiver IS an action server**: `swarm_bt_nodes.cpp:903` — `rclcpp_action::create_server<LlmCommand>(node, "/llm/command", ...)`. Fixed.

## Accepted patterns (not bugs)

- `/formations/config` TRANSIENT_LOCAL RELIABLE depth=1: confirmed correct in both publisher (formation_manager_node.py) and all subscribers (pbs_motion_controller:123, lns_motion_controller:214, formation_monitor_node:133).
- `/bt/state` published RELIABLE QoS(20) (via `bt_state_qos()` in swarm_bt_nodes.hpp:208), passive_observer + chat_server subscribe BEST_EFFORT: compatible.
- RViz panel subscribes `/bt/state` with `QoS(20).reliable()`: matches publisher exactly.
- AMCL disabled intentionally — static `map → robot_N/odom` TF from static_transform_publisher per robot in robot_local_nav.launch.py.
- `/mapf_grid` (LNS2 debug): orphan publisher — still active, now elevated to Warning (W-6) due to TRANSIENT_LOCAL memory overhead.
- `/llm/events`: 3 publishers (decision_server, passive_observer, chat_server), 1 subscriber (RViz panel). Legitimate fan-in; file-based logging independent of topic.
- Two `/bt/state` reader groups (passive_observer BE/20, chat_server BE/20, RViz panel R/20) all compatible with RELIABLE publisher.
- `/fleet/mode`, `/fleet/mapf_ok`, `/fleet/formation_enabled`: orphan publishers from bt_runner for external monitoring — accepted.
- LNS2 only: PBS planner never publishes `/robot_N/mapf_plan`; LNS2 never publishes `/robot_N/mapf_path`. The two planners are mutually exclusive and launched with matching controller type.

**Why:** AMCL disabled, static TF only. 20-robot DDS ceiling, CycloneDDS required. Planners are mutually exclusive in all launch files (conditioned on `planner:=lns|pbs`).
