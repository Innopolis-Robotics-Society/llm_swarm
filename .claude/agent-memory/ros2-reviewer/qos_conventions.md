---
name: QoS conventions
description: Per-topic QoS settings confirmed in the codebase; used to skip false-positive mismatches
type: reference
---

## Confirmed QoS settings

| Topic | Publisher QoS | Subscriber QoS | Notes |
|-------|--------------|----------------|-------|
| `/map` | TransientLocal Reliable depth-1 (nav2_map_server) | TransientLocal Reliable depth-1 (both planners) | Correct match |
| `/formations/config` | TransientLocal Reliable depth-1 (formation_manager) | TransientLocal Reliable depth-1 (motion_controller, path_follower, formation_monitor) | Correct match |
| `/formations/status` | Reliable depth-10 (formation_monitor) | Reliable depth-10 (BTStatePublisher) | Correct match |
| `/bt/state` | Reliable depth-20 (BTStatePublisher) | BestEffort depth-20 (PassiveObserver) | BestEffort sub under Reliable pub is compatible per DDS spec |
| `/robot_N/odom` | BestEffort sensor_data (stage_ros2, upstream default, no override in workspace) | BROKEN in PBS planner + pbs_motion_controller + formation nodes; CORRECT in LNS2 planner + lns_motion_controller |
| `/robot_N/mapf_path` | depth-10 default (PBS planner) | n/a | Consumers are motion_controller and path_follower |
| `/mapf_grid` | TransientLocal Reliable depth-1 (LNS2, debug) | n/a | |
| `/llm/events` | Reliable depth-10 | n/a | |

## Confirmed broken /robot_N/odom subscribers (use Reliable default, publisher is BestEffort)

- `iros_llm_swarm_mapf/src/mapf_planner_node.cpp:162-163` — `create_subscription(..., 10, ...)` — bare int, expands to QoS(10) = Reliable VOLATILE KEEP_LAST(10). Receives nothing from Stage.
- `iros_llm_swarm_robot/src/pbs_motion_controller.cpp:118-120` — own odom: `create_subscription(..., "/" + ns_ + "/odom", 10, ...)` — same default.
- `iros_llm_swarm_robot/src/pbs_motion_controller.cpp:188-190` — leader odom: `create_subscription(..., "/" + leader_ns_ + "/odom", 10, ...)` — same default.
- `iros_llm_swarm_formation/iros_llm_swarm_formation/formation_manager_node.py:173-176` — `create_subscription(Odometry, f"/{ns}/odom", ..., 10)` — int 10, rclpy default = Reliable.
- `iros_llm_swarm_formation/iros_llm_swarm_formation/formation_monitor_node.py:244-249` — `create_subscription(Odometry, f"/{ns}/odom", ..., 10)` — int 10, rclpy default = Reliable.

## Confirmed correct /robot_N/odom subscribers

- `iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp:301-304` — `rclcpp::SensorDataQoS()` — BestEffort VOLATILE KEEP_LAST(5). Matches Stage publisher.
- `iros_llm_swarm_robot/src/lns_motion_controller.cpp:209-211` — own odom: `rclcpp::SensorDataQoS()`.
- `iros_llm_swarm_robot/src/lns_motion_controller.cpp:290-292` — leader odom: `rclcpp::SensorDataQoS()`.

## Stage QoS override status

No QoS override parameters exist for `stage_ros2` in the workspace. `warehouse_swarm.launch.py` passes only `world_file`, `one_tf_tree`, `enforce_prefixes`, `use_sim_time`, `is_depth_canonical`, `base_watchdog_timeout` to the stage node. The upstream `rmw_qos_profile_sensor_data` (BestEffort) is in effect.
