---
name: Known dataflow issues — 2026-05-10 audit
description: Confirmed bugs and accepted/known patterns from full dataflow audit on 2026-05-10
type: project
---

## Confirmed bugs (not yet fixed as of 2026-05-10)

### Critical
1. **PBS odom QoS mismatch**: `pbs_motion_controller.cpp:119` subscribes to `/robot_N/odom` with `QoS(10)` = RELIABLE. Stage_ros2 publishes odom with sensor_data QoS (BEST_EFFORT). Does NOT connect. PBS formation-follower mode also broken via leader odom (same QoS). LNS2 correctly uses `SensorDataQoS()`.

2. **formation_manager + formation_monitor odom QoS mismatch**: Both subscribe to `/robot_N/odom` with `QoS(10)` (Python default = RELIABLE). Same Stage BEST_EFFORT mismatch as above.

3. **PBS planner odom QoS mismatch**: `mapf_planner_node.cpp:163` subscribes with `QoS(10)` = RELIABLE vs Stage BEST_EFFORT.

4. **DATAFLOW_AUDIT.md documents /llm/execute as action name** but execute_server.py:52 actually serves `/llm/execute_plan`. The enumeration doc is wrong; code and RViz panel agree on `/llm/execute_plan`.

### Warnings
1. **Orphan publishers**: `/fleet/mode`, `/fleet/mapf_ok`, `/fleet/formation_enabled` published by test_bt_runner — no in-workspace subscribers (external observability use only via ros2 topic echo or rqt).
2. **Double /bt/state subscription in chat_server**: chat_server.py:91 creates its own sub AND BTLeafSender.py:55 creates a second sub on the same node. Not a bug but wasteful.
3. **DATAFLOW_AUDIT.md says /llm/command is "published by passive_observer"** (action section) — wrong framing. passive_observer is the action CLIENT; LlmCommandReceiver (BT) is the server.

## Accepted patterns (not bugs)

- `/formations/config` TRANSIENT_LOCAL RELIABLE depth=1: confirmed correct in both publisher (formation_manager_node.py:125-129) and all subscribers (pbs_motion_controller:123, lns_motion_controller:214, formation_monitor:128-131).
- `/bt/state` published RELIABLE QoS(20), passive_observer + chat_server subscribe BEST_EFFORT: compatible (pub reliable → sub best_effort degrades guarantee but connects).
- RViz panel subscribes `/bt/state` with `QoS(20).reliable()`: matches publisher exactly.
- AMCL disabled intentionally — static `map → robot_N/odom` TF from static_transform_publisher per robot.
- `/mapf_grid` (LNS2 debug): orphan publisher — documented debug interface, acceptable.
- `/llm/events`: 3 publishers (decision_server, passive_observer, chat_server), 1 subscriber (RViz panel). Legitimate fan-in to observability panel.
- Two `/bt/state` readers (passive_observer, chat_server, RViz panel, BTLeafSender) all on the same topic with compatible QoS: intentional multi-consumer design.
