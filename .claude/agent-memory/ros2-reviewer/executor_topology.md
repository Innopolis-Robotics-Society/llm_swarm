---
name: Executor topology
description: Executor and callback-group choices for every node in the workspace
type: reference
---

## C++ nodes

| Node | Executor | Callback groups | Notes |
|------|----------|-----------------|-------|
| `mapf_planner_node` (PBS) | MultiThreadedExecutor(2) | Default (no explicit groups) | Planning runs in detached thread; replanning runs in timer callback (executor thread) |
| `mapf_lns2_node` (LNS2) | MultiThreadedExecutor(2) | Default (no explicit groups) | Planning runs in `execute_thread_` (joined); cancel poll runs in `cancel_thread_` |
| `motion_controller_node` | SingleThreadedExecutor (rclcpp::spin) | Default | |
| `path_follower_node` (LNS2) | SingleThreadedExecutor | Default | |
| `test_bt_runner` | MultiThreadedExecutor(4) | Default | BT runs in scenario thread; ROS callbacks in executor threads |

## Python nodes

| Node | Executor | Notes |
|------|----------|-------|
| `LlmDecisionServer` | MultiThreadedExecutor(4) | `_execute` bridges ROS executor thread to private asyncio loop via `run_coroutine_threadsafe` + `fut.result()` — this BLOCKS the executor thread |
| `ExecuteServer` | MultiThreadedExecutor(4) | Same blocking bridge pattern |
| `PassiveObserver` | MultiThreadedExecutor(4) | `_on_state` callback dispatches to asyncio loop non-blocking; correct |
| `FormationManagerNode` | SingleThreadedExecutor | Services are synchronous; correct |
| `FormationMonitorNode` | SingleThreadedExecutor | Timer + odom callbacks; correct |

## Critical finding
`LlmDecisionServer._execute` and `ExecuteServer._execute` block the ROS executor thread for the full LLM inference duration (up to `timeout_sec` = 10-15s). With `MultiThreadedExecutor(4)` and up to 4 concurrent action goals, all 4 threads can be consumed by LLM waits, starving map/odom subscriptions.
