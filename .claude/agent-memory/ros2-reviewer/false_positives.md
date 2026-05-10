---
name: False positive patterns
description: Patterns that look wrong but are intentional — skip on future reviews
type: feedback
---

## Intentional patterns

- **`sleep_for` in `test_bt_runner.cpp`**: These are in `run_scenario()`, a detached scenario thread, not in a ROS callback. They are deliberate pacing delays between BT steps.

- **`wait_for_action_server(2s)` in `MapfPlan::onStart()`**: Called from BT node `onStart()`, which runs in the BT tick thread (not a ROS executor callback). Blocking for 2s there is awkward but does not deadlock the executor.

- **`wait_for_service(2s)` in `SetFormation::start_service_call()` and `DisableFormation::start_service_call()`**: Same — BT tick thread. Blocks the BT tree for 2s if service is down. Not a ROS executor deadlock.

- **`wait_for_action_server(2s)` in `path_follower_node.cpp:400` and `motion_controller_node.cpp:337`**: Called from result callbacks (executor thread). This IS a blocking call in a callback — see issue tracker.

- **`LlmCommandReceiver` action server created inside a BT node**: Intentional architecture — the action server's `handle_accepted` writes to a mutex-protected slot; actual blackboard writes happen only in `tick()` (BT thread). Thread model is sound.

- **No callback groups declared**: Both PBS and LNS2 nodes use `MultiThreadedExecutor(2)` with default (MutuallyExclusive) callback groups. This is correct for their design: planning runs outside the executor entirely (detached/joined thread), so the two executor threads only run subscriptions and timers.

- **`TransientLocal` publisher for `/bt/state` with `BestEffort` subscriber (PassiveObserver)**: A Reliable publisher and BestEffort subscriber are compatible — the subscriber receives whatever it can. This is a deliberate bandwidth trade-off.

**Why:** Confirmed by reading the code comments and architecture. These are not bugs.
