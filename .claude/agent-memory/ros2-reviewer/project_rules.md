---
name: Project rules
description: Hard rules from CLAUDE.md used as reviewer ammunition
type: project
---

## Hard project rules (from CLAUDE.md)

- No allocation in `update()`, `read()`, `write()`, or any >100 Hz timer callback.
- No `time.sleep` / `std::this_thread::sleep_for` in callbacks.
- QoS must be justified; the four defaults (sensor/cmd/status/latched) are presumed.
- Lifecycle nodes for hardware-owning nodes; plain nodes for everything else.
- `MutuallyExclusive` callback groups by default; `Reentrant` requires comment proving thread-safety.
- Every parameter has a descriptor with type and range.
- C++: no `new`/`delete`, RAII, `noexcept` destructors.
- Python: type hints, no bare `except`, numpy-vectorise anything >16 elements.
- 20-robot ceiling — CycloneDDS with `cyclonedds_swarm.xml` mandatory.
- AMCL intentionally disabled — do not recommend re-enabling.
- LNS2 package (`iros_llm_swarm_mapf_lns`) is planned for full refactor — flag what's broken but note the refactor context.

## Known project-wide anti-patterns (from 2026-05-27 full audit)

- **Lock-held-across-DDS-publish**: `mapf_planner_node.cpp` holds `state_mutex_` across `publisher->publish()` calls in `do_plan()` (line 631/649), `handle_cancel()` (line 334/343), and `trigger_replan()` (~1003/1006). Also `dynamic_obstacle_manager.cpp:152-162` holds `mutex_` across `map_pub_->publish()`. Fix: snapshot data, release lock, then publish.

- **`wait_for_action_server(N s)` in result/timer/subscription callbacks**: Both `pbs_motion_controller.cpp:337` and `lns_motion_controller.cpp:459` block executor threads for 2s inside callbacks. Pattern is systemic — check any new motion controller or nav2 client wrapper.

- **`fut.result()` asyncio bridge without timeout guard**: `decision_server.py:83-87` and `execute_server.py:63-66` bridge ROS executor thread to asyncio loop but `.result()` blocks the executor thread for the full LLM inference duration. Fix: use `fut.result(timeout=N)` or restructure as non-blocking.

- **Parameters declared without descriptors**: `mapf_planner_node.cpp` (18 params, no descriptors), `bt_runner.cpp` (1 param), `FormationManagerNode` (7 params), `FormationMonitorNode` (6 params), `LlmDecisionServer` (14 params). LNS2 node and lns_motion_controller are the positive examples — they use helper functions `make_int_desc()`, `make_double_desc()`, etc.

**Why:** These come directly from the project's CLAUDE.md and represent the team's explicit quality bar.
**How to apply:** Use these as the primary benchmark when flagging issues. A violation of these rules is always worth flagging regardless of whether it appears in a diff or a full-repo audit.
