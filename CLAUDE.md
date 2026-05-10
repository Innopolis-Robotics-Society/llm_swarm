# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ROS 2 Humble (Jazzy forward-compatible) multi-robot swarm simulation. ~20 differential-drive robots in a 2D Stage world with per-robot Nav2, two MAPF planners (PBS in `iros_llm_swarm_mapf`, LNS2 in `iros_llm_swarm_mapf_lns`), leader-follower formations, BehaviorTree.CPP v3 orchestration, and an LLM orchestrator with two control channels (reactive + passive observer). All packages live as a flat `colcon` workspace mounted into the dev container at `/home/fabian/ros2_ws/src`.

## Development environment

All development happens inside Docker. Two service variants share `x-common`: `terminal` (GPU, default) and `terminal-cpu`.

```bash
docker compose up terminal         # or: docker compose up terminal-cpu
docker compose exec terminal bash
```

Inside the container, every shell needs the swarm DDS env (the compose file exports these for the entry shell, but new shells / `tmux` panes must source it):

```bash
source /home/fabian/ros2_ws/install/setup.bash
source /home/fabian/ros2_ws/src/scripts/setup_swarm_env.sh
# equivalent to setting RMW_IMPLEMENTATION=rmw_cyclonedds_cpp,
# ROS_DOMAIN_ID=42, CYCLONEDDS_URI=file://.../scripts/cyclonedds_swarm.xml
```

CycloneDDS with `cyclonedds_swarm.xml` is **mandatory** above ~10 robots — Fast DDS discovery storms make the swarm unstable. `setup_swarm_env.sh` also bumps `net.core.rmem_*` via sudo; failure there is non-fatal.

## Build

`iros_llm_swarm_interfaces` defines msg/srv/action used by every other package — build it (and `iros_llm_swarm_mapf_lns`, which generates more types into the action server) **before** the rest when those types change:

```bash
cd /home/fabian/ros2_ws
colcon build --packages-select iros_llm_swarm_interfaces iros_llm_swarm_mapf_lns
colcon build                                            # then everything
```

`--symlink-install` is the default for Python packages. `iros_llm_swarm_docs` carries a `COLCON_IGNORE` and is not part of the normal build.

## Launch

Top-level launch files live in `iros_llm_swarm_bringup/launch/`:

| Launch file | Purpose |
| --- | --- |
| `swarm_warehouse.launch.py` | Stage + Nav2 + RViz only (no MAPF, no BT) |
| `swarm_mapf.launch.py` | Stage + Nav2 + PBS planner + RViz |
| `swarm_lns.launch.py` | Stage + Nav2 + LNS2 planner + RViz |
| `swarm_mapf_formation.launch.py` / `swarm_lns_formation.launch.py` | Above + formation manager |
| `swarm_full_demo.launch.py` | Full stack + BT runner + LLM orchestrator (mock LLM) |

Launch arguments (`swarm_warehouse.launch.py` and downstream): `scenario` (`cave` \| `large_cave` \| `warehouse_2` \| `warehouse_4`, default `cave`), `num_robots` (default `20`), `world_file`, `rviz_cfg`, `use_sim_time`. The simulator always spawns the robots defined in the world file; `num_robots` only limits how many are *controlled*.

The full demo additionally accepts `enable_passive_observer:=true` to activate channel 2 (the proactive observer that watches `/bt/state` and pushes `LlmCommand` on `/llm/command`). Channel 1 (BT-driven `/llm/decision` action) is always on.

## Testing

There is no formal `colcon test` suite for the swarm logic — testing is interactive against a running stack. After launching one of the stacks above:

```bash
# All robots to a single point (PBS or LNS2 stack)
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 13.5 --goal-y 16.5

# Random goals within a radius of map centre, custom timeout
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0 --timeout 300

# Goals from JSON: {"goals":[{"id":0,"gx":15.0,"gy":12.5}, ...]}
ros2 run iros_llm_swarm_mapf test_send_goals --json-file src/iros_llm_swarm_mapf/config/goals_1.json

# Nav2-only smoke test (drives every robot toward the goal until they hit a wall)
ros2 run iros_llm_swarm_local_nav test_local_planner --num 20 --goal-x X --goal-y Y

# BT runner under the full demo
ros2 run iros_llm_swarm_bt test_bt_runner
ros2 run iros_llm_swarm_bt fleet_cmd --scenario {simple|stress|unreachable|idle}
```

`test_send_goals` calls `/swarm/set_goals` and stays attached for the entire plan→execute→arrive lifecycle; `Ctrl+C` cancels the action and stops every robot.

When unit-style tests do exist (e.g. `iros_llm_orchestrator/test/`), run them with the standard:

```bash
colcon test --packages-select <pkg> && colcon test-result --verbose
```

## Architecture

### Package map

| Package | Lang | Role |
| --- | --- | --- |
| `iros_llm_swarm_interfaces` | msg/srv/action | Shared types — single source of truth |
| `iros_llm_swarm_mapf` | C++17 | PBS planner: Euclidean A\* + capsule conflict + gradient inflation |
| `iros_llm_swarm_mapf_lns` | C++17 | LNS2 planner — same `/swarm/set_goals` contract, scalable alternative |
| `iros_llm_swarm_local_nav` | Python | Spawns one Nav2 stack per robot under `robot_N/` namespace |
| `iros_llm_swarm_costmap_plugins` | C++17 | `ResettingObstacleLayer` — fixes ghost-trail bug in `nav2_costmap_2d::ObstacleLayer` |
| `iros_llm_swarm_formation` | Python | Leader-follower formations + manager + monitor |
| `iros_llm_swarm_bt` | C++17 | BehaviorTree.CPP v3 nodes (`MapfPlan`, `SetFormation`, `DisableFormation`, `CheckMode`) + `test_bt_runner`, `fleet_cmd` |
| `iros_llm_orchestrator` | Python | LLM glue: `decision_server` (channel 1), `passive_observer` (channel 2), `chat_server`, `execute_server`, `user_chat_node` |
| `iros_llm_rviz_panel` | C++ (Qt) | RViz2 operator panel — chat tab, status, goal markers, STOP ALL |
| `iros_llm_rviz_tool` | C++ | RViz2 tool plugin (goal pickers etc.) |
| `iros_llm_swarm_simulation_lite` | Python | Stage 2D simulator launcher, world files, robot `.inc` |
| `iros_llm_swarm_simulation` | Python | Gazebo Harmonic (3D) — **not stable for 20 robots, prefer `_lite`** |
| `iros_llm_swarm_robot` | mixed | Per-robot motion controllers launch |
| `iros_llm_swarm_bringup` | Python | Top-level launch composition |
| `iros_llm_swarm_docs` | rosdoc2 | Carries `COLCON_IGNORE` |

### Central contracts

- **`/swarm/set_goals`** (`iros_llm_swarm_interfaces/action/SetGoals`) — long-lived action that owns the entire `validating → planning → executing → replanning → succeeded/failed` lifecycle for the whole fleet. Both PBS and LNS2 servers expose it identically. Cancellation stops every robot. Schedule monitoring + replanning are internal; clients (BT, LLM) see one atomic op.
- **`/robot_{id}/mapf_path`** (`nav_msgs/Path`) — per-robot path stream; each `PoseStamped.header.stamp = plan_time + step × time_step_sec` (temporal sync). Followers chunk and dispatch to Nav2 `follow_path`.
- **`/formations/config`** — `TRANSIENT_LOCAL`, `RELIABLE`, depth 1 (latched). Full snapshot of every formation. Status flows in parallel on `/formations/status` (`STATE_INACTIVE → FORMING → STABLE ↔ DEGRADED → BROKEN`).
- **`/llm/decision`** (action, channel 1) — BT nodes call it on WARN/ERROR; reactive.
- **`/llm/command`** (`LlmCommand`, channel 2) — `passive_observer` pushes commands here from its own analysis of `/bt/state`. `LlmCommandReceiver` applies them to the BT blackboard.
- **`/bt/state`** — heartbeat/status of the running behaviour tree; both LLM channels and the RViz panel consume it.
- LLM datasets accumulate in `~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl` (channel 1) and `~/.ros/llm_commands/decisions_YYYYMMDD.jsonl` (channel 2).

### PBS planner internals (`iros_llm_swarm_mapf`)

- Grid downsampled from the 0.05 m map to `pbs_resolution` (default 0.2 m). N-connected move set auto-sized: `reach = max(1, floor(max_speed * time_step_sec / resolution))`. Multi-cell moves traced with Bresenham — any blocked cell on the trace rejects the move and wall-gradient penalties accumulate along it.
- Two radii per agent: hard footprint (collision-free) + soft inflation zone (gradient penalty, curve `quadratic` by default). Same model for walls.
- Cost = `urgency * max_speed * dt + Euclidean distance + wall penalty + agent proximity penalty`. `urgency < 1.0` is unsupported.
- Conflict detection is capsule-vs-capsule (segment-segment distance ≤ Σ radii) — replaces the old vertex/edge split.
- Heuristic: backward Dijkstra from each unique goal cell, cached, using the same move set as A\*.
- PBS orchestration: sequential plan furthest→stationary first; if blocked, fall back to independent A\* and let PBS resolve. Branch heuristic = furthest-goal agent gets priority.
- Schedule monitor at `replan_check_hz` (2 Hz). Triggers replan when any robot is more than `replan_threshold_m` (1.0 m) off plan; static `replan_cooldown_sec` (15 s) between replans.
- **Constraint enforced at startup**: `footprint_radius >= 0.7 * pbs_resolution`. Defaults (0.22 m, 0.2 m) just satisfy it.

### Per-robot Nav2 stack (`iros_llm_swarm_local_nav`)

For each `robot_N`: `controller_server` (DWB/MPPI) + `behavior_server` + `lifecycle_manager` + a static `map → robot_N/odom` TF publisher. Robots are spawned with a 0.3 s stagger to avoid lifecycle races. A single global `map_server` is shared. **AMCL is intentionally disabled** — at 20 robots, cross-robot LiDAR interference makes localization drift. The static identity TF is the localization until that is solved.

Custom costmap layer `ResettingObstacleLayer` (in `iros_llm_swarm_costmap_plugins`) is the drop-in replacement for `nav2_costmap_2d::ObstacleLayer`. Use it; the stock layer leaves ghost-trail cells between LiDAR rays.

Nav2 YAML uses `<robot_namespace>` placeholders rewritten at launch time via `ReplaceString`, then routed through `RewrittenYaml(root_key=<namespace>)` so node-level params land correctly under the namespace.

### TF / namespacing

Stage runs with `one_tf_tree: true` + `enforce_prefixes: true`, so every robot frame is `robot_N/...` and the whole fleet shares one TF tree. There is exactly one `map → robot_N/odom` per robot (static). Don't add more — AMCL must remain off.

### LLM orchestrator (`iros_llm_orchestrator`)

Two-channel design:

- **Channel 1 (reactive)** — `decision_server` exposes `/llm/decision` (action). BT nodes (`MapfPlan`, `SetFormation`) call it directly when they hit WARN/ERROR. Always on in the full demo.
- **Channel 2 (proactive)** — `passive_observer` subscribes to `/bt/state`, decides on its own whether to intervene, and writes `LlmCommand` to `/llm/command`. Off by default (`enable_passive_observer:=false`); enabling it does **not** disable channel 1 — both run.

`chat_server` exposes the `/llm/chat` action consumed by the RViz panel. `execute_server` owns command execution. The default backend is a mock; swap it via the orchestrator config to point at a real model.

## Stack and tooling conventions

- **ROS 2 Humble** is the target; **Jazzy** must keep working. Flag any Rolling-only API.
- **Python** for mission logic, glue, MoveIt2 high-level. **C++17/20** for control loops, perception, ros2_control hardware, anything > 100 Hz.
- **Python over MATLAB** for all numerical work — `numpy` + `scipy`.
- **Embedded:** ESP32 + micro-ROS, TMC2209 over UART, motor / sensor I/O over CAN (SocketCAN).
- **Build:** `colcon` + `ament_cmake` (C++) / `ament_python` (Python). `--symlink-install` default.

### Style

- C++: clang-format with the ROS 2 default (`.clang-format` from `ros2/style`), clang-tidy via `ament_clang_tidy`.
- Python: `black`, `ruff` (replaces flake8 + isort), `mypy --strict` for new modules. Type hints mandatory on public functions.
- Commits: conventional commits (`feat(perception): ...`).

## Hard project rules (reviewer ammunition)

- No allocation in `update()`, `read()`, `write()`, or any > 100 Hz timer.
- No `time.sleep` / `std::this_thread::sleep_for` in callbacks.
- QoS must be justified; the four defaults in `ros2-implementer.md` are presumed.
- Lifecycle nodes for hardware-owning nodes; plain nodes for everything else.
- `MutuallyExclusive` callback groups by default; `Reentrant` requires a comment proving thread-safety.
- E-stop and watchdog topics use `Reliable, TransientLocal` and have unit + `launch_testing` coverage.
- Every parameter has a descriptor with type and range.
- C++: no `new` / `delete`, RAII for every resource, `noexcept` destructors.
- Python: type hints, no bare `except`, numpy-vectorise anything > 16 elements.

## Known constraints

- 20-robot ceiling on a single host — driven by DDS discovery cost. CycloneDDS with `cyclonedds_swarm.xml` is required, not optional.
- AMCL disabled (see TF section). Don't re-enable without a plan for cross-robot laser interference.
- 3D Gazebo (`iros_llm_swarm_simulation`) is not stable yet — use `_simulation_lite` (Stage).
- `iros_llm_rviz_panel` does Qt from ROS callbacks via `Qt::QueuedConnection` signals only — never touch widgets from an executor thread (segfaults under load, passes silently on a quiet workstation).
- Always rebuild `iros_llm_swarm_interfaces` (and `_mapf_lns` if its types changed) before dependents when adding messages — symlink-install will not save you here.

## Sub-agent routing

### Default chain (no slash command)
For any non-trivial change the orchestrator runs:
`researcher` → `planner` → **PAUSE for user** → `ros2-implementer` → `/deep-review` → `test-writer` → `test-reviewer` → `synthesizer`.

This is exactly what `/full-cycle` does. Use it.

### Parallel dispatch
Launch in parallel only when **all** of:
- 3+ independent review axes (architecture, ROS 2, C++, Python, performance, security, tests).
- No shared writes between agents (reviewers are read-only — always safe).
- Clear file scope per agent.

### Sequential dispatch
- `researcher` before `planner`.
- `planner` before `ros2-implementer`.
- `ros2-implementer` before any reviewer.
- All reviewers before `synthesizer`.
- `test-writer` before `test-reviewer`.

### Background dispatch
Web research and external doc lookups (MoveIt2 / Nav2 / PCL docs) → background sub-agent so the main thread keeps moving.

### Routing cheatsheet
- "How is X done in this codebase?" → `researcher`.
- "Design Y" → `planner`.
- "Implement the approved plan" → `ros2-implementer`.
- "Review my changes" → `/deep-review`.
- "Audit this whole package" → `/ros2-audit`.
- "Build the whole thing from a one-line description" → `/full-cycle`.

### Output formatting expectations
- Reviewer output uses `🔴 / 🟡 / 🟢` with `file:line`.
- Synthesis output ends in a single verdict: ✅ / 🟡 / 🔴.
- Commits and PR descriptions follow conventional commits and reference findings by `file:line`.
- Plans use the structure in `planner.md`.

### What reviewers MUST NOT do
- Hedge with "consider", "perhaps", "might want to". Take a position.
- Flag below 80 % confidence.
- Edit files (they are read-only by tool allowlist).
