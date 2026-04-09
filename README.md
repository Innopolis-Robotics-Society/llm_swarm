# IROS LLM Swarm — Simulation

Multi-robot simulation on ROS 2 Humble with Nav2 local navigation stack. Runs 20 differential-drive robots in a 2D Stage environment with per-robot navigation controllers and a shared map.

## Quick Start (Docker)

Run container:

```bash
docker compose up terminal
```

Attach to the container:

```bash
docker compose exec terminal bash
```

Inside the container - launch the full stack:

```bash
ros2 launch iros_llm_swarm_bringup swarm_warehouse.launch.py
```

> **GUI/RViz:** for graphical output make sure X11 forwarding or similar display passthrough is configured in `docker-compose.yaml` (e.g. `DISPLAY`, `/tmp/.X11-unix` volume mount, `xhost +local:docker` on the host).

### Launch Arguments

| Argument       | Default           | Description                                                                                        |
| -------------- | ----------------- | -------------------------------------------------------------------------------------------------- |
| `num_robots`   | `20`              | Number of robots to controll by system (still spawn as many robots as there are in the world file) |
| `world_file`   | `warehouse.world` | Stage world file path                                                                              |
| `rviz_cfg`     | `swarm_20.rviz`   | common RViz config for 20 robots                                                                   |
| `use_sim_time` | `true`            | Use simulation clock                                                                               |

```bash
# Example: launch with 5 robots under nav2 controll
ros2 launch iros_llm_swarm_bringup swarm_warehouse.launch.py num_robots:=5
```

### Simple tests

You can play around with swarm a little:

You can send everyone to the same point by following command:

```bash
ros2 run iros_llm_swarm_local_nav test_local_planne --num 20 --goal-x <replace> --goal-y <replace>
```

Or send everyone to their own random point within a radius:

```bash
ros2 run iros_llm_swarm_local_nav test_local_planne --num 20 --radius <replace>
```

> _notes:_ num - determines the number of robots that will receive the command

> _Behaviour explanation:_ They will all approach the obstacle and stop, since the local planner is unable to handle obstacles; it will simply get upset at the wall and that's it

### MAPF tests

Launch the full MAPF stack first:

```bash
ros2 launch iros_llm_swarm_bringup swarm_mapf.launch.py
```

Then send goals via the `/swarm/set_goals` service:

```bash
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0
ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json
```

## Packages

### `iros_llm_swarm_bringup`

Top-level launch orchestrator. `swarm_warehouse.launch.py` brings up the full stack in order:

1. Stage simulator
2. Nav2 local navigation (with 1 s delay for simulator init)
3. RViz visualization

### `iros_llm_swarm_simulation_lite`

2D simulation layer based on Stage (`stage_ros2`). Contains the warehouse world file, map files (`.pgm` / `.yaml`), and robot model description (`warehouse_robot.inc`). Stage runs with a unified TF tree (`one_tf_tree: true`) and namespace-enforced prefixes.

### `iros_llm_swarm_local_nav`

Per-robot Nav2 navigation stack. For each robot spawns:

- **controller_server** - local trajectory tracking (e.g. DWB / MPPI)
- **behavior_server** - recovery behaviors (spin, backup, wait)
- **lifecycle_manager** - manages the above nodes
- **static TF** - `map to <robot_ns>/odom` identity transform

Robots are launched with staggered timers (0.3 s interval) to avoid startup race conditions. A global **map_server** is shared across all robots.

> **Note:** AMCL is currently disabled - localization uses a static `map to odom` transform. This is intentional for the current development stage; dynamic localization with 20 robots causes drift due to cross-robot laser interference.

### `iros_llm_swarm_costmap_plugins`

Custom Nav2 costmap layer plugin. Provides `ResettingObstacleLayer` — a drop-in replacement for `nav2_costmap_2d::ObstacleLayer` that resets its internal grid every cycle before marking. Fixes ghost obstacle trails caused by cells between LiDAR rays never being cleared via raytracing.

### `iros_llm_swarm_mapf`

PBS (Priority-Based Search) MAPF planner with gradient inflation. Header-only C++17 solver (`pbs_solver.hpp`). Accepts goals via `/swarm/set_goals` service (SetGoals), reads start positions from odometry and footprints from Nav2. Plans conflict-free time-indexed paths on a downsampled grid (0.2m/cell).

#### How the solver works

Each agent has two radii: physical footprint (hard — overlap forbidden) and inflation zone (soft — gradient penalty). Closer to the hard boundary = higher cost. Robots naturally keep distance but can approach when forced. Same model for walls.

```
  distance:   [ 0 ····· hard ············· soft ··· ∞ ]
  cost:       [ FORBIDDEN | max penalty -> 0 |  free  ]
```

Initial paths are computed sequentially — agents with furthest goals plan first (most freedom), stationary agents plan last (must yield). If sequential planning fails for an agent (too constrained by previously reserved paths), it falls back to independent planning and lets PBS resolve the conflicts.

PBS then searches for a conflict-free ordering: detects physical-overlap conflicts (hard radius only), branches by assigning priority to one agent over another, replans the lower-priority agent around the higher. Branch heuristic: further-goal agent gets priority first.

Low-level pathfinding uses Space-Time A\* with a BFS-precomputed heuristic (true step-distance from goal, accounting for wall topology). Cost model: every timestep costs 1, movement adds 1 more — robots prefer waiting over unnecessary walking. Wall gradient penalty charged only on cell entry, agent proximity penalty per-timestep. Agents can start at positions overlapping with reservations (needed during replanning when robots are close together) — A\* finds a diverging path from there.

During PBS replanning, agents whose starts physically overlap get a grace period: the overlapping agent's reservation entries are skipped for the first few timesteps, giving both agents time to separate before collision avoidance kicks in.

After paths are published, schedule monitoring compares robot positions to the plan. On deviation, triggers replanning from current positions with adaptive cooldown. Arrived robots keep their original goals to prevent drift-induced conflicts.

Path followers chunk paths at hold points and send to Nav2 `follow_path` with temporal synchronization.

#### Service API

Service `/swarm/set_goals` (`iros_llm_swarm_interfaces/srv/SetGoals`):

**Request:** `uint32[] robot_ids` + `geometry_msgs/Point[] goals` (x, y in map frame, arrays must be same length). Start positions are read automatically from `/robot_{id}/odom`.

**Response:**

| Field | Type | Description |
|---|---|---|
| `success` | bool | Whether planning succeeded |
| `message` | string | "OK" or warning/error description |
| `planning_time_ms` | float64 | Wall-clock planning time (ms) |
| `num_agents_planned` | uint32 | Number of agents actually planned |
| `pbs_expansions` | uint32 | PBS tree node expansions |
| `max_path_length` | uint32 | Longest path in steps |
| `path_lengths` | uint32[] | Per-agent path lengths (parallel to `robot_ids`) |
| `astar_ok_count` | uint32 | Successful A\* calls |
| `astar_fail_count` | uint32 | Failed A\* calls (hit expansion cap) |
| `astar_avg_exp` | uint32 | Avg expansions per successful A\* call |
| `astar_max_exp` | uint32 | Max expansions in a single successful A\* call |

Paths are published as `nav_msgs/Path` on `/robot_{id}/mapf_path`. Each `PoseStamped` contains the scheduled arrival time (`header.stamp = plan_time + step × time_step_sec`) and orientation pointing toward the next waypoint.

Sending goals:
```bash
# All robots to a point (distributed in grid pattern with 1m spacing)
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0

# Random goals within radius of map center (15, 15)
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0

# From JSON file
ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json
```

JSON format:
```json
{"goals": [{"id": 0, "gx": 15.0, "gy": 12.5}, {"id": 1, "gx": 16.2, "gy": 14.8}]}
```

#### Log output

**Planner node** (`mapf_planner`) on success:
```
PBS solved in 13256.5 ms (1 PBS exp, A*: 20 ok (avg 425000 / max 890000 exp), 0 failed), publishing 20 paths
```

**Planner node** on failure:
```
PBS failed (2709.9 ms, 1 expansions)
  reason: root A* failed, max_t=536, branches tried=0 failed=2
  root A* failed for agent 11: start=(45,12) goal=(120,85) footprint=0.220m inflation=0.750m
```

**test_send_goals** on success:
```
20 agents, 13256.5 ms, 1 PBS exp, A*: 20 ok (avg 425000 / max 890000 exp), 0 failed, max path 134 steps
```

#### Key parameters

| Parameter | Default | Description |
|---|---|---|
| `default_robot_radius` | 0.22 | Physical footprint radius (m) |
| `inflation_radius` | 0.5 (0.75 in launch) | Gradient zone width beyond footprint (m) |
| `cost_curve` | "quadratic" | Gradient curve shape ("linear", "quadratic", "cubic") |
| `proximity_penalty` | 50 | Cost per step at the hard boundary |
| `pbs_resolution` | 0.2 | PBS grid cell size (m), map downsampled from 0.05 |
| `max_pbs_expansions` | 5000 | PBS node expansion limit |
| `max_astar_expansions` | 100000 (launch) | Per-A* expansion limit (root planning uncapped) |
| `time_step_sec` | 0.4 | Seconds per PBS grid step |
| `replan_check_hz` | 2.0 | Schedule deviation check rate (Hz) |
| `replan_threshold_m` | 1.0 | Deviation distance to trigger replan (m) |
| `replan_cooldown_sec` | 5.0 | Min cooldown between replans (s) |

### `iros_llm_swarm_formation`

Leader-follower formation control with PD controllers (Kp=1.2, Kd=0.3, 20 Hz). Pre-defined formations (wedge, line, diamond) in `formations.yaml`. Runtime create/disband via `/formation/set` and `/formation/disband` services.

### `iros_llm_swarm_interfaces`

Custom ROS 2 messages and services: FormationConfig, SetFormation, DisbandFormation, SetGoals.

### `iros_llm_swarm_simulation`

Gazebo Harmonic (full 3D) simulation package - prepared for later development stages. Contains URDF/Xacro robot descriptions and an SDF world.

> **Note:** The simulation will be able to launch 20 robots, but work with the Nav2 stack and overall stable operation is not guaranteed!

## Architecture Notes

- **TF tree:** unified via Stage's `one_tf_tree` + `enforce_prefixes`. Each robot's frames are prefixed with `robot_N/`. A static publisher bridges `map to robot_N/odom` per robot.
- **Namespacing:** all Nav2 nodes are pushed into `robot_N` namespaces. The `<robot_namespace>` placeholder in YAML configs is replaced at launch time via `ReplaceString`.
- **Parameter handling:** Nav2 params are processed through `RewrittenYaml` with `root_key=<namespace>` for proper node-level scoping.
- **Robot placement:** two groups of 10 robots on opposite sides of the warehouse (bottom-left facing up, top-right facing down).

## Environment Setup

DDS and performance tuning scripts are in `src/scripts/`:

- `cyclonedds_swarm.xml` — CycloneDDS config optimized for multi-robot comms
- `setup_swarm_env.sh` — environment variable setup (DDS config, domain ID, etc.)

## Project Roadmap

- [x] 2D Stage simulation with 20 robots + Nav2 local nav
- [x] MAPF with PBS planner, gradient inflation, and service API
- [x] Schedule-based replanning with adaptive cooldown
- [x] Formation control (leader-follower with PD controllers)
- [ ] MAPF improvements
  - [ ] Formation-aware MAPF (plan formations as single entities with compound footprints)
  - [ ] 8-connected grid movement (diagonal paths)
  - [ ] Orientation-aware PBS for complex polygon footprints
- [ ] Fault tolerance and communication loss handling
- [ ] LLM reasoning agent integration (fleet-level task allocation)
- [ ] Transition to Gazebo Harmonic for 3D simulation
