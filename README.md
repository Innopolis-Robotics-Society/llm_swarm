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

Then send goals via the `/swarm/set_goals` action. The action stays alive until all robots arrive at their goals (long-lived action). The test script prints feedback during execution and exits when the mission completes. Press `Ctrl+C` to cancel the mission and stop all robots.

```bash
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0
ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json
```

### Formations and MAPF tests

Launch the full stack:
```bash
ros2 launch iros_llm_swarm_bringup swarm_formation.launch.py
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

PBS (Priority-Based Search) MAPF planner with gradient inflation and N-connected Euclidean A\*. Plans conflict-free time-indexed paths on a downsampled grid (0.2m/cell).

Path followers execute paths with temporal synchronization via Nav2 `follow_path` action and follow formation if assigned to one.

#### How the solver works

**Gradient inflation.** Each agent has two radii: physical footprint (hard — overlap forbidden) and inflation zone (soft — gradient penalty). Closer to the hard boundary = higher cost. Robots naturally keep distance but can approach when forced. Same model for walls.

```
  distance:   [ 0 ····· hard ············· soft ··· ∞ ]
  cost:       [ FORBIDDEN | max penalty -> 0 |  free  ]
```

**N-connected movement.** A\* movement connectivity is auto-generated from physical parameters: `reach = max(1, floor(max_speed * time_step_sec / resolution))`. At reach=1 (default), agents have 9 moves: 4 cardinal + 4 diagonal + wait. Increasing `time_step_sec` grows the reach — at reach=2, agents can jump 2 cells per timestep (~13 moves). Multi-cell moves are validated by tracing the center-line through the grid (Bresenham): any blocked cell along the trace rejects the move, and wall gradient penalties are summed across all traversed cells.

**Cost model.** Costs are in physical units:

```
cost = urgency * max_speed * time_step_sec   (time: opportunity cost of not moving)
     + sqrt(dr² + dc²) * resolution          (distance in metres)
     + wall_penalty_sum * resolution          (wall gradient, normalized)
     + agent_proximity_penalty                (from reservation table)
```

The `urgency` coefficient (default 1.0) controls time-vs-distance tradeoff. At 1.0, one timestep of waiting costs as much as the distance the robot could have covered at max speed. Values below 1.0 are not recommended.

**Capsule-based conflict detection.** Each agent's movement per timestep sweeps a capsule (circle along a line segment). Conflicts are detected by checking segment-segment distance between agent pairs against their combined footprint radii. This catches both positional overlap and mid-step crossing — a single test replaces the old vertex/edge distinction.

**Dijkstra heuristic.** Backward Dijkstra from each goal using the same move set as A\*. Produces tight distance estimates for any connectivity level. Cached per unique goal cell.

**PBS orchestration.** Initial paths are computed sequentially — agents with furthest goals plan first (most freedom), stationary agents plan last (must yield). If sequential planning fails for an agent (too constrained by previously reserved paths), it falls back to independent planning and lets PBS resolve the conflicts.

PBS then searches for a conflict-free ordering: detects capsule-overlap conflicts (hard radius only), branches by assigning priority to one agent over another, replans the lower-priority agent around the higher. Branch heuristic: further-goal agent gets priority first.

Agents can start at positions overlapping with reservations (needed during replanning when robots are close together) — A\* finds a diverging path from there. Agents whose starts physically overlap get a grace period: the overlapping agent's reservation entries are skipped for the first few timesteps, giving both time to separate.

**Schedule monitoring.** After paths are published, periodic checks compare robot positions to the plan. On deviation, triggers replanning from current positions with a static cooldown (from replan end to next replan start). Arrived robots keep their original goals to prevent drift-induced conflicts.

Path followers chunk paths at hold points and send to Nav2 `follow_path` with temporal synchronization.

#### Action API

Action `/swarm/set_goals` (`iros_llm_swarm_interfaces/action/SetGoals`) - long-lived action that covers the entire plan-execute-arrive cycle.

**Lifecycle:**

1. Client sends goal (robot_ids + target positions)
2. Server plans paths (feedback: `status="planning"`)
3. Server dispatches paths to `/robot_{id}/mapf_path` and monitors execution (feedback: `status="executing"`)
4. On schedule deviation, server replans autonomously (feedback: `status="replanning"`)
5. Action succeeds when all robots arrive at goals
6. Client can cancel at any time - all robots are stopped

**Goal:** `uint32[] robot_ids` + `geometry_msgs/Point[] goals` (x, y in map frame, arrays must be same length). Start positions are read automatically from `/robot_{id}/odom`.

**Result:**

| Field                | Type     | Description                                      |
| -------------------- | -------- | ------------------------------------------------ |
| `success`            | bool     | Whether the mission completed successfully       |
| `message`            | string   | "All robots arrived" or error description        |
| `error_code`         | uint16   | Enumerated error (NONE=0, NO_VALID_AGENTS=201, PBS_FAILED=202, TIMEOUT=203, CANCELLED=204) |
| `planning_time_ms`   | float64  | Wall-clock planning time (ms), last plan         |
| `num_agents_planned` | uint32   | Number of agents actually planned                |
| `pbs_expansions`     | uint32   | PBS tree node expansions                         |
| `max_path_length`    | uint32   | Longest path in steps                            |
| `path_lengths`       | uint32[] | Per-agent path lengths (parallel to `robot_ids`) |
| `astar_ok_count`     | uint32   | Successful A\* calls                             |
| `astar_fail_count`   | uint32   | Failed A\* calls (hit expansion cap)             |
| `astar_avg_exp`      | uint32   | Avg expansions per successful A\* call           |
| `astar_max_exp`      | uint32   | Max expansions in a single successful A\* call   |
| `total_replans`      | uint32   | Number of replans during execution               |
| `total_execution_sec`| float64  | Total time from planning to all-arrived (s)      |

**Feedback** (published periodically during all phases):

| Field             | Type   | Description                                          |
| ----------------- | ------ | ---------------------------------------------------- |
| `status`          | string | Phase: "validating", "planning", "executing", "replanning", "failed" |
| `elapsed_ms`      | uint32 | Milliseconds since action started                    |
| `robots_arrived`  | uint32 | Robots that reached their goals (execution phase)    |
| `robots_active`   | uint32 | Robots still en-route (execution phase)              |
| `robots_deviated` | uint32 | Robots off-schedule (execution phase)                |
| `replans_done`    | uint32 | Replan count so far (execution phase)                |

Paths are published as `nav_msgs/Path` on `/robot_{id}/mapf_path`. Each `PoseStamped` contains the scheduled arrival time (`header.stamp = plan_time + step × time_step_sec`) and orientation pointing toward the next waypoint.

Sending goals:

```bash
# All robots to a point (distributed in grid pattern with 1m spacing)
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 13.5 --goal-y 16.5

# Random goals within radius of map center (15, 15)
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0

# From JSON file
ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json

# Custom timeout (default 600s)
ros2 run iros_llm_swarm_mapf test_send_goals --random --timeout 300
```

JSON format:

```json
{
  "goals": [
    { "id": 0, "gx": 15.0, "gy": 12.5 },
    { "id": 1, "gx": 16.2, "gy": 14.8 }
  ]
}
```

#### Log output

**Planner node** (`mapf_planner`) on success:

```
PBS solved in 8267.2 ms (1 PBS exp, A*: 20 ok (avg 308227 / max 661387 exp), 0 failed), publishing 20 paths
Planning complete (8267.2 ms), entering execution phase. Action stays alive until all robots arrive.
```

**Planner node** when all robots arrive:

```
==== ALL 20 ROBOTS REACHED THEIR GOALS! ====
```

**Planner node** on failure:

```
PBS failed (2709.9 ms, 1 expansions)
  reason: root A* failed, max_t=536, branches tried=0 failed=2
  root A* failed for agent 11: start=(45,12) goal=(120,85) footprint=0.220m inflation=0.750m
```

**test_send_goals** output:

```
[INFO] Goal accepted — mission in progress...
[INFO]   [  8267 ms] planning
[INFO]   [  8270 ms] publishing
[INFO]   [     0 ms] executing: 0/20 arrived, 0 replans
[INFO]   [  5000 ms] executing: 3/20 arrived, 0 replans
[WARN]   [ 15000 ms] replanning: 2 deviated, 0 replans so far
[INFO]   [ 45000 ms] executing: 20/20 arrived, 1 replans
[INFO] === MISSION COMPLETE ===
         20 agents, 8267.2 ms planning
         PBS: 1 exp, A*: 20 ok (avg 308227 / max 661387), 0 failed
         max path 127 steps
         1 replans, 45.0 s execution
```

#### Key parameters

| Parameter              | Default         | Description                                                                |
| ---------------------- | --------------- | -------------------------------------------------------------------------- |
| `default_robot_radius` | 0.22            | Physical footprint radius (m)                                              |
| `inflation_radius`     | 0.5             | Gradient zone width beyond footprint (m)                                   |
| `cost_curve`           | "quadratic"     | Gradient curve shape ("linear", "quadratic", "cubic")                      |
| `proximity_penalty`    | 15              | Max gradient penalty at the hard boundary (walls and agents)               |
| `pbs_resolution`       | 0.2             | PBS grid cell size (m), map downsampled from 0.05                          |
| `max_pbs_expansions`   | 5000            | PBS node expansion limit                                                   |
| `max_astar_expansions` | 100000 (launch) | Per-A\* expansion limit (root planning uncapped)                           |
| `time_step_sec`        | 0.1             | Seconds per PBS grid step                                                  |
| `max_speed`            | 0.5             | Max robot speed (m/s), determines movement connectivity with time_step_sec |
| `urgency`              | 1.0             | Time cost coefficient: 1 = balanced, >1 = rush. Below 1.0 not recommended  |
| `replan_check_hz`      | 2.0             | Schedule deviation check rate (Hz)                                         |
| `replan_threshold_m`   | 1.0             | Deviation distance to trigger replan (m)                                   |
| `replan_cooldown_sec`  | 15.0            | Cooldown from replan end to next replan start (s)                          |

**Constraint:** `footprint_radius >= 0.7 * pbs_resolution` must hold when using the Euclidean planner. The planner asserts this at startup. With defaults (0.22m footprint, 0.2m resolution) the constraint is satisfied. Increasing `pbs_resolution` beyond `footprint_radius / 0.7` requires a proportionally larger footprint or finer resolution.

### `iros_llm_swarm_formation`

Centralized management of robot formations and distributes formation configuration to all robots.

#### Topics

- `/formations/config`

Type: `iros_llm_swarm_interfaces/msg/FormationsState`

QoS: `TRANSIENT_LOCAL`, `RELIABLE`, `depth=1` (latched)

Description: Publishes the complete state of all formations in the system.
Each message is a full snapshot, allowing late subscribers to immediately receive the current configuration.

Message Structure:
```text
std_msgs/Header header
FormationConfig[] formations
```

FormationConfig.msg:
```text
std_msgs/Header header

string   formation_id        # unique name e.g. "wedge"
string   leader_ns           # e.g. "robot_0"
string[] follower_ns         # e.g. ["robot_1", "robot_2"]

# Per-follower offset in leader BODY frame (forward=+x, left=+y).
# Length must match follower_ns.
geometry_msgs/Point[] offsets

# Bounding footprint for MAPF planner (Polygon in leader body frame).
geometry_msgs/Polygon footprint

bool active  # false = formation dissolved, followers go autonomous
```

Example:
```text
header:
  stamp:
    sec: 1710000000
    nanosec: 0
  frame_id: ""

formations:
- formation_id: "line"
  leader_ns: "robot_0"
  follower_ns: ["robot_1", "robot_2"]

  offsets:
  - {x: 1.0, y: 0.0, z: 0.0}
  - {x: 2.0, y: 0.0, z: 0.0}

  footprint:
    points:
    - {x: 2.5, y: 0.0, z: 0.0}
    - {x: 1.8, y: 1.8, z: 0.0}
    <...>

  active: true

- formation_id: "triangle"
  leader_ns: "robot_3"
  follower_ns: ["robot_4", "robot_5"]

  offsets:
  - {x: -1.0, y: 1.0, z: 0.0}
  - {x: -1.0, y: -1.0, z: 0.0}

  footprint:
    points:
    <...>

  active: false
```

#### Services

- `/formation/set`

Type: `iros_llm_swarm_interfaces/srv/SetFormation`

Description: Creates or updates a formation. 

Defenition:
```text
string formation_id     # unique name; if exists, updates this formation
string leader_ns        # e.g. "robot_0"
string[] follower_ns    # e.g. ["robot_1", "robot_2"]
float64[] offsets_x     # parallel arrays — offset in leader body frame
float64[] offsets_y     # length must equal follower_ns length
bool activate           # true = activate immediately after setting
---
bool success
string message
```

Example:
```bash
ros2 service call /formation/set iros_llm_swarm_interfaces/srv/SetFormation "
formation_id: 'line'
leader_ns: 'robot_0'
follower_ns: ['robot_1', 'robot_2']
offsets_x: [1.0, 2.0]
offsets_y: [0.0, 0.0]
activate: true
"
```

- `/formation/disband`

Type: `iros_llm_swarm_interfaces/srv/DisbandFormation`

Description: Deactivates a formation without removing it. Followers resume autonomous operation. Still be published in `/formations/config`.

Defenition:
```text
string formation_id
---
bool success
string message
```

Example:
```bash
ros2 service call /formation/disband iros_llm_swarm_interfaces/srv/DisbandFormation "formation_id: 'line'"
```

### `iros_llm_swarm_interfaces`

Custom ROS 2 messages, services, and actions: FormationConfig, SetFormation, DisbandFormation, SetGoals.

### `iros_llm_swarm_simulation`

Gazebo Harmonic (full 3D) simulation package - prepared for later development stages. Contains URDF/Xacro robot descriptions and an SDF world.

> **Note:** The simulation will be able to launch 20 robots, but work with the Nav2 stack and overall stable operation is not guaranteed!

## Architecture Notes

- **TF tree:** unified via Stage's `one_tf_tree` + `enforce_prefixes`. Each robot's frames are prefixed with `robot_N/`. A static publisher bridges `map to robot_N/odom` per robot.
- **Namespacing:** all Nav2 nodes are pushed into `robot_N` namespaces. The `<robot_namespace>` placeholder in YAML configs is replaced at launch time via `ReplaceString`.
- **Parameter handling:** Nav2 params are processed through `RewrittenYaml` with `root_key=<namespace>` for proper node-level scoping.
- **Robot placement:** two groups of 10 robots on opposite sides of the warehouse (bottom-left facing up, top-right facing down).
- **MAPF action lifecycle:** the `/swarm/set_goals` action is long-lived — it stays active from initial planning through execution until all robots arrive. Internal schedule monitoring and replanning happen autonomously within the action server. External clients (BT, LLM) see this as a single atomic "navigate fleet" operation and receive structured feedback throughout. Cancellation stops all robots immediately.

## Environment Setup

DDS and performance tuning scripts are in `src/scripts/`:

- `cyclonedds_swarm.xml` — CycloneDDS config optimized for multi-robot comms
- `setup_swarm_env.sh` — environment variable setup (DDS config, domain ID, etc.)

## Project Roadmap

- [x] 2D Stage simulation with 20 robots + Nav2 local nav
- [x] MAPF with PBS planner, gradient inflation, and action API
- [x] N-connected Euclidean A\* with capsule-based conflict detection
- [x] Schedule-based replanning with static cooldown
- [x] Formation control (leader-follower with PD controllers)
- [x] Long-lived MAPF action (plan-execute-arrive lifecycle with feedback)
- [ ] BT integration
  - [ ] Swarm BT Navigator (single BT for the whole swarm)
  - [ ] Custom BT nodes: ExecuteMAPF, ExecuteFormation, EmergencyStop
  - [ ] Mode switching (MAPF - formation) via BT control flow
- [ ] MAPF improvements
  - [ ] Better conflict resolution — PBS is currently brute force, needs smarter strategy (CBS, ECBS, or improved PBS heuristics)
  - [ ] Path following temporal accuracy — robots drift from planned schedule (too fast/slow), causing unnecessary replans
  - [ ] Replan trigger tuning — replanning fires too often, needs hysteresis or less sensitive thresholds
  - [ ] Per-agent time horizon — short-path agents don't need the full max_t, reducing their state space
  - [ ] Formation-aware MAPF (plan formations as single entities with compound footprints)
  - [ ] Orientation-aware PBS for complex polygon footprints
- [ ] Fault tolerance and communication loss handling
- [ ] LLM reasoning agent integration (fleet-level task allocation)
- [ ] Transition to Gazebo Harmonic for 3D simulation