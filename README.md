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

| Argument | Default | Description |
|---|---|---|
| `num_robots` | `20` | Number of robots to controll by system (still spawn as many robots as there are in the world file) |
| `world_file` | `warehouse.world` | Stage world file path |
| `rviz_cfg` | `swarm_20.rviz` | common RViz config for 20 robots |
| `use_sim_time` | `true` | Use simulation clock |

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

> *notes:* num - determines the number of robots that will receive the command

> *Behaviour explanation:* They will all approach the obstacle and stop, since the local planner is unable to handle obstacles; it will simply get upset at the wall and that's it

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

Footprint-aware PBS (Priority-Based Search) MAPF planner. Accepts goals via `/swarm/set_goals` service (SetGoals), reads start positions from odometry and footprints from Nav2. Plans conflict-free time-indexed paths on a downsampled grid (0.2m/cell) with per-agent inflated maps and distance-based reservation tables.
Path followers execute paths with temporal synchronization via Nav2 `follow_path` action and follow formation if assigned to one.

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
- [x] MAPF with footprint-aware PBS planner + service API
- [x] Formation control (leader-follower with PD controllers)
- [ ] MAPF improvements
  - [ ] Formation-aware MAPF (plan formations as single entities with compound footprints)
  - [ ] Schedule-based replanning (re-plan when robots fall behind)
  - [ ] 8-connected grid movement (diagonal paths)
  - [ ] Orientation-aware PBS for complex polygon footprints
- [ ] Fault tolerance and communication loss handling
- [ ] LLM reasoning agent integration (fleet-level task allocation)
- [ ] Transition to Gazebo Harmonic for 3D simulation
