# IROS LLM Swarm — Project Context for a Fresh Claude Session

This file is a self-contained briefing for picking up work on the
**LLM + Behavior Tree** layer of the swarm. It assumes no prior
exposure to the codebase. Drop it into a new Claude chat and continue
from there.

The project lives in `/home/mook/projects/llm_swarm` (ROS 2 workspace
sources directly at the repo root — there's no `src/` indirection).
It builds and runs inside Docker.

---

## 1. What this project is

A ROS 2 Humble research stack for orchestrating **20 differential-drive
robots** in a 2D Stage simulation. The headline feature is a fleet-level
**Multi-Agent Path Finding (MAPF)** action — `/swarm/set_goals` —
that takes per-robot target positions and produces a conflict-free,
time-indexed schedule. The same action is implemented by two
interchangeable backends (PBS and LNS2). On top of MAPF the project
adds:

- **Nav2 local navigation** per robot (Regulated Pure Pursuit, custom
  costmap layer).
- **Leader-follower formations** with a centralized registry and a PD
  controller in each per-robot follower.
- **Behavior trees** (BehaviorTree.CPP v3) with four custom action
  nodes — **the layer we're working on**.
- **LLM advisory channel** wired into the BT — also our work.
- A 2D Stage simulator (supported) and a 3D Gazebo Fortress preview
  (not stable yet).

The project ships full per-package documentation built with `rosdoc2`
(see [§13](#13-where-to-find-more) below).

---

## 2. Where the work is heading

The user's focus is **LLM + BT integration**. Specifically:

- The BT scaffolding (`MapfPlan`, `SetFormation`, `DisableFormation`,
  `CheckMode` + a runner + reference tree) **already exists and works**
  — the project's `README` lists "BT integration" as TODO, but read
  that as *expansion*, not "starts from scratch".
- The LLM is reached through an `/llm/decision` action defined in
  `iros_llm_swarm_interfaces` (`LlmDecision.action`). The LLM agent
  itself is **out of tree** — the BT side already calls the action
  asynchronously, accumulates feedback in a ring buffer, and acts on
  the LLM's verdict. We're filling in the LLM agent and richer BT
  control flow that uses it.
- The PBS planner (`iros_llm_swarm_mapf`) is **legacy**. New work
  should target the LNS2 stack (`iros_llm_swarm_mapf_lns`). PBS
  bringup launches still exist but are not maintained.

---

## 3. High-level architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    IROS LLM SWARM SIMULATION                     │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Behavior Tree (test_bt_runner)                                  │
│   ├── MapfPlan           → /swarm/set_goals (action client)      │
│   ├── SetFormation       → /formation/set    (service client)    │
│   ├── DisableFormation   → /formation/deactivate                 │
│   └── CheckMode          (blackboard mode dispatch)              │
│                          ↑↓ /llm/decision (LLM advisory)         │
│                                                                  │
│  ┌─────────────┐   ┌──────────────┐   ┌────────────────────────┐ │
│  │  Stage 2D   │   │     RViz     │   │      CycloneDDS        │ │
│  │  Simulator  │   │    Viewer    │   │     domain_id = 42     │ │
│  └──────┬──────┘   └──────────────┘   └────────────────────────┘ │
│         │ /tf, /robot_*/odom, /robot_*/base_scan                 │
│  ┌──────▼──────────────────────────────────────────────────────┐ │
│  │   MAPF PLANNER  (LNS2 — supported, or PBS — legacy)         │ │
│  │   action: /swarm/set_goals                                  │ │
│  │   pub:    /robot_*/mapf_plan  (LNS2)                        │ │
│  │          /robot_*/mapf_path  (PBS — legacy)                 │ │
│  │   sub:   /map, /robot_*/odom, /robot_*/follower_status      │ │
│  └──────┬──────────────────────────────────────────────────────┘ │
│         │ per-robot plan / path                                  │
│  ┌──────▼──────────────────────────────────────────────────────┐ │
│  │   PER-ROBOT FOLLOWER  (one per robot, dual-mode)            │ │
│  │   LNS stack: path_follower_node     (iros_llm_swarm_mapf_lns)│ │
│  │   PBS stack: motion_controller_node (iros_llm_swarm_robot)  │ │
│  │   AUTONOMOUS  → /robot_*/follow_path  (Nav2 action)         │ │
│  │   FORMATION   → /robot_*/cmd_vel       (PD controller)      │ │
│  │   sub: /formations/config                                   │ │
│  └──────┬───────────────────────────┬──────────────────────────┘ │
│         │ AUTONOMOUS                │ FORMATION                  │
│  ┌──────▼──────────────┐   ┌────────▼─────────────────────────┐  │
│  │  Nav2 LOCAL NAV     │   │  FORMATION LAYER                 │  │
│  │  (per robot)        │   │  /formations/config  (latched)   │  │
│  │  controller_server  │   │  /formations/status  (10 Hz)     │  │
│  │  behavior_server    │   │  services: /formation/*          │  │
│  │  shared map_server  │   │  formation_manager_node          │  │
│  │  ResettingObst.Layer│   │  formation_monitor_node          │  │
│  └─────────────────────┘   └──────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

**Vertical pipeline.** Simulator publishes ground-truth state →
MAPF turns fleet-level goals into time-indexed schedules → per-robot
followers turn each schedule into Nav2 `follow_path` actions or, in
formation mode, into direct `cmd_vel`.

**Mission API.** Everything fleet-level goes through the long-lived
`/swarm/set_goals` action. Cancel = stop everyone. Feedback = phase +
counters. Clients (BT, scripted tests, future LLM) treat one mission
as one atomic call.

---

## 4. Package map

| Package | Role | Status |
|---|---|---|
| `iros_llm_swarm_interfaces` | Shared `msg`/`srv`/`action` types. **Build first.** | active |
| `iros_llm_swarm_mapf_lns` | **LNS2 planner + per-robot path follower.** Supported. | active, refactor planned |
| `iros_llm_swarm_mapf` | **PBS planner.** Legacy — don't extend. | legacy |
| `iros_llm_swarm_robot` | PBS-paired follower (`motion_controller_node`). | legacy by association |
| `iros_llm_swarm_formation` | Formation registry + monitor (Python). | active |
| `iros_llm_swarm_bt` | **BT plugin (4 nodes) + runner + trees.** | active — our focus |
| `iros_llm_swarm_local_nav` | Per-robot Nav2 stack spawner + tuned params. | active |
| `iros_llm_swarm_costmap_plugins` | `ResettingObstacleLayer` Nav2 plugin. | active |
| `iros_llm_swarm_simulation_lite` | 2D Stage simulator. | supported |
| `iros_llm_swarm_simulation` | 3D Gazebo Fortress. | preview, unstable |
| `iros_llm_swarm_bringup` | Top-level launch orchestration. | active |
| `iros_llm_swarm_docs` | rosdoc2 aggregator (built site). | active |

**Where the per-package docs live:** each package has
`doc/index.rst` plus `rosdoc2.yaml`. Build the site with:

```bash
cd /home/mook/projects/llm_swarm
rosdoc2 build --package-path iros_llm_swarm_docs --output-directory /tmp/docs
# repeat for any package you want browseable
python3 -m http.server 8765 --directory /tmp/docs
```

---

## 5. Type contracts (`iros_llm_swarm_interfaces`)

These are the messages the rest of the stack (and the BT) cares about.
Field-by-field detail in `iros_llm_swarm_interfaces/doc/index.rst`.

### `SetGoals.action`

Long-lived fleet navigation action. **One goal covers
plan→execute→replan→arrive**.

- **Goal**: `uint32[] robot_ids` + `geometry_msgs/Point[] goals`.
- **Result**: `success`, `error_code`, `planning_time_ms`,
  `num_agents_planned`, `total_replans`, `total_execution_sec`,
  per-agent path lengths, A* stats.
- **Feedback** (~2 Hz): `status` ∈ `"validating" | "planning" |
  "executing" | "replanning" | "failed"`, counters
  (`robots_arrived` / `robots_active` / `robots_deviated` /
  `robot_stall` / `replans_done`), `info` (periodic), `warning`
  (one-shot).
- **Error codes**: `NONE=0`, `UNKNOWN=200`, `NO_VALID_AGENTS=201`,
  `PBS_FAILED=202` (LNS reuses this for "planner failed"),
  `TIMEOUT=203`, `CANCELLED=204`.

### `LlmDecision.action`

The LLM advisory channel.

- **Goal**: `level ∈ {"WARN", "ERROR", "INFO"}`, `event` (one-line
  description), `log_buffer[]` (last N feedback lines).
- **Result**: `decision ∈ {"wait", "abort", "replan"}`, `reasoning`
  (human-readable, not used by BT logic).
- **Feedback**: `reasoning_stream` (the LLM's thinking, streamed
  back to user UI).

### `MAPFPlan.msg` + `MAPFStep.msg` (LNS-side per-robot plan)

```
MAPFPlan:
  std_msgs/Header header
  uint32 plan_id
  MAPFStep[] steps

MAPFStep:
  geometry_msgs/Point target
  float32 hold_sec
```

- `plan_id` is monotonically increasing. Followers echo it in
  `FollowerStatus` so the planner can drop stale statuses across
  replans.
- **Empty `steps` = CANCEL.** Follower aborts in-flight Nav2 goal,
  goes IDLE.

### `FollowerStatus.msg`

Per-robot heartbeat (default 2 Hz) **and** state-transition publish.
Lets the planner tell *frozen* from *legitimately holding*.

```
header, plan_id
state ∈ {IDLE=0, NAVIGATING=1, HOLDING=2, PLAN_COMPLETE=3, FAILED=4}
current_step_index, num_steps, hold_remaining_sec, nav2_failures
last_failure_reason ∈ {NONE, NAV2_ABORTED, NAV2_REJECTED, NAV2_SERVER_DOWN}
```

### `FormationConfig.msg` / `FormationsConfig.msg`

`FormationConfig` is one formation definition (id, leader_ns,
follower_ns, body-frame offsets, footprint polygon, active flag).
`FormationsConfig` is the fleet-wide snapshot, published latched
(TRANSIENT_LOCAL + RELIABLE + depth=1) on `/formations/config`.

### `FormationStatus.msg` / `FormationsStatus.msg`

Runtime state machine: `INACTIVE → FORMING → STABLE ↔ DEGRADED →
BROKEN → INACTIVE`. Published at 10 Hz on `/formations/status`. Carries
per-follower position errors, max/mean aggregates, failure code +
reason on BROKEN.

### `/formation/*` services

CRUD on the registry, all served by `formation_manager_node`:
`set`, `remove`, `activate`, `deactivate`, `get`, `list`, `load`,
`save`. **Removing an active formation is refused** — disband first.

---

## 6. Navigation deep-dive (the substrate the BT drives)

### 6.1 MAPF planner (LNS2 — supported)

`iros_llm_swarm_mapf_lns/mapf_lns2_node`:

- One ~2400-line C++ node. **Refactor planned** — when reading the
  source, expect it to be split into the helpers already next to it
  (`node_utils`, `plan_publisher`, `robot_lifecycle`).
- **Algorithm**: LNS2 (Li, Chan, Ma, Koenig 2022).
  - *Soft A**: space-time A* with collision penalty added to edge cost
    (`penalty > horizon × step_cost` so lex order is `(collisions,
    length)`).
  - *Collision table*: footprint-aware vertex+edge occupancy with O(1)
    lookups, incremental updates.
  - *Destroy operators* (4): CollisionBased, RandomWalk, Random,
    Bottleneck. ALNS adaptive weights with segment-based reaction.
  - *Warm start*: **common** time shift (not per-agent — old
    per-agent shift was a bug) + per-agent classification
    (`OnTrack` / `Spliced` / `StubFar` / `StubNew` /
    `StubGoal` / `StubInvalid` / `StubArrived` / `Missing`).
  - Falls back to cold start if `should_warm_start()` rejects the
    seed.
- **Action server**: `/swarm/set_goals`. Lifecycle was already
  described in §5.
- **Per-robot output**: `/robot_<id>/mapf_plan` (`MAPFPlan`).
  Empty plan = cancel.
- **Schedule monitor**: ticks at `replan_check_hz` (default 2 Hz),
  triggers replan on deviation > `replan_threshold_m`, on stall
  (`stall_timeout_sec` without movement), or on
  `state == STATE_FAILED`. Cooldown = `replan_cooldown_sec`.
- **"Lives" / unplanable system**: on N consecutive empty solves or
  stalled ticks, bench the robot. After `unplanable_retry_delay_sec`
  bring it back. `max_lives` failures → permanent skip.

Full param set in `iros_llm_swarm_mapf_lns/config/mapf_lns2.yaml` and
`doc/ros_api.rst`.

### 6.2 Per-robot follower (LNS-side: `path_follower_node`)

Lives in `iros_llm_swarm_mapf_lns`. **Dual-mode**, mode chosen by
`/formations/config`:

- **AUTONOMOUS** (default):
  - Subscribe `/<ns>/mapf_plan`.
  - Walk the steps. A *segment* = longest run of zero-`hold_sec`
    steps; segments are dispatched to Nav2's
    `/<ns>/follow_path` action as one goal each. After the segment
    ends, sleep `hold_sec`, then proceed.
  - Publish `/<ns>/follower_status` heartbeat (2 Hz) + on every
    transition.
  - State machine: `IDLE → NAVIGATING(seg) → HOLDING → NAVIGATING(next) → ... → PLAN_COMPLETE` (or `FAILED` on Nav2 abort).
- **FORMATION_FOLLOWER**:
  - PD controller. Target = `leader_pos + R(leader_yaw) · offset`
    (offset in leader body frame: `+x` forward, `+y` left).
  - Project world-frame error into robot body frame to get
    `(linear, angular)` velocity; clip to `max_v` / `max_omega`.
  - Publish directly to `/<ns>/cmd_vel`.
  - PD timer at `control_hz` (default 20).

Mode switch is event-driven by `/formations/config` (latched). Node
never restarts.

### 6.3 Nav2 layer (`iros_llm_swarm_local_nav`)

Per-robot Nav2 stack spawner. Important quirks:

1. **AMCL is disabled.** Static identity TF `map → robot_<id>/odom`
   replaces it. At 20 robots cross-LiDAR interference makes AMCL
   drift. Stage's `one_tf_tree` already publishes
   `robot_<id>/odom → robot_<id>/base_link`, so identity is
   sufficient.
2. **Controller = Regulated Pure Pursuit (not DWB).** RPP follows the
   MAPF polyline literally + has three regulators (curvature
   slowdown, proximity slowdown, collision lookahead). DWB would
   either ignore obstacles (low BaseObstacle weight) or invent
   detours (high BaseObstacle weight) — both wrong for fleet-driven
   schedules.
3. **Local costmap uses `ResettingObstacleLayer`** from
   `iros_llm_swarm_costmap_plugins`. It wipes the layer's grid every
   `updateBounds()` before re-marking. Fixes ghost-obstacle trails
   left by other moving robots: with sparse-ray scans (180 rays @ 360°
   FOV) cells *between* rays are never explicitly cleared by
   raytracing.
4. **YAML namespace substitution**: `robot_nav2_params.yaml` uses
   `<robot_namespace>` literal placeholder; launch transforms it via
   `nav2_common.launch.ReplaceString` then wraps with `RewrittenYaml`
   under root key `robot_<id>`.
5. **Staggered launch**: `TimerAction(period=0.3 * i)` per robot to
   avoid lifecycle race conditions.

### 6.4 Formation layer (`iros_llm_swarm_formation`)

Two Python nodes:

- `formation_manager_node`: in-memory registry + 8 services. Latched
  publish of `/formations/config` after every mutation. Computes
  `footprint` polygon as a 16-point circle approximation.
  **PD math is NOT here** — it's in the per-robot followers.
- `formation_monitor_node`: subscribes to registry + per-robot odom,
  computes per-follower error vs target offset, publishes
  `/formations/status` at `monitor_hz` (default 10) with the state
  machine. **BROKEN is sticky** — only registry change (via
  manager) clears it.

**YAML schema** (different field names than service request!):

```yaml
formations:
  - id: "wedge"          # ↔ formation_id
    leader: "robot_1"    # ↔ leader_ns
    followers:
      - ns: "robot_2"    # ↔ entry in follower_ns
        dx: -1.0         # ↔ entry in offsets_x (leader body frame)
        dy:  0.6         # ↔ entry in offsets_y
```

---

## 7. Behavior Tree deep-dive (`iros_llm_swarm_bt`) ⭐

This is the layer the work targets. Sources:
- `include/iros_llm_swarm_bt/swarm_bt_nodes.hpp`
- `src/swarm_bt_nodes.cpp` (BT plugin — `BT_REGISTER_NODES`)
- `src/test_bt_runner.cpp` (runner + scenario)
- `behavior_trees/swarm_navigate_to_pose.xml` (main tree)
- `behavior_trees/test_*.xml` (per-node smoke trees)
- `launch/test_bt_runner.launch.py`
- Built docs: `iros_llm_swarm_bt/doc/index.rst`

### 7.1 BT nodes (4)

All four register under their unqualified class name in the plugin.

#### `MapfPlan` — the centerpiece

`BT::StatefulActionNode`. Wraps **two** action clients:
- `/swarm/set_goals` (the MAPF action)
- `/llm/decision` (the LLM advisory action)

**Ports**:

| Name | Dir | Type | Notes |
|---|---|---|---|
| `robot_ids` | in | `vector<int>` | cast to `uint32` for the action |
| `goals` | in | `vector<geometry_msgs/Point>` | parallel to `robot_ids` |
| `mapf_ok` | out | `bool` | true on success |
| `mapf_info` | out | `string` | summary line |
| `mapf_warn` | out | `string` | last warning from feedback |

**Lifecycle (`onStart` → `onRunning` → terminal)**:

1. `onStart()`: validate inputs, wait 2 s for action server, send goal
   with feedback callback. Reset all state (buffers, decisions,
   futures).
2. `onRunning()` is **strictly poll-based**, never blocks:
   - If LLM produced a verdict, act on it: `"abort"` → cancel
     MAPF + FAILURE; `"replan"` → cancel + FAILURE + write
     `@mapf_decision = "replan"` to blackboard for the outer
     controller; `"wait"` → continue RUNNING.
   - Drive the LLM future state machine
     (goal_handle → result_future).
   - Drive the MAPF future state machine.
   - On terminal MAPF result: populate outputs, return SUCCESS or
     FAILURE.
3. `onHalted()`: cancel both MAPF and LLM goals, clear buffered
   decision.

**LLM feedback path (the integration point)**:

```
on_feedback(MAPF feedback) →
  build one-line summary →
  push to ring buffer (max_info_buffer_, default 50) →
  if feedback.warning != "":
    setOutput("mapf_warn", warning)
    send_to_llm("WARN", warning)        # immediate, async
  elif feedback.info != "" and llm_log_interval_sec > 0:
    if since_last >= llm_log_interval_sec:
      send_to_llm("INFO", info)          # periodic
```

`send_to_llm()` builds a `LlmDecision` goal with `level`, `event`,
and the **entire current ring buffer** as `log_buffer`. Skips if
another LLM call is in flight (`llm_pending_`) or if the action
server isn't ready. **MAPF is NEVER blocked by LLM unavailability.**

**Failure modes** that complete with FAILURE:
- Missing/mismatched/empty input ports
- `/swarm/set_goals` server unavailable for 2 s
- Goal rejected
- Action transport error (`ResultCode != SUCCEEDED`)
- Result with `num_agents_planned == 0`
- LLM verdict `"abort"` or `"replan"`

**Subtle**: a *partial* result (`success=false` but
`num_agents_planned > 0`) returns SUCCESS with a WARN log — read
`mapf_info` if the caller cares.

**Node-level ROS parameters** (declared on first `MapfPlan`
construction):
- `llm_log_interval_sec` (default `0.0` = disabled, only WARN events
  trigger LLM)
- `llm_info_buffer_size` (default `50`)

#### `SetFormation` — wraps `/formation/set`

`BT::StatefulActionNode`. Inputs: `formation_id`, `leader_ns`,
`follower_ns`, `offsets_x`, `offsets_y`, `activate` (default `true`).
Outputs: `formation_enabled` (= `success && activate`),
`active_formation` (echo of `formation_id`). Validates that
`follower_ns` / `offsets_x` / `offsets_y` lengths match before
sending.

#### `DisableFormation` — wraps `/formation/deactivate`

Inputs: `formation_id`. Output: `formation_enabled` (always `false`
on completion).

#### `CheckMode` — synchronous condition

Inputs: `mode` (typically bound to `{@mode}` blackboard), `expected`
(typically a literal). Returns SUCCESS if equal, FAILURE otherwise.

### 7.2 Reference tree

`behavior_trees/swarm_navigate_to_pose.xml`:

```
ReactiveFallback "ModeDispatch"
├── CheckMode mode={@mode} expected="idle"
├── Sequence "MapfBranch"
│   ├── CheckMode mode={@mode} expected="mapf"
│   └── MapfPlan robot_ids={@robot_ids} goals={@goals}
│                mapf_ok={@mapf_ok} mapf_info={@mapf_info}
└── Sequence "FormationBranch"
    ├── CheckMode mode={@mode} expected="formation"
    └── SetFormation formation_id={@formation_id} leader_ns={@leader_ns}
                     follower_ns={@follower_ns}
                     offsets_x={@offsets_x} offsets_y={@offsets_y}
                     activate="true"
                     formation_enabled={@formation_enabled}
                     active_formation={@active_formation}
```

**Why ReactiveFallback**: it re-evaluates *every* child on *every*
tick. Switching `@mode` mid-execution → currently-running action
gets `onHalted()` → next tick picks the new branch. This is the
clean way to interrupt a mission.

**No `DisableFormation` branch in the main tree.** Disbanding
happens implicitly via mode flip (formation → idle); the
`DisableFormation` BT class exists for future expansion and is
exercised only by `behavior_trees/test_disable_formation.xml`.

**XML version note**: file uses `BTCPP_format="4"` while CMake links
against `behaviortree_cpp_v3`. Today's BT.CPP v3 builds accept
format-4 XML; if the BT.CPP version changes, expect to revisit.

### 7.3 Blackboard convention

The runner pre-creates the blackboard with:
- `"node"` → the host `rclcpp::Node` (every BT class reads this to
  create ROS clients)
- `@mode = "idle"`, `@robot_ids = []`, `@goals = []`

| Key | Owner / use |
|---|---|
| `@mode` | Operator input — `"idle"` / `"mapf"` / `"formation"` |
| `@robot_ids`, `@goals` | Operator input — MAPF mission |
| `@formation_id`, `@leader_ns`, `@follower_ns`, `@offsets_x`, `@offsets_y` | Operator input — formation definition |
| `@mapf_ok`, `@mapf_info`, `@mapf_warn` | `MapfPlan` outputs |
| `@formation_enabled`, `@active_formation` | `SetFormation` / `DisableFormation` outputs |
| `@mapf_decision` | `MapfPlan` writes `"replan"` on LLM verdict; **outer controller must consume it, set new goals, flip `@mode` back to `"mapf"`** |
| `@mapf_failed`, `@formation_failed` | Set by the runner on terminal FAILURE; read by the embedded scenario thread to abort |

### 7.4 The runner (`test_bt_runner`)

Two modes via the `scenario` ROS parameter:

- **`scenario:=false` (default — idle runner)**: subscribes to
  `/fleet/cmd` (`std_msgs/String`, `"key=value"` format) and writes
  the corresponding entry to the blackboard. Recognized keys:
  `mode`, `formation_id`, `leader_ns`, `robot_ids` (CSV ints),
  `goals` (CSV `x,y,x,y,...`), `follower_ns` (CSV), `offsets_x`,
  `offsets_y` (CSV doubles).

- **`scenario:=true` (embedded integration scenario)**: side thread
  drives a 6-step 20-robot scenario:
  1. MAPF all 20 → grid centered at `(15, 15)`.
  2. Formation WEDGE_20 — orange squad `robot_0..9`, `robot_1`
     leader.
  3. Formation LINE_BLUE — blue squad `robot_10..14`, `robot_10`
     leader.
  4. MAPF cross-swap — orange to top-right, blue to bottom-left
     (stress test).
  5. MAPF home.
  6. Idle.

**Tick loop** (10 Hz):
- `spin_some` ROS, read `@mode`, publish to `/fleet/mode`.
- On transition INTO `"idle"` → `haltTree()` immediately.
- While `@mode == "idle"` → don't tick the tree at all.
- On terminal SUCCESS/FAILURE → publish results on `/fleet/mapf_ok`
  and `/fleet/formation_enabled`, halt, and on FAILURE flip
  `@mode = "idle"`.

**Published topics** (for external observers):
- `/fleet/mode` (`std_msgs/String`, latched) — mirror of `@mode`
- `/fleet/mapf_ok` (`std_msgs/String`, "true"/"false")
- `/fleet/formation_enabled` (`std_msgs/Bool`)

Run with:
```bash
ros2 launch iros_llm_swarm_bt test_bt_runner.launch.py
# or with scenario:
ros2 run iros_llm_swarm_bt test_bt_runner --ros-args -p scenario:=true
```

### 7.5 LLM integration point — exactly where to plug in

The BT side **is already wired**. To complete the loop you need:

1. **An LLM agent process** that exposes `/llm/decision` as an action
   server. Goal contains `level` (WARN/ERROR/INFO), `event` (one
   line), `log_buffer[]` (last ~50 feedback lines). Return
   `decision ∈ {"wait", "abort", "replan"}` and a human-readable
   `reasoning` string. Optionally stream `reasoning_stream` as
   feedback.

2. **A bridge for `@mapf_decision = "replan"`.** When the LLM says
   "replan", `MapfPlan` cancels the mission and writes
   `@mapf_decision = "replan"` to the blackboard. Today nothing
   reads it. The outer controller (a new BT runner subscriber, an
   extra BT branch, or LLM-driven goal generation) must consume that
   signal and prepare new `@robot_ids` / `@goals`.

3. **Tune `llm_log_interval_sec`.** Default `0.0` → only WARN/ERROR
   events fire LLM calls. Setting it to e.g. `5.0` gets a periodic
   INFO ping every 5 s during a mission, useful for the LLM to
   update its model of progress.

What the LLM sees: a one-line summary built per feedback tick:
```
[t=12345ms status=executing arrived=3 active=15 stall=2 replans=1] WARN: 2 unplanable skipped
```
plus the last `llm_info_buffer_size` such lines as context.

---

## 8. Mission lifecycle (concrete example)

A `swarm/set_goals` call for 20 robots heading to `(13.5, 16.5)`:

```
Client (BT MapfPlan)
    │ SetGoals.Goal(robot_ids=[0..19], goals=[(13.5,16.5)×20 distributed])
    ▼
mapf_lns2_node
    │ feedback: status="validating"   (build agents from /odom + /footprint + /map)
    │ feedback: status="planning"     (LNS2 initial soln + repair to 0 collisions)
    ├── /robot_*/mapf_plan (per robot, plan_id=1)
    │ feedback: status="executing", arrived=0, active=20
    ▼
path_follower_node × 20
    │ subscribe mapf_plan (plan_id=1)
    │ dispatch segment to /robot_<id>/follow_path (Nav2 action)
    │ publish /robot_<id>/follower_status heartbeat (2 Hz)
    ▼
Nav2 controller_server (RPP) → cmd_vel → Stage

Meanwhile:
  - planner ticks schedule monitor at replan_check_hz=2
  - on deviation > replan_threshold_m: warm-start replan
    → publish new mapf_plan with plan_id=2
    → followers see new plan_id, abort current Nav2 goal, restart
  - feedback: status="replanning", replans_done++
  - on stall: bench robot, publish empty mapf_plan to it, continue
    others. After unplanable_retry_delay_sec, revive.

When all 20 arrive:
  - feedback: status="executing", arrived=20, active=0
  - mission completes with success=true, total_replans, total_execution_sec
```

**MapfPlan BT** also runs an LLM side channel:
- Every WARN in feedback → immediate LLM call.
- Every INFO at most every `llm_log_interval_sec` → LLM call.
- LLM `"abort"` → BT cancels mission and returns FAILURE.
- LLM `"replan"` → BT cancels mission, writes `@mapf_decision`,
  returns FAILURE (outer controller picks it up).
- LLM `"wait"` → BT continues running.

---

## 9. Build / launch / test

### 9.1 Inside the Docker container

```bash
# from host
docker compose up terminal
docker compose exec terminal bash

# inside container
source /home/fabian/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///home/fabian/ros2_ws/src/scripts/cyclonedds_swarm.xml

# Build (interfaces first when types change)
cd /home/fabian/ros2_ws
colcon build --packages-select iros_llm_swarm_interfaces
colcon build
```

### 9.2 Launch (LNS2 stack, supported)

```bash
# Stack only, no formations
ros2 launch iros_llm_swarm_bringup swarm_lns.launch.py scenario:=warehouse_2

# With formation manager (loads formations_2.yaml, auto_activate=true)
ros2 launch iros_llm_swarm_bringup swarm_lns_formation.launch.py
```

After "MAPF stack ready" (~15 s):

```bash
# Send goals (test client)
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 13.5 --goal-y 16.5
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0
ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json
```

### 9.3 Run the BT runner against the stack

```bash
# Idle runner — drive it from /fleet/cmd
ros2 launch iros_llm_swarm_bt test_bt_runner.launch.py

# In another terminal:
ros2 topic pub --once /fleet/cmd std_msgs/msg/String \
  "data: 'mode=mapf'"

# Or run the embedded scenario
ros2 run iros_llm_swarm_bt test_bt_runner --ros-args -p scenario:=true
```

### 9.4 Useful inspection commands

```bash
# What does the planner see?
ros2 topic echo /robot_0/follower_status
ros2 topic echo /formations/status
ros2 topic echo /fleet/mode

# Feedback during a mission (rich):
ros2 action send_goal /swarm/set_goals \
  iros_llm_swarm_interfaces/action/SetGoals \
  "{robot_ids: [0,1,2], goals: [{x: 5,y: 5},{x: 6,y: 5},{x: 5,y: 6}]}" \
  --feedback
```

---

## 10. Gotchas and legacy notes

1. **PBS launches are legacy.** `swarm_mapf.launch.py` and
   `swarm_mapf_formation.launch.py` boot the legacy PBS planner
   (`iros_llm_swarm_mapf`) + its companion follower
   (`iros_llm_swarm_robot/motion_controller_node`). For new work use
   `swarm_lns.launch.py` and `swarm_lns_formation.launch.py`. The
   code is kept in tree but won't be evolved.

2. **PBS vs LNS follower diverge in topic names.**
   - PBS: planner publishes `nav_msgs/Path` on `/<ns>/mapf_path`,
     follower (`motion_controller_node`) consumes it. **No
     `FollowerStatus` topic.**
   - LNS: planner publishes `MAPFPlan` on `/<ns>/mapf_plan`,
     follower (`path_follower_node`) consumes and **publishes
     `FollowerStatus`**.

3. **`num_robots` does NOT shrink the simulator.** Stage worlds
   hard-code 20 robots. The `num_robots` argument controls how many
   get a Nav2 stack + follower; extras sit idle.

4. **AMCL is disabled.** Do not "fix" the static `map → odom` TF —
   it's intentional. AMCL drifts at 20 robots.

5. **`ResettingObstacleLayer` is mandatory** in the local costmap
   for multi-robot scenarios on the Stage scanner. Without it,
   ghost-obstacle trails appear behind every other robot.

6. **`large_cave` reuses `cave.pgm`** at half resolution. Not a
   typo.

7. **Formation YAML has different field names** than the service
   request: `id` vs `formation_id`, `leader` vs `leader_ns`,
   `followers[].dx/dy` vs `offsets_x/y`. Don't conflate.

8. **`auto_activate: true` is hard-coded** in formation bringups —
   formations from YAML start active. Override or call
   `/formation/deactivate` if you need a passive registry.

9. **Two LNS launch files have inconsistent CLI**: `swarm_lns.launch.py`
   uses `scenario:=...`, `swarm_lns_formation.launch.py` uses
   `world_file:=...`. Pass arguments explicitly when scripting.

10. **20-robot ceiling.** Above this, DDS discovery overhead
    dominates; CycloneDDS profile (`cyclonedds_swarm.xml`) is
    required even at 20.

11. **Gazebo (`iros_llm_swarm_simulation`) is preview.** Different
    scan topic (`/scan` vs `/base_scan`), different lidar specs (10 m
    vs 3.5 m), no map in the world. Nav2 params are tuned for Stage —
    switching needs rework.

---

## 11. Design questions worth checking when you start

(Things to ask the user / read more about before making changes.)

- **What does "the LLM agent" look like in this project?** It's not
  in tree. Is the plan to add it as a new ROS package, run it
  out-of-process and connect to `/llm/decision`, or front it through
  a gateway?
- **What should `@mapf_decision = "replan"` do downstream?** Today
  nothing reads it. Where does the decision-→-new-goals flow
  belong: a new BT branch, a Python wrapper around the runner, or
  the LLM agent itself?
- **Does the LLM see the full feedback log per call?** The current
  buffer is the last 50 lines; decide if that's the right context
  shape.
- **Should the LLM be able to drive formations directly, not just
  vote on MAPF?** The `SetFormation` and `DisableFormation` BT
  nodes are ready to be triggered.
- **Mode-switching control flow**: README mentions "ExecuteMAPF",
  "ExecuteFormation", "EmergencyStop" custom BT nodes as roadmap.
  Worth scoping which of those land first.

---

## 12. Memory on the user (for context)

- **Evgenii**, ROS 2 robotics engineer.
- Working primarily on the LNS2 MAPF planner refactor
  (`iros_llm_swarm_mapf_lns/mapf_lns2_node.cpp` decomposition along
  the `node_utils` / `plan_publisher` / `robot_lifecycle` boundaries
  already in tree).
- Russian-speaking; project's own docs and code are in English.
- An Obsidian wiki exists at
  `~/projects/claude-obsidian/wiki/llm-swarm/` with pre-existing
  notes on LNS2 internals (refactoring analysis, soft A*,
  collision table, destroy operators, warm start).

---

## 13. Where to find more

Each package has its own `doc/index.rst`. Per-package depth:

| Package | What's inside the docs |
|---|---|
| `iros_llm_swarm_docs` | Aggregator with overview, architecture, packages, getting started — the entry point. |
| `iros_llm_swarm_interfaces` | Field-by-field reference for every msg/srv/action. |
| `iros_llm_swarm_mapf_lns` | 3 pages: overview, **algorithm internals**, ROS API. |
| `iros_llm_swarm_robot` | PBS-side dual-mode follower. |
| `iros_llm_swarm_formation` | Manager + monitor + YAML schema + state machine. |
| `iros_llm_swarm_bt` | **Most relevant for this work.** All four nodes, reference tree, runner, blackboard convention. |
| `iros_llm_swarm_local_nav` | Nav2 wiring, RPP rationale, scenario flow, `test_local_planne`. |
| `iros_llm_swarm_costmap_plugins` | `ResettingObstacleLayer` rationale and limits. |
| `iros_llm_swarm_simulation_lite` | Stage worlds, maps, robot model, scenario YAML. |
| `iros_llm_swarm_simulation` | Gazebo Fortress (preview, full compatibility punch-list). |
| `iros_llm_swarm_bringup` | All five launch files + inconsistencies/gotchas. |

Build the rendered HTML (sphinx + rosdoc2):

```bash
cd /home/mook/projects/llm_swarm
rm -rf /tmp/swarm_docs_build /home/mook/projects/llm_swarm/docs_build
for pkg in iros_llm_swarm_interfaces iros_llm_swarm_mapf_lns \
           iros_llm_swarm_robot iros_llm_swarm_formation \
           iros_llm_swarm_bt iros_llm_swarm_costmap_plugins \
           iros_llm_swarm_local_nav iros_llm_swarm_simulation_lite \
           iros_llm_swarm_simulation iros_llm_swarm_bringup \
           iros_llm_swarm_docs; do
  rosdoc2 build --package-path "$pkg" --output-directory /tmp/swarm_docs_build
done
python3 -m http.server 8765 --directory /tmp/swarm_docs_build
# open http://localhost:8765/iros_llm_swarm_docs/index.html
```

Each per-package page links back to the aggregator via a
`« back to overview` link at the top.

---

## 14. Quick orientation map for a fresh session

1. Read `iros_llm_swarm_bt/include/iros_llm_swarm_bt/swarm_bt_nodes.hpp`
   and `src/swarm_bt_nodes.cpp` — the BT nodes themselves.
2. Read `iros_llm_swarm_bt/src/test_bt_runner.cpp` — how the BT is
   driven, idle vs scenario mode, blackboard wiring.
3. Read `iros_llm_swarm_bt/behavior_trees/swarm_navigate_to_pose.xml`
   — the mode-dispatch idiom.
4. Read `iros_llm_swarm_interfaces/action/SetGoals.action` and
   `LlmDecision.action` — the action contracts.
5. Skim `iros_llm_swarm_mapf_lns/src/mapf_lns2_node.cpp` *only* if
   you need to know what the planner publishes — focus on the
   action server registration and feedback structure.
6. Skim `iros_llm_swarm_mapf_lns/src/path_follower_node.cpp` — the
   per-robot consumer of `MAPFPlan`. Important for understanding
   the state machine the planner observes via `FollowerStatus`.
7. Check `iros_llm_swarm_formation/iros_llm_swarm_formation/formation_manager_node.py`
   — the service implementations the BT formation nodes call.

After that, the rosdoc2 site (built site or the `doc/index.rst`
sources directly) covers everything else.

— end —
