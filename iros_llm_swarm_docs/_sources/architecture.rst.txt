System Architecture
===================

The stack is organized as a vertical pipeline. The simulator publishes
ground-truth state, the MAPF planner turns fleet-level goals into
time-indexed paths, per-robot motion controllers turn those paths into
either Nav2 ``follow_path`` actions or direct ``cmd_vel`` commands when
a formation is active, and an optional behavior tree + LLM orchestrator
sit on top as the mission front-end. A dynamic-obstacle manager and a
pair of RViz plugins (operator panel + click-to-command tools) round
out the operator-facing surface.

Layered view
------------

::

    ┌──────────────────────────────────────────────────────────────────────┐
    │                        IROS LLM SWARM SIMULATION                     │
    ├──────────────────────────────────────────────────────────────────────┤
    │                                                                      │
    │  ┌─────────────┐ ┌──────────────┐ ┌────────────────────────────────┐ │
    │  │  Stage 2D   │ │     RViz     │ │         CycloneDDS             │ │
    │  │  Simulator  │ │  + plugins   │ │       domain_id = 42           │ │
    │  └──────┬──────┘ └──────┬───────┘ └────────────────────────────────┘ │
    │         │               │ /llm/chat, /llm/execute_plan,             │
    │         │               │ /llm/command, /llm/events, /bt/state      │
    │         │               │ /obstacles/*, /doors/*                    │
    │         │ /tf, /robot_*/odom, /robot_*/scan                         │
    │  ┌──────▼──────────────────────────────────────────────────────────┐ │
    │  │  DYNAMIC OBSTACLES (optional, iros_llm_swarm_obstacles)         │ │
    │  │  sub: /raw_map  → merge circles/rects/doors → pub /map (latched)│ │
    │  │  srvs: /obstacles/{add_*, remove, list}, /doors/{open, close}   │ │
    │  └──────┬──────────────────────────────────────────────────────────┘ │
    │         │ /map (TRANSIENT_LOCAL)                                    │
    │  ┌──────▼──────────────────────────────────────────────────────────┐ │
    │  │  MAPF PLANNER  (PBS or LNS2 — selectable at launch)             │ │
    │  │  action: /swarm/set_goals                                       │ │
    │  │  PBS  pub: /robot_*/mapf_path  (nav_msgs/Path)                  │ │
    │  │  LNS2 pub: /robot_*/mapf_plan  (MAPFPlan, with hold + plan_id)  │ │
    │  │  sub: /map, /robot_*/odom, /robot_*/follower_status (LNS2)      │ │
    │  └──────┬──────────────────────────────────────────────────────────┘ │
    │         │ /robot_*/mapf_path  or  /robot_*/mapf_plan                │
    │  ┌──────▼──────────────────────────────────────────────────────────┐ │
    │  │  PER-ROBOT FOLLOWER  (one per robot, dual-mode)                 │ │
    │  │  PBS stack:  pbs_motion_controller  (iros_llm_swarm_robot)      │ │
    │  │  LNS2 stack: lns_motion_controller  (iros_llm_swarm_robot)      │ │
    │  │  AUTONOMOUS  → /robot_*/follow_path  (Nav2 action)              │ │
    │  │  FORMATION   → /robot_*/cmd_vel       (PD controller)           │ │
    │  │  sub: /formations/config                                        │ │
    │  └──────┬───────────────────────────┬──────────────────────────────┘ │
    │         │ AUTONOMOUS                │ FORMATION                      │
    │  ┌──────▼──────────────┐   ┌────────▼─────────────────────────┐      │
    │  │  Nav2 LOCAL NAV     │   │  FORMATION LAYER                 │      │
    │  │  (per robot)        │   │  /formations/config (latched)    │      │
    │  │  controller_server  │   │  /formations/status (10 Hz)      │      │
    │  │  behavior_server    │   │  services: /formation/*          │      │
    │  │  zone_map_server    │   │  formation_manager_node          │      │
    │  │  ResettingObst.Layer│   │  formation_monitor_node          │      │
    │  └─────────────────────┘   └──────────────────────────────────┘      │
    │                                                                      │
    │  ┌─────────────────────────────────────────────────────────────────┐ │
    │  │  BT RUNNER + LLM ORCHESTRATOR  (optional, full_demo)            │ │
    │  │  BehaviorTree.CPP v3 nodes → /swarm/set_goals, /formation/*     │ │
    │  │  pub: /bt/state (heartbeat)                                     │ │
    │  │  Channel 1 reactive:  BT → /llm/decision (action)               │ │
    │  │  Channel 2 proactive: passive_observer → /llm/command           │ │
    │  │  Channel 3 chat:      RViz → /llm/chat → /llm/execute_plan      │ │
    │  └─────────────────────────────────────────────────────────────────┘ │
    └──────────────────────────────────────────────────────────────────────┘

Topic graph
-----------

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Topic
     - Type
     - Publisher
     - Subscribers
   * - ``/robot_N/odom``
     - ``nav_msgs/Odometry``
     - Stage simulator
     - MAPF planner, motion controller, formation monitor
   * - ``/robot_N/mapf_path``
     - ``nav_msgs/Path``
     - PBS planner (``iros_llm_swarm_mapf``)
     - PBS follower (``pbs_motion_controller``)
   * - ``/robot_N/mapf_plan``
     - ``iros_llm_swarm_interfaces/MAPFPlan``
     - LNS2 planner (``iros_llm_swarm_mapf_lns``)
     - LNS2 follower (``lns_motion_controller``)
   * - ``/robot_N/cmd_vel``
     - ``geometry_msgs/Twist``
     - Per-robot follower (FORMATION mode)
     - Stage simulator
   * - ``/robot_N/follower_status``
     - ``iros_llm_swarm_interfaces/FollowerStatus``
     - LNS2 follower (``lns_motion_controller``) — *PBS follower does
       not publish this*
     - LNS2 planner
   * - ``/raw_map``
     - ``nav_msgs/OccupancyGrid``
     - ``map_server`` (when obstacles manager is in the loop)
     - ``dynamic_obstacle_manager``
   * - ``/map``
     - ``nav_msgs/OccupancyGrid`` (TRANSIENT_LOCAL)
     - ``zone_map_server`` *or* ``dynamic_obstacle_manager`` (merged
       static + dynamic)
     - MAPF planner, Nav2 costmaps
   * - ``/obstacles/markers``
     - ``visualization_msgs/MarkerArray`` (TRANSIENT_LOCAL)
     - ``dynamic_obstacle_manager``
     - RViz
   * - ``/formations/config``
     - ``FormationsConfig``
     - ``formation_manager``
     - Per-robot followers (TRANSIENT_LOCAL, latched)
   * - ``/formations/status``
     - ``FormationsStatus``
     - ``formation_monitor``
     - LLM orchestrator, RViz panel, external clients
   * - ``/bt/state``
     - ``iros_llm_swarm_interfaces/BTState``
     - ``test_bt_runner``
     - ``passive_observer``, ``chat_server``, RViz panel
   * - ``/llm/events``
     - ``iros_llm_swarm_interfaces/LlmEvent``
     - ``decision_server``, ``passive_observer``, ``chat_server``
     - RViz panel (Events tab)
   * - ``/llm_panel/markers``
     - ``visualization_msgs/MarkerArray``
     - RViz LLM panel (30 Hz)
     - RViz

Actions
-------

.. list-table::
   :header-rows: 1
   :widths: 25 30 25 20

   * - Action
     - Type
     - Server
     - Clients
   * - ``/swarm/set_goals``
     - ``iros_llm_swarm_interfaces/action/SetGoals``
     - ``mapf_planner_node`` *or* ``mapf_lns2_node``
     - BT (``MapfPlan`` node), test scripts, ``execute_server``
   * - ``/robot_N/follow_path``
     - ``nav2_msgs/action/FollowPath``
     - Nav2 ``controller_server``
     - Per-robot follower (AUTONOMOUS mode)
   * - ``/llm/decision``
     - ``iros_llm_swarm_interfaces/action/LlmDecision``
     - ``decision_server`` (channel 1)
     - BT nodes on WARN/ERROR
   * - ``/llm/command``
     - ``iros_llm_swarm_interfaces/action/LlmCommand``
     - ``LlmCommandReceiver`` in BT runner
     - ``passive_observer`` (channel 2), RViz STOP-ALL, ``SendLlmGoalTool``
   * - ``/llm/chat``
     - ``iros_llm_swarm_interfaces/action/LlmChat``
     - ``chat_server`` (channel 3)
     - RViz panel chat tab
   * - ``/llm/execute_plan``
     - ``iros_llm_swarm_interfaces/action/LlmExecutePlan``
     - ``execute_server``
     - RViz panel "Execute pending plan" button

Formation services
------------------

All formation operations are CRUD-style services on the
``/formation/*`` namespace:

* ``/formation/set`` — create or update a formation.
* ``/formation/activate`` / ``/formation/deactivate`` — toggle whether
  followers track their offsets.
* ``/formation/get`` / ``/formation/list`` — read state.
* ``/formation/load`` / ``/formation/save`` — YAML persistence.

The full list and message definitions live in
``iros_llm_swarm_interfaces``.

Dynamic-obstacle services
-------------------------

When ``iros_llm_swarm_obstacles`` is launched, the
``dynamic_obstacle_manager`` exposes:

* ``/obstacles/add_circle`` / ``/obstacles/add_rectangle`` /
  ``/obstacles/add_door`` — add obstacles by id with geometry.
* ``/obstacles/remove`` — remove an obstacle by id.
* ``/obstacles/list`` — read all current obstacles.
* ``/doors/open`` / ``/doors/close`` — toggle door state (open doors do
  not block cells; closed doors do).

The merged map is republished on ``/map`` with ``TRANSIENT_LOCAL`` QoS,
so any new MAPF planner or Nav2 costmap that joins late receives the
latest state automatically.

TF tree
-------

A unified TF tree is enforced via Stage's ``one_tf_tree: true`` and
``enforce_prefixes`` options. AMCL is disabled, so each robot has a
static identity transform from ``map`` to its ``odom`` frame::

    map
     └── robot_N/odom        (static, identity — AMCL disabled)
          └── robot_N/base_link   (published by Stage)

Mission lifecycle
-----------------

The ``/swarm/set_goals`` action is **long-lived**: a single goal covers
the entire mission, from initial planning to all robots reaching their
targets. Internal schedule monitoring and replanning happen autonomously
inside the action server. External clients see one atomic operation::

    validating → planning → executing → replanning (on deviation) → succeeded
                                                                  ↘ failed
                                                                  ↘ cancelled

Feedback is published periodically (~500 ms) and contains:

* ``status`` — phase string.
* ``elapsed_ms`` — time since the goal was accepted.
* ``robots_arrived`` / ``robots_active`` / ``robots_deviated``.
* ``replans_done`` — count of triggered replans.

Cancellation stops all robots immediately by clearing their assigned
paths and sending zero velocities.

Per-robot follower
------------------

Each robot runs one dual-mode follower from ``iros_llm_swarm_robot``.
Both PBS and LNS2 stacks share the same launch file
(``motion_controllers.launch.py``); the executable is selected by the
``controller_type`` argument:

* **PBS stack** uses ``pbs_motion_controller``. Consumes
  ``nav_msgs/Path`` on ``/<ns>/mapf_path`` (each pose carries a
  scheduled arrival time); chunks the path at hold points to keep the
  robot inside ``schedule_tolerance_sec`` of the PBS schedule. Does
  **not** publish ``FollowerStatus`` — the PBS planner detects deviation
  by polling odometry.
* **LNS2 stack** uses ``lns_motion_controller``. Consumes ``MAPFPlan``
  on ``/<ns>/mapf_plan`` (explicit ``MAPFStep`` entries with
  ``hold_sec`` and a ``plan_id`` token); publishes ``FollowerStatus``
  with state, current step, ``plan_id``, hold countdown, and Nav2
  failure count so the planner can distinguish frozen from holding
  followers.

Both implementations behave the same in **FORMATION_FOLLOWER** mode:
a PD controller tracks ``leader_pos + R(leader_yaw) · offset`` in
world coordinates and publishes directly to ``cmd_vel``. Mode
switching is event-driven by ``/formations/config``; the node does not
restart.

LLM orchestrator
----------------

When ``swarm_full_demo.launch.py`` is used, the
``iros_llm_orchestrator`` package adds an LLM advisory layer with
**three independent channels**:

* **Channel 1 — reactive (always on).** BT nodes (``MapfPlan``,
  ``SetFormation``) call ``/llm/decision`` directly when they hit
  WARN/ERROR. The action server returns one of ``wait``, ``abort``,
  ``replan``. Lowest latency; one decision per BT event.
* **Channel 2 — proactive (opt-in).** ``passive_observer`` subscribes
  to ``/bt/state`` and decides on its own whether to intervene. When it
  fires, it calls ``/llm/command`` (received by ``LlmCommandReceiver``
  in the BT runner) to push commands into the BT blackboard. Disabled
  by default; enable with ``enable_passive_observer:=true``. Channel 1
  remains active alongside.
* **Channel 3 — operator chat.** The RViz panel sends free-form text
  to ``/llm/chat``. ``chat_server`` streams the reply back chunk-by-
  chunk, parses an executable plan JSON, optionally previews it as a
  tree, and dispatches leaves through ``/llm/execute_plan`` →
  ``execute_server``. Includes a remediation loop (default two retries)
  that re-prompts the LLM on execution failure with a refreshed runtime
  context.

The chat channel can ground itself in live system state through a
**read-only MCP** context provider (``mcp_readonly``, default). The
provider spawns an MCP subprocess (``uvx ros-mcp --transport=stdio``),
calls only allowlisted tools (``get_topics``, ``get_topic_type``,
``subscribe_once``, ``get_nodes``, ``get_services``, ``get_actions``…),
and returns a bounded snapshot to the prompt. The LLM never sees the
tools directly — only the snapshot — so it cannot write or execute via
MCP. On execution failure, the remediation loop refreshes context with
failure-aware tool selection (e.g. formation issues re-subscribe to
``/formations/status``).

Every LLM call is appended to JSONL files for SFT dataset collection:

* Channel 1 → ``~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl``.
* Channel 2 → ``~/.ros/llm_commands/decisions_YYYYMMDD.jsonl``.

Channels 1 & 2 share the BT command bus and never collide: channel 2
checks the ``llm_thinking`` flag on ``/bt/state`` and skips its own
trigger while channel 1 is active.

Operator surface (RViz)
-----------------------

Two C++/Qt RViz plugins ship with the stack:

* ``iros_llm_rviz_panel`` — a single docked panel with five tabs (Chat,
  MAPF, Events, BT, Info) and a sticky status bar. The Chat tab drives
  channel 3; the BT tab shows the current ``/bt/state`` row by row; the
  Events tab streams ``/llm/events``; the MAPF tab tracks per-robot
  arrival progress and active counts. A red **STOP ALL** button bypasses
  the LLM and publishes ``LlmCommand{mode=idle}`` directly to
  ``/llm/command``. All ROS callbacks marshal into Qt slots through
  ``Qt::QueuedConnection`` — never touch widgets from an executor
  thread.
* ``iros_llm_rviz_tool`` — three click-to-command tools registered with
  RViz:

  * **SendLlmGoalTool** (``g``) — pick a robot group, click on the map,
    and N goals are spread around the click and sent to ``/llm/command``
    as a ``mode=mapf`` command. No LLM in the loop.
  * **PlaceObstacleTool** (``b``) — left-click adds a circle / rectangle
    / door (shape and size from the Tool Properties panel); right-click
    removes the nearest obstacle.
  * **DoorTool** (``d``) — left-click opens, right-click closes the door
    whose ID is set in the Tool Properties panel.

Startup sequence
----------------

The bringup launch files apply staggered timers so DDS discovery and
costmap initialization can settle before the next layer comes online.
Approximate timing for ``swarm_full_demo.launch.py``:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Time
     - Component
   * - 0 s
     - Stage simulator + RViz (with ``iros_llm_rviz_panel`` /
       ``iros_llm_rviz_tool`` loaded from the rviz config)
   * - 1 s
     - Nav2 (``zone_map_server`` + per-robot controller / behavior)
   * - 10 s
     - MAPF planner (PBS or LNS2 — selected by ``planner:=pbs|lns``)
   * - 12 s
     - Per-robot followers ×N + formation manager / monitor (when
       ``enable_formation:=true``)
   * - 18 s
     - LLM orchestrator (``decision_server``, ``chat_server``,
       ``execute_server``, plus ``passive_observer`` and
       ``rosbridge_server`` when enabled)
   * - 20 s
     - BT runner

Environment notes
-----------------

* **DDS profile.** ``scripts/cyclonedds_swarm.xml`` tunes CycloneDDS for
  20-robot discovery overhead. Set
  ``CYCLONEDDS_URI=file:///…/cyclonedds_swarm.xml`` and
  ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`` before launching anything.
* **Domain ID.** The project pins ``ROS_DOMAIN_ID=42`` to keep simulation
  traffic isolated from other ROS 2 work on the same host.
* **Time source.** ``use_sim_time=true`` is the default everywhere — the
  Stage simulator owns the clock.
