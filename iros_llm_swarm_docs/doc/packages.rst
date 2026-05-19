Package Map
===========

The repository is split into small, single-purpose ROS 2 packages. The
table below summarizes each package; the per-package generated
documentation (linked from the main :doc:`index`) goes deeper into APIs,
parameters, and internals.

Overview table
--------------

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Package
     - Language
     - Role
   * - ``iros_llm_swarm_interfaces``
     - msg / srv / action
     - Shared ROS 2 type definitions used by every other package
       (MAPF, formation, LLM, dynamic obstacles, BT state).
   * - ``iros_llm_swarm_mapf``
     - C++17
     - PBS Multi-Agent Path Finding planner with N-connected Euclidean
       A\* and capsule-based conflict detection.
   * - ``iros_llm_swarm_mapf_lns``
     - C++17
     - LNS2 Multi-Agent Path Finding planner — drop-in alternative to
       PBS, sharing the same action interface.
   * - ``iros_llm_swarm_robot``
     - C++17
     - Per-robot dual-mode followers shared by both planners:
       ``pbs_motion_controller`` (PBS) and ``lns_motion_controller``
       (LNS2).
   * - ``iros_llm_swarm_formation``
     - Python
     - Centralized leader-follower formation manager and monitor.
   * - ``iros_llm_swarm_bt``
     - C++17
     - BehaviorTree.CPP v3 action nodes wrapping the MAPF and formation
       interfaces, plus the BT runner and ``LlmCommandReceiver``.
   * - ``iros_llm_swarm_local_nav``
     - Python
     - Per-robot Nav2 stack spawner (controller, behavior, lifecycle).
   * - ``iros_llm_swarm_costmap_plugins``
     - C++17
     - ``ResettingObstacleLayer`` Nav2 costmap plugin and
       ``zone_map_server`` (the project's map server).
   * - ``iros_llm_swarm_obstacles``
     - C++17
     - ``dynamic_obstacle_manager`` — overlays runtime-added circles,
       rectangles, and stateful doors onto the static map.
   * - ``iros_llm_orchestrator``
     - Python
     - Three-channel LLM advisory layer (reactive ``/llm/decision``,
       proactive ``passive_observer``, operator ``chat_server`` with
       MCP-grounded context and a remediation loop).
   * - ``iros_llm_rviz_panel``
     - C++ / Qt
     - RViz2 operator panel — chat, MAPF status, BT state, events log,
       STOP-ALL.
   * - ``iros_llm_rviz_tool``
     - C++ / Qt
     - RViz2 tool plugins — click-to-send goals, place obstacles, and
       toggle doors.
   * - ``iros_llm_swarm_simulation_lite``
     - Python
     - 2D Stage simulator launch, world files, and robot models.
   * - ``iros_llm_swarm_simulation``
     - Python
     - Gazebo Harmonic 3D simulation (preview, not yet stable).
   * - ``iros_llm_swarm_bringup``
     - Python
     - Top-level launch orchestration for the full stack.
   * - ``iros_llm_swarm_docs``
     - —
     - This package: rosdoc2 aggregator that produces the unified site.

Per-package roles
-----------------

iros_llm_swarm_interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~

Type-only package. Defines every custom ``msg``, ``srv``, and ``action``
shared across the swarm:

* **Actions:** ``SetGoals`` (the fleet-level MAPF action), ``LlmDecision``
  (channel 1), ``LlmCommand`` (channels 2 + RViz STOP-ALL),
  ``LlmChat`` (channel 3 streaming), ``LlmExecutePlan`` (operator-
  confirmed plan replay).
* **Messages:** ``MAPFPlan`` / ``MAPFStep`` (LNS2 follower contract),
  ``FollowerStatus``, ``BTState``, ``LlmEvent``, ``FormationConfig`` /
  ``FormationsConfig`` / ``FormationStatus`` / ``FormationsStatus``,
  obstacle messages (``CircleObstacle``, ``RectangleObstacle``,
  ``Door``).
* **Services:** the ``/formation/*`` CRUD set, plus the obstacle and
  door services consumed by ``iros_llm_swarm_obstacles``
  (``AddCircle``, ``AddRectangle``, ``AddDoor``, ``RemoveObstacle``,
  ``ListObstacles``, ``OpenDoor``, ``CloseDoor``).

**Build this package first** when adding new types — every other
package depends on its generated headers and Python modules.

iros_llm_swarm_mapf
~~~~~~~~~~~~~~~~~~~

Reference MAPF planner. Implements PBS (Priority-Based Search) on top of
N-connected Euclidean A*, with backward-Dijkstra heuristics, gradient
inflation (hard footprint plus soft penalty zone), and capsule-based
conflict detection. Hosts the ``/swarm/set_goals`` action server and
publishes time-indexed paths to ``/robot_*/mapf_path``. Includes a
schedule monitor that triggers replanning when robots deviate beyond a
configurable threshold.

Entry point: ``src/mapf_planner_node.cpp`` (executable
``mapf_planner_node``).

iros_llm_swarm_mapf_lns
~~~~~~~~~~~~~~~~~~~~~~~

The actively maintained MAPF stack. ``mapf_lns2_node`` shares the
fleet-level ``/swarm/set_goals`` action with the legacy PBS planner
but uses Large Neighborhood Search (LNS2) internally — ALNS-style
destroy/repair operators, soft-constraint A\* (collision penalties
rather than hard blocks), and warm-start from the previous plan on
replan. The follower contract is ``MAPFPlan`` on
``/<ns>/mapf_plan``, with explicit ``hold_sec`` per step and a
``plan_id`` echo via ``FollowerStatus``. The corresponding follower
executable lives in ``iros_llm_swarm_robot`` (``lns_motion_controller``).

Entry points: ``src/mapf_lns2_node.cpp`` and supporting
``plan_publisher`` / ``robot_lifecycle`` translation units. Launch
helper: ``launch/mapf_lns2.launch.py`` (used by ``swarm_lns*``
bringup).

iros_llm_swarm_robot
~~~~~~~~~~~~~~~~~~~~

Per-robot dual-mode follower **shared by both planners**. Two
executables are built from this single package:

* ``pbs_motion_controller`` (``src/pbs_motion_controller.cpp``) — paired
  with the PBS planner. Consumes ``nav_msgs/Path`` on
  ``/<ns>/mapf_path`` and dispatches it to Nav2's ``follow_path``
  action in chunks bounded by ``schedule_tolerance_sec``, so the robot
  stays close to the PBS schedule. Does **not** publish
  ``FollowerStatus`` — the PBS planner detects deviation by polling
  odometry.
* ``lns_motion_controller`` (``src/lns_motion_controller.cpp``) — paired
  with the LNS2 planner. Consumes ``MAPFPlan`` on ``/<ns>/mapf_plan``
  with explicit ``hold_sec`` per step, executes hold-then-go, and
  publishes ``FollowerStatus`` with state, current step, ``plan_id``,
  hold countdown, and Nav2 failure count.

Both executables share the **FORMATION_FOLLOWER** code path: a PD
controller tracks the leader's body-frame offset directly via
``cmd_vel``. Mode switching is event-driven by ``/formations/config``;
the node does not restart. Launch helper:
``motion_controllers.launch.py`` selects the executable via the
``controller_type`` argument.

iros_llm_swarm_formation
~~~~~~~~~~~~~~~~~~~~~~~~

Centralized formation layer. ``formation_manager_node`` exposes the
``/formation/*`` CRUD services and publishes the latched
``/formations/config`` topic. ``formation_monitor_node`` watches odometry
of leaders and followers, computes per-follower position errors, and
publishes a state machine
(``INACTIVE → FORMING → STABLE ↔ DEGRADED → BROKEN``) on
``/formations/status``.

iros_llm_swarm_bt
~~~~~~~~~~~~~~~~~

BehaviorTree.CPP v3 action nodes that wrap the swarm's ROS 2 interfaces:

* ``MapfPlan`` — async client for ``/swarm/set_goals`` with optional LLM
  callback hook (``/llm/decision``).
* ``SetFormation`` / ``DisableFormation`` — formation service wrappers.
* ``CheckMode`` — blackboard mode-transition checks.

The package also ships the ``test_bt_runner`` executable (loads a tree
and publishes ``/bt/state``), the ``fleet_cmd`` CLI helper
(``simple|stress|unreachable|idle`` scenarios), and an
``LlmCommandReceiver`` that applies channel-2 commands onto the BT
blackboard. Reference tree:
``behavior_trees/swarm_navigate_to_pose.xml``.

iros_llm_swarm_local_nav
~~~~~~~~~~~~~~~~~~~~~~~~

Spawns a Nav2 stack per robot:

* ``controller_server`` for local trajectory tracking.
* ``behavior_server`` for recovery actions.
* ``lifecycle_manager`` for the above.
* A static ``map → robot_N/odom`` transform (AMCL is intentionally
  disabled — at 20 robots, cross-robot LiDAR interference makes
  localization drift).

A single shared ``zone_map_server`` (from
``iros_llm_swarm_costmap_plugins``) serves all robots. Per-robot
launches are staggered (~0.3 s apart) to avoid startup race conditions,
and Nav2 parameter files are routed through ``ReplaceString`` and
``RewrittenYaml`` so the same templates work for every namespace.

iros_llm_swarm_costmap_plugins
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two C++ artefacts:

* **``ResettingObstacleLayer``** — a Nav2 costmap layer plugin
  registered through ``costmap_plugins.xml``. A drop-in replacement for
  ``nav2_costmap_2d::ObstacleLayer`` that resets its internal grid every
  ``updateBounds()`` call before re-marking. This fixes ghost-obstacle
  trails caused by laser cells that fall *between* rays and are never
  explicitly cleared via raytracing.
* **``zone_map_server``** — a small subclass of
  ``nav2_map_server::MapServer`` used in place of the stock
  ``map_server`` in ``robot_local_nav.launch.py``. It is the actual map
  publisher in the per-robot Nav2 launch.

iros_llm_swarm_obstacles
~~~~~~~~~~~~~~~~~~~~~~~~

Dynamic-obstacle layer for runtime scenario edits. The single node
``dynamic_obstacle_manager`` subscribes to ``/raw_map`` (the static
occupancy grid produced by ``map_server`` / ``zone_map_server``), keeps
an in-memory list of circles, rectangles, and stateful doors, and
republishes the merged grid on ``/map`` with ``TRANSIENT_LOCAL`` QoS.
At startup the obstacle list can be hydrated from a YAML scenario file
via the ``scenario_file`` and ``scenario_name`` parameters; at runtime
it can be edited through eight services:

* ``/obstacles/add_circle`` / ``/obstacles/add_rectangle`` /
  ``/obstacles/add_door`` — add by id with geometry.
* ``/obstacles/remove`` — remove by id.
* ``/obstacles/list`` — read all current obstacles.
* ``/doors/open`` / ``/doors/close`` — toggle door state. Open doors do
  not block cells; closed doors do.

A debug ``MarkerArray`` is published on ``/obstacles/markers`` for RViz
visualisation. The package is **not yet wired into the bringup
launches**; activate it manually or compose it into a custom launch
file. ``iros_llm_rviz_tool`` already exposes click-to-edit tools that
talk to these services.

iros_llm_orchestrator
~~~~~~~~~~~~~~~~~~~~~

The LLM advisory layer. Five Python nodes:

* ``decision_server`` — channel 1, ``/llm/decision`` action server. BT
  nodes call it on WARN/ERROR; returns ``wait | abort | replan``.
* ``passive_observer`` — channel 2, watches ``/bt/state`` and pushes
  ``LlmCommand`` on ``/llm/command`` when intervention is warranted
  (cooldown-gated, opt-in via ``enabled:=true``).
* ``chat_server`` — channel 3, ``/llm/chat`` action server. Streams
  replies, parses JSON plans, runs a remediation loop (default 2
  retries) on execution failure. Supports formation auto-staging and an
  MCP read-only context provider.
* ``execute_server`` — replays a plan JSON via ``/llm/execute_plan`` so
  the operator can preview-then-confirm in RViz.
* ``user_chat`` — CLI chatbot for offline testing of the channel-3
  pipeline.

Backends are selected by the ``llm_backend`` argument
(``mock | ollama | http | local``); the HTTP backend is OpenAI-
compatible. All decisions and commands are appended to JSONL files
under ``~/.ros/llm_decisions/`` and ``~/.ros/llm_commands/`` for SFT
dataset collection. Launch helper: ``launch/orchestrator.launch.py``.

iros_llm_rviz_panel
~~~~~~~~~~~~~~~~~~~

Single RViz2 ``Panel`` plugin (``iros_llm_rviz_panel/LLM-Panel``) with
five tabs:

* **Chat** — drives ``/llm/chat`` and previews the parsed plan as a
  tree; the **Execute** button confirms the plan via
  ``/llm/execute_plan``.
* **MAPF** — per-robot arrival progress, active counts, sparklines from
  the ``action_summary`` tail of ``/bt/state``.
* **Events** — live tail of ``/llm/events`` with channel filters.
* **BT** — current ``/bt/state`` row by row, plus a transition log.
* **Info** — async-refreshed parameter snapshot.

A sticky status bar at the top exposes the **STOP ALL** button, which
publishes ``LlmCommand{mode=idle}`` directly to ``/llm/command`` with
no LLM in the loop. All ROS callbacks marshal into Qt slots via
``Qt::QueuedConnection`` — never touch widgets from an executor thread.
Goal markers are published on ``/llm_panel/markers`` at 30 Hz when
``mode=mapf``.

iros_llm_rviz_tool
~~~~~~~~~~~~~~~~~~

Three RViz2 ``Tool`` plugins, all click-to-command (no LLM in the loop):

* ``SendLlmGoalTool`` (shortcut ``g``) — pick a robot group, click on
  the map, and N goals are spread around the click point and sent to
  ``/llm/command`` as a ``mode=mapf`` goal.
* ``PlaceObstacleTool`` (shortcut ``b``) — left-click adds a circle,
  rectangle, or door (shape/size from the Tool Properties panel);
  right-click removes the nearest obstacle.
* ``DoorTool`` (shortcut ``d``) — left-click opens, right-click closes
  the door whose ID is set in the Tool Properties panel.

iros_llm_swarm_simulation_lite
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The supported 2D simulation backend. Wraps ``stage_ros2`` with project
world files (``cave``, ``large_cave``, ``warehouse``, ``warehouse_four``,
``amongus``), ``.pgm`` / ``.yaml`` map files, and the differential-drive
robot model (``warehouse_robot.inc``). Stage runs with a unified TF tree
and namespace-enforced prefixes so all robots coexist in the same TF
graph. The launch ``scenario`` argument is mapped to the corresponding
world file via ``scenario/common_scenarios.yaml``.

iros_llm_swarm_simulation
~~~~~~~~~~~~~~~~~~~~~~~~~

Gazebo Harmonic (full 3D) simulation. Contains URDF / Xacro robot
descriptions and an SDF world. **Not yet recommended for daily use** —
it can spawn 20 robots, but Nav2 stability at this density is not
guaranteed. Kept in tree as the target for future work; use
``_lite`` until further notice.

iros_llm_swarm_bringup
~~~~~~~~~~~~~~~~~~~~~~

Top-level launch orchestration only — no node code lives here. Six
launch files cover the supported scenarios:

* ``swarm_warehouse.launch.py`` — base scenario without the MAPF
  layer (simulator + Nav2 + RViz only).
* ``swarm_mapf.launch.py`` — PBS planner stack.
* ``swarm_lns.launch.py`` — LNS2 planner stack (delegates the planner
  to ``mapf_lns2.launch.py`` from ``iros_llm_swarm_mapf_lns``).
* ``swarm_mapf_formation.launch.py`` — PBS + formation layer.
* ``swarm_lns_formation.launch.py`` — LNS2 + formation layer.
* ``swarm_full_demo.launch.py`` — full stack: simulator + Nav2 +
  selectable planner (``planner:=pbs|lns``) + optional formation +
  BT runner + LLM orchestrator (channel 1 always on; channel 2
  opt-in via ``enable_passive_observer:=true``) + optional rosbridge.

Each one composes the simulator, Nav2, planner, per-robot followers,
and (optionally) the formation manager / monitor with the staggered
timing described in :doc:`architecture`.

iros_llm_swarm_docs
~~~~~~~~~~~~~~~~~~~

The aggregator package you are reading. Builds a unified Sphinx site
that links to each package's own rosdoc2-generated documentation. Its
own source lives under ``doc/``; per-package docs are produced by
running ``rosdoc2 build`` from the workspace root.
