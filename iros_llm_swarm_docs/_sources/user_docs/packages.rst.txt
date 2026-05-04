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
     - Shared ROS 2 type definitions used by every other package.
   * - ``iros_llm_swarm_mapf``
     - C++17
     - PBS Multi-Agent Path Finding planner with N-connected Euclidean
       A* and capsule-based conflict detection.
   * - ``iros_llm_swarm_mapf_lns``
     - C++17
     - LNS2 Multi-Agent Path Finding planner — drop-in alternative to
       PBS, sharing the same action interface.
   * - ``iros_llm_swarm_robot``
     - C++17
     - Per-robot dual-mode follower paired with the legacy PBS
       planner (``motion_controller_node``). LNS2 uses its own
       follower from ``iros_llm_swarm_mapf_lns``.
   * - ``iros_llm_swarm_formation``
     - Python
     - Centralized leader-follower formation manager and monitor.
   * - ``iros_llm_swarm_bt``
     - C++17
     - BehaviorTree.CPP v3 action nodes wrapping the MAPF and formation
       interfaces.
   * - ``iros_llm_swarm_local_nav``
     - Python
     - Per-robot Nav2 stack spawner (controller, behavior, lifecycle).
   * - ``iros_llm_swarm_costmap_plugins``
     - C++17
     - ``ResettingObstacleLayer`` Nav2 costmap plugin.
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
shared across the swarm: ``SetGoals.action``, ``FormationsConfig.msg``,
``FollowerStatus.msg``, the ``/formation/*`` service definitions, and so
on. **Build this package first** when adding new types — the C++ and
Python packages depend on its generated headers and modules.

iros_llm_swarm_mapf
~~~~~~~~~~~~~~~~~~~

Reference MAPF planner. Implements PBS (Priority-Based Search) on top of
N-connected Euclidean A*, with backward-Dijkstra heuristics, gradient
inflation (hard footprint plus soft penalty zone), and capsule-based
conflict detection. Hosts the ``/swarm/set_goals`` action server and
publishes time-indexed paths to ``/robot_*/mapf_path``. Includes a
schedule monitor that triggers replanning when robots deviate beyond a
configurable threshold.

Entry point: ``src/mapf_planner_node.cpp``.

iros_llm_swarm_mapf_lns
~~~~~~~~~~~~~~~~~~~~~~~

The actively maintained MAPF stack. ``mapf_lns2_node`` shares the
fleet-level ``/swarm/set_goals`` action with the legacy PBS planner
but uses Large Neighborhood Search (LNS2) internally — ALNS-style
destroy/repair operators, soft-constraint A* (collision penalties
rather than hard blocks), and warm-start from the previous plan on
replan. ``path_follower_node`` is the per-robot follower for this
stack; it consumes ``MAPFPlan`` (with explicit ``hold_sec`` and a
``plan_id`` echo via ``FollowerStatus``) and is the LNS-side
counterpart to ``iros_llm_swarm_robot``'s ``motion_controller_node``.

Entry points: ``src/mapf_lns2_node.cpp``, ``src/path_follower_node.cpp``.

iros_llm_swarm_robot
~~~~~~~~~~~~~~~~~~~~

Per-robot dual-mode follower paired with the **legacy PBS planner**
(``iros_llm_swarm_mapf``). In **AUTONOMOUS** mode it consumes
``nav_msgs/Path`` on ``/<ns>/mapf_path`` and dispatches it to Nav2's
``follow_path`` action in chunks bounded by
``schedule_tolerance_sec``, so the robot stays close to the PBS
schedule. In **FORMATION_FOLLOWER** mode a PD controller tracks the
leader's body-frame offset directly via ``cmd_vel``. Mode switching is
event-driven by ``/formations/config``; the node does not restart.

Entry point: ``src/motion_controller_node.cpp``. Spawned by
``swarm_mapf*.launch.py``. Does **not** publish ``FollowerStatus`` —
the PBS planner detects deviation by polling odometry. The LNS2 stack
uses a different follower; see ``iros_llm_swarm_mapf_lns``.

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
  callback hook.
* ``SetFormation`` / ``DisableFormation`` — formation service wrappers.
* ``CheckMode`` — blackboard mode-transition checks.

Reference tree: ``behavior_trees/swarm_navigate_to_pose.xml``. The BT
runner is intended as the integration point for higher-level mission
logic (including future LLM agents).

iros_llm_swarm_local_nav
~~~~~~~~~~~~~~~~~~~~~~~~

Spawns a Nav2 stack per robot:

* ``controller_server`` for local trajectory tracking.
* ``behavior_server`` for recovery actions.
* ``lifecycle_manager`` for the above.
* A static ``map → robot_N/odom`` transform (AMCL is disabled).

A single shared ``map_server`` serves all robots. Per-robot launches are
staggered (~0.3 s apart) to avoid startup race conditions, and Nav2
parameter files are routed through ``ReplaceString`` and
``RewrittenYaml`` so the same templates work for every namespace.

iros_llm_swarm_costmap_plugins
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Custom Nav2 costmap layer: ``ResettingObstacleLayer``. A drop-in
replacement for ``nav2_costmap_2d::ObstacleLayer`` that resets its
internal grid every ``updateBounds()`` call before re-marking. This
fixes ghost-obstacle trails caused by laser cells that fall *between*
rays and are never explicitly cleared via raytracing.

iros_llm_swarm_simulation_lite
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The supported 2D simulation backend. Wraps ``stage_ros2`` with project
world files (``cave``, ``warehouse_2``, ``warehouse_4``, ``large_cave``),
``.pgm`` / ``.yaml`` map files, and the differential-drive robot model
(``warehouse_robot.inc``). Stage runs with a unified TF tree and
namespace-enforced prefixes so all robots coexist in the same TF graph.

iros_llm_swarm_simulation
~~~~~~~~~~~~~~~~~~~~~~~~~

Gazebo Harmonic (full 3D) simulation. Contains URDF / Xacro robot
descriptions and an SDF world. **Not yet recommended for daily use** —
it can spawn 20 robots, but Nav2 stability at this density is not
guaranteed. Kept in tree as the target for future work; use
``_lite`` until further notice.

iros_llm_swarm_bringup
~~~~~~~~~~~~~~~~~~~~~~

Top-level launch orchestration only — no node code lives here. Provides
the four primary launch files:

* ``swarm_mapf.launch.py`` — PBS planner stack.
* ``swarm_lns.launch.py`` — LNS2 planner stack.
* ``swarm_mapf_formation.launch.py`` — PBS + formation layer.
* ``swarm_lns_formation.launch.py`` — LNS2 + formation layer.
* ``swarm_warehouse.launch.py`` — base scenario without the MAPF
  layer (simulator + Nav2 + RViz only).

Each one composes the simulator, Nav2, planner, per-robot followers,
and (optionally) the formation manager / monitor with the staggered
timing described in :doc:`architecture`.

iros_llm_swarm_docs
~~~~~~~~~~~~~~~~~~~

The aggregator package you are reading. Builds a unified Sphinx site
that links to each package's own rosdoc2-generated documentation. Its
own source lives under ``doc/``; per-package docs are produced by
running ``rosdoc2 build`` from the workspace root.
