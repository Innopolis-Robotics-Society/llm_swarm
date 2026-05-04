System Architecture
===================

The stack is organized as a vertical pipeline: the simulator publishes
ground-truth state, the MAPF planner turns fleet-level goals into
time-indexed paths, and per-robot motion controllers turn those paths
into either Nav2 ``follow_path`` actions or direct ``cmd_vel`` commands
when a formation is active.

Layered view
------------

::

    ┌──────────────────────────────────────────────────────────────────┐
    │                    IROS LLM SWARM SIMULATION                     │
    ├──────────────────────────────────────────────────────────────────┤
    │                                                                  │
    │  ┌─────────────┐   ┌──────────────┐   ┌────────────────────────┐ │
    │  │  Stage 2D   │   │     RViz     │   │      CycloneDDS        │ │
    │  │  Simulator  │   │    Viewer    │   │     domain_id = 42     │ │
    │  └──────┬──────┘   └──────────────┘   └────────────────────────┘ │
    │         │ /tf, /robot_*/odom, /robot_*/scan                      │
    │  ┌──────▼──────────────────────────────────────────────────────┐ │
    │  │   MAPF PLANNER  (PBS or LNS2 — selectable at launch)        │ │
    │  │   action: /swarm/set_goals                                  │ │
    │  │   pub:    /robot_*/mapf_path  (nav_msgs/Path)               │ │
    │  │   sub:    /map, /robot_*/odom, /robot_*/follower_status     │ │
    │  └──────┬──────────────────────────────────────────────────────┘ │
    │         │ /robot_*/mapf_path                                     │
    │  ┌──────▼──────────────────────────────────────────────────────┐ │
    │  │   PER-ROBOT FOLLOWER  (one per robot, dual-mode)            │ │
    │  │   PBS stack: motion_controller_node (iros_llm_swarm_robot)  │ │
    │  │   LNS stack: path_follower_node     (iros_llm_swarm_mapf_lns)│ │
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
    │                                                                  │
    │   Behavior Tree (optional front-end) → /swarm/set_goals          │
    │                                       → /formation/* services    │
    └──────────────────────────────────────────────────────────────────┘

Topic graph
-----------

.. list-table::
   :header-rows: 1
   :widths: 25 25 20 30

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
     - PBS follower (``motion_controller_node``)
   * - ``/robot_N/mapf_plan``
     - ``iros_llm_swarm_interfaces/MAPFPlan``
     - LNS2 planner (``iros_llm_swarm_mapf_lns``)
     - LNS2 follower (``path_follower_node``)
   * - ``/robot_N/cmd_vel``
     - ``geometry_msgs/Twist``
     - Per-robot follower (FORMATION mode)
     - Stage simulator
   * - ``/robot_N/follower_status``
     - ``iros_llm_swarm_interfaces/FollowerStatus``
     - LNS2 follower (``path_follower_node``) — *PBS follower does
       not publish this*
     - LNS2 planner
   * - ``/map``
     - ``nav_msgs/OccupancyGrid``
     - ``map_server``
     - MAPF planner, Nav2 costmaps
   * - ``/formations/config``
     - ``FormationsConfig``
     - ``formation_manager``
     - Per-robot followers (TRANSIENT_LOCAL, latched)
   * - ``/formations/status``
     - ``FormationsStatus``
     - ``formation_monitor``
     - External clients

Actions
-------

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Action
     - Type
     - Server
     - Clients
   * - ``/swarm/set_goals``
     - ``iros_llm_swarm_interfaces/action/SetGoals``
     - ``mapf_planner_node`` *or* ``mapf_lns2_node``
     - BT, LLM agent, test scripts
   * - ``/robot_N/follow_path``
     - ``nav2_msgs/action/FollowPath``
     - Nav2 ``controller_server``
     - Per-robot follower (AUTONOMOUS mode)

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

Each robot runs one dual-mode follower. Which executable plays this
role depends on which planner stack is active:

* **PBS stack** uses ``motion_controller_node`` from
  ``iros_llm_swarm_robot``. Consumes ``nav_msgs/Path`` on
  ``/<ns>/mapf_path`` (each pose carries a scheduled arrival time);
  chunks the path at hold points to keep the robot inside
  ``schedule_tolerance_sec`` of the PBS schedule. Does **not** publish
  ``FollowerStatus`` — the PBS planner detects deviation by polling
  odometry.
* **LNS2 stack** uses ``path_follower_node`` from
  ``iros_llm_swarm_mapf_lns``. Consumes ``MAPFPlan`` on
  ``/<ns>/mapf_plan`` (explicit ``MAPFStep`` entries with
  ``hold_sec`` and a ``plan_id`` token); publishes ``FollowerStatus``
  with state, current step, ``plan_id``, hold countdown, and Nav2
  failure count so the planner can distinguish frozen from holding
  followers.

Both implementations behave the same in **FORMATION_FOLLOWER** mode:
a PD controller tracks ``leader_pos + R(leader_yaw) · offset`` in
world coordinates and publishes directly to ``cmd_vel``. Mode
switching is event-driven by ``/formations/config``; the node does not
restart.

Startup sequence
----------------

The bringup launch files apply staggered timers so DDS discovery and
costmap initialization can settle before the next layer comes online.
Approximate timing for ``swarm_mapf.launch.py``:

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Time
     - Component
   * - 0 s
     - Stage simulator + RViz
   * - 1 s
     - Nav2 (shared ``map_server`` + per-robot controller / behavior)
   * - 10 s
     - MAPF planner node (waits for ``/map``)
   * - 12 s
     - Per-robot followers ×20 (``motion_controller_node`` for PBS,
       ``path_follower_node`` for LNS2)

Formation launch files add ``formation_manager`` and
``formation_monitor`` at the same step as the followers.

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
