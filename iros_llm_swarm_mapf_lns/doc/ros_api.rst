ROS API
=======

This page documents the **ROS contract** of both executables built by
the package: ``mapf_lns2_node`` (fleet-level planner) and
``path_follower_node`` (per-robot follower).

The action and message types referenced here are owned by
``iros_llm_swarm_interfaces``; see its docs for field-level semantics.

.. contents::
   :local:
   :depth: 2

mapf_lns2_node
--------------

Single instance per fleet. Subscribes to the shared occupancy grid and
to every robot's odometry / footprint / follower status; hosts the
``/swarm/set_goals`` action server and publishes per-robot
``MAPFPlan`` messages.

Action server
~~~~~~~~~~~~~

Action: ``/swarm/set_goals`` of type
``iros_llm_swarm_interfaces/action/SetGoals``.

Long-lived semantics — see the action description in
``iros_llm_swarm_interfaces``. The lifecycle phases reported in
``feedback.status`` are ``"validating"``, ``"planning"``,
``"executing"``, ``"replanning"``, ``"failed"``.

Failure path: planning errors complete the action with
``error_code = PBS_FAILED`` (the constant is reused as a generic
"planner failed" by the LNS backend) plus a populated
``feedback.warning`` and a ``message`` describing the cause.
Cancellation completes with ``error_code = CANCELLED`` after broadcasting
zero ``Twist`` and an empty ``MAPFPlan`` to every active robot.

Topics
~~~~~~

Subscribed:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - QoS / notes
   * - ``/map`` (configurable)
     - ``nav_msgs/OccupancyGrid``
     - ``KeepLast(1) + transient_local + reliable``. The topic name is
       parameterized via ``map_topic``.
   * - ``/robot_<id>/odom``
     - ``nav_msgs/Odometry``
     - ``depth=10``. Polled to derive each agent's current cell at
       plan / replan time.
   * - ``/robot_<id>/local_costmap/published_footprint``
     - ``geometry_msgs/PolygonStamped``
     - ``depth=1``. Per-robot footprint radius is computed as the
       maximum vertex distance from the polygon's centroid. Falls
       back to ``default_robot_radius`` until first message arrives.
   * - ``/robot_<id>/follower_status``
     - ``iros_llm_swarm_interfaces/FollowerStatus``
     - ``depth=10``. Used by the schedule monitor to detect
       PLAN_COMPLETE, FAILED, and stale-vs-current ``plan_id``.

Published:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - QoS / notes
   * - ``/robot_<id>/mapf_plan``
     - ``iros_llm_swarm_interfaces/MAPFPlan``
     - Per-robot plan. ``plan_id`` increments on every publish; an
       empty ``steps`` list cancels the follower.
   * - ``/robot_<id>/cmd_vel``
     - ``geometry_msgs/Twist``
     - Used only on cancel for emergency stop (broadcast zero
       velocity).
   * - ``/mapf_grid`` (optional)
     - ``nav_msgs/OccupancyGrid``
     - Debug visualization of the planner's downsampled grid.
       Enabled by ``publish_debug_grid``;
       ``KeepLast(1) + transient_local + reliable``.

Parameters
~~~~~~~~~~

All parameters are read once at startup; the production defaults live
in ``config/mapf_lns2.yaml`` and are loaded by ``mapf_lns2.launch.py``.

Fleet and physics:

.. list-table::
   :header-rows: 1
   :widths: 32 14 54

   * - Parameter
     - Default
     - Meaning
   * - ``num_robots``
     - 20
     - Number of robots the planner controls. External robot IDs are
       ``[0, num_robots)``.
   * - ``time_step_sec``
     - 0.5
     - Wall-clock seconds per planning timestep. Must match what the
       follower expects (see ``schedule_tolerance_sec`` below).
   * - ``map_topic``
     - ``/map``
     - Topic name for the static occupancy grid.
   * - ``grid_resolution``
     - 0.2
     - Meters per cell on the planner grid (downsampled from the map).
   * - ``default_robot_radius``
     - 0.22
     - Fallback footprint radius (m) until ``published_footprint``
       arrives.
   * - ``inflation_radius``
     - 0.0
     - Extra radius added to each footprint on top of the measured
       value.
   * - ``max_speed``
     - 0.5
     - Informational; the LNS solver does not use it directly.
   * - ``goal_reached_m``
     - 0.5
     - Arrival tolerance for the "all done" check.

LNS2 core:

.. list-table::
   :header-rows: 1
   :widths: 32 14 54

   * - Parameter
     - Default
     - Meaning
   * - ``collision_penalty``
     - 10000
     - Soft A* penalty per collision unit. Must satisfy
       ``penalty > horizon × step_cost``.
   * - ``horizon_steps``
     - 300
     - Hard upper bound on planning timesteps.
   * - ``max_astar_expansions``
     - 200000
     - Per-A*-call expansion cap during the **initial** mission solve.
       Values up to 900000 are used in production for crowded fleets.
   * - ``neighborhood_size``
     - 8
     - Destroy-set size per LNS iteration (~40% of fleet at 20
       robots).
   * - ``time_budget_ms``
     - 500
     - Wall-clock budget for the repair loop on the initial solve
       (production: 10000).
   * - ``plateau_limit``
     - 200
     - Iterations with no improvement before the loop terminates
       (production: 1500).
   * - ``segment_size``
     - 50
     - Number of iterations between ALNS weight updates.
   * - ``alns_reaction``
     - 0.1
     - EMA reaction coefficient for ALNS weight updates.
   * - ``diagonal_moves``
     - ``false``
     - 4-connected by default; diagonals are MAPF-unsafe in dense
       fleets.
   * - ``initial_build_time_budget_ms``
     - 6000
     - Wall-clock cap for the prioritized greedy construction phase
       during ``execute_goal``. 0 = no limit.

Replan and warm start:

.. list-table::
   :header-rows: 1
   :widths: 38 14 48

   * - Parameter
     - Default
     - Meaning
   * - ``replan_check_hz``
     - 2.0
     - Schedule monitor tick rate.
   * - ``replan_cooldown_sec``
     - 2.0
     - Minimum gap between consecutive replans.
   * - ``replan_time_budget_ms``
     - 1500
     - Wall-clock budget for both warm and cold replans.
   * - ``replan_segment_size``
     - 8
     - ALNS segment size during replans (smaller than the initial
       solve so weights adapt quickly to the smaller iteration count).
   * - ``replan_max_astar_expansions``
     - 200000
     - A* cap for cold replans. Lower than ``max_astar_expansions`` to
       keep cold-replan latency bounded.
   * - ``warm_start_enabled``
     - ``true``
     - When ``false``, every replan is a cold start.
   * - ``warm_patch_radius_cells``
     - 10
     - Max L∞ distance (cells) between expected and actual cell that
       still triggers a splice instead of a stub.
   * - ``warm_splice_max_expansions``
     - 2000
     - BFS budget when splicing actual → tail. ``0`` disables splice.
   * - ``warm_max_collisions_per_agent``
     - 4
     - Per-agent seed collision threshold; exceeding it falls back to
       cold start.

Stall detection and lives:

.. list-table::
   :header-rows: 1
   :widths: 38 14 48

   * - Parameter
     - Default
     - Meaning
   * - ``stall_timeout_sec``
     - 4.0
     - Time without meaningful movement before a robot is considered
       stalled.
   * - ``stall_move_thresh_m``
     - 0.15
     - Movement below this is "not moving".
   * - ``progress_log_interval_sec``
     - 3.0
     - Periodic summary cadence; 0 disables.
   * - ``empty_fails_before_unplanable``
     - 3
     - Consecutive empty-path solves before benching the robot
       (production override: 6).
   * - ``stall_iters_before_unplanable``
     - 10
     - Consecutive stalled monitor ticks before benching
       (production override: 13).
   * - ``max_lives``
     - 3
     - Total bench events a robot can survive (production: 6).
       ``max_lives = 1`` recovers permanent-on-first-fail behaviour.
   * - ``unplanable_retry_delay_sec``
     - 8.0
     - How long a benched robot stays out before being put back in
       the planning pool (production: 5.0).
   * - ``publish_debug_grid``
     - ``true``
     - Publish ``/mapf_grid`` for RViz inspection.

Internal modules
~~~~~~~~~~~~~~~~

These currently live as separate translation units alongside
``mapf_lns2_node.cpp`` and will be the target of the upcoming refactor:

* ``node_utils.{hpp,cpp}`` — pure helpers: world↔cell conversion,
  grid path → ``MAPFStep[]`` compression, blocking arrived / skipped
  robot cells on the planner grid.
* ``plan_publisher.{hpp,cpp}`` — owns per-robot ``MAPFPlan`` and
  ``cmd_vel`` publishers and the monotonic ``plan_id`` counter.
  Also implements ``publish_stop_all`` (used by cancel) and
  ``publish_cancel_for(rid)`` (used to cancel a single benched
  robot).
* ``robot_lifecycle.{hpp,cpp}`` — bundles per-robot mission-scoped
  state (arrival latch, last-movement baseline, three failure
  counters, lives remaining, unplanable membership and bench
  timestamp). Exposes ``reset_for_new_mission``, ``bench``,
  ``revive``, and ``mark_movement``.

path_follower_node
------------------

Spawned per robot by the launch file. Each instance is parameterized
by ``robot_id``; its namespace is ``robot_<id>``. The follower has two
modes selected at runtime by ``/formations/config``:

* **AUTONOMOUS** (default) — receives ``MAPFPlan`` and dispatches it
  segment-by-segment to Nav2's ``follow_path``.
* **FORMATION_FOLLOWER** — runs a PD controller against the leader's
  ``odom`` and the offset declared in ``/formations/config``,
  publishing directly to ``cmd_vel``.

State machine (AUTONOMOUS mode)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    IDLE  ── new plan ──▶  NAVIGATING(seg)  ── seg done + hold > 0 ──▶  HOLDING
                                  │                                          │
                          seg done + last step                        hold expires
                                  │                                          │
                                  ▼                                          ▼
                            PLAN_COMPLETE                          NAVIGATING(next seg)

A *segment* is the longest run of steps from ``current_step_index``
where every step except the final one has ``hold_sec == 0``. The
follower dispatches each segment as a single ``FollowPath`` action
goal — there is no per-waypoint chunking.

A new ``MAPFPlan`` with a different ``plan_id`` cancels any in-flight
``follow_path`` goal and starts at step 0 of the new plan. An empty
``MAPFPlan.steps`` list is treated as a cancel request and transitions
the follower to IDLE.

Topics and actions
~~~~~~~~~~~~~~~~~~

Subscribed:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - Notes
   * - ``/<ns>/mapf_plan``
     - ``iros_llm_swarm_interfaces/MAPFPlan``
     - Always subscribed; only acted on in AUTONOMOUS mode.
   * - ``/<ns>/odom``
     - ``nav_msgs/Odometry``
     -
   * - ``/<leader_ns>/odom``
     - ``nav_msgs/Odometry``
     - FORMATION mode only, dynamically subscribed when this
       follower's leader changes.
   * - ``/formations/config``
     - ``iros_llm_swarm_interfaces/FormationsConfig``
     - ``KeepLast(1) + transient_local + reliable`` (latched).

Published:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - Notes
   * - ``/<ns>/follower_status``
     - ``iros_llm_swarm_interfaces/FollowerStatus``
     - Heartbeat at ``status_pub_hz`` plus immediate publishes on
       state transitions.
   * - ``/<ns>/cmd_vel``
     - ``geometry_msgs/Twist``
     - FORMATION mode only.

Action client:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Action
     - Type
     - Notes
   * - ``/<ns>/follow_path``
     - ``nav2_msgs/action/FollowPath``
     - One in-flight goal at a time; cancelled when a new plan
       arrives or the follower transitions to IDLE.

Parameters
~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 14 56

   * - Parameter
     - Default
     - Meaning
   * - ``robot_id``
     - 0
     - Index used to derive the namespace ``robot_<id>``.
   * - ``path_frame``
     - ``map``
     - Frame the MAPF plan and ``follow_path`` goals are published in.
   * - ``status_pub_hz``
     - 2.0
     - Heartbeat rate for ``follower_status``.
   * - ``schedule_tolerance_sec``
     - 5.0
     - Soft tolerance for arrival timing relative to the planner's
       schedule. Set in the launch file (not a node default).
   * - ``kp``
     - 1.2
     - PD proportional gain (FORMATION mode).
   * - ``kd``
     - 0.3
     - PD derivative gain (FORMATION mode).
   * - ``max_v``
     - 0.5
     - Max linear velocity (m/s) (FORMATION mode).
   * - ``max_omega``
     - 1.0
     - Max angular velocity (rad/s) (FORMATION mode).
   * - ``control_hz``
     - 20.0
     - PD loop rate (FORMATION mode).

Note on motion-controller naming
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The aggregator's architecture page describes a generic dual-mode
"motion controller". For the LNS2 stack, that role is filled by
**this** ``path_follower_node``, not by ``iros_llm_swarm_robot``'s
``motion_controller_node`` — the LNS launch file
(``mapf_lns2.launch.py``) spawns one ``path_follower_node`` per robot.
The two implementations cover the same contract but evolved
independently; the LNS follower will be retained as the supported
follower for this planner.

Launch
------

The package's own launch file brings up just the planner and
followers; the simulator, Nav2, and (optionally) formation layer are
composed by ``iros_llm_swarm_bringup`` (e.g. ``swarm_lns.launch.py``,
``swarm_lns_formation.launch.py``).

.. code-block:: bash

   ros2 launch iros_llm_swarm_mapf_lns mapf_lns2.launch.py
   ros2 launch iros_llm_swarm_mapf_lns mapf_lns2.launch.py num_robots:=10

Launch arguments:

.. list-table::
   :header-rows: 1
   :widths: 22 28 50

   * - Argument
     - Default
     - Description
   * - ``num_robots``
     - 20
     - Number of ``path_follower_node`` instances to spawn.
   * - ``use_sim_time``
     - ``true``
     - Use simulator clock for both planner and followers.
   * - ``params_file``
     - ``config/mapf_lns2.yaml``
     - YAML parameter file for ``mapf_lns2_node``.
   * - ``log_level``
     - ``info``
     - ``rclcpp`` log level (``debug`` / ``info`` / ``warn`` /
       ``error``).
