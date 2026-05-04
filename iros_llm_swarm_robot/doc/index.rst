iros_llm_swarm_robot
====================

Per-robot dual-mode motion controller for the **PBS** MAPF stack
(``iros_llm_swarm_mapf``). One executable, ``motion_controller_node``,
spawned once per robot by the launch file. It receives the per-robot
schedule from the planner, drives Nav2's ``follow_path`` action while
respecting the schedule's timing tolerance, and switches to a PD
formation-follower mode when ``/formations/config`` activates a
formation containing this robot.

.. note::

   **Legacy pairing.** This controller is the follower paired with the
   legacy PBS planner. The bringup launches that use it are
   ``swarm_mapf.launch.py`` and ``swarm_mapf_formation.launch.py``.

   The newer LNS2 stack uses a different follower —
   ``path_follower_node`` from ``iros_llm_swarm_mapf_lns`` — and a
   different per-robot input contract (``MAPFPlan`` with explicit
   ``hold_sec`` per step + ``plan_id`` echo via ``FollowerStatus``).
   The two followers cover overlapping responsibilities but are not
   interchangeable: ``motion_controller_node`` consumes
   ``nav_msgs/Path`` and does not publish ``FollowerStatus``.

.. contents::
   :local:
   :depth: 2

Role in the stack
-----------------

For each robot ``id``, one ``motion_controller_node`` runs in the
``robot_<id>`` namespace and:

* Receives a ``nav_msgs/Path`` on ``/<ns>/mapf_path`` from
  ``mapf_planner_node``. Each ``PoseStamped`` carries a *scheduled*
  arrival time in ``header.stamp``.
* Drives Nav2's ``/<ns>/follow_path`` action chunk-by-chunk so the
  robot does not run ahead of the PBS schedule by more than
  ``schedule_tolerance_sec``.
* Listens to ``/formations/config`` (latched). If a formation is
  active and lists this robot as a follower, transitions to the
  FORMATION_FOLLOWER mode and runs a PD controller that tracks the
  leader's body-frame offset, publishing ``cmd_vel`` directly.
* Switches back to AUTONOMOUS when the formation is dissolved or this
  robot is removed from it.

The node never restarts; mode switches happen at runtime by event.

Modes
-----

The active mode is one of:

* **AUTONOMOUS** (default) — consume ``mapf_path``, dispatch to
  Nav2's ``follow_path``.
* **FORMATION_FOLLOWER** — track the leader's body-frame offset via a
  PD controller publishing ``cmd_vel``.

Mode transitions are driven by ``/formations/config``:

* A ``FormationConfig`` arrives with ``active = true`` and this
  robot's namespace in ``follower_ns`` → switch to FORMATION_FOLLOWER
  with the listed leader and offset. Cancel any in-flight Nav2 goal
  before subscribing to the leader's odom and starting the PD timer.
* The same formation arrives with ``active = false``, this robot is
  removed from ``follower_ns``, or no formation snapshot lists this
  robot at all → switch back to AUTONOMOUS. Stop the PD timer,
  publish a zero ``Twist``, drop the leader subscription.

AUTONOMOUS mode: schedule-aware path execution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ``mapf_path`` is consumed in *chunks*. Starting from the current
waypoint index, the controller walks the pose list looking for the
first pose whose ``header.stamp`` is more than
``schedule_tolerance_sec`` in the future. That pose is the next
*hold point*; everything from the current index up to and including
that pose is sent to Nav2 as a single ``follow_path`` goal. When Nav2
reports the goal complete, the controller waits the residual time
(if any) until the hold point's scheduled arrival time, then dispatches
the next chunk. The cycle ends when the path's last pose has been
sent.

An empty ``mapf_path`` is a **cancel** signal: any in-flight Nav2 goal
is cancelled and the active path is dropped. The PBS planner uses
this to halt deviated robots before replanning.

If no own-odometry has been received yet when a path arrives, the
first chunk is deferred via a 100 ms wall timer that polls until odom
is available.

FORMATION_FOLLOWER mode: PD controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Target position (world frame) is computed from the leader's pose and
the assigned body-frame offset:

.. code-block:: text

   T = leader_pos + R(leader_yaw) · offset

with ``offset`` taken from ``FormationConfig.offsets[my_slot]`` (``+x``
forward, ``+y`` left). The position error is fed through a PD
controller in world coordinates, then projected into the robot's body
frame to produce ``cmd_vel``:

.. code-block:: text

   ex = T.x - own.x
   ey = T.y - own.y
   vx_w = kp·ex + kd·(ex - prev_ex)
   vy_w = kp·ey + kd·(ey - prev_ey)
   v     = clamp( cos(own_yaw)·vx_w + sin(own_yaw)·vy_w,  ±max_v)
   omega = clamp(-sin(own_yaw)·vx_w + cos(own_yaw)·vy_w,  ±max_omega)

The PD timer fires at ``control_hz``. If either own-odom or
leader-odom is missing in a tick, the step is skipped (no ``cmd_vel``
is published).

Re-subscription to a different leader's odom happens implicitly when
the formation snapshot reports a new ``leader_ns`` for this robot.

Topics, action client, services
-------------------------------

Subscribed:

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Topic
     - Type
     - Notes
   * - ``/<ns>/mapf_path``
     - ``nav_msgs/Path``
     - Always subscribed; only acted on in AUTONOMOUS mode. Empty list
       = cancel signal.
   * - ``/<ns>/odom``
     - ``nav_msgs/Odometry``
     - Always.
   * - ``/<leader_ns>/odom``
     - ``nav_msgs/Odometry``
     - FORMATION_FOLLOWER only; subscription created when entering
       formation, replaced when the leader changes, dropped on exit.
   * - ``/formations/config``
     - ``iros_llm_swarm_interfaces/FormationsConfig``
     - ``KeepLast(1) + transient_local + reliable`` (latched).

Published:

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Topic
     - Type
     - Notes
   * - ``/<ns>/cmd_vel``
     - ``geometry_msgs/Twist``
     - FORMATION_FOLLOWER mode (PD output) and on AUTONOMOUS entry
       (single zero-Twist to stop drift).

Action client:

.. list-table::
   :header-rows: 1
   :widths: 35 30 35

   * - Action
     - Type
     - Notes
   * - ``/<ns>/follow_path``
     - ``nav2_msgs/action/FollowPath``
     - One in-flight goal at a time. Cancelled on path replacement,
       on empty-path receipt, and on entering FORMATION mode.
       Goal payload uses ``controller_id = "FollowPath"`` and
       ``goal_checker_id = "goal_checker"``.

.. note::

   This node does **not** publish
   ``/<ns>/follower_status``. The PBS planner detects schedule
   deviation by polling odometry directly. The LNS2 follower in
   ``iros_llm_swarm_mapf_lns`` does publish ``FollowerStatus`` —
   keep this in mind when reading code that expects the status
   contract.

Parameters
----------

Defaults below are the node's compile-time defaults. The values used
in the supported launch file come from ``config/motion_controller.yaml``
and override several of these — both columns are listed.

.. list-table::
   :header-rows: 1
   :widths: 30 16 16 38

   * - Parameter
     - Node default
     - YAML value
     - Meaning
   * - ``robot_id``
     - 0
     - per-robot
     - Index used to derive ``robot_<id>``. Set per spawn by
       ``motion_controllers.launch.py``.
   * - ``path_frame``
     - ``"map"``
     - ``"map"``
     - Frame for chunks sent to Nav2.
   * - ``schedule_tolerance_sec``
     - 0.5
     - 0.3
     - Max look-ahead in time at which a pose is still considered
       "in the same chunk". Larger ⇒ longer Nav2 chunks, less hold
       overhead, looser PBS schedule compliance.
   * - ``kp``
     - 1.2
     - 2.5
     - PD proportional gain (FORMATION mode).
   * - ``kd``
     - 0.3
     - 0.8
     - PD derivative gain (FORMATION mode).
   * - ``max_v``
     - 0.5
     - 0.8
     - Max linear velocity (m/s) (FORMATION mode).
   * - ``max_omega``
     - 1.0
     - 1.8
     - Max angular velocity (rad/s) (FORMATION mode).
   * - ``control_hz``
     - 20.0
     - 20.0
     - PD loop rate (FORMATION mode).

Internal cancellation invariants
--------------------------------

``cancel_nav2()`` is the single chokepoint that brings the node back
to a clean state. It cancels any pending hold timer, the
deferred-start timer (used when entering AUTONOMOUS before odom has
arrived), the in-flight Nav2 goal (if any), drops the active path,
and resets the waypoint index. It is called on every mode transition
and on every fresh ``mapf_path`` arrival.

Build and launch
----------------

The package builds one executable. ``iros_llm_swarm_interfaces`` must
be built first — its message types are required headers::

    colcon build --packages-select iros_llm_swarm_interfaces
    colcon build --packages-select iros_llm_swarm_robot

The package's own launch file spawns one controller per robot::

    ros2 launch iros_llm_swarm_robot motion_controllers.launch.py
    ros2 launch iros_llm_swarm_robot motion_controllers.launch.py num_robots:=10

Arguments:

.. list-table::
   :header-rows: 1
   :widths: 22 28 50

   * - Argument
     - Default
     - Description
   * - ``num_robots``
     - 20
     - Number of ``motion_controller_node`` instances to spawn.
   * - ``use_sim_time``
     - ``true``
     - Use the simulator clock.

In the integrated bringup this launch is included by
``swarm_mapf.launch.py`` (PBS) and ``swarm_mapf_formation.launch.py``
(PBS + formation), starting at ``t = 12 s`` after the simulator,
Nav2, and the PBS planner have settled. The LNS bringup
(``swarm_lns*.launch.py``) does **not** include this controller — it
spawns ``path_follower_node`` from ``iros_llm_swarm_mapf_lns``
instead.

Cross-references
----------------

* PBS planner that produces the ``mapf_path`` consumed here:
  ``iros_llm_swarm_mapf``.
* Newer follower paired with the LNS2 planner:
  ``iros_llm_swarm_mapf_lns`` (``path_follower_node``).
* Formation publisher / monitor and YAML schema:
  ``iros_llm_swarm_formation``.
* Message and service types: ``iros_llm_swarm_interfaces`` (in
  particular ``FormationsConfig`` and ``FormationConfig``).
