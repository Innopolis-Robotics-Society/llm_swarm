iros_llm_swarm_local_nav
========================

Per-robot Nav2 stack spawner. The package is a Python launch
orchestrator plus a tuned Nav2 parameter file: a single launch file
brings up one shared ``map_server`` and N copies of
``controller_server`` / ``behavior_server`` / lifecycle managers, one
under each ``robot_<id>`` namespace, with namespace substitution in
the YAML happening at launch time.

A standalone smoke-test client (``test_local_planne``) is also
provided for exercising Nav2 in isolation, without the MAPF layer.

.. contents::
   :local:
   :depth: 2

What this package does — and does not
-------------------------------------

* **Spawns** the per-robot Nav2 stack for the swarm.
* **Owns** the Nav2 parameter file (``robot_nav2_params.yaml``) tuned
  for 20 differential-drive robots in 2D Stage simulation.
* **Defines** the static ``map → robot_<id>/odom`` transform that
  replaces AMCL.
* **Does not** know about the MAPF layer at all. It exposes a vanilla
  per-robot ``follow_path`` action; whoever consumes it
  (``iros_llm_swarm_robot``'s ``motion_controller_node``,
  ``iros_llm_swarm_mapf_lns``'s ``path_follower_node``, or the
  bundled ``test_local_planne`` test client) can speak to it directly.
* **Does not** spawn the simulator; the Stage world ships from
  ``iros_llm_swarm_simulation_lite`` and is launched separately by
  the bringup files.

Layered output of the launch
----------------------------

For ``num_robots = N``, the launch produces the following nodes::

    Once globally:
      map_server
      lifecycle_manager_map  (autostart, manages [map_server])

    Per robot, namespace robot_i, staggered by 0.3 * i seconds:
      static_transform_publisher  map -> robot_i/odom  (identity)
      controller_server
      behavior_server
      lifecycle_manager_nav        (autostart, manages
                                     [controller_server, behavior_server])

The 0.3 s stagger between robots is set via ``TimerAction(period=0.3 * i)``
to spread DDS discovery and Nav2 startup over time — bringing all 20
stacks up simultaneously triggers race conditions in lifecycle
transitions.

AMCL is *not* spawned by default. The launch file contains the AMCL
node block commented out, with a TODO note explaining why: at 20
robots the cross-robot LiDAR interference pollutes the particle
filter, the localisation drifts within seconds, and Nav2 then
mis-corrects the costmap relative to the simulator. The replacement
is the static ``map → robot_i/odom`` identity transform — Stage's
``one_tf_tree`` mode already publishes ``robot_i/odom →
robot_i/base_link`` correctly, so an identity ``map → odom`` is
sufficient as long as the simulator is the ground-truth source of
both frames. AMCL can be re-enabled by uncommenting the block in
``robot_local_nav.launch.py``; the YAML already has matching ``amcl:``
parameters under the per-robot namespace.

Per-robot YAML processing
-------------------------

The Nav2 parameter file uses ``<robot_namespace>`` as a textual
placeholder anywhere a frame name or topic should be per-robot:

.. code-block:: yaml

   amcl:
     ros__parameters:
       odom_frame_id: "<robot_namespace>/odom"
       base_frame_id: "<robot_namespace>/base_link"

   local_costmap:
     local_costmap:
       ros__parameters:
         global_frame:     "<robot_namespace>/odom"
         robot_base_frame: "<robot_namespace>/base_link"
         obstacle_layer:
           base_scan:
             topic: "/<robot_namespace>/base_scan"

The launch routes the file through two transformations from
``nav2_common.launch``:

#. **``ReplaceString``** — substitutes every literal
   ``<robot_namespace>`` with ``robot_i``. Produces a per-robot
   intermediate file in memory.
#. **``RewrittenYaml``** — wraps the result under the root key
   ``robot_i`` so each Nav2 node, when launched in that namespace,
   reads its parameters from
   ``robot_i.<original_section>.<...>``. ``convert_types: true`` is
   set so YAML-ints/floats keep their original types.

The result is fed to every Nav2 node in the per-robot
``GroupAction`` via a ``ParameterFile(... allow_substs=True)``.

Tooling: nodes spawned per robot
--------------------------------

``controller_server``
~~~~~~~~~~~~~~~~~~~~~

Drives the local trajectory. The shipped configuration uses
**Regulated Pure Pursuit**, *not* DWB. The YAML carries a long
in-line rationale; the short version:

* DWB is a reactive sampling planner that treats the MAPF path as a
  *suggestion* and assembles its own trajectories from a critic stack.
  At low ``BaseObstacle`` weight it ignores the costmap and runs
  through obstacles; at high weight it invents detours that diverge
  from the MAPF schedule. Both behaviours are wrong for a fleet that
  is supposed to follow a centrally-planned conflict-free schedule.
* Regulated Pure Pursuit follows the polyline literally: it picks a
  carrot point ``lookahead_dist`` ahead and steers toward it, with
  three regulators on top — slow on tight curvature, slow near
  obstacles in the costmap, and stop on a near-future predicted
  collision. No trajectory invention.

Key parameters (full set in
``config/robot_nav2_params.yaml``):

.. list-table::
   :header-rows: 1
   :widths: 38 14 48

   * - Parameter
     - Default
     - Meaning
   * - ``controller_frequency``
     - 10.0
     - Loop rate (Hz). Pure Pursuit is light enough to run at 20 Hz
       too if motion looks chunky.
   * - ``FollowPath.desired_linear_vel``
     - 0.5
     - Cruise speed.
   * - ``FollowPath.lookahead_dist``
     - 0.6
     - Carrot radius. ~3 MAPF cells at 0.2 m resolution. Lower =
       tighter tracking; higher = smoother but corner-cutting.
   * - ``use_regulated_linear_velocity_scaling``
     - ``true``
     - Slow on tight curvature.
   * - ``use_cost_regulated_linear_velocity_scaling``
     - ``true``
     - Slow when neighbouring robots appear in the local costmap.
   * - ``use_collision_detection``
     - ``true``
     - Predict collisions ``max_allowed_time_to_collision_up_to_carrot``
       seconds ahead and stop if any are predicted.
   * - ``use_rotate_to_heading``
     - ``true``
     - Rotate in place if the carrot is more than 45° off-axis (only
       applies to differential-drive base).

``behavior_server``
~~~~~~~~~~~~~~~~~~~

Hosts the recovery behaviours: ``spin``, ``backup``, ``wait``. No
behaviour-tree-driven recovery sequence is configured at this layer —
recovery is invoked by the controller stack on its own when needed.

``lifecycle_manager`` (per robot)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Manages ``controller_server`` and ``behavior_server`` with
``autostart: true``. ``amcl`` is intentionally absent from the
managed list — the commented launch block, if re-enabled, also
contains the matching addition to ``node_names``.

Local and global costmaps
-------------------------

**Local costmap** (rolling 8 × 8 m window at 0.05 m / cell, updated
at 20 Hz, published at 5 Hz):

* ``obstacle_layer`` — uses
  ``iros_llm_swarm_costmap_plugins/ResettingObstacleLayer`` (see that
  package's documentation). The plugin wipes the layer's grid every
  cycle before re-marking, which is the swarm-specific fix for the
  ghost-trail problem caused by sparse-ray scans + many moving
  neighbours. Configured with ``observation_persistence: 0.0`` to
  match: every observation must re-occur every cycle to stay
  visible.
* ``inflation_layer`` — standard
  ``nav2_costmap_2d::InflationLayer`` with
  ``inflation_radius: 0.40`` and ``cost_scaling_factor: 3.0``.

**Global costmap** uses the static-map flow:

* ``static_layer`` — subscribes to ``/map`` (transient local).
* ``inflation_layer`` — same plugin as in the local costmap, with
  ``inflation_radius: 0.5``.

The global costmap is **not** wiped per cycle (the static layer
relies on a single transient_local snapshot of the world map).

Static ``map → robot_i/odom`` TF
--------------------------------

For each robot, the launch spawns a ``static_transform_publisher``
publishing identity (zero translation, zero rotation) from ``map`` to
``robot_<id>/odom``. Because Stage runs in ``one_tf_tree`` mode and
publishes ``robot_<id>/odom → robot_<id>/base_link`` itself, the
identity ``map → odom`` is sufficient: every robot's frames live on
the same TF graph as the simulator's ground truth, with the
simulator as the canonical source of the localisation chain.

This identity TF replaces AMCL for the reasons described above. If
ground-truth-equivalent localisation ever needs to be relaxed, the
AMCL block in the launch file is the entry point — it already
contains the matching ``set_initial_pose`` call wired to per-robot
``initial_poses``.

Scenario / initial-pose loading
-------------------------------

The launch ingests initial-pose lists from two sources:

#. **Direct argument.** If ``initial_poses`` is anything other than
   the literal string ``"None"``, it is parsed as
   ``"x,y,yaw; x,y,yaw; ..."`` (semicolons separate robots, commas
   separate components). Tuples that fail to parse are silently
   skipped.
#. **Scenario YAML.** When ``initial_poses == "None"`` (the default),
   the launch reads
   ``iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml``
   and looks up the entry under
   ``scenarios.<scenario>``. Each scenario provides ``initial_poses``
   (a list of ``[x, y, yaw]``) and optionally a ``map`` filename
   (relative to ``iros_llm_swarm_simulation_lite/stage_sim/``). When
   the scenario provides a map, it overrides the ``map_file``
   argument; otherwise the launch falls back to ``map_file``.

If fewer poses are available than ``num_robots``, missing slots are
zero-padded with ``(0.0, 0.0, 0.0)``. AMCL would consume the per-pose
``set_initial_pose`` parameters; with AMCL disabled they are
currently inert, but the parsing is preserved so toggling AMCL back
on is a one-block change.

Launch arguments
----------------

.. list-table::
   :header-rows: 1
   :widths: 24 26 50

   * - Argument
     - Default
     - Description
   * - ``num_robots``
     - 20
     - Number of robots to bring up.
   * - ``scenario``
     - ``cave``
     - Scenario name to look up in the scenarios YAML.
   * - ``scenarios_file``
     - ``iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml``
     - YAML containing scenario definitions.
   * - ``initial_poses``
     - ``"None"``
     - Inline ``"x,y,yaw; ..."`` poses. ``"None"`` means use the
       scenarios YAML.
   * - ``map_file``
     - ``iros_llm_swarm_simulation_lite/stage_sim/cave.yaml``
     - Map YAML used when the scenario does not specify one.
   * - ``nav2_params_file``
     - ``iros_llm_swarm_local_nav/config/robot_nav2_params.yaml``
     - Nav2 parameter file.

Smoke-test client: ``test_local_planne``
----------------------------------------

A standalone Python client that bypasses the MAPF layer and sends
``FollowPath`` action goals directly to each robot's
``controller_server``. Useful for sanity-checking the per-robot Nav2
stack in isolation: if every robot accepts a goal, drives toward it,
and stops on encountering an obstacle, the local-nav layer is wired
correctly.

.. note::

   The console-script name has a trailing-letter typo —
   ``test_local_planne`` (no ``r``) — preserved across releases.
   The repository's ``README`` and ``CLAUDE.md`` reference both
   ``test_local_planner`` and ``test_local_planne``; only the latter
   actually resolves to a binary. Treat that spelling as canonical
   when invoking it::

       ros2 run iros_llm_swarm_local_nav test_local_planne ...

The client has two modes, selected by command-line argument:

* **Random in odom** (default) — for each robot, pick a uniformly
  random goal within ``--radius`` metres in the robot's odom frame
  and send a 10-step straight-line ``FollowPath`` goal toward it.
* **Fixed goal** — when ``--goal-x`` and ``--goal-y`` are provided,
  send every robot to the same global point. ``--goal-frame``
  selects ``"map"`` (default) or ``"odom"``. In ``map`` mode the
  client uses ``tf2_ros`` to transform the goal into each robot's
  ``odom`` frame, since ``FollowPath`` operates per-robot in the
  local frame.

Arguments::

    --num         Number of robots to drive (default: 20)
    --radius      Random goal radius in odom (default: 2.0)
    --goal-x      Fixed-goal x (default: None)
    --goal-y      Fixed-goal y (default: None)
    --goal-frame  Frame for fixed goal: "map" or "odom" (default: "map")

Behaviour caveats:

* The client builds a 10-step straight-line ``Path`` between the
  robot's current odom pose and the goal. There is **no obstacle
  awareness** in the client itself — the controller stack is what
  must avoid obstacles. With Regulated Pure Pursuit and a populated
  costmap the robot will slow and stop in front of obstacles; the
  goal is then unreachable until the obstacle clears. This is the
  expected demonstration of "what Nav2 does without MAPF" in the
  bundled README.
* "Random in map" is intentionally **not** supported in the current
  client — the in-line warning in the source file explains why
  (computing a random offset in map requires the inverse map↔odom
  TF and the implementation does not bother). Pass the goal in
  ``odom`` if you want random goals.

Build, install, run
-------------------

Pure Python package; only ``iros_llm_swarm_simulation_lite`` is a
runtime dependency for default scenarios::

    colcon build --packages-select iros_llm_swarm_local_nav

Standalone launch::

    ros2 launch iros_llm_swarm_local_nav robot_local_nav.launch.py
    ros2 launch iros_llm_swarm_local_nav robot_local_nav.launch.py num_robots:=10
    ros2 launch iros_llm_swarm_local_nav robot_local_nav.launch.py \
      initial_poses:='2,2,0; 3,3,0; 4,4,0' num_robots:=3

Smoke test against a running stack::

    ros2 run iros_llm_swarm_local_nav test_local_planne --num 20 \
      --goal-x 13.5 --goal-y 16.5

In the integrated bringup launches
(``swarm_mapf*.launch.py`` / ``swarm_lns*.launch.py``), this launch
file is included with a 1 s delay after the simulator starts.

Cross-references
----------------

* Nav2 plugin loaded by the local costmap:
  ``iros_llm_swarm_costmap_plugins`` (``ResettingObstacleLayer``).
* Map server input: maps and worlds ship from
  ``iros_llm_swarm_simulation_lite``.
* Scenario definitions:
  ``iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml``.
* Action ``follow_path`` consumers:
  ``iros_llm_swarm_robot/motion_controller_node`` (PBS stack) and
  ``iros_llm_swarm_mapf_lns/path_follower_node`` (LNS stack).
