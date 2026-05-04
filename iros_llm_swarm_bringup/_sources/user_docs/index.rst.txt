iros_llm_swarm_bringup
======================

Top-level launch orchestration for the swarm. The package contains
**no node code** — five launch files plus the shared RViz config.
Each launch file composes the per-package launches from
``iros_llm_swarm_simulation_lite``, ``iros_llm_swarm_local_nav``,
the active MAPF planner, the per-robot followers, and (optionally)
the formation layer, with a fixed staggered timing pattern.

This is the entry point for every full-system run.

.. contents::
   :local:
   :depth: 2

Picking a launch
----------------

.. list-table::
   :header-rows: 1
   :widths: 32 12 56

   * - Launch file
     - MAPF
     - Components
   * - ``swarm_warehouse.launch.py``
     - none
     - Stage + Nav2 + RViz only. Sanity-check the simulator and Nav2
       layer without any planner.
   * - ``swarm_mapf.launch.py``
     - PBS
     - Adds ``iros_llm_swarm_mapf/mapf_planner_node`` and the
       ``iros_llm_swarm_robot/motion_controller_node`` followers.
   * - ``swarm_mapf_formation.launch.py``
     - PBS
     - Same as above plus ``iros_llm_swarm_formation`` manager and
       monitor.
   * - ``swarm_lns.launch.py``
     - LNS2
     - Adds the ``iros_llm_swarm_mapf_lns/mapf_lns2.launch.py`` stack
       (LNS2 planner and its bundled ``path_follower_node``).
   * - ``swarm_lns_formation.launch.py``
     - LNS2
     - Same as above plus formation manager and monitor.

The two **PBS** launches use the legacy planner
(``iros_llm_swarm_mapf``) and its companion follower
(``iros_llm_swarm_robot/motion_controller_node``). The two **LNS2**
launches use the supported planner stack (``iros_llm_swarm_mapf_lns``)
which spawns its own ``path_follower_node`` per robot — the
``iros_llm_swarm_robot`` follower is **not** included in LNS launches.

Common architecture
-------------------

Every launch follows the same staggered shape::

    t = 0  s : Stage simulator + RViz
    t = 1  s : Nav2 (one map_server + per-robot controller / behavior /
               lifecycle, plus the static map → robot_N/odom transforms)
    t = 10 s : MAPF planner    (waits for /map; transient_local QoS
                                makes the late subscribe a no-cost
                                receive)
    t = 12 s : Per-robot followers (and, if applicable, formation
                                manager + monitor — they start
                                together with the followers so that
                                followers are subscribed to
                                /formations/config before its first
                                latched message arrives)

The stagger is implemented by wrapping each layer's
``IncludeLaunchDescription`` (or ``Node``) in a
``TimerAction(period=N.0, actions=[...])``. ``LogInfo`` messages on
the same timer make the boot order visible in the launch output.

The full-system launches do **not** publish anything themselves; they
only compose the layer launches with hard-coded delays. To inspect or
modify timing, edit the relevant ``.launch.py`` directly.

Per-launch detail
-----------------

swarm_warehouse.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~

Bare environment without a planner. Useful for verifying the
simulator and Nav2 are correctly wired before bringing up MAPF.

* **Components:** Stage (``warehouse_swarm.launch.py``) at t=0;
  Nav2 (``robot_local_nav.launch.py``) at t=1; RViz at t=0.
* **Default world:** ``warehouse.world``.
* **Arguments:** ``world_file``, ``num_robots`` (default ``20``),
  ``rviz_cfg``, ``use_sim_time`` (default ``true``).

Smoke test against this stack with ``test_local_planne`` from
``iros_llm_swarm_local_nav``.

swarm_mapf.launch.py
~~~~~~~~~~~~~~~~~~~~

PBS planner stack on Stage.

* **Default world:** ``warehouse_four.world``.
* **t=10s — ``mapf_planner_node`` parameters explicit in the launch:**

  * ``time_step_sec`` (arg, default 0.4)
  * ``inflation_radius: 0.75``
  * ``max_pbs_expansions: 5000``
  * ``max_astar_expansions: 1000000``
  * ``cost_curve: "quadratic"``
  * ``urgency: 1.0``
  * ``replan_threshold_m: 1.5``
  * ``proximity_penalty: 50``
  * ``pbs_resolution: 0.2``
  * ``replan_cooldown_sec: 30.0``

* **t=12s — followers:** ``iros_llm_swarm_robot/motion_controllers.launch.py``.
* **Arguments:** ``num_robots``, ``use_sim_time``,
  ``time_step_sec``, ``world_file``, ``rviz_cfg``.

A trailing ``LogInfo`` at t=15 s announces the stack is ready.

swarm_mapf_formation.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

PBS planner with formation overlay.

* **Default world:** ``warehouse.world`` *(differs from*
  ``swarm_mapf``\ *)*.
* **Default formations YAML:** ``formations_1.yaml``
  (wedge around ``robot_1``, line around ``robot_5`` —
  matching the warehouse two-squad layout).
* **t=10s — ``mapf_planner_node`` parameters minimal in the launch:**
  only ``num_robots``, ``time_step_sec``, ``use_sim_time``,
  ``map_topic`` and ``default_robot_radius=0.22`` are set inline;
  everything else takes the planner's compile-time defaults. This is
  intentionally a thinner override than ``swarm_mapf.launch.py`` —
  expect different planner behaviour even with the same map.
* **t=12s — followers + formation_manager + formation_monitor in the
  same TimerAction.** This is critical: a follower that subscribes
  to the latched ``/formations/config`` topic *after* the manager's
  first publish would still receive the latched message, but the
  three nodes are started together to keep the boot trace
  predictable.
* **formation_manager parameters:** ``config_file:
  formations_cfg`` (default
  ``iros_llm_swarm_formation/config/formations_1.yaml``),
  ``auto_activate: True``, ``footprint_padding: 0.2``,
  ``robot_radius: 0.3``.
* **formation_monitor parameters:** ``monitor_hz: 10``,
  ``stable_thresh_m: 0.15``, ``degraded_thresh_m: 0.35``,
  ``odom_timeout_s: 1.0``, ``stuck_window_s: 3.0``,
  ``stuck_delta_m: 0.05``.
* **Arguments:** ``num_robots``, ``use_sim_time``,
  ``time_step_sec``, ``world_file``, ``rviz_cfg``,
  ``formations_cfg``.

swarm_lns.launch.py
~~~~~~~~~~~~~~~~~~~

LNS2 planner stack on Stage. The cleanest of the four MAPF
launches: it delegates planner + follower spawning entirely to
``iros_llm_swarm_mapf_lns/mapf_lns2.launch.py``.

* **Scenario-based simulator selection.** This launch is the only
  one that uses the ``scenario`` argument with strict choices
  (``cave`` / ``large_cave`` / ``warehouse_2`` / ``warehouse_4``)
  and the ``scenarios_file`` argument, both forwarded to the Stage
  launch. There is no ``world_file`` argument here — set
  ``scenario`` instead.
* **Default scenario:** ``cave``.
* **t=10s — full LNS2 stack** via include: this single
  ``IncludeLaunchDescription`` produces both the planner node
  (``mapf_lns2_node``) and ``num_robots`` per-robot
  ``path_follower_node`` instances. The motion-controller include
  from ``iros_llm_swarm_robot`` is **not** added — the LNS follower
  replaces it.
* **Planner parameters** come from
  ``iros_llm_swarm_mapf_lns/config/mapf_lns2.yaml``, not from this
  bringup file. This launch passes only ``num_robots`` and
  ``use_sim_time`` through.
* **Arguments:** ``scenario``, ``scenarios_file``, ``num_robots``,
  ``use_sim_time``, ``rviz_cfg``.

  ``time_step_sec`` is **not** declared here even though the
  ``swarm_mapf*`` launches define it; the LNS planner reads its own
  ``time_step_sec`` from its YAML.

swarm_lns_formation.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

LNS2 planner with formation overlay.

* **Default world:** ``warehouse_four.world`` *(differs from*
  ``swarm_mapf_formation``\ *)*. Note also that this launch uses
  ``world_file`` instead of ``scenario``, unlike the non-formation
  ``swarm_lns.launch.py`` — the two LNS launches expose
  **different** simulator-selection interfaces.
* **Default formations YAML:** ``formations_2.yaml``
  (wedge around ``robot_1``, line around ``robot_17`` —
  matching the four-corner layout where ``robot_5`` does not exist
  in the same place as in ``warehouse``).
* **t=10s:** LNS2 stack via include (planner + path_followers).
* **t=12s:** ``formation_manager`` only (inside ``TimerAction``).
  ``formation_monitor`` is **launched at root, not inside a
  TimerAction** — so it starts at t=0. This is asymmetric with
  ``swarm_mapf_formation.launch.py`` where both formation nodes
  are wrapped in the t=12 timer. The monitor sits idle until the
  manager begins publishing on ``/formations/config``, so the
  asymmetry is functionally fine but worth knowing when reading
  the boot trace.
* **Arguments:** same as ``swarm_lns_formation`` minus the
  ``scenario`` flow — ``num_robots``, ``use_sim_time``,
  ``time_step_sec`` *(declared but unused)*, ``world_file``,
  ``rviz_cfg``, ``formations_cfg``.

Argument reference
------------------

Common arguments (where they appear):

.. list-table::
   :header-rows: 1
   :widths: 22 24 16 38

   * - Argument
     - Default
     - In launches
     - Meaning
   * - ``num_robots``
     - 20
     - all
     - Number of robots to attach a Nav2 stack to (and, in MAPF
       launches, to plan for). Stage always spawns the world's full
       robot list — see *Inconsistencies* below.
   * - ``use_sim_time``
     - ``true``
     - all
     - Use the simulator clock everywhere.
   * - ``rviz_cfg``
     - ``share/iros_llm_swarm_bringup/rviz/swarm_20.rviz``
     - all
     - RViz config file.
   * - ``world_file``
     - varies (see below)
     - ``swarm_warehouse``, ``swarm_mapf``,
       ``swarm_mapf_formation``, ``swarm_lns_formation``
     - Stage world file. Forwarded to
       ``warehouse_swarm.launch.py``.
   * - ``scenario``
     - ``cave`` (with choices)
     - ``swarm_lns`` *only*
     - Named entry in the scenarios YAML; resolves the world file
       indirectly.
   * - ``scenarios_file``
     - ``share/iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml``
     - ``swarm_lns`` *only*
     - Override the scenarios catalogue.
   * - ``time_step_sec``
     - 0.4
     - ``swarm_mapf``, ``swarm_mapf_formation``,
       ``swarm_lns_formation`` (declared but unused)
     - Per-step time for the PBS planner; must match the follower's
       schedule tolerance.
   * - ``formations_cfg``
     - varies (``formations_1.yaml`` or ``formations_2.yaml``)
     - ``swarm_mapf_formation``, ``swarm_lns_formation``
     - Formation YAML loaded by ``formation_manager_node`` with
       ``auto_activate: True``.

Default world per launch:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Launch file
     - Default world
     - Notes
   * - ``swarm_warehouse``
     - ``warehouse.world``
     - 30 × 30 m, two squads.
   * - ``swarm_mapf``
     - ``warehouse_four.world``
     - 30 × 30 m, four-corner layout. *Different* from the formation
       PBS launch.
   * - ``swarm_mapf_formation``
     - ``warehouse.world``
     - Two-squad layout to match ``formations_1.yaml``.
   * - ``swarm_lns``
     - ``cave.world`` (via ``scenario: cave``)
     - Only launch using the scenario flow.
   * - ``swarm_lns_formation``
     - ``warehouse_four.world``
     - Four-corner layout to match ``formations_2.yaml``.

Inconsistencies and gotchas
---------------------------

Several details of the launch files are surprising on first read.
Capturing them so they don't get rediscovered repeatedly.

1. **Two simulator-selection interfaces.** ``swarm_lns`` is the only
   launch that uses the ``scenario`` / ``scenarios_file`` argument
   pair. The others take a literal ``world_file`` path. Picking
   ``swarm_lns`` and ``swarm_lns_formation`` looks symmetric from
   the outside but uses **different** argument shapes — see
   ``ros2 launch iros_llm_swarm_bringup <name> --show-args``.
2. **Different default worlds across the four MAPF launches.** PBS:
   ``warehouse_four.world`` (no formation) vs ``warehouse.world``
   (with formation). LNS: ``cave`` (no formation) vs
   ``warehouse_four.world`` (with formation). Stick to passing
   ``world_file`` / ``scenario`` explicitly when scripting runs.
3. **PBS launches drive a legacy planner.** Both ``swarm_mapf*``
   launches spawn ``iros_llm_swarm_mapf/mapf_planner_node`` as the
   planner. That package is the older PBS implementation that the
   project is moving away from; for new work prefer the
   ``swarm_lns*`` launches even when "MAPF" is the task description.
4. **``num_robots`` does not shrink the simulator.** Stage spawns
   every robot listed in the world file (always 20 in shipped
   worlds); the ``num_robots`` argument only caps how many of those
   get a Nav2 stack and a follower attached. Robots beyond the cap
   sit idle at their initial poses with no controller.
5. **``time_step_sec`` is declared but unused in
   ``swarm_lns_formation``.** Leftover from the PBS template; LNS
   reads its own ``time_step_sec`` from
   ``iros_llm_swarm_mapf_lns/config/mapf_lns2.yaml``. Setting it on
   the bringup CLI does nothing.
6. **Asymmetric formation_monitor placement in
   ``swarm_lns_formation``.** ``formation_monitor`` is launched at
   root (effectively t=0) instead of inside the t=12 ``TimerAction``
   that holds ``formation_manager``. The monitor harmlessly idles
   until the manager begins publishing — but the boot trace will
   show the monitor much earlier than the manager.
7. **MAPF parameter coverage differs.** ``swarm_mapf.launch.py``
   sets a long list of PBS parameters inline; the formation variant
   sets very few and relies on planner defaults. Consequently the
   two PBS launches do not produce identical planner behaviour out
   of the box.
8. **``auto_activate: True`` is hard-coded** in both formation
   launches. The YAML-loaded formations become active immediately
   at startup. To start with a passive registry, override the
   parameter (e.g. ``auto_activate:=False`` from the CLI is **not**
   wired through — edit the ``.launch.py`` or call
   ``/formation/deactivate`` after startup).

RViz configuration
------------------

The shared config ``rviz/swarm_20.rviz`` contains:

* Static displays: ``Grid``, ``TF``, two ``Map`` displays
  (global + local costmap pre-wired), ``LaserScan``,
  ``PointCloud2``, ``PoseWithCovariance``.
* One ``Polygon`` display for the formation footprint.
* One ``Odometry`` display.
* Twenty ``Path`` displays — pre-wired to the per-robot MAPF path
  topics so all 20 schedules are visible during a mission.

Override with ``rviz_cfg:=`` to point at a different file.

Build, install, run
-------------------

Pure Python ament package; just launch and config files installed::

    colcon build --packages-select iros_llm_swarm_bringup

Examples (each assumes the workspace's ``setup.bash`` is sourced and
``CYCLONEDDS_URI`` points at ``cyclonedds_swarm.xml`` — see the
project ``README``)::

    # Stage + Nav2 only
    ros2 launch iros_llm_swarm_bringup swarm_warehouse.launch.py
    ros2 launch iros_llm_swarm_bringup swarm_warehouse.launch.py num_robots:=5

    # PBS stack
    ros2 launch iros_llm_swarm_bringup swarm_mapf.launch.py
    ros2 launch iros_llm_swarm_bringup swarm_mapf_formation.launch.py

    # LNS2 stack (preferred for new work)
    ros2 launch iros_llm_swarm_bringup swarm_lns.launch.py scenario:=warehouse_2
    ros2 launch iros_llm_swarm_bringup swarm_lns_formation.launch.py \
      formations_cfg:=/abs/path/formations.yaml

After the stack reports "MAPF stack ready" (~15 s), drive it via the
``/swarm/set_goals`` action — for example::

    ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 13.5 --goal-y 16.5

Cross-references
----------------

* Stage simulator launch (always at t=0):
  ``iros_llm_swarm_simulation_lite``.
* Nav2 layer (always at t=1): ``iros_llm_swarm_local_nav``.
* PBS planner (in ``swarm_mapf*``): ``iros_llm_swarm_mapf``.
* LNS planner (in ``swarm_lns*``): ``iros_llm_swarm_mapf_lns``
  (planner *and* per-robot followers).
* PBS-side per-robot follower (in ``swarm_mapf*``):
  ``iros_llm_swarm_robot``.
* Formation registry / monitor (in ``swarm_*_formation``):
  ``iros_llm_swarm_formation``.
* Test clients to drive the stack: see ``iros_llm_swarm_mapf``
  (``test_send_goals``) and ``iros_llm_swarm_local_nav``
  (``test_local_planne``).
