iros_llm_swarm_simulation_lite
==============================

The **supported** simulator for the swarm: a thin wrapper around
``stage_ros2`` (a 2D Stage simulation backend for ROS 2) plus the
project's own world files, occupancy maps, robot model, and scenario
catalogue. Used by the integrated bringup launches; the
heavier 3D Gazebo alternative ``iros_llm_swarm_simulation`` is preview
and not yet recommended.

This is a pure Python ament package — it ships only configuration and
launch logic. The simulator itself comes from ``stage_ros2``.

.. contents::
   :local:
   :depth: 2

What's in the package
---------------------

* ``launch/warehouse_swarm.launch.py`` — the only launch file. Spawns
  one ``stage_ros2/stage_ros2`` instance with the world file selected
  by the ``scenario`` argument.
* ``world/`` — four Stage world files (``cave``, ``large_cave``,
  ``warehouse``, ``warehouse_four``). Each ``.world`` includes the
  shared robot model, declares a ``floorplan`` against an occupancy
  bitmap, and statically lists all 20 robots with explicit poses.
* ``map/`` — three Nav2-format ``.yaml`` map descriptors plus their
  PGM bitmaps. Used by the global ``map_server`` started by
  ``iros_llm_swarm_local_nav``.
* ``robot_description/warehouse_robot.inc`` — the differential-drive
  robot model used by every world.
* ``scenario/common_scenarios.yaml`` — the scenario catalogue
  consumed by both this package and ``iros_llm_swarm_local_nav``.

Install layout
~~~~~~~~~~~~~~

The ``setup.py`` deliberately installs world files, maps, and the
robot ``.inc`` into the **same** install directory
(``share/<pkg>/stage_sim``). Stage resolves bitmap paths in
``floorplan { bitmap "warehouse.pgm" }`` relative to the world file's
location — colocation in the install tree is what makes that
resolution work without absolute paths in the worlds.

Scenarios are installed under ``share/<pkg>/scenario/``, separate
from the simulator artefacts. Launch files outside this package
(notably ``iros_llm_swarm_local_nav``) read the scenario catalogue
from there.

Launch flow
-----------

The launch file is intentionally minimal: read scenario name, look up
the world file in the scenario YAML, start one Stage process. There is
**no per-robot processing here** — Stage spawns every robot listed in
the world file at simulation startup.

::

    arg: scenario        ─┐
    arg: scenarios_file  ─┼─▶ resolve world file
    arg: world_file      ─┘     (scenario.world OR fallback)
                                       │
                                       ▼
                          stage_ros2 (one_tf_tree=True,
                                       enforce_prefixes=True,
                                       use_sim_time=True,
                                       is_depth_canonical=True,
                                       base_watchdog_timeout=0.2)

Launch arguments
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 28 50

   * - Argument
     - Default
     - Meaning
   * - ``scenario``
     - ``cave``
     - Scenario name. Looked up in ``scenarios_file``; if found and
       the entry has a ``world:`` field, that file is used.
   * - ``scenarios_file``
     - ``share/iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml``
     - Scenario catalogue YAML. Override to use an external file.
   * - ``world_file``
     - ``share/iros_llm_swarm_simulation_lite/stage_sim/cave.world``
     - Fallback world file used only if the scenario lookup fails or
       the entry has no ``world:`` field. Override directly to bypass
       the scenario flow.

There is **no** ``num_robots`` argument in this launch. Each world
file already declares 20 robots inline; Stage spawns whatever the
world contains, and the upstream layers (``iros_llm_swarm_local_nav``
and the bringup files) decide how many of those to attach Nav2 stacks
to via *their* ``num_robots`` argument. Lowering ``num_robots`` at
the bringup level does not stop Stage from spawning the unused
robots; it just leaves them sitting at their initial poses.

Stage parameters
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 14 56

   * - Parameter
     - Value
     - Why
   * - ``one_tf_tree``
     - ``true``
     - All robots' TF frames are published into a single shared TF
       tree, so ``map → robot_N/odom → robot_N/base_link`` chains
       coexist on one topic without per-robot TF prefixes that would
       require a TF static publisher per pair.
   * - ``enforce_prefixes``
     - ``true``
     - Stage prefixes every frame and topic with the robot name.
       Combined with the world file's per-robot ``name "robot_N"``,
       this is what produces ``/robot_N/odom`` and
       ``robot_N/base_link``.
   * - ``use_sim_time``
     - ``true``
     - The simulator owns ``/clock``; everything else in the stack
       uses ``use_sim_time: true`` to match.
   * - ``is_depth_canonical``
     - ``true``
     - Format toggle inside ``stage_ros2``; kept for forward
       compatibility.
   * - ``base_watchdog_timeout``
     - 0.2
     - Stage stops a robot if no ``cmd_vel`` arrives within this many
       seconds. The followers in the swarm publish ``cmd_vel`` only
       in formation mode; in autonomous mode Nav2 controls velocity
       through a different path (``follow_path`` action), so the
       watchdog is per-robot timing rather than fleet-wide. 0.2 s is
       generous enough to survive minor scheduling jitter.

Worlds and maps
---------------

The four bundled worlds:

.. list-table::
   :header-rows: 1
   :widths: 22 16 18 44

   * - World
     - Map bitmap
     - Map ``.yaml`` resolution / origin
     - Layout
   * - ``cave.world``
     - ``cave.pgm``
     - 0.05 m / cell, origin ``[-37.5, -37.5, 0]``
     - 75 × 75 m cave map. Five spawn zones distributed across the
       cave; default warehouse-like 20-robot fleet at varying
       headings.
   * - ``large_cave.world``
     - ``cave.pgm`` *(reused)*
     - 0.1 m / cell, origin ``[-75, -75, 0]``
     - 150 × 150 m world that **reuses the same** ``cave.pgm`` at
       half the original resolution. Five spawn zones spread out at
       the larger scale. Recommended only at fine-grained tuning;
       default planner parameters are sized for ``cave``.
   * - ``warehouse.world``
     - ``warehouse.pgm``
     - 0.05 m / cell, origin ``[0, 0, 0]``
     - 30 × 30 m warehouse. Two spawn zones: orange squad
       (``robot_0..9``) bottom-left at ~``(2-4, 2-8)``; blue squad
       (``robot_10..19``) top-right at ~``(26-28, 22-28)``.
   * - ``warehouse_four.world``
     - ``warehouse.pgm``
     - 0.05 m / cell, origin ``[0, 0, 0]``
     - Same 30 × 30 m warehouse, but the 20 robots are distributed
       across all four corners (5 each).

The ``warehouse`` map sits at world-aligned origin (``[0, 0]``)
because the world declares ``floorplan { pose [15, 15, 0, 0] size
[30, 30, 0.5] }`` — bottom-left of the bitmap is the world origin.
The two cave maps centre the bitmap at ``(0, 0)``, hence the
negative ``origin`` values in the YAMLs.

Robot model (``warehouse_robot.inc``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* **Footprint:** 30 × 30 × 15 cm bounding box; the actual collision
  shape is an 8-vertex octagon approximating a 30 cm circle.
* **Drive:** ``diff`` (Stage's differential-drive model).
* **Sensor:** one 360°-FOV planar laser at body centre (slightly
  below: ``pose [0, 0, -0.05, 0]``). 180 samples, 0.12 – 3.5 m
  range. Published as ``/robot_N/base_scan``.
* **Behavioural flags:** ``obstacle_return 1``, ``ranger_return 1.0``
  — robots see each other on the lidar. This is what causes the
  ghost-trail problem that ``iros_llm_swarm_costmap_plugins`` exists
  to fix.

The robot's ``color`` is overridden per-spawn in each world file,
which is purely cosmetic but useful in RViz and the Stage GUI for
distinguishing squads.

.. note::

   The Gazebo-side robot in ``iros_llm_swarm_simulation`` uses
   ``/robot_N/scan`` (no ``base_`` prefix) and a very different
   sensor profile (181 samples, 10 m max, Gaussian noise). The Nav2
   parameter file in ``iros_llm_swarm_local_nav`` is tuned for the
   Stage-side topic name and is not portable between simulators
   without changes.

Scenario catalogue (``common_scenarios.yaml``)
----------------------------------------------

Schema:

.. code-block:: yaml

   scenarios:
     <scenario_name>:
       world: "<filename>"          # relative to share/<pkg>/stage_sim/
       map: "<filename>"            # relative to share/<pkg>/stage_sim/
       num_robots: <int>            # informational; Stage uses the world's robot list
       initial_poses:
         - [x, y, yaw]
         - ...

The four shipped scenarios:

.. list-table::
   :header-rows: 1
   :widths: 18 22 60

   * - Scenario
     - World / map
     - Notes
   * - ``cave``
     - ``cave.world`` / ``cave.yaml``
     - Default. 20 robots distributed across five cave zones at
       varying headings.
   * - ``large_cave``
     - ``large_cave.world`` / ``large_cave.yaml``
     - Same robot count and zone layout but spread across the
       150 × 150 m world. **Not recommended on default planner
       parameters** — distances are large and the default MAPF
       horizon may be too short.
   * - ``warehouse_2``
     - ``warehouse.world`` / ``warehouse.yaml``
     - Two-squad layout: orange ``robot_0..9`` bottom-left, blue
       ``robot_10..19`` top-right.
   * - ``warehouse_4``
     - ``warehouse_four.world`` / ``warehouse.yaml``
     - Four-corner layout: 5 robots per corner. Same map; different
       world.

The catalogue is consumed in two places:

#. **This package's launch** uses the ``world:`` field to choose the
   world file.
#. **``iros_llm_swarm_local_nav``'s launch** uses the
   ``initial_poses`` field for per-robot AMCL setup (currently inert
   — AMCL is disabled) and the ``map:`` field to pick the global
   map. The two consumers operate independently; the same scenario
   YAML drives both.

Topics published by Stage (per robot)
-------------------------------------

With ``one_tf_tree`` and ``enforce_prefixes`` set, Stage publishes
the following per robot. The ``stage_ros2`` defaults are followed:

* ``/robot_N/odom`` — ``nav_msgs/Odometry``
* ``/robot_N/base_scan`` — ``sensor_msgs/LaserScan``
* ``/robot_N/base_pose_ground_truth`` —
  ``nav_msgs/Odometry`` (ground truth, useful for evaluation).
* ``/tf`` (shared) — frames ``robot_N/odom →
  robot_N/base_link``. ``map → robot_N/odom`` is supplied by
  ``iros_llm_swarm_local_nav``'s static publisher (AMCL is off).
* ``/clock`` — ``rosgraph_msgs/Clock``.

Stage subscribes to ``/robot_N/cmd_vel`` (``geometry_msgs/Twist``)
to drive each robot's diff-drive model.

Build, install, run
-------------------

The package is pure Python — no build step beyond ``colcon``::

    colcon build --packages-select iros_llm_swarm_simulation_lite

Standalone launch (Stage only — no Nav2, no MAPF, no followers)::

    ros2 launch iros_llm_swarm_simulation_lite warehouse_swarm.launch.py
    ros2 launch iros_llm_swarm_simulation_lite warehouse_swarm.launch.py \
      scenario:=warehouse_2

In the integrated bringup launches (``swarm_mapf*.launch.py`` /
``swarm_lns*.launch.py``) this launch is included at ``t = 0`` and
everything else is layered on top.

Cross-references
----------------

* Nav2 stack that reads the maps and consumes ``/robot_N/base_scan``:
  ``iros_llm_swarm_local_nav``.
* The custom costmap layer that compensates for inter-robot lidar
  cross-marking on this simulator's sparse scanner:
  ``iros_llm_swarm_costmap_plugins``.
* Alternative 3D simulator (preview, not yet stable):
  ``iros_llm_swarm_simulation``.
* Bringup orchestration that includes this launch:
  ``iros_llm_swarm_bringup``.
