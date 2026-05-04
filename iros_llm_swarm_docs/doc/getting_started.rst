Getting Started
===============

This page walks through bringing the stack up from a fresh clone, running
the first mission, and pointing at the right launch file for each
scenario. For the *why* behind each component, see :doc:`overview` and
:doc:`architecture`.

Prerequisites
-------------

* Docker with Compose v2.
* X11 forwarding configured if you want RViz / Stage GUIs (the provided
  ``docker-compose.yaml`` mounts ``/tmp/.X11-unix`` and forwards
  ``DISPLAY``; run ``xhost +local:docker`` on the host first).
* No host-side ROS 2 install is required — everything lives in the
  container.

Bring up the container
----------------------

From the repository root::

    docker compose up terminal
    docker compose exec terminal bash

Inside the container, source the workspace and pin the DDS profile::

    source /home/fabian/ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42
    export CYCLONEDDS_URI=file:///home/fabian/ros2_ws/src/scripts/cyclonedds_swarm.xml

The CycloneDDS profile is required even at 20 robots — the default
profile struggles with the discovery fan-out.

Build
-----

When adding new types, build ``iros_llm_swarm_interfaces`` first so the
generated headers and Python modules are available to the rest of the
workspace::

    cd /home/fabian/ros2_ws
    colcon build --packages-select iros_llm_swarm_interfaces
    colcon build

For routine work, ``colcon build`` alone is enough. Per-package builds
are useful when iterating on one component::

    colcon build --packages-select iros_llm_swarm_mapf_lns

Launch a mission
----------------

The supported scenarios all live in ``iros_llm_swarm_bringup``:

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Launch file
     - Stack
   * - ``swarm_warehouse.launch.py``
     - Stage + Nav2 + RViz only — useful for poking at Nav2 without
       the MAPF layer.
   * - ``swarm_mapf.launch.py``
     - Stage + Nav2 + PBS planner + per-robot followers
       (``motion_controller_node``).
   * - ``swarm_lns.launch.py``
     - Stage + Nav2 + LNS2 planner + per-robot followers
       (``path_follower_node``).
   * - ``swarm_mapf_formation.launch.py``
     - PBS planner with formation manager / monitor.
   * - ``swarm_lns_formation.launch.py``
     - LNS2 planner with formation manager / monitor.

Example::

    ros2 launch iros_llm_swarm_bringup swarm_lns.launch.py

Useful launch arguments (all four files):

.. list-table::
   :header-rows: 1
   :widths: 18 18 64

   * - Argument
     - Default
     - Description
   * - ``scenario``
     - ``cave``
     - World preset: ``cave``, ``large_cave``, ``warehouse_2``,
       ``warehouse_4``.
   * - ``num_robots``
     - ``20``
     - Number of robots controlled by the planner. The world always
       spawns its full robot set; this caps how many receive goals.
   * - ``world_file``
     - ``warehouse.world``
     - Stage world file.
   * - ``rviz_cfg``
     - ``swarm_20.rviz``
     - RViz config file.
   * - ``use_sim_time``
     - ``true``
     - Use the simulator clock.

Send a mission
--------------

Once the stack is up and ``/map`` is available, the planner is ready to
accept goals on ``/swarm/set_goals``. Convenience clients live in
``iros_llm_swarm_mapf``::

    # All robots toward a single point (auto-distributed in a 1 m grid)
    ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 13.5 --goal-y 16.5

    # Random goals within a radius around the map center (15, 15)
    ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0

    # Goals from a JSON file
    ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json

JSON goal format::

    {
      "goals": [
        { "id": 0, "gx": 15.0, "gy": 12.5 },
        { "id": 1, "gx": 16.2, "gy": 14.8 }
      ]
    }

The action is long-lived: it stays active until every robot arrives, or
until the client cancels with ``Ctrl+C``. Feedback is printed during
each phase (``planning`` → ``executing`` → ``replanning`` → completion).

Local-nav-only smoke test
-------------------------

To exercise just the Nav2 layer without MAPF::

    ros2 run iros_llm_swarm_local_nav test_local_planner --num 20 \
        --goal-x 13.5 --goal-y 16.5

Robots will drive straight at the obstacle and stop — the local planner
alone cannot resolve fleet conflicts. This is expected; the MAPF layer
is what makes the swarm coherent.

Behavior tree smoke test
------------------------

To run the included behavior tree from a single command::

    ros2 run iros_llm_swarm_bt test_bt_runner

This loads ``behavior_trees/swarm_navigate_to_pose.xml`` and exercises
the ``MapfPlan`` / ``SetFormation`` / ``DisableFormation`` /
``CheckMode`` action nodes against a running stack.

Where to go next
----------------

* :doc:`packages` — pick a package and dive into its API and parameters.
* The per-package documentation linked from :doc:`index` covers internals
  not repeated here (planner cost models, BT XML schema, formation YAML
  layout, costmap plugin parameters, …).
