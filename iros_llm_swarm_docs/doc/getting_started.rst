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
       (``pbs_motion_controller``).
   * - ``swarm_lns.launch.py``
     - Stage + Nav2 + LNS2 planner + per-robot followers
       (``lns_motion_controller``).
   * - ``swarm_mapf_formation.launch.py``
     - PBS planner with formation manager / monitor.
   * - ``swarm_lns_formation.launch.py``
     - LNS2 planner with formation manager / monitor.
   * - ``swarm_full_demo.launch.py``
     - Full stack: simulator + Nav2 + selectable planner + optional
       formation + BT runner + LLM orchestrator (chat / decision /
       execute / passive observer) + optional rosbridge. The
       operator-facing launch file.

Example::

    ros2 launch iros_llm_swarm_bringup swarm_lns.launch.py

Useful launch arguments (shared across the bringup files):

.. list-table::
   :header-rows: 1
   :widths: 18 18 64

   * - Argument
     - Default
     - Description
   * - ``scenario``
     - ``cave`` (``amongus`` for ``swarm_full_demo``)
     - World preset: ``cave``, ``large_cave``, ``warehouse_2``,
       ``warehouse_4``, ``amongus``. Maps to a ``.world`` file via
       ``iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml``.
   * - ``num_robots``
     - ``20``
     - Number of robots controlled by the planner. The world always
       spawns its full robot set; this caps how many receive goals.
   * - ``world_file``
     - scenario-dependent
     - Override the Stage world file directly.
   * - ``rviz_cfg``
     - ``swarm_20.rviz``
     - RViz config file (loads the LLM panel and click-to-command
       tools when running ``swarm_full_demo``).
   * - ``use_sim_time``
     - ``true``
     - Use the simulator clock.

``swarm_full_demo.launch.py`` adds a few more arguments:

.. list-table::
   :header-rows: 1
   :widths: 24 16 60

   * - Argument
     - Default
     - Description
   * - ``planner``
     - ``lns``
     - Planner backend: ``lns`` or ``pbs``.
   * - ``enable_formation``
     - ``true``
     - Start ``formation_manager`` + ``formation_monitor``.
   * - ``enable_passive_observer``
     - ``false``
     - Start the channel-2 observer that watches ``/bt/state`` and
       pushes ``LlmCommand`` autonomously. Channel 1 (reactive
       ``/llm/decision``) is always on.
   * - ``enable_rosbridge``
     - ``true``
     - Start ``rosbridge_server`` (used by the MCP context provider).
   * - ``llm_backend``
     - ``http``
     - LLM backend: ``http`` (OpenAI-compatible), ``ollama``,
       ``mock``, or ``local`` (HuggingFace Transformers).
   * - ``llm_endpoint`` / ``llm_model``
     - empty
     - Override the YAML defaults in ``orchestrator.yaml``.

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

Operator chat (LLM)
-------------------

When running ``swarm_full_demo.launch.py``, the RViz instance loads
``iros_llm_rviz_panel`` (Panels → Add New Panel →
``iros_llm_rviz_panel/LLM-Panel``). Type a free-form command in the
**Chat** tab and press *Send* — the panel calls ``/llm/chat``, streams
the reply back chunk-by-chunk, and (if *Preview* is checked) shows the
parsed plan as a tree before executing it via ``/llm/execute_plan``.
The red **STOP ALL** button bypasses the LLM and halts every robot
through ``/llm/command{mode=idle}``.

Three click-to-command tools from ``iros_llm_rviz_tool`` are also
loaded:

* **g** — ``SendLlmGoalTool``: pick a robot group, click on the map,
  goals are spread around the click and dispatched to ``/llm/command``.
* **b** — ``PlaceObstacleTool``: left-click adds an obstacle, right-
  click removes the nearest. Talks to ``/obstacles/*`` (requires
  ``iros_llm_swarm_obstacles`` to be running).
* **d** — ``DoorTool``: left-click opens, right-click closes a door
  by id.

Every LLM call is appended to ``~/.ros/llm_decisions/`` (channel 1) and
``~/.ros/llm_commands/`` (channel 2) as one JSONL record per call.

Where to go next
----------------

* :doc:`examples` — operator usage guide for the LLM panel: chat
  patterns, robot groups and locations, plan structure, formation
  staging, click-to-command tools, channel 1 / 2 / 3 triggers,
  dataset inspection, backend selection, debugging.
* :doc:`packages` — pick a package and dive into its API and parameters.
* The per-package documentation linked from :doc:`index` covers internals
  not repeated here (planner cost models, BT XML schema, formation YAML
  layout, costmap plugin parameters, …).
