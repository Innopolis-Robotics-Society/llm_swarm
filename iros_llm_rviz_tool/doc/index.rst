iros_llm_rviz_tool
==================

Three RViz2 ``Tool`` plugins for click-to-command operation. None of
them call the LLM — they translate a mouse click directly into a ROS
service or action call.

Plugins
-------

.. list-table::
   :header-rows: 1
   :widths: 20 12 68

   * - Plugin
     - Shortcut
     - What it does
   * - ``SendLlmGoalTool``
     - ``g``
     - Inherits ``rviz_default_plugins::tools::PoseTool``. After the
       click pose is set, the tool loads the map YAML from
       ``iros_llm_orchestrator``, pops a ``QInputDialog::getItem`` to
       pick a robot group (e.g. ``cyan``, ``magenta``), spreads N goals
       around the click point at 1.5 m spacing, and dispatches them as
       a single ``LlmCommand{mode=mapf, robot_ids=[...], goals=[...]}``
       on ``/llm/command``.
   * - ``PlaceObstacleTool``
     - ``b``
     - Inherits ``rviz_common::Tool``. Shape (Circle / Rectangle / Door),
       radius / width / height, and door-open default come from the
       Tool Properties panel. **Left-click** raycasts to the ground
       plane and calls ``/obstacles/add_circle``,
       ``/obstacles/add_rectangle``, or ``/obstacles/add_door`` with
       the click position. **Right-click** calls ``/obstacles/list``,
       finds the nearest obstacle to the click, and calls
       ``/obstacles/remove`` with its id.
   * - ``DoorTool``
     - ``d``
     - Inherits ``rviz_common::Tool``. The door id comes from the Tool
       Properties panel. **Left-click** calls ``/doors/open``;
       **right-click** calls ``/doors/close``.

The ``PlaceObstacleTool`` and ``DoorTool`` need
``iros_llm_swarm_obstacles`` running — they are no-ops if the obstacle
manager is not in the graph.

ROS interfaces
--------------

**Action client (``SendLlmGoalTool``):**

* ``/llm/command`` (``iros_llm_swarm_interfaces/action/LlmCommand``)

**Service clients (``PlaceObstacleTool``):**

* ``/obstacles/add_circle`` (``AddCircle``)
* ``/obstacles/add_rectangle`` (``AddRectangle``)
* ``/obstacles/add_door`` (``AddDoor``)
* ``/obstacles/list`` (``ListObstacles``)
* ``/obstacles/remove`` (``RemoveObstacle``)

**Service clients (``DoorTool``):**

* ``/obstacles/list`` (``ListObstacles``) — to enumerate ids.
* ``/doors/open`` (``OpenDoor``)
* ``/doors/close`` (``CloseDoor``)

Source layout
-------------

* ``include/send_llm_goal_tool.hpp`` / ``src/send_llm_goal_tool.cpp``
* ``include/place_obstacle_tool.hpp`` / ``src/place_obstacle_tool.cpp``
* ``include/door_tool.hpp`` / ``src/door_tool.cpp``
* ``icons/send_llm_goal.svg`` — toolbar icon shared by the tools.

Build and registration
----------------------

The three classes are registered with pluginlib through
``plugin_description.xml`` (``iros_llm_rviz_tool/SendLlmGoalTool``,
``iros_llm_rviz_tool/PlaceObstacle``, ``iros_llm_rviz_tool/DoorControl``)
and exposed at install time via
``pluginlib_export_plugin_description_file(rviz_common
plugin_description.xml)``. Tools run on the RViz render / GUI thread;
service and action calls are made through the shared RViz node
abstraction. Icons install to
``share/iros_llm_rviz_tool/icons``.

Loading at runtime
------------------

In RViz2: **Tools panel → ``+`` → pick a tool from the list**, or use
the keyboard shortcut listed above when the render area has focus. The
bundled ``swarm_20.rviz`` config preloads all three.
