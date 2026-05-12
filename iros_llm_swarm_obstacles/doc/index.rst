iros_llm_swarm_obstacles
========================

Dynamic-obstacle layer for the swarm. The single C++ node
``dynamic_obstacle_manager`` overlays runtime-added circles,
rectangles, and stateful doors onto the static map produced by
``map_server`` / ``zone_map_server`` and republishes the merged grid on
``/map`` with ``TRANSIENT_LOCAL`` QoS — so any planner or Nav2 costmap
that joins late automatically receives the latest state.

The package is **not yet wired into the bringup launches**. To use it,
either compose it into a custom launch file or run it directly with the
parameters described below; the click-to-edit tools in
``iros_llm_rviz_tool`` already speak its service API.

Node
----

.. list-table::
   :header-rows: 1
   :widths: 28 28 44

   * - Executable
     - Class
     - Role
   * - ``dynamic_obstacle_manager``
     - ``DynamicObstacleManager``
     - Maintains the obstacle list, merges into the base map, hosts the
       ``/obstacles/*`` and ``/doors/*`` services.

ROS interfaces
--------------

**Publishers:**

* ``/map`` (``nav_msgs/OccupancyGrid``, ``TRANSIENT_LOCAL``, depth 1) —
  merged occupancy grid (base + obstacles).
* ``/obstacles/markers`` (``visualization_msgs/MarkerArray``,
  ``TRANSIENT_LOCAL``, depth 1) — RViz visualisation. Circles render as
  ``CYLINDER``, rectangles as ``CUBE``, doors as ``CUBE`` with colour
  encoding state (red = closed, green = open).

**Subscriber:**

* ``/raw_map`` (``nav_msgs/OccupancyGrid``, ``TRANSIENT_LOCAL``) — the
  static base map. The topic name is configurable via
  ``raw_map_topic``.

**Services:**

.. list-table::
   :header-rows: 1
   :widths: 32 32 36

   * - Service
     - Type
     - Effect
   * - ``/obstacles/add_circle``
     - ``AddCircle``
     - Add a circular obstacle (id, position, radius).
   * - ``/obstacles/add_rectangle``
     - ``AddRectangle``
     - Add a rectangular obstacle (id, position, width, height).
   * - ``/obstacles/add_door``
     - ``AddDoor``
     - Add a door (id, position, width, height, ``is_open``).
   * - ``/obstacles/remove``
     - ``RemoveObstacle``
     - Remove any obstacle by id.
   * - ``/obstacles/list``
     - ``ListObstacles``
     - Read all current obstacles (circles, rectangles, doors).
   * - ``/doors/open``
     - ``OpenDoor``
     - Set door ``is_open=true`` (does not block cells).
   * - ``/doors/close``
     - ``CloseDoor``
     - Set door ``is_open=false`` (blocks cells).

Parameters
----------

.. list-table::
   :header-rows: 1
   :widths: 22 22 56

   * - Parameter
     - Default
     - Purpose
   * - ``raw_map_topic``
     - ``/raw_map``
     - Topic for the base static map.
   * - ``scenario_file``
     - empty
     - Path to a YAML file defining one or more scenarios. If empty, no
       scenario is loaded.
   * - ``scenario_name``
     - empty
     - Name of the scenario inside ``scenario_file`` (e.g. ``cave``,
       ``warehouse_4``). If empty, no scenario is loaded.

Scenario YAML schema::

    scenarios:
      <scenario_name>:
        obstacles:
          circles:
            - { id: c1, x: 5.0, y: 7.0, radius: 0.4 }
          rectangles:
            - { id: r1, x: 12.0, y: 4.5, width: 1.0, height: 2.0 }
          doors:
            - { id: d1, x: 9.0, y: 6.0, width: 1.5, height: 0.2,
                default_open: true }

``default_open`` is optional and defaults to ``true`` when omitted.

Algorithms
----------

* **Circle blocking.** Iterate cells inside the AABB of the circle;
  mark a cell occupied when the world-frame distance from the cell
  centre to the circle centre is ≤ radius.
* **Rectangle blocking.** Clip the AABB to the grid bounds; mark every
  cell in the resulting range as ``100``.
* **Door state.** Closed doors block their cells; open doors do not.
* **Marker lifecycle.** A ``DELETEALL`` marker is published before each
  refresh so RViz drops stale geometry.
* **Scenario load.** YAML root → ``scenarios`` → ``<scenario_name>`` →
  ``obstacles`` → ``{circles[], rectangles[], doors[]}``.

Integration notes
-----------------

The manager publishes on the same ``/map`` topic as ``map_server`` and
``zone_map_server`` with the same QoS; only one of those should be
active for ``/map`` at a time. The intended pattern is:

1. ``map_server`` (or ``zone_map_server``) publishes the static base on
   ``/raw_map`` (or ``/map_server/map``) instead of ``/map``.
2. ``dynamic_obstacle_manager`` subscribes to that topic and publishes
   the merged grid on ``/map``.
3. Nav2 costmaps and the MAPF planners continue to subscribe to
   ``/map`` and pick up dynamic obstacles transparently.

Source layout
-------------

* ``src/dynamic_obstacle_manager.cpp`` — single-file implementation:
  map ingestion, cell blocking for circles and rectangles, marker
  publication, the seven service handlers, and YAML scenario loading.
