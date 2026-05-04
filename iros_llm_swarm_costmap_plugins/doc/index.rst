iros_llm_swarm_costmap_plugins
==============================

A single-class Nav2 costmap plugin: ``ResettingObstacleLayer``. Drop-in
replacement for ``nav2_costmap_2d::ObstacleLayer`` that wipes its
internal grid every cycle before re-marking observations. Used by the
swarm's per-robot local costmaps to eliminate ghost-obstacle trails
left by other moving robots.

.. contents::
   :local:
   :depth: 2

Why this plugin exists
----------------------

The default ``ObstacleLayer`` clears costmap cells via **raytracing**:
for each laser ray that does not hit an obstacle, every cell it passes
through is marked free. Cells that fall **between** consecutive rays
of a sparse 2D scanner are never traversed by any ray, and rely on
some *other* ray either marking them or clearing them on a future
scan.

In a single-robot scenario this is rarely a problem — the robot moves
relative to static obstacles, so cells that were marked occupied by a
ray on one scan are usually re-cleared by an adjacent ray on the next
scan. With **multi-robot** scans, however, each robot's neighbours are
*moving* obstacles whose cells get marked, then never cleared because
the sparse rays do not pass through the exact cell the neighbour just
vacated. Over a few seconds the local costmap builds up "ghost" trails
behind every other robot, the local planner steers around obstacles
that are not there, and the controller misbehaves.

This plugin sidesteps the problem by **discarding the entire previous
costmap state** before every ``updateBounds()`` call. The next
mark-from-scan pass then re-creates a costmap that reflects only what
is observed *right now* — no leftover state, no ghosts.

Implementation
--------------

The whole plugin is a 15-line subclass:

.. code-block:: cpp

   namespace iros_llm_swarm_costmap_plugins {

   class ResettingObstacleLayer : public nav2_costmap_2d::ObstacleLayer {
   public:
     void updateBounds(
       double robot_x, double robot_y, double robot_yaw,
       double * min_x, double * min_y, double * max_x, double * max_y) override
     {
       resetMaps();
       ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw,
                                    min_x, min_y, max_x, max_y);
     }
   };

   }  // namespace iros_llm_swarm_costmap_plugins

   PLUGINLIB_EXPORT_CLASS(
     iros_llm_swarm_costmap_plugins::ResettingObstacleLayer,
     nav2_costmap_2d::Layer)

What ``resetMaps()`` does, inherited from ``Costmap2D``: walks the
internal ``costmap_`` byte array and writes ``NO_INFORMATION`` (255)
to every cell. The base ``ObstacleLayer::updateBounds()`` then runs
its normal mark + raytrace passes against the latest observation
buffer, producing a costmap that has only the obstacles currently
visible to this robot's scanner.

Pluginlib registration
~~~~~~~~~~~~~~~~~~~~~~

The plugin is registered with the ``nav2_costmap_2d::Layer`` base type
in ``resetting_obstacle_layer.xml``:

.. code-block:: xml

   <library path="resetting_obstacle_layer">
     <class name="iros_llm_swarm_costmap_plugins/ResettingObstacleLayer"
            type="iros_llm_swarm_costmap_plugins::ResettingObstacleLayer"
            base_class_type="nav2_costmap_2d::Layer">
       <description>ObstacleLayer that resets its internal grid every cycle before marking</description>
     </class>
   </library>

The exported class name is the **string** Nav2 looks for in the
costmap ``plugin:`` field (see usage below). The ``library path``
matches the shared library installed by ``CMakeLists.txt``
(``libresetting_obstacle_layer.so``).

How to use it
-------------

In any costmap section (local or global), set the layer's
``plugin`` to the exported class name. The plugin behaves exactly
like the standard ``ObstacleLayer`` for every other parameter — the
override is invisible to configuration:

.. code-block:: yaml

   local_costmap:
     local_costmap:
       ros__parameters:
         plugins: ["obstacle_layer", "inflation_layer"]
         obstacle_layer:
           plugin: "iros_llm_swarm_costmap_plugins/ResettingObstacleLayer"
           enabled: true
           observation_persistence: 0.0   # see note below
           clearing: true
           marking: true
           observation_sources: base_scan
           base_scan:
             topic: "/<robot_namespace>/base_scan"
             data_type: "LaserScan"
             obstacle_max_range: 4.0
             raytrace_max_range:  5.0
             max_obstacle_height: 2.0

The swarm's own per-robot local costmap config in
``iros_llm_swarm_local_nav/config/robot_nav2_params.yaml`` uses the
plugin exactly this way.

Considerations and limits
-------------------------

* **Marking-only behaviour.** Because the costmap is wiped before every
  update, ``observation_persistence`` becomes effectively meaningless —
  every cell that is not re-marked by the current scan disappears
  immediately. Setting ``observation_persistence: 0.0`` makes this
  intent explicit in the config.
* **Local costmap only, in practice.** This plugin is designed for the
  *rolling-window local costmap*, where the wipe-and-remark cost is
  bounded by the small window size. Wiping a full-map costmap every
  cycle is much more expensive and removes the only memory the
  costmap had of unobserved obstacles — usually not what you want for
  a global costmap. The swarm's global costmap uses the standard
  ``StaticLayer`` + ``InflationLayer`` instead.
* **Non-laser sources.** The reset happens before *all* observation
  sources are processed, regardless of type. If you mix laser and
  pointcloud sources on the same layer, every pointcloud mark must
  also re-occur every cycle to remain visible.
* **Plugin ABI.** ``resetMaps()`` is a public method on
  ``Costmap2D``; ``updateBounds`` is virtual on ``Layer``. Both are
  stable Nav2 surfaces. If a future Nav2 release renames or
  re-shapes them, this plugin will need a small port.

Build and install
-----------------

A single shared library, no headers exported::

    colcon build --packages-select iros_llm_swarm_costmap_plugins

Built artefacts:

* ``libresetting_obstacle_layer.so`` — the shared library.
* ``share/iros_llm_swarm_costmap_plugins/resetting_obstacle_layer.xml``
  — the pluginlib description (auto-discovered by Nav2 via
  ``pluginlib_export_plugin_description_file``).

Sourcing the workspace's ``setup.bash`` exposes the plugin to any
Nav2 instance running in that environment.

Cross-references
----------------

* Producer of the YAML that loads this plugin:
  ``iros_llm_swarm_local_nav`` (per-robot Nav2 launch + config).
* Upstream class documentation: ``nav2_costmap_2d::ObstacleLayer``.
