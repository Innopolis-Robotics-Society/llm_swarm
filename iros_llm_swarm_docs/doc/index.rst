IROS LLM Swarm Documentation
============================

A ROS 2 Humble research stack for orchestrating 20 differential-drive
robots in a 2D Stage simulation: fleet-level MAPF planning, per-robot
Nav2 navigation, leader-follower formations, a behavior-tree
front-end behind a single ``/swarm/set_goals`` action, and an LLM
advisory layer with three channels (reactive, proactive, operator
chat) plus an RViz operator panel and click-to-command tools.

Start with :doc:`overview` for a tour of what the project does and why,
then move on to :doc:`getting_started` to bring the stack up. Once the
demo is running, :doc:`examples` walks through the operator workflow —
talking to the LLM via the RViz panel, sending robots, formations,
dynamic obstacles, channel triggers, dataset inspection, and the
fallbacks. The :doc:`architecture` and :doc:`packages` pages are the
reference for how the pieces fit together.

Project overview
----------------

.. toctree::
   :maxdepth: 2

   overview
   getting_started
   examples
   architecture
   packages

Per-package documentation
-------------------------

The links below open each package's own rosdoc2-generated site, built
alongside this aggregator.

.. toctree::
   :maxdepth: 1

   iros_llm_swarm_bringup <../iros_llm_swarm_bringup/index.html#http://>
   iros_llm_swarm_bt <../iros_llm_swarm_bt/index.html#http://>
   iros_llm_swarm_costmap_plugins <../iros_llm_swarm_costmap_plugins/index.html#http://>
   iros_llm_swarm_formation <../iros_llm_swarm_formation/index.html#http://>
   iros_llm_swarm_interfaces <../iros_llm_swarm_interfaces/index.html#http://>
   iros_llm_swarm_local_nav <../iros_llm_swarm_local_nav/index.html#http://>
   iros_llm_swarm_mapf <../iros_llm_swarm_mapf/index.html#http://>
   iros_llm_swarm_mapf_lns <../iros_llm_swarm_mapf_lns/index.html#http://>
   iros_llm_swarm_obstacles <../iros_llm_swarm_obstacles/index.html#http://>
   iros_llm_swarm_robot <../iros_llm_swarm_robot/index.html#http://>
   iros_llm_swarm_simulation <../iros_llm_swarm_simulation/index.html#http://>
   iros_llm_swarm_simulation_lite <../iros_llm_swarm_simulation_lite/index.html#http://>
   iros_llm_orchestrator <../iros_llm_orchestrator/index.html#http://>
   iros_llm_rviz_panel <../iros_llm_rviz_panel/index.html#http://>
   iros_llm_rviz_tool <../iros_llm_rviz_tool/index.html#http://>
