Project Overview
================

**IROS LLM Swarm** is a ROS 2 Humble research stack for orchestrating a fleet
of 20 differential-drive robots in a 2D simulated environment. The project
combines a Multi-Agent Path Finding (MAPF) layer, the Nav2 local navigation
stack, leader-follower formation control, and a behavior tree front-end into
a single, reproducible simulation.

The codebase is built around a clean separation between *fleet-level*
decisions (where each robot should go and in what order) and *robot-level*
execution (how a single robot follows an assigned path or holds an offset
inside a formation). The same fleet-level action API — ``/swarm/set_goals``
— is shared by two interchangeable MAPF backends, so the higher-level
behavior tree, scripted tests, or future LLM agents do not need to know
which planner is currently running.

What the project does
---------------------

* Runs **20 differential-drive robots** in a 2D Stage simulator, each with
  its own Nav2 stack (controller, behavior tree, costmaps).
* Plans **conflict-free, time-indexed paths** for the whole fleet through
  one of two MAPF planners:

  * **PBS** (Priority-Based Search) with gradient inflation and
    capsule-based conflict detection — the original baseline.
  * **LNS2** (Large Neighborhood Search) — a more scalable alternative
    sharing the same action interface.

* Coordinates **leader-follower formations** with a centralized
  configuration topic and a per-robot dual-mode motion controller that
  switches between *autonomous* (Nav2 path-following) and *formation*
  (PD offset tracking) modes without restarting nodes.
* Exposes a long-lived **MAPF action** that covers the full
  ``validate → plan → execute → replan → succeed`` lifecycle, so external
  clients (behavior trees, LLM agents, test scripts) can treat fleet
  navigation as one atomic operation.
* Provides a small set of **BehaviorTree.CPP** action nodes that wrap the
  MAPF and formation interfaces, ready to be composed into higher-level
  mission trees.
* Ships an **LLM advisory layer** (``iros_llm_orchestrator``) with three
  independent channels — reactive (``/llm/decision`` action called from
  BT nodes on WARN/ERROR), proactive (``passive_observer`` watches
  ``/bt/state`` and pushes commands), and an operator chat
  (``/llm/chat`` driven from the RViz panel, with a remediation loop and
  optional read-only MCP context).
* Adds an **operator surface in RViz** (``iros_llm_rviz_panel`` plus the
  click-to-command tools in ``iros_llm_rviz_tool``) for chat,
  per-robot status, BT state, event log, STOP-ALL, and direct
  goal / obstacle / door manipulation.
* Includes a **dynamic obstacle layer** (``iros_llm_swarm_obstacles``)
  that overlays runtime-added circles, rectangles, and stateful doors
  on top of the static map and republishes the merged grid on ``/map``.

Design goals
------------

* **Single fleet-level API.** All planners and front-ends agree on
  ``/swarm/set_goals``; backends can be swapped without touching the rest
  of the system.
* **Reproducible by default.** Everything runs inside Docker with a fixed
  CycloneDDS configuration tuned for multi-robot discovery overhead.
* **Safe primitives, simple glue.** Conflict detection, inflation, and
  schedule monitoring live inside the planner. The behavior tree only
  decides *which* mission to run, not *how* to keep robots from colliding.
* **Pluggable LLM reasoning.** The current release ships
  ``iros_llm_orchestrator`` with three channels (reactive, proactive,
  operator chat) and a backend factory that selects between a mock
  client, a local Ollama instance, an OpenAI-compatible HTTP endpoint,
  and an in-process HuggingFace Transformers backend without changing
  any node code. The LLM never executes outside the BT/Nav2 primitives —
  it produces decisions and plan JSON which are then dispatched through
  the same ``/swarm/set_goals`` and ``/llm/command`` interfaces used by
  the rest of the stack.

Scope and known limits
----------------------

* **20-robot ceiling.** DDS discovery overhead with the default RMW
  becomes the dominant cost above this number; the included
  ``cyclonedds_swarm.xml`` profile is required even for 20 robots.
* **Static localization only.** AMCL is intentionally disabled because
  cross-robot laser interference at this density makes it drift.
  Localization is replaced by a static ``map → robot_N/odom`` transform
  per robot.
* **Stage, not Gazebo.** The 2D Stage simulator
  (``iros_llm_swarm_simulation_lite``) is the supported simulation
  backend. A 3D Gazebo Harmonic package
  (``iros_llm_swarm_simulation``) exists for future work but is not yet
  stable with 20 robots and Nav2.

Roadmap (high level)
--------------------

The current release covers MAPF (both PBS and LNS2), formation control,
the long-lived action lifecycle, and Stage-based simulation. Upcoming
work includes deeper BT integration, smarter MAPF conflict resolution,
formation-aware planning, fault tolerance under communication loss, and
a transition to a stable 3D Gazebo simulation. See the project ``README``
for the full task list.

Where to go next
----------------

* :doc:`getting_started` — bring up the stack inside Docker and run the
  first mission.
* :doc:`architecture` — system layers, topics, actions, and the lifecycle
  of a fleet mission.
* :doc:`packages` — per-package roles and entry points; jump from there
  into the per-package generated documentation.
