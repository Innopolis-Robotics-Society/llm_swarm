iros_llm_swarm_mapf_lns
=======================

LNS2 (Large Neighborhood Search v2) Multi-Agent Path Finding planner for
the ``iros_llm_swarm`` simulation, plus the per-robot follower that
executes its plans against Nav2.

The package exposes the same fleet-level contract as the legacy PBS
planner — ``/swarm/set_goals`` action, per-robot ``mapf_plan`` topics,
``follower_status`` feedback — so launch files and behavior trees can
swap the planner backend without further wiring.

.. toctree::
   :maxdepth: 2
   :hidden:

   algorithm
   ros_api

.. note::

   **Refactoring in progress.** ``mapf_lns2_node.cpp`` is currently a
   single ~2400-line file that interleaves the LNS2 control loop, the
   action server, schedule monitoring, life-cycle bookkeeping, and ROS
   plumbing. A refactor that decomposes it along the module boundaries
   already used by the planner code (``node_utils``, ``plan_publisher``,
   ``robot_lifecycle``, plus an extracted "mission" object) is planned.
   This document describes the **current behaviour and contract**;
   internals will move, but the ROS API will not.

Role in the stack
-----------------

Two executables are built from this package:

* ``mapf_lns2_node`` — fleet-level planner. Hosts the
  ``/swarm/set_goals`` action server, runs the LNS2 solver on the
  downsampled occupancy grid, publishes time-indexed plans on
  ``/robot_*/mapf_plan``, and monitors execution to trigger replans on
  deviation, stall, or follower failure.
* ``path_follower_node`` — per-robot follower (one instance per robot
  spawned by the launch file). Subscribes to its own ``mapf_plan``,
  drives Nav2's ``follow_path`` action segment by segment, and reports
  state on ``follower_status``. Also implements a PD-based formation
  follower mode driven by ``/formations/config``.

Both executables are bound by the contract laid out in
``iros_llm_swarm_interfaces`` (``MAPFPlan``, ``MAPFStep``,
``FollowerStatus``, ``SetGoals.action``).

A static library ``lns2`` is also installed and exported via
``ament_export_targets`` so other packages can link the pure-algorithm
solver without dragging in the ROS layer.

Mission flow
------------

The fleet-level lifecycle for one ``/swarm/set_goals`` call::

    Client                         mapf_lns2_node                    path_follower_node × N
       │                                  │                                    │
       ├─── SetGoals(robot_ids, goals) ──▶│                                    │
       │                                  │ build agents from current odom     │
       │                                  │ + footprints + map                 │
       │  feedback: validating            │                                    │
       │◀─────────────────────────────────│                                    │
       │  feedback: planning              │ LNS2: prioritized initial soln     │
       │◀─────────────────────────────────│ + destroy/repair to 0 collisions   │
       │                                  │                                    │
       │                                  │── MAPFPlan (per robot) ───────────▶│
       │  feedback: executing             │                                    │ FollowPath segments
       │◀─────────────────────────────────│  watch follower_status + odom      │ → Nav2
       │                                  │  → arrived/active/deviated counts  │
       │                                  │                                    │
       │                                  │  on deviation/stall: trigger       │
       │  feedback: replanning            │  warm-start (or cold) replan       │
       │◀─────────────────────────────────│  → publish new MAPFPlan(plan_id+1) │
       │                                  │                                    │
       │  result: success/failure         │  all robots arrived OR             │
       │◀─────────────────────────────────│  unplanable budget exhausted       │

Cancellation broadcasts an empty ``MAPFPlan`` and a zero ``Twist`` to
every active robot, then completes the action with
``error_code = CANCELLED``.

What's documented in this package
---------------------------------

* :doc:`algorithm` — LNS2 algorithm internals: types, soft A*, collision
  table, destroy operators, warm start, and the module layout under
  ``include/iros_llm_swarm_mapf/lns2/``.
* :doc:`ros_api` — full ROS contract: action server, topics, parameters
  for both ``mapf_lns2_node`` and ``path_follower_node``.

Type definitions consumed and produced by this package live in
``iros_llm_swarm_interfaces``; see its docs for field semantics.

Build
-----

The package depends on ``iros_llm_swarm_interfaces``. Build that first
when adding new types::

    colcon build --packages-select iros_llm_swarm_interfaces
    colcon build --packages-select iros_llm_swarm_mapf_lns

The pure-algorithm side (``lns2`` static library) has no ROS
dependencies and is exported by ``ament_export_targets(lns2Targets)``
for downstream consumers (e.g. unit tests, alternative front-ends).

Smoke and stress tests
----------------------

Two standalone executables under ``src/lns2/`` exercise the algorithm
without ROS:

* ``smoke_test.cpp`` — deterministic small-scenario sanity checks.
* ``stress_test.cpp`` — randomized fleet/horizon sweeps used during
  algorithm tuning.

These are not built by default by ``CMakeLists.txt`` — wire them in
manually when iterating on the solver.
