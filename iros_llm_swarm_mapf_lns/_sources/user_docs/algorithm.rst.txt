LNS2 Algorithm
==============

The ``lns2`` static library under ``include/iros_llm_swarm_mapf/lns2/``
implements the algorithm side of the planner: pure C++17, no ROS
dependencies. The ROS node (:doc:`ros_api`) wraps it but does not poke
into its data structures directly.

This page describes the **algorithm as currently shipped**. It is the
component most likely to change during the upcoming refactor; treat
function names and small data layout details as advisory rather than
load-bearing.

.. contents::
   :local:
   :depth: 2

Big picture
-----------

LNS2 (`Li, Chan, Ma, Koenig 2022`) is a metaheuristic for MAPF that
treats path-finding as a repair problem: build an initial joint plan
that may contain collisions, then iteratively *destroy* a small subset
of agent paths and *repair* them with a single-agent solver until the
plan is collision-free (or a budget is exhausted).

The implementation has three pillars:

#. **Soft A*** — a single-agent space-time A* whose edge cost mixes
   path length with a *soft* collision count derived from the current
   joint plan. Penalty weight is set high enough that the lex order is
   ``(collisions, path_length)``: minimizing the soft cost minimizes
   collisions first, length second.

#. **Collision table** — footprint-aware vertex + edge occupancy index
   with O(1) lookups and incremental updates. The destroy/repair loop
   only re-plans a few agents per iteration; the table makes
   "how many agents currently sit in this cell at this timestep?" cheap.

#. **Adaptive Large Neighborhood Search (ALNS)** — four destroy
   operators compete via segment-based weight updates. A reward is
   recorded for each operator on each iteration; periodically the
   weights are updated by an EMA toward the average observed reward.

Module layout
-------------

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - File
     - Purpose
   * - ``types.hpp``
     - ``Cell``, ``CellOffset``, ``GridMap``, ``Path``,
       ``FootprintModel``, ``Agent``, ``Collision``. Scalar aliases
       (``AgentId``, ``Timestep``, ``CellIdx``, ``Cost``).
   * - ``gridmap_utils.hpp``
     - Footprint-aware static reachability (BFS) and
       ``find_nearest_free_cell``.
   * - ``soft_astar.hpp``
     - ``SoftAStarParams``, ``SoftAStarResult``, ``HeuristicCache``,
       and the ``soft_astar()`` function.
   * - ``collision_table.hpp``
     - ``CollisionTable`` — vertex + edge occupancy index with
       incremental ``add_path`` / ``remove_path`` and O(1) count
       lookups.
   * - ``solution.hpp``
     - ``Solution`` — paths + collision table + per-agent
       ``hold_until``, with ``Snapshot`` / ``restore`` for rollback.
   * - ``destroy_operators.hpp``
     - ``DestroyOp`` enum, four operator classes
       (CollisionBased, RandomWalk, Random, Bottleneck), and
       ``ALNSWeights`` (segment EMA reaction).
   * - ``warm_start.hpp``
     - ``build_warm_seed`` — constructs a ``Solution`` from the
       previous plan shifted forward in time, plus
       ``should_warm_start`` decision heuristic.
   * - ``lns2_solver.hpp``
     - ``LNS2Solver`` — top-level entry point with ``solve()`` (cold
       start) and ``solve_from()`` (warm start).

Types
-----

``Agent`` is the input shape used by the solver:

.. code-block:: cpp

   struct Agent {
     AgentId        id;          // dense index in [0, N)
     Cell           start;
     Cell           goal;
     FootprintModel footprint;   // owned per-agent; circular envelope
   };

The contract is that ``id`` equals the agent's position in the
``std::vector<Agent>`` passed to the solver. Sparse external IDs (e.g.
``robot_12`` and ``robot_17`` out of a fleet of 20) are remapped to a
dense range by the caller; the ROS node keeps the external ↔ internal
mapping at the call site (``planned_ext_ids`` in ``node_utils``).

``Path`` is a ``std::vector<Cell>`` indexed by discrete timestep:
``path[k]`` is the agent's cell at ``t = k``. After ``path.size() - 1``
the agent is assumed to remain at its goal — the *hold-at-goal* tail —
and ``Solution`` tracks a ``hold_until_`` timestep to which that tail
extends in the collision table.

``FootprintModel`` is a heading-independent set of cell offsets
relative to base_link. Built once per agent via ``from_radius``; the
solver never rotates or rebuilds it during a search.

Soft A*
-------

``soft_astar()`` is the per-agent inner solver. State is ``(cell, t)``;
edge cost is

.. code-block:: text

   step_cost
     + collision_penalty * (# other agents occupying next footprint cells at t+1)
     + collision_penalty * (# other agents swap-traversing the edge at t)

The heuristic ``h`` is the true distance on the static graph,
precomputed by a backward BFS from the goal and cached per
``(goal, diagonal)`` key in ``HeuristicCache``. The cache invalidates
itself if a different ``GridMap`` pointer is passed than last time —
sufficient for the solver, but callers that mutate ``blocked`` cells in
place must call ``HeuristicCache::clear()`` themselves.

Termination is when ``f_min`` over OPEN ≥ best known total cost
(``g + tail``); failure is exhaustion of either the timestep horizon
or the per-call expansion cap.

Parameters (``SoftAStarParams``):

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Field
     - Default
     - Meaning
   * - ``step_cost``
     - 1
     - Cost of one move (including ``stay``).
   * - ``collision_penalty``
     - 10000
     - Cost per collision unit. Must satisfy
       ``penalty > horizon × step_cost`` so the lex order is
       ``(collisions, length)``.
   * - ``horizon``
     - 300
     - Hard upper bound on ``t`` — the search fails if it cannot land
       at the goal within this horizon.
   * - ``max_expansions``
     - 200000
     - Per-call expansion cap. Prevents pathological searches from
       blocking the loop forever.
   * - ``diagonal_moves``
     - ``false``
     - 4-connected by default; diagonals are MAPF-unsafe in dense
       fleets.

Collision table
---------------

``CollisionTable`` is a footprint-aware vertex + edge occupancy index:

* **Vertex occupancy**: ``vertex_occ_[(cell, t)] -> {agents}``. Every
  footprint cell of every agent at every timestep up to its
  ``hold_until`` contributes a record.
* **Edge occupancy**: ``edge_occ_[(from, to, t)] -> {agents}``. Each
  agent step at time ``t`` writes one entry. Swap detection asks
  "who's traversing ``(to, from, t)``?" — if anyone is, both have an
  edge collision.

Counters are kept in lockstep with the maps:

* ``total_collisions()`` — sum over unordered agent pairs of shared
  ``(cell, t)`` and swap edges.
* ``per_agent_collisions[i]`` — sum of pair collisions involving ``i``.
  ``sum(per_agent) == 2 × total_collisions()``.
* ``touches_[i]`` — superset of agents that currently share at least
  one entry with ``i``. May over-approximate after removals;
  ``rebuild_touches()`` resets it.

The table is updated incrementally by ``add_path`` /
``remove_path`` calls funneled through ``Solution``. ``Solution`` owns
the only mutating path; direct mutation outside that wrapper would
desync ``per_agent_`` and the maps.

For sampling, ``sample_random_collision()`` returns a uniformly chosen
``(a, b, cell, t, kind)`` tuple to seed the destroy step.

Destroy operators
-----------------

Four operators implement the ``DestroyOperator`` interface and live
under ``destroy_operators.hpp``:

* **CollisionBased** — sample a random collision, take its two agents,
  expand outward by ``touches_`` until the requested neighborhood size
  is reached.
* **RandomWalk** — pick a random agent and walk along its
  conflict-graph neighbours.
* **Random** — uniform random subset.
* **Bottleneck** — bias toward agents with the highest collision count.

Selection between operators is adaptive (ALNS):

* ``ALNSWeights::sample()`` picks an operator proportionally to its
  current weight.
* ``ALNSWeights::record(op, reward)`` accumulates rewards. Reward
  constants are:

  .. code-block:: text

     R_IMPROVE_BEST  = 10.0   // better than any previous solution
     R_IMPROVE_LOCAL = 5.0    // better than before this iteration
     R_NO_CHANGE     = 1.0    // rollback

* ``flush_segment(reaction, w_min)`` is called every ``segment_size``
  iterations and applies an EMA update toward each operator's average
  reward. Operators **not used** in the segment decay slowly toward
  ``w_min`` so a never-selected operator does not stay frozen at a
  high initial weight — and a stuck-at-floor one is not trapped there
  forever.

Solution wrapper
----------------

``Solution`` ties paths, footprints, hold-at-goal timesteps, and the
collision table into a single object:

* The only mutators are ``set_path()`` and ``clear_path()``; both
  funnel through the table to keep counts consistent.
* ``set_global_hold(h)`` adjusts ``hold_until`` for *all* agents to a
  shared horizon (typically ``max(path.size()) - 1`` across the
  current repair set, or the full horizon after a successful repair).
* ``take_and_clear(ids)`` produces a ``Snapshot`` and clears the named
  agents; ``restore(snap)`` undoes it. This is the rollback primitive
  used when an iteration produces a worse solution than its starting
  point.

Top-level solver loop
---------------------

``LNS2Solver`` exposes two entry points:

* ``solve()`` — cold start. Builds an initial solution greedily
  (agents sorted by ``h``-distance to goal; each one planned ignoring
  unplanned siblings, then committed to the table), then runs the
  repair loop.
* ``solve_from(seed, …)`` — warm start. Takes a pre-built ``Solution``
  (typically produced by ``build_warm_seed``) and runs the repair loop
  directly.

The repair loop iterates until any of:

* ``time_budget_ms`` wall time exhausted,
* ``plateau_limit`` iterations with no improvement,
* the solution becomes collision-free.

One iteration:

#. Sample destroy operator from ``ALNSWeights``.
#. Operator returns ``neighborhood_size`` agent IDs.
#. ``Solution::take_and_clear`` snapshots and clears them.
#. For each agent (in random order), call ``soft_astar`` against the
   current table; commit the new path with ``set_path``.
#. Compare collision count to the snapshot. Accept (keep new paths and
   ``record(op, +reward)``) or rollback (``restore(snap)`` and
   ``record(op, R_NO_CHANGE)``).

Diagnostics live in ``LNS2Stats``: total iterations, accepted
iterations, A* call counts and total expansions, per-operator usage
and improvement counts, final operator weights, wall time.

Initial-solution time cap
~~~~~~~~~~~~~~~~~~~~~~~~~

``LNS2Params::initial_time_budget_ms`` caps the wall-clock cost of the
prioritized greedy construction phase. When the budget is exhausted the
loop stops early; remaining agents get no initial path and the repair
loop handles them as stubs. Set to 0 to disable. Used by the ROS node
to keep ``execute_goal`` latency bounded under dense start
configurations.

Warm start
----------

``build_warm_seed`` produces a ``Solution`` that the solver can repair
in place, starting from the last successful plan shifted forward in
time. The current implementation applies a **common** time shift
``delta_steps`` to every agent (this fixes the older per-agent k-shift
bug, which destroyed the relative timing of the schedule whenever one
robot fell behind). For each agent it then classifies the seed:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - ``SeedStatus``
     - Description
   * - ``OnTrack``
     - Actual cell == expected cell at ``t = delta``; tail of the
       previous path is reused directly.
   * - ``Spliced``
     - Actual near expected; a short BFS splice (≤
       ``patch_radius_cells`` Chebyshev) joins ``actual_cell`` back
       onto the tail.
   * - ``StubFar``
     - Actual far from expected; one-cell hold ("frozen"). Repair must
       re-plan it.
   * - ``StubNew``
     - Agent was not in the previous plan; one-cell hold at actual.
   * - ``StubGoal``
     - Goal changed since the previous plan; one-cell hold at actual.
   * - ``StubInvalid``
     - Previous path no longer statically valid (e.g. footprint no
       longer fits because the map changed); one-cell hold.
   * - ``StubArrived``
     - Previous plan fully elapsed and the robot is at its goal.
   * - ``Missing``
     - No odometry; agent left out of the repair set.

``WarmStartReport`` aggregates these counts and computes seed
collisions. ``should_warm_start()`` decides whether the seed is worth
running ``solve_from`` on (vs. paying for a cold start) using two
thresholds:

* ``max_collisions_per_agent`` — per-agent collision ceiling on the
  seed.
* ``max_stubs_ratio`` — fraction of stubs the repair must actively
  resolve.

If either threshold is exceeded, the ROS node falls back to ``solve()``.

Cross-references
----------------

* ROS-side parameter mapping (``time_budget_ms`` ↔ params YAML): see
  :doc:`ros_api`.
* Type definitions (``Path``, ``Cell``, ``Agent``) are owned by this
  algorithm library; they are **not** the ROS message types. The ROS
  side converts between grid cells and world coordinates via
  ``world_to_cell`` / ``cell_to_world`` in ``node_utils.hpp``.
