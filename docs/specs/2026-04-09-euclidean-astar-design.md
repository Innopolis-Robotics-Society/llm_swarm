# Euclidean A* with Multi-Cell Movement — Design Spec

## Problem

The current PBS/A* solver uses 4-connected (cardinal) movement: robots move one cell per timestep in N/S/E/W directions only. This produces Manhattan-distance paths (longer, less natural) and requires many timesteps to cover distance (large state space, 25-45s planning time for 20 robots).

## Goals

- Shorter, more natural paths via diagonal and multi-cell movement
- Smaller state space (fewer timesteps) for faster planning
- Connectivity derived automatically from physical parameters (speed, timestep, resolution)
- Correct conflict detection for arbitrary move distances
- Backward-compatible: old planner remains available via config switch

## Non-Goals

- Continuous-time planning (CCBS)
- Optimal solutions (PBS is already suboptimal; flexibility > optimality)
- Non-circular footprints

## Design

### Section 1: Refactor — Split `pbs_solver.hpp`

Extract the existing single header into three files with no behavioral changes:

| Header | Contents |
|--------|----------|
| `mapf_types.hpp` | `Cell`, `Path`, `GridMap`, `MoveSet` (initially empty), `ReservationTable`, constants (`INF`, etc.), capsule geometry functions |
| `space_time_astar.hpp` | Existing `SpaceTimeAStarPlanner` — unchanged, includes `mapf_types.hpp` |
| `pbs_solver.hpp` | `ConflictDetector`, `PriorityGraph`, `PBSSolver` — includes planner header (old or new) |

Purpose: establish clean boundaries before modifying internals. Everything compiles and works identically after this step.

### Section 2: New A* Planner — `EuclideanAStarPlanner`

New file `euclidean_astar.hpp`, coexists with the old planner.

#### Move Table Generation

```
struct Move { int dr; int dc; float cost; };

MoveSet::generate(max_speed, time_step_sec, resolution):
  reach = max(1, floor(max_speed * time_step_sec / resolution))
  moves = [{0, 0, 0.0}]  // wait always present
  for dr in [-reach, reach], dc in [-reach, reach]:
    if dr*dr + dc*dc <= reach*reach and (dr, dc) != (0, 0):
      moves.append({dr, dc, sqrt(dr*dr + dc*dc)})
```

- `reach=1` -> 9 moves (4 cardinal + 4 diagonal + wait)
- `reach=2` -> ~13 moves
- `reach=3` -> ~29 moves
- Minimum reach of 1 guarantees at least 8-connected + wait

The planner constructs its `MoveSet` internally from `max_speed`, `time_step_sec`, `resolution` passed at construction.

#### Cost Model

Float g-values replace int. Cost per move:

```
g_new = g_current + time_cost + move.cost + penalties
```

- `time_cost`: base cost per timestep (constant, e.g. 1.0)
- `move.cost`: Euclidean distance in cells (0.0 for wait, 1.0 for cardinal, ~1.41 for diagonal, etc.)
- `penalties`: wall gradient (sum along trace) + agent proximity (from reservations)

Waiting costs only `time_cost`. Moving always costs more. Preserves "prefer waiting over unnecessary movement."

#### Heuristic — Dijkstra (replaces BFS)

Backward Dijkstra from each goal using the same `MoveSet`. Produces correct minimum-cost distances for any connectivity. Cached per unique goal cell.

With 4-connected integer costs, Dijkstra degenerates to BFS — strict generalization.

Slightly more expensive than BFS (priority queue vs FIFO), but one-time cost per goal on the downsampled PBS grid.

#### Line-of-Sight Trace for Moves

Each move is validated by tracing the center-line through the grid (DDA/Bresenham).

- **Blocked check**: if any cell on the center-line trace is hard-blocked on the agent's inflated map, the move is invalid
- **Wall gradient penalty**: **sum** of `wall_cost` across all cells in the center-line trace. At each cell, `wall_cost` is already the max of overlapping wall gradients (computed during inflation). Sum penalizes longer traversals through gradient zones proportionally.

The per-agent inflated map already bakes in the footprint radius (cells within `footprint_radius` of walls are hard-blocked), so center-line checks against the inflated map are correct for the robot's body. Between cell centers, the worst-case deviation (~0.7 cells at half diagonal) is covered by the inflation margin — no thickening needed as long as `footprint_radius >= 0.7 * pbs_resolution`. The planner must assert this at startup and reject configurations that violate it.

### Section 3: Conflict Detection — Capsule Overlap

Replaces the vertex/edge conflict distinction with a single capsule overlap test. A capsule is the swept volume of a circular robot along its movement segment.

#### Core function

```
capsule_overlap(seg_a, radius_a, seg_b, radius_b) -> bool:
  min_dist = segment_segment_distance(seg_a.from, seg_a.to, seg_b.from, seg_b.to)
  return min_dist < radius_a + radius_b
```

`segment_segment_distance`: closed-form closest approach between two line segments (~15 lines, no iteration). When a segment degenerates to a point (wait move), this reduces to point-segment or point-point distance.

#### Used in two places

**A. PBS-level `ConflictDetector`** — after pathfinding, checks all agent pairs per timestep. Returns `std::optional<Conflict>` where `Conflict = {agent1, agent2, time}`. The `ConflictType` enum is removed.

**B. A*-level reservation checks in `relax()`** — during pathfinding, checks candidate move segment against reserved segments. Replaces `vertex_penalty()` and `is_edge_blocked()`:
- Hard reject: `min_dist < combined_footprint_radius`
- Soft gradient penalty: applied when `min_dist` falls between combined footprint and combined inflation radius, using the existing gradient curve

Lives in `mapf_types.hpp` since both A* and PBS use it.

#### Grace period

Unchanged in concept. Overlapping-start agents skip capsule checks for the grace timesteps, giving them time to separate.

### Section 4: Reservation Table

Entries change from point reservations to segment reservations:

```
struct Reservation {
  size_t agent_id;
  size_t time;
  Cell from;
  Cell to;
  float radius;
};
```

Indexed by timestep only. At each `t`, a flat list of reserved segments. `relax()` iterates all reservations at the relevant timestep and checks capsule overlap.

With 20 agents, this is at most 20 capsule checks per `relax()` call — fast enough without a spatial index.

Wait moves: `from == to`, capsule degenerates to a circle — identical to current point reservation behavior.

### Section 5: PBS Solver Changes

Minimal changes to the PBS orchestration layer:

- **Conflict detection**: `ConflictDetector::find_first()` uses capsule overlap. Returns `std::optional<Conflict>`.
- **Planner selection**: imports `euclidean_astar.hpp` or `space_time_astar.hpp` based on `planner_type` parameter.
- **Planner construction**: passes `max_speed`, `time_step_sec`, `resolution` to the planner. Old planner ignores extra params. New planner builds `MoveSet` internally.
- **`max_t` adaptation**: uses Dijkstra heuristic distances instead of BFS step count. Formula: `max_dist_dijkstra * 2 + 32` (tunable).

Everything else untouched: priority graph, branching, DAG cycle detection, branch ordering heuristic, sequential root planning, `solve()` loop.

### Section 6: Configuration

| Parameter | Default | Notes |
|-----------|---------|-------|
| `max_speed` | 0.5 m/s | Existing robot spec. Combined with `time_step_sec` and `pbs_resolution` to compute reach |
| `time_step_sec` | 0.4s | Existing. Increase to grow reach and shrink state space |
| `pbs_resolution` | 0.2m | Existing. Unchanged |
| `planner_type` | `"euclidean"` | New. `"euclidean"` or `"classic"` — selects planner implementation |

Single-knob tuning: adjust `time_step_sec` to trade temporal resolution for state-space size. `max_speed` caps reach physically. Minimum reach of 1 cell guaranteed.

**Constraint**: `footprint_radius >= 0.7 * pbs_resolution` must hold for wall-clipping safety. The planner asserts this at startup. With current defaults (0.22m footprint, 0.2m resolution) the constraint is satisfied. Increasing `pbs_resolution` beyond `footprint_radius / 0.7` requires a proportionally larger footprint or finer resolution.

**Documentation**: this constraint must be added to the README MAPF section during implementation.

## File Summary

| File | Action |
|------|--------|
| `include/.../mapf_types.hpp` | New — extracted shared types + capsule geometry |
| `include/.../space_time_astar.hpp` | New — existing A* planner moved here, unchanged |
| `include/.../euclidean_astar.hpp` | New — new planner with move table, float costs, Dijkstra heuristic |
| `include/.../pbs_solver.hpp` | Modified — imports split headers, updated conflict detector, planner selection |
| `src/mapf_planner_node.cpp` | Modified — new `planner_type` parameter, passes physical params to planner |
| `launch/swarm_mapf.launch.py` | Modified — `planner_type` launch argument |
