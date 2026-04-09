# Euclidean A* with Multi-Cell Movement — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend PBS A* from 4-connected cardinal movement to N-connected Euclidean movement with auto-generated move tables, capsule-based conflict detection, and segment reservation tables.

**Architecture:** Split the monolithic `pbs_solver.hpp` into `mapf_types.hpp` (shared types + geometry), `space_time_astar.hpp` (existing planner), and `euclidean_astar.hpp` (new planner). PBS solver selects planner at compile time via include. Conflict detection upgraded from point-based to segment-segment capsule overlap.

**Tech Stack:** C++17 header-only, ROS 2 Humble, ament_cmake

**Spec:** `docs/specs/2026-04-09-euclidean-astar-design.md`

---

### Task 1: Extract `mapf_types.hpp` from `pbs_solver.hpp`

Move shared types into a new header. No behavioral changes.

**Files:**
- Create: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp`
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp`

- [ ] **Step 1: Create `mapf_types.hpp`**

Create the new header with the types extracted from `pbs_solver.hpp`. Cut these sections (lines 1-100, 120-149 of current `pbs_solver.hpp`):

```cpp
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

// ---------------------------------------------------------------------------
// Базовые структуры
// ---------------------------------------------------------------------------

enum class CostCurve { Linear, Quadratic, Cubic };

inline float apply_curve(float ratio, CostCurve curve) {
  switch (curve) {
    case CostCurve::Linear:    return ratio;
    case CostCurve::Quadratic: return ratio * ratio;
    case CostCurve::Cubic:     return ratio * ratio * ratio;
  }
  return ratio;
}

struct Cell {
  size_t row = 0;
  size_t col = 0;
};

struct Agent {
  size_t id = 0;
  Cell start;
  Cell goal;
  float footprint_radius = 0.0f;
  float inflation = 0.0f;
};

struct GridMap {
  size_t rows = 0;
  size_t cols = 0;
  std::vector<uint8_t> blocked;
  std::vector<int>     wall_cost;

  GridMap inflate_gradient(float hard_m, float soft_m, float resolution_m,
                           int max_penalty = 10,
                           CostCurve cost_curve = CostCurve::Quadratic) const {
    // (exact existing implementation, unchanged)
    if (soft_m <= 0.0f) return *this;
    const float hard_r = hard_m / resolution_m;
    const float soft_r = soft_m / resolution_m;
    const int scan_r = static_cast<int>(std::ceil(soft_r));

    GridMap out;
    out.rows = rows;
    out.cols = cols;
    out.blocked.assign(rows * cols, 0);
    out.wall_cost.assign(rows * cols, 0);

    const float hard_r_sq = hard_r * hard_r;
    const float soft_r_sq = soft_r * soft_r;

    for (size_t row = 0; row < rows; ++row) {
      for (size_t col = 0; col < cols; ++col) {
        if (blocked[row * cols + col] == 0) continue;
        for (int dr = -scan_r; dr <= scan_r; ++dr) {
          for (int dc = -scan_r; dc <= scan_r; ++dc) {
            const float dist_sq = static_cast<float>(dr * dr + dc * dc);
            if (dist_sq > soft_r_sq) continue;
            const int nr = static_cast<int>(row) + dr;
            const int nc = static_cast<int>(col) + dc;
            if (nr < 0 || nr >= static_cast<int>(rows)) continue;
            if (nc < 0 || nc >= static_cast<int>(cols)) continue;
            const size_t idx = nr * cols + nc;
            if (dist_sq <= hard_r_sq) {
              out.blocked[idx] = 1;
            } else {
              const float dist = std::sqrt(dist_sq);
              const float ratio = (soft_r - dist) / (soft_r - hard_r);
              const int cost = static_cast<int>(
                  apply_curve(ratio, cost_curve) * max_penalty);
              out.wall_cost[idx] = std::max(out.wall_cost[idx], cost);
            }
          }
        }
      }
    }
    return out;
  }
};

using Path = std::vector<Cell>;

// ---------------------------------------------------------------------------
// Статистика решения
// ---------------------------------------------------------------------------

enum class FailReason {
  None,
  RootPathFailed,
  BranchExhausted,
  MaxExpansions,
};

struct SolveDiagnostics {
  FailReason fail_reason = FailReason::None;
  size_t     fail_agent = 0;
  size_t     max_t_used = 0;
  size_t     branches_tried = 0;
  size_t     branches_failed = 0;
};

struct AStarStats {
  size_t ok_total_exp = 0;
  size_t ok_max_exp = 0;
  size_t ok_count = 0;
  size_t fail_count = 0;
};

struct SolveStats {
  size_t expansions = 0;
  size_t max_path_length = 0;
  AStarStats astar;
  SolveDiagnostics diag;
};
```

Note: `SolveDiagnostics` drops the `first_conflict` field (which was `Conflict` type) — that type is being removed. If diagnostic logging of first conflict is needed later, it can use `std::optional<Conflict>` from the new conflict types.

- [ ] **Step 2: Update `pbs_solver.hpp`**

Replace the extracted sections at the top of `pbs_solver.hpp` with:

```cpp
#pragma once

#include "iros_llm_swarm_mapf/mapf_types.hpp"

#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
```

Remove the duplicated type definitions (`CostCurve`, `Cell`, `Agent`, `GridMap`, `Path`, stats structs) — they now come from `mapf_types.hpp`. Keep everything from `ConflictType` onward (line 106+) in `pbs_solver.hpp`.

Also remove the `Conflict` field from `SolveDiagnostics` since it references `ConflictType`/`Conflict` which stay in `pbs_solver.hpp`. (The `first_conflict` diagnostic was only used for logging.)

- [ ] **Step 3: Build and verify**

Run inside Docker container:

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds successfully with no errors.

- [ ] **Step 4: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp \
       iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp
git commit -m "refactor(mapf): extract mapf_types.hpp from pbs_solver.hpp"
```

---

### Task 2: Extract `space_time_astar.hpp` from `pbs_solver.hpp`

Move the existing `SpaceTimeAStarPlanner` and `ReservationTable` into their own header.

**Files:**
- Create: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/space_time_astar.hpp`
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp`

- [ ] **Step 1: Create `space_time_astar.hpp`**

Move `ReservationTable` (lines 159-308 of original) and `SpaceTimeAStarPlanner` (lines 315-513 of original) into the new header:

```cpp
#pragma once

#include "iros_llm_swarm_mapf/mapf_types.hpp"

#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// ReservationTable
// ---------------------------------------------------------------------------

class ReservationTable {
  // (exact existing implementation, unchanged — the full class from pbs_solver.hpp)
};

// ---------------------------------------------------------------------------
// Space-Time A*
// ---------------------------------------------------------------------------

class SpaceTimeAStarPlanner {
  // (exact existing implementation, unchanged — the full class from pbs_solver.hpp)
};
```

- [ ] **Step 2: Update `pbs_solver.hpp`**

Add `#include "iros_llm_swarm_mapf/space_time_astar.hpp"` and remove the moved classes. `pbs_solver.hpp` now contains only: `ConflictType`, `Conflict`, `ConflictDetector`, `PriorityGraph`, `PBSSolver`.

- [ ] **Step 3: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds successfully with no errors.

- [ ] **Step 4: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/space_time_astar.hpp \
       iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp
git commit -m "refactor(mapf): extract space_time_astar.hpp from pbs_solver.hpp"
```

---

### Task 3: Add capsule geometry to `mapf_types.hpp`

Implement segment-segment distance and capsule overlap — the foundation for conflict detection and reservation checks.

**Files:**
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp`

- [ ] **Step 1: Add geometry functions to `mapf_types.hpp`**

Add at the end of the file, after the `SolveStats` struct:

```cpp
// ---------------------------------------------------------------------------
// Segment geometry (capsule collision)
// ---------------------------------------------------------------------------

struct Segment {
  float r0, c0;  // start point (row, col) as float
  float r1, c1;  // end point

  static Segment from_cells(const Cell& a, const Cell& b) {
    return {static_cast<float>(a.row), static_cast<float>(a.col),
            static_cast<float>(b.row), static_cast<float>(b.col)};
  }

  // Degenerate (wait move): both endpoints are the same cell
  static Segment from_cell(const Cell& a) {
    return from_cells(a, a);
  }
};

// Minimum squared distance between two line segments.
// Each segment is parameterized as P(s) = A + s*(B-A), s in [0,1].
// Returns the squared distance at the closest point pair.
inline float segment_segment_dist_sq(const Segment& s1, const Segment& s2) {
  const float d1r = s1.r1 - s1.r0, d1c = s1.c1 - s1.c0;
  const float d2r = s2.r1 - s2.r0, d2c = s2.c1 - s2.c0;
  const float rr = s1.r0 - s2.r0,  rc = s1.c0 - s2.c0;

  const float a = d1r * d1r + d1c * d1c;  // |d1|^2
  const float e = d2r * d2r + d2c * d2c;  // |d2|^2
  const float b = d1r * d2r + d1c * d2c;  // d1 . d2
  const float c = d1r * rr  + d1c * rc;   // d1 . r
  const float f = d2r * rr  + d2c * rc;   // d2 . r

  const float denom = a * e - b * b;
  float s, t;

  if (denom < 1e-8f) {
    // Segments are parallel or degenerate
    s = 0.0f;
    t = (e > 1e-8f) ? f / e : 0.0f;
  } else {
    s = (b * f - c * e) / denom;
    t = (a * f - b * c) / denom;
  }

  // Clamp s to [0,1] and recompute t
  s = std::max(0.0f, std::min(1.0f, s));
  t = (e > 1e-8f) ? (b * s + f) / e : 0.0f;

  // Clamp t to [0,1] and recompute s
  t = std::max(0.0f, std::min(1.0f, t));
  if (a > 1e-8f) {
    s = (b * t - c) / a;
    s = std::max(0.0f, std::min(1.0f, s));
  }

  const float pr = s1.r0 + s * d1r - (s2.r0 + t * d2r);
  const float pc = s1.c0 + s * d1c - (s2.c0 + t * d2c);
  return pr * pr + pc * pc;
}

// Check if two capsules overlap (swept circles along segments).
// Returns true if minimum distance between segments < radius_a + radius_b.
inline bool capsule_overlap(const Segment& a, float radius_a,
                            const Segment& b, float radius_b) {
  const float combined = radius_a + radius_b;
  if (combined <= 0.0f) {
    // Point agents: exact cell check at closest approach
    return segment_segment_dist_sq(a, b) < 1e-6f;
  }
  return segment_segment_dist_sq(a, b) < combined * combined;
}

// Gradient penalty between two capsules.
// Returns: -1 = hard overlap (forbidden), 0 = free, >0 = soft penalty.
// hard_dist: combined footprint radii (hard boundary).
// soft_dist: combined soft radii (outer boundary).
inline int capsule_penalty(const Segment& a, float hard_a, float soft_a,
                           const Segment& b, float hard_b, float soft_b,
                           int max_penalty = 50,
                           CostCurve cost_curve = CostCurve::Quadratic) {
  const float dist_sq = segment_segment_dist_sq(a, b);
  const float hard_dist = hard_a + hard_b;
  const float soft_dist = soft_a + soft_b;

  if (hard_dist > 0.0f && dist_sq < hard_dist * hard_dist) return -1;
  if (soft_dist <= 0.0f || dist_sq >= soft_dist * soft_dist) return 0;

  const float dist = std::sqrt(dist_sq);
  const float ratio = (soft_dist - dist) / (soft_dist - hard_dist);
  return static_cast<int>(apply_curve(ratio, cost_curve) * max_penalty);
}
```

- [ ] **Step 2: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors.

- [ ] **Step 3: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp
git commit -m "feat(mapf): add capsule geometry (segment-segment distance, overlap, penalty)"
```

---

### Task 4: Add `MoveSet` and Bresenham trace to `mapf_types.hpp`

**Files:**
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp`

- [ ] **Step 1: Add `MoveSet` struct and Bresenham trace**

Add after the capsule geometry section:

```cpp
// ---------------------------------------------------------------------------
// Move table
// ---------------------------------------------------------------------------

struct Move {
  int   dr;
  int   dc;
  float cost;  // Euclidean distance in cells (0 for wait)
};

struct MoveSet {
  std::vector<Move> moves;

  // Generate all reachable moves from physical parameters.
  // reach = max(1, floor(max_speed * time_step / resolution))
  // Includes wait (0,0) and all (dr,dc) where dr²+dc² <= reach².
  static MoveSet generate(float max_speed, float time_step_sec,
                           float resolution) {
    const int reach = std::max(1,
        static_cast<int>(std::floor(max_speed * time_step_sec / resolution)));
    MoveSet ms;
    ms.moves.push_back({0, 0, 0.0f});  // wait
    for (int dr = -reach; dr <= reach; ++dr) {
      for (int dc = -reach; dc <= reach; ++dc) {
        if (dr == 0 && dc == 0) continue;
        if (dr * dr + dc * dc <= reach * reach) {
          ms.moves.push_back({dr, dc,
              std::sqrt(static_cast<float>(dr * dr + dc * dc))});
        }
      }
    }
    return ms;
  }
};

// ---------------------------------------------------------------------------
// Bresenham line trace
// ---------------------------------------------------------------------------

// Trace cells along line from (r0,c0) to (r1,c1) using Bresenham.
// Callback receives each cell (row, col). Returns false from callback to stop.
template <typename Fn>
void bresenham_trace(int r0, int c0, int r1, int c1, Fn&& fn) {
  int dr = r1 > r0 ? r1 - r0 : r0 - r1;
  int dc = c1 > c0 ? c1 - c0 : c0 - c1;
  int sr = r0 < r1 ? 1 : -1;
  int sc = c0 < c1 ? 1 : -1;
  int err = dr - dc;

  while (true) {
    if (!fn(r0, c0)) return;
    if (r0 == r1 && c0 == c1) break;
    const int e2 = 2 * err;
    if (e2 > -dc) { err -= dc; r0 += sr; }
    if (e2 <  dr) { err += dr; c0 += sc; }
  }
}

// Check if a move is valid (no blocked cells along trace) and compute
// the sum of wall_cost along the trace.
// Returns: {valid, wall_penalty_sum}
inline std::pair<bool, float> trace_move(const GridMap& map,
                                          size_t from_r, size_t from_c,
                                          int dr, int dc) {
  const int tr = static_cast<int>(from_r) + dr;
  const int tc = static_cast<int>(from_c) + dc;

  // Bounds check
  if (tr < 0 || tr >= static_cast<int>(map.rows)) return {false, 0.0f};
  if (tc < 0 || tc >= static_cast<int>(map.cols)) return {false, 0.0f};

  bool valid = true;
  float penalty = 0.0f;

  bresenham_trace(static_cast<int>(from_r), static_cast<int>(from_c),
                  tr, tc, [&](int r, int c) -> bool {
    const size_t idx = static_cast<size_t>(r) * map.cols + static_cast<size_t>(c);
    if (map.blocked[idx]) { valid = false; return false; }
    if (!map.wall_cost.empty()) penalty += static_cast<float>(map.wall_cost[idx]);
    return true;
  });

  return {valid, penalty};
}
```

- [ ] **Step 2: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors.

- [ ] **Step 3: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp
git commit -m "feat(mapf): add MoveSet generation and Bresenham line trace"
```

---

### Task 5: Implement segment-based `ReservationTable`

New reservation table that stores segments instead of points. Used by the new Euclidean A* planner.

**Files:**
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp`

- [ ] **Step 1: Add `SegmentReservationTable` to `mapf_types.hpp`**

Add after the Bresenham section:

```cpp
// ---------------------------------------------------------------------------
// Segment-based Reservation Table
// ---------------------------------------------------------------------------

class SegmentReservationTable {
 public:
  void clear() {
    entries_.clear();
    held_goals_.clear();
  }

  // Reserve an agent's path as a sequence of segments.
  // footprint_cells: physical (hard) radius in cells.
  // soft_radius_cells: footprint + inflation in cells.
  // skip_until: grace period — skip entries for t < skip_until.
  void reserve_path(const Path& path, size_t hold_goal_until_time,
                    float footprint_cells, float soft_radius_cells,
                    size_t skip_until = 0) {
    if (path.empty()) return;
    for (size_t t = std::max(skip_until, size_t(1)); t < path.size(); ++t) {
      entries_[t].push_back(
          {path[t - 1], path[t], footprint_cells, soft_radius_cells});
    }
    // Also store wait at t=0 (or skip_until) as a point segment
    if (skip_until == 0) {
      entries_[0].push_back(
          {path[0], path[0], footprint_cells, soft_radius_cells});
    }
    // Hold goal: agent stays at final cell after arrival
    if (path.size() <= hold_goal_until_time) {
      held_goals_.push_back({path.back(), footprint_cells, soft_radius_cells,
                             std::max(path.size(), skip_until)});
    }
  }

  // Check a candidate move segment against reservations.
  // Returns: 0 = free, >0 = soft penalty, -1 = forbidden.
  int segment_penalty(const Cell& from, const Cell& to, size_t time,
                      float my_footprint, float my_soft,
                      int max_penalty = 50,
                      CostCurve cost_curve = CostCurve::Quadratic) const {
    const Segment my_seg = Segment::from_cells(from, to);
    int worst = 0;

    auto check = [&](const SegEntry& e) {
      const Segment other = Segment::from_cells(e.from, e.to);
      const int pen = capsule_penalty(
          my_seg, my_footprint, my_soft,
          other, e.footprint_cells, e.soft_radius_cells,
          max_penalty, cost_curve);
      if (pen < 0) { worst = -1; return; }
      worst = std::max(worst, pen);
    };

    // Check held goals
    for (const auto& hg : held_goals_) {
      if (time >= hg.from_time) {
        const Segment goal_seg = Segment::from_cell(hg.cell);
        const int pen = capsule_penalty(
            my_seg, my_footprint, my_soft,
            goal_seg, hg.footprint_cells, hg.soft_radius_cells,
            max_penalty, cost_curve);
        if (pen < 0) return -1;
        worst = std::max(worst, pen);
      }
    }

    // Check entries at this timestep
    auto it = entries_.find(time);
    if (it != entries_.end()) {
      for (const auto& e : it->second) {
        check(e);
        if (worst < 0) return -1;
      }
    }
    return worst;
  }

  // Can an agent hold its goal from from_time onward?
  bool can_hold_goal(size_t row, size_t col, size_t from_time,
                     size_t max_time, float my_footprint_cells) const {
    const Segment my_seg = Segment::from_cell({row, col});
    for (const auto& hg : held_goals_) {
      const Segment other = Segment::from_cell(hg.cell);
      if (capsule_overlap(my_seg, my_footprint_cells, other, hg.footprint_cells))
        return false;
    }
    for (size_t t = from_time; t <= max_time; ++t) {
      auto it = entries_.find(t);
      if (it == entries_.end()) continue;
      for (const auto& e : it->second) {
        const Segment other = Segment::from_cells(e.from, e.to);
        if (capsule_overlap(my_seg, my_footprint_cells, other, e.footprint_cells))
          return false;
      }
    }
    return true;
  }

 private:
  struct SegEntry {
    Cell  from;
    Cell  to;
    float footprint_cells;
    float soft_radius_cells;
  };
  struct HeldGoal {
    Cell   cell;
    float  footprint_cells;
    float  soft_radius_cells;
    size_t from_time;
  };

  std::unordered_map<size_t, std::vector<SegEntry>> entries_;
  std::vector<HeldGoal> held_goals_;
};
```

- [ ] **Step 2: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors.

- [ ] **Step 3: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/mapf_types.hpp
git commit -m "feat(mapf): add SegmentReservationTable with capsule-based checks"
```

---

### Task 6: Implement `EuclideanAStarPlanner`

New A* planner with move table, float costs, Dijkstra heuristic, and segment-based reservation checks.

**Files:**
- Create: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/euclidean_astar.hpp`

- [ ] **Step 1: Create `euclidean_astar.hpp`**

```cpp
#pragma once

#include "iros_llm_swarm_mapf/mapf_types.hpp"

#include <queue>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// Euclidean Space-Time A* with N-connected movement
// ---------------------------------------------------------------------------

class EuclideanAStarPlanner {
 public:
  EuclideanAStarPlanner() = default;

  void reset_map(const GridMap* map) { map_ = map; }
  const GridMap* current_map() const { return map_; }
  void set_goal_dists(const std::vector<float>* d) { goal_dists_ = d; }
  size_t last_expansions() const { return last_expansions_; }

  void set_move_set(const MoveSet& ms) { moves_ = ms; }

  bool find_path(const Cell& start, const Cell& goal,
                 const SegmentReservationTable& reservations,
                 float my_footprint_cells, float my_soft_cells,
                 size_t max_time, Path& path,
                 CostCurve cost_curve = CostCurve::Quadratic,
                 int proximity_penalty = 50,
                 size_t max_astar_expansions = 0) {
    my_footprint_ = my_footprint_cells;
    my_soft_ = my_soft_cells;
    cost_curve_ = cost_curve;
    proximity_penalty_ = proximity_penalty;

    path.clear();
    if (!map_) return false;
    if (is_blocked(start.row, start.col) || is_blocked(goal.row, goal.col))
      return false;

    const size_t spatial = map_->rows * map_->cols;
    const size_t state_count = spatial * (max_time + 1);

    ensure_capacity(state_count);
    next_generation();

    const size_t start_idx = idx(start.row, start.col);
    const size_t goal_idx  = idx(goal.row, goal.col);
    const size_t start_state = state_idx(start_idx, 0, spatial);

    set_g(start_state, 0.0f);
    parent_[start_state] = INVALID;

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
    open.push({start_idx, 0, 0.0f, heuristic(start, goal)});

    last_expansions_ = 0;
    while (!open.empty()) {
      ++last_expansions_;
      if (max_astar_expansions > 0 && last_expansions_ > max_astar_expansions)
        return false;

      const Node cur = open.top(); open.pop();
      const size_t cs = state_idx(cur.idx, cur.t, spatial);

      if (is_closed(cs)) continue;
      if (!is_alive(cs) || cur.g != g_[cs]) continue;
      mark_closed(cs);

      if (cur.idx == goal_idx) {
        if (reservations.can_hold_goal(goal.row, goal.col, cur.t,
                                        max_time, my_footprint_cells)) {
          restore(cs, spatial, path);
          return true;
        }
      }

      if (cur.t >= max_time) continue;

      const Cell from = cell(cur.idx);
      const size_t nt = cur.t + 1;

      for (const auto& mv : moves_.moves) {
        relax(cur.idx, cur.t, from, mv, nt, goal, reservations, open, spatial);
      }
    }
    return false;
  }

 private:
  static constexpr float INF_F  = std::numeric_limits<float>::max() / 4.0f;
  static constexpr size_t INVALID = std::numeric_limits<size_t>::max();

  struct Node { size_t idx, t; float g, f; };
  struct NodeCmp {
    bool operator()(const Node& a, const Node& b) const {
      return a.f != b.f ? a.f > b.f : a.g < b.g;
    }
  };

  const GridMap* map_ = nullptr;
  const std::vector<float>* goal_dists_ = nullptr;
  MoveSet moves_;
  float my_footprint_ = 0.0f;
  float my_soft_ = 0.0f;
  CostCurve cost_curve_ = CostCurve::Quadratic;
  int proximity_penalty_ = 50;
  size_t last_expansions_ = 0;

  std::vector<float>    g_;
  std::vector<size_t>   parent_;
  std::vector<uint8_t>  closed_;
  std::vector<uint32_t> gen_;
  uint32_t current_gen_ = 1;

  size_t idx(size_t r, size_t c) const { return r * map_->cols + c; }
  Cell cell(size_t i) const { return {i / map_->cols, i % map_->cols}; }

  static size_t state_idx(size_t ci, size_t t, size_t sp) { return t * sp + ci; }
  static size_t state_cell(size_t si, size_t sp) { return si % sp; }

  bool is_blocked(size_t r, size_t c) const {
    return map_->blocked[idx(r, c)] != 0;
  }

  float heuristic(const Cell& a, const Cell& /*goal*/) const {
    if (goal_dists_) {
      const float d = (*goal_dists_)[idx(a.row, a.col)];
      return d < INF_F ? d : INF_F;
    }
    return 0.0f;  // no heuristic available
  }

  void ensure_capacity(size_t n) {
    if (g_.size() >= n) return;
    g_.resize(n, INF_F);
    parent_.resize(n, INVALID);
    closed_.resize(n, 0);
    gen_.resize(n, 0);
  }

  void next_generation() {
    if (++current_gen_ == 0) {
      current_gen_ = 1;
      std::fill(gen_.begin(), gen_.end(), 0);
    }
  }

  bool   is_alive(size_t s)  const { return gen_[s] == current_gen_; }
  float  get_g(size_t s)     const { return is_alive(s) ? g_[s] : INF_F; }
  void   set_g(size_t s, float v)  { gen_[s] = current_gen_; g_[s] = v; closed_[s] = 0; }
  bool   is_closed(size_t s) const { return is_alive(s) && closed_[s]; }
  void   mark_closed(size_t s)     { closed_[s] = 1; }

  void relax(size_t ci, size_t ct, const Cell& from, const Move& mv,
             size_t nt, const Cell& goal,
             const SegmentReservationTable& res,
             std::priority_queue<Node, std::vector<Node>, NodeCmp>& open,
             size_t sp) {
    // Compute target cell
    const int nr = static_cast<int>(from.row) + mv.dr;
    const int nc = static_cast<int>(from.col) + mv.dc;
    if (nr < 0 || nr >= static_cast<int>(map_->rows)) return;
    if (nc < 0 || nc >= static_cast<int>(map_->cols)) return;

    const size_t ni = idx(static_cast<size_t>(nr), static_cast<size_t>(nc));
    const bool moving = (ni != ci);
    const Cell to_cell = {static_cast<size_t>(nr), static_cast<size_t>(nc)};

    // Trace the move: check blocked cells and sum wall penalties
    float wall_pen = 0.0f;
    if (moving) {
      auto [valid, wpen] = trace_move(*map_, from.row, from.col, mv.dr, mv.dc);
      if (!valid) return;
      wall_pen = wpen;
    }

    // Segment-based reservation check (replaces vertex_penalty + is_edge_blocked)
    const int agent_pen = res.segment_penalty(
        from, to_cell, nt, my_footprint_, my_soft_,
        proximity_penalty_, cost_curve_);
    if (agent_pen < 0) return;

    const size_t cs = state_idx(ci, ct, sp);
    const size_t ns = state_idx(ni, nt, sp);
    if (is_closed(ns)) return;

    // Cost: base time step (1.0) + Euclidean move cost + penalties
    const float tg = get_g(cs) + 1.0f + mv.cost
                     + wall_pen + static_cast<float>(agent_pen);
    if (tg < get_g(ns)) {
      set_g(ns, tg);
      parent_[ns] = cs;
      open.push({ni, nt, tg, tg + heuristic(to_cell, goal)});
    }
  }

  void restore(size_t goal_state, size_t sp, Path& path) const {
    for (size_t cur = goal_state; cur != INVALID; cur = parent_[cur])
      path.push_back(cell(state_cell(cur, sp)));
    std::reverse(path.begin(), path.end());
  }
};

// ---------------------------------------------------------------------------
// Dijkstra heuristic precomputation (replaces BFS)
// ---------------------------------------------------------------------------

// Backward Dijkstra from goal using the given MoveSet.
// Returns minimum-cost distances (float) from every cell to goal.
// Accounts for blocked cells but excludes penalties.
inline std::vector<float> dijkstra_from(const GridMap& map, const Cell& goal,
                                         const MoveSet& moves) {
  static constexpr float INF_F = std::numeric_limits<float>::max() / 4.0f;
  const size_t N = map.rows * map.cols;
  std::vector<float> dist(N, INF_F);

  struct DNode { size_t idx; float d; };
  struct DCmp { bool operator()(const DNode& a, const DNode& b) { return a.d > b.d; } };
  std::priority_queue<DNode, std::vector<DNode>, DCmp> pq;

  const size_t gi = goal.row * map.cols + goal.col;
  dist[gi] = 0.0f;
  pq.push({gi, 0.0f});

  while (!pq.empty()) {
    const auto [ci, d] = pq.top(); pq.pop();
    if (d > dist[ci]) continue;

    const size_t r = ci / map.cols;
    const size_t c = ci % map.cols;

    for (const auto& mv : moves.moves) {
      if (mv.dr == 0 && mv.dc == 0) continue;  // skip wait
      // Reverse direction: we're going backwards from goal
      const int nr = static_cast<int>(r) - mv.dr;
      const int nc = static_cast<int>(c) - mv.dc;
      if (nr < 0 || nr >= static_cast<int>(map.rows)) continue;
      if (nc < 0 || nc >= static_cast<int>(map.cols)) continue;

      // Trace: check that the reversed move doesn't cross blocked cells
      // (trace from neighbor TO current, same cells either direction)
      auto [valid, _wpen] = trace_move(map, static_cast<size_t>(nr),
                                        static_cast<size_t>(nc), mv.dr, mv.dc);
      if (!valid) continue;

      const size_t ni = static_cast<size_t>(nr) * map.cols + static_cast<size_t>(nc);
      const float nd = d + mv.cost;
      if (nd < dist[ni]) {
        dist[ni] = nd;
        pq.push({ni, nd});
      }
    }
  }
  return dist;
}
```

- [ ] **Step 2: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors (header is included by nothing yet, but syntax-checked by including it temporarily or via next task).

- [ ] **Step 3: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/euclidean_astar.hpp
git commit -m "feat(mapf): implement EuclideanAStarPlanner with move table and Dijkstra heuristic"
```

---

### Task 7: Update `ConflictDetector` to use capsule overlap

Modify the PBS-level conflict detector to use segment-segment checks.

**Files:**
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp`

- [ ] **Step 1: Replace `ConflictDetector::find_first()`**

Remove the `ConflictType` enum. Replace `Conflict` struct:

```cpp
struct Conflict {
  size_t agent1 = 0;
  size_t agent2 = 0;
  size_t time   = 0;
};
```

Replace `ConflictDetector::find_first()` to use capsule overlap:

```cpp
class ConflictDetector {
 public:
  // Returns std::nullopt if no conflict found.
  static std::optional<Conflict> find_first(
      const std::vector<Path>& paths,
      const std::vector<float>& footprint_cells) {
    const size_t n = paths.size();
    const size_t T = max_len(paths);

    // Precompute grace periods
    std::vector<std::vector<size_t>> grace(n, std::vector<size_t>(n, 0));
    for (size_t i = 0; i < n; ++i)
      for (size_t j = i + 1; j < n; ++j) {
        const size_t g = start_grace(paths[i][0], footprint_cells[i],
                                     paths[j][0], footprint_cells[j]);
        grace[i][j] = grace[j][i] = g;
      }

    for (size_t t = 0; t < T; ++t) {
      for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
          if (t < grace[i][j]) continue;

          // Build segments for this timestep
          const Cell ci_from = at(paths[i], t > 0 ? t - 1 : 0);
          const Cell ci_to   = at(paths[i], t);
          const Cell cj_from = at(paths[j], t > 0 ? t - 1 : 0);
          const Cell cj_to   = at(paths[j], t);

          const Segment si = Segment::from_cells(ci_from, ci_to);
          const Segment sj = Segment::from_cells(cj_from, cj_to);

          if (capsule_overlap(si, footprint_cells[i], sj, footprint_cells[j])) {
            return Conflict{i, j, t};
          }
        }
      }
    }
    return std::nullopt;
  }

  static size_t start_grace(const Cell& a, float ra, const Cell& b, float rb) {
    // (unchanged from current implementation)
    const float combined = ra + rb;
    if (combined <= 0.0f) return 0;
    const float dr = static_cast<float>(a.row) - static_cast<float>(b.row);
    const float dc = static_cast<float>(a.col) - static_cast<float>(b.col);
    const float dist = std::sqrt(dr * dr + dc * dc);
    if (dist >= combined) return 0;
    return static_cast<size_t>(std::ceil(combined - dist)) + 1;
  }

 private:
  static Cell at(const Path& p, size_t t) {
    if (p.empty()) return {};
    return t < p.size() ? p[t] : p.back();
  }
  static size_t max_len(const std::vector<Path>& ps) {
    size_t r = 0;
    for (const auto& p : ps) r = std::max(r, p.size());
    return r;
  }
};
```

- [ ] **Step 2: Update PBS solver to use `std::optional<Conflict>`**

In `PBSSolver::solve()`, change the conflict detection check:

```cpp
// Old:
const Conflict c = ConflictDetector::find_first(node.paths, footprint_cells_);
if (c.type == ConflictType::None) {

// New:
const auto c = ConflictDetector::find_first(node.paths, footprint_cells_);
if (!c) {
```

And update the references to `c.agent1` etc to `c->agent1`:

```cpp
size_t hi = c->agent1, lo = c->agent2;
```

Remove the `first_conflict` diagnostic assignment (the `Conflict` struct no longer has cell fields). Add `#include <optional>` to the includes.

- [ ] **Step 3: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors.

- [ ] **Step 4: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp
git commit -m "feat(mapf): upgrade ConflictDetector to capsule overlap, remove vertex/edge distinction"
```

---

### Task 8: Wire `EuclideanAStarPlanner` into `PBSSolver`

Make PBS use the new planner. The old planner remains available via `space_time_astar.hpp`.

**Files:**
- Modify: `iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp`

- [ ] **Step 1: Update `PBSSolver` to use the new planner**

Change the include at the top of `pbs_solver.hpp`:

```cpp
// Replace:
#include "iros_llm_swarm_mapf/space_time_astar.hpp"
// With:
#include "iros_llm_swarm_mapf/euclidean_astar.hpp"
```

In the `PBSSolver` private members, replace:

```cpp
// Old:
SpaceTimeAStarPlanner  low_level_;

// New:
EuclideanAStarPlanner  low_level_;
MoveSet                move_set_;
```

Add a setter for physical parameters used to generate the move set:

```cpp
public:
  void set_movement_params(float max_speed, float time_step_sec,
                            float resolution) {
    move_set_ = MoveSet::generate(max_speed, time_step_sec, resolution);
    low_level_.set_move_set(move_set_);
  }
```

In `solve()`, replace the BFS distance cache with Dijkstra:

```cpp
// Old:
std::unordered_map<size_t, std::vector<int>> dist_cache_;

// New:
std::unordered_map<size_t, std::vector<float>> dist_cache_;
```

Update `bfs_from` calls to `dijkstra_from`:

```cpp
// Old:
dist_cache_[key] = bfs_from(*low_level_.current_map(), agents[i].goal);

// New:
dist_cache_[key] = dijkstra_from(*low_level_.current_map(), agents[i].goal, move_set_);
```

Update `agent_goal_dists_` type:

```cpp
// Old:
std::vector<const std::vector<int>*> agent_goal_dists_;

// New:
std::vector<const std::vector<float>*> agent_goal_dists_;
```

Update `agent_dists_` computation (now float-based):

```cpp
const float d = (*agent_goal_dists_[i])[si];
agent_dists_[i] = d < std::numeric_limits<float>::max() / 4.0f
    ? static_cast<size_t>(d) : 0;
```

Update `max_t` formula:

```cpp
// Old:
const size_t max_t = std::min<size_t>(max_dist * 4 + 64, 600);

// New:
const size_t max_t = std::min<size_t>(max_dist * 2 + 32, 600);
```

Replace `ReservationTable` with `SegmentReservationTable` in root planning and `replan()`. The `reserve_path` API is the same. Update `find_path` calls to use the new reservation table type.

Remove the `bfs_from` static method (no longer needed).

- [ ] **Step 2: Add `footprint_radius >= 0.7 * resolution` assertion**

At the start of `solve()`, after computing `footprint_cells_`, add a safety assertion (`pbs_solver.hpp` is header-only, no rclcpp):

```cpp
#include <cassert>  // add to includes at top

// In solve(), after the footprint_cells_ loop:
for (size_t i = 0; i < n; ++i) {
  if (resolution > 0.0f && agents[i].footprint_radius > 0.0f) {
    assert(agents[i].footprint_radius >= 0.7f * resolution &&
           "footprint_radius must be >= 0.7 * pbs_resolution to prevent wall clipping");
  }
}
```

- [ ] **Step 3: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors.

- [ ] **Step 4: Commit**

```bash
git add iros_llm_swarm_mapf/include/iros_llm_swarm_mapf/pbs_solver.hpp
git commit -m "feat(mapf): wire EuclideanAStarPlanner into PBSSolver"
```

---

### Task 9: Update `mapf_planner_node.cpp` and launch file

Pass physical parameters to the solver and add the `planner_type` config.

**Files:**
- Modify: `iros_llm_swarm_mapf/src/mapf_planner_node.cpp`
- Modify: `iros_llm_swarm_mapf/launch/mapf.launch.py`

- [ ] **Step 1: Update `mapf_planner_node.cpp`**

Add parameter declarations in the constructor:

```cpp
declare_parameter("max_speed", 0.5);
declare_parameter("planner_type", std::string("euclidean"));
```

Read the new parameters:

```cpp
max_speed_ = get_parameter("max_speed").as_double();
planner_type_ = get_parameter("planner_type").as_string();
```

Add member variables:

```cpp
double max_speed_;
std::string planner_type_;
```

In the `set_goals` callback, after setting up the solver's map, call:

```cpp
solver_.set_movement_params(
    static_cast<float>(max_speed_),
    static_cast<float>(time_step_sec_),
    static_cast<float>(pbs_resolution_));
```

This must be called before `solver_.solve()`.

- [ ] **Step 2: Update launch file**

Add new launch arguments in `mapf.launch.py`:

```python
DeclareLaunchArgument(
    'max_speed', default_value='0.5',
    description='Max robot speed (m/s), used with time_step_sec to compute movement reach'
),
DeclareLaunchArgument(
    'planner_type', default_value='euclidean',
    description='A* planner type: euclidean (N-connected) or classic (4-connected)'
),
```

Add to the node parameters dict:

```python
'max_speed': LaunchConfiguration('max_speed'),
'planner_type': LaunchConfiguration('planner_type'),
```

- [ ] **Step 3: Build and verify**

```bash
colcon build --packages-select iros_llm_swarm_mapf
```

Expected: builds with no errors.

- [ ] **Step 4: Commit**

```bash
git add iros_llm_swarm_mapf/src/mapf_planner_node.cpp \
       iros_llm_swarm_mapf/launch/mapf.launch.py
git commit -m "feat(mapf): add max_speed and planner_type params, wire to solver"
```

---

### Task 10: Update README with new constraint

**Files:**
- Modify: `README.md` (MAPF section)

- [ ] **Step 1: Add constraint to README**

Find the MAPF key parameters section and add:

```markdown
**Constraint**: `footprint_radius >= 0.7 * pbs_resolution` must hold when using the Euclidean planner.
The planner asserts this at startup. With defaults (0.22m footprint, 0.2m resolution) the constraint is satisfied.
```

Also document the new `planner_type` and `max_speed` parameters in the key parameters table.

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs(mapf): add Euclidean planner constraint and new params to README"
```

---

### Task 11: Integration test

Test the full stack with the new planner.

**Files:** No new files — uses existing test tooling.

- [ ] **Step 1: Build everything**

```bash
colcon build --packages-select iros_llm_swarm_mapf
source install/setup.bash
```

- [ ] **Step 2: Launch the system**

In one terminal:
```bash
ros2 launch iros_llm_swarm_bringup swarm_mapf.launch.py
```

- [ ] **Step 3: Send test goals — single point**

```bash
ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0
```

Expected: planning succeeds, planning time reported, paths published.

- [ ] **Step 4: Send test goals — random**

```bash
ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0
```

Expected: planning succeeds with random targets.

- [ ] **Step 5: Compare timing with classic planner**

Re-launch with `planner_type:=classic` (requires reverting the include in `pbs_solver.hpp` to `space_time_astar.hpp` or adding a runtime switch — for now, compare by rebuilding with the old include). Note planning times for comparison.

- [ ] **Step 6: Commit any fixes**

If integration testing reveals issues, fix and commit with appropriate messages.
