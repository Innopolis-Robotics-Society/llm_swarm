#pragma once

#include "iros_llm_swarm_mapf/mapf_types.hpp"

#include <queue>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// Euclidean Space-Time A* with N-connected movement
//
// Replaces the 4-connected SpaceTimeAStarPlanner with a connectivity-
// agnostic planner. The move set is auto-generated from physical
// parameters (speed, timestep, resolution) via MoveSet::generate().
//
// Key differences from the classic planner:
//   - Float g-values (Euclidean move costs instead of integer step costs)
//   - Dijkstra heuristic (replaces BFS — correct for non-uniform costs)
//   - Move loop over MoveSet instead of hardcoded cardinal neighbors
//   - Bresenham trace validates multi-cell jumps (blocked check + wall penalty sum)
//   - Segment-based reservation checks via SegmentReservationTable
//     (replaces separate vertex_penalty + is_edge_blocked)
// ---------------------------------------------------------------------------

class EuclideanAStarPlanner {
 public:
  EuclideanAStarPlanner() = default;

  void reset_map(const GridMap* map) { map_ = map; }
  const GridMap* current_map() const { return map_; }
  void set_goal_dists(const std::vector<float>* d) { goal_dists_ = d; }
  size_t last_expansions() const { return last_expansions_; }

  void set_move_set(const MoveSet& ms) { moves_ = ms; }
  void set_cost_params(float time_cost, float resolution) {
    time_cost_ = time_cost;
    resolution_ = resolution;
  }

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

    // Don't reject starts overlapping reservations at t=0 — during
    // replanning agents must be allowed to start at their physical position.
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
  float time_cost_  = 1.0f;   // urgency * max_speed * time_step_sec
  float resolution_ = 1.0f;   // PBS grid cell size in metres
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

  // Dijkstra-precomputed distance from this cell to the goal.
  // Falls back to 0 if no heuristic is set (still correct, just slower).
  float heuristic(const Cell& a, const Cell& /*goal*/) const {
    if (goal_dists_) {
      const float d = (*goal_dists_)[idx(a.row, a.col)];
      return d < INF_F ? d : INF_F;
    }
    return 0.0f;
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

    const float tg = get_g(cs) + time_cost_ + resolution_ * mv.cost
                     + wall_pen * resolution_ + static_cast<float>(agent_pen);
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
// Dijkstra heuristic precomputation
//
// Backward Dijkstra from goal using the same MoveSet as A*.
// Produces correct minimum-cost distances for any connectivity.
// With 4-connected integer costs, degenerates to BFS.
// One-time cost per unique goal on the downsampled PBS grid.
// ---------------------------------------------------------------------------

inline std::vector<float> dijkstra_from(const GridMap& map, const Cell& goal,
                                         const MoveSet& moves,
                                         float time_cost, float resolution) {
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
      // Reverse direction: expanding backwards from goal
      const int nr = static_cast<int>(r) - mv.dr;
      const int nc = static_cast<int>(c) - mv.dc;
      if (nr < 0 || nr >= static_cast<int>(map.rows)) continue;
      if (nc < 0 || nc >= static_cast<int>(map.cols)) continue;

      // Validate the reversed move doesn't cross blocked cells
      auto [valid, _wpen] = trace_move(map, static_cast<size_t>(nr),
                                        static_cast<size_t>(nc), mv.dr, mv.dc);
      if (!valid) continue;

      const size_t ni = static_cast<size_t>(nr) * map.cols + static_cast<size_t>(nc);
      const float nd = d + time_cost + resolution * mv.cost;
      if (nd < dist[ni]) {
        dist[ni] = nd;
        pq.push({ni, nd});
      }
    }
  }
  return dist;
}
