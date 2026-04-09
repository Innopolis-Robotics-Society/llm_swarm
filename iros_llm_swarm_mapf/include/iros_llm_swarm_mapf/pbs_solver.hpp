#pragma once

#include "iros_llm_swarm_mapf/mapf_types.hpp"

#include <deque>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ---------------------------------------------------------------------------
// Conflict types
// ---------------------------------------------------------------------------

enum class ConflictType { None, Vertex, Edge };

struct Conflict {
  ConflictType type = ConflictType::None;
  size_t agent1 = 0;
  size_t agent2 = 0;
  size_t time   = 0;
  Cell   cell1{};
  Cell   cell2{};
};

// SolveDiagnostics, AStarStats, SolveStats defined in mapf_types.hpp

// ---------------------------------------------------------------------------
// ReservationTable
// ---------------------------------------------------------------------------
// Stores centres and radii of higher-priority agents.
// Conflict check via Euclidean distance between centres:
// if distance < (r_a + r_b), agents are considered in conflict.
// With zero radii the behaviour is identical to point-based checking.

class ReservationTable {
 public:
  void clear() {
    entries_.clear();
    held_goals_.clear();
  }

  // Reserve agent path with the given radius (in cells).
  // footprint_cells: physical (hard) radius in cells.
  // soft_radius_cells: footprint + inflation in cells (outer penalty boundary).
  // skip_until: don't reserve entries for t < skip_until (grace period
  // for agents whose starts overlap — lets them separate first).
  void reserve_path(const Path& path,
                    size_t hold_goal_until_time,
                    float footprint_cells, float soft_radius_cells,
                    size_t skip_until = 0) {
    if (path.empty()) return;
    for (size_t t = skip_until; t < path.size(); ++t) {
      entries_[t].push_back({path[t], footprint_cells, soft_radius_cells});
    }
    // Goal hold: agent stays in place after arrival
    if (path.size() <= hold_goal_until_time) {
      held_goals_.push_back({path.back(), footprint_cells, soft_radius_cells,
                             std::max(path.size(), skip_until)});
    }
  }

  // Gradient penalty for placing an agent at (row,col) at time t.
  // Returns: 0 = free, >0 = soft penalty, -1 = FORBIDDEN (physical overlap).
  // my_footprint/my_soft: agent's own radii in cells.
  // max_penalty: cost at the hard boundary (decreasing to 0 at soft boundary).
  // Multiple nearby agents: takes the greatest penalty, not sum.
  int vertex_penalty(size_t row, size_t col, size_t time,
                     float my_footprint, float my_soft,
                     int max_penalty = 50,
                     CostCurve cost_curve = CostCurve::Quadratic) const {
    int total = 0;
    auto check = [&](const Cell& c, float fp, float sr) {
      const float dr = static_cast<float>(row) - static_cast<float>(c.row);
      const float dc = static_cast<float>(col) - static_cast<float>(c.col);
      const float dist_sq = dr * dr + dc * dc;
      const float hard = my_footprint + fp;
      const float soft = my_soft + sr;
      if (hard > 0.0f && dist_sq < hard * hard) { total = -1; return; }
      if (soft > 0.0f && dist_sq < soft * soft) {
        const float dist = std::sqrt(dist_sq);
        const float ratio = (soft - dist) / (soft - hard);
        const int cost = static_cast<int>(
            apply_curve(ratio, cost_curve) * max_penalty);
        total = std::max(total, cost);
      }
    };

    // Check held goals
    for (const auto& hg : held_goals_) {
      if (time >= hg.from_time) {
        check(hg.cell, hg.footprint_cells, hg.soft_radius_cells);
        if (total < 0) return -1;
      }
    }
    // Check positions at the specific timestep
    auto it = entries_.find(time);
    if (it != entries_.end()) {
      for (const auto& e : it->second) {
        check(e.cell, e.footprint_cells, e.soft_radius_cells);
        if (total < 0) return -1;
      }
    }
    return total;
  }

  // Edge conflict: checks opposing movement between steps time and time+1.
  // Uses footprint (hard) radius only — physical swap prevention.
  bool is_edge_blocked(size_t fr, size_t fc, size_t tr, size_t tc,
                        size_t time, float my_footprint_cells) const {
    auto it = entries_.find(time);
    auto it_next = entries_.find(time + 1);
    if (it == entries_.end() || it_next == entries_.end()) return false;

    // For each agent: check if it moves in the opposite direction.
    // NOTE: pairing by vector index assumes reserve_path() is called
    // once per agent in a fixed order.  With skip_until, an agent's
    // entries start later, so indices at the grace boundary may differ —
    // the min(size, size) guard safely skips unmatched trailing entries.
    // TODO: edge conflicts are silently missed at the grace boundary
    // timestep. Fix by pairing entries by agent ID, not vector index.
    for (size_t i = 0; i < it->second.size() && i < it_next->second.size(); ++i) {
      const auto& cur  = it->second[i];
      const auto& next = it_next->second[i];
      // Edge conflict: after swap both agents end up too close
      // to each other's previous position
      if (cells_conflict(fr, fc, my_footprint_cells,
                          next.cell.row, next.cell.col, cur.footprint_cells) &&
          cells_conflict(tr, tc, my_footprint_cells,
                          cur.cell.row, cur.cell.col, cur.footprint_cells))
        return true;
    }
    return false;
  }

  // Can the agent hold its goal starting from from_time?
  // Uses footprint (hard) radius — agent can hold goal as long as no
  // physical overlap with other agents' paths or held goals.
  bool can_hold_goal(size_t row, size_t col, size_t from_time,
                      size_t max_time, float my_footprint_cells) const {
    for (const auto& hg : held_goals_) {
      if (cells_conflict(row, col, my_footprint_cells,
                          hg.cell.row, hg.cell.col, hg.footprint_cells))
        return false;
    }
    for (size_t t = from_time; t <= max_time; ++t) {
      auto it = entries_.find(t);
      if (it == entries_.end()) continue;
      for (const auto& e : it->second) {
        if (cells_conflict(row, col, my_footprint_cells,
                            e.cell.row, e.cell.col, e.footprint_cells))
          return false;
      }
    }
    return true;
  }

 private:
  struct Entry {
    Cell cell;
    float footprint_cells;    // physical (hard) radius in cells
    float soft_radius_cells;  // footprint + inflation in cells
  };
  struct HeldGoal {
    Cell cell;
    float footprint_cells;
    float soft_radius_cells;
    size_t from_time;
  };

  std::unordered_map<size_t, std::vector<Entry>> entries_;  // time -> entries
  std::vector<HeldGoal> held_goals_;

  // Two-circle conflict check: distance between centres < sum of radii.
  // With zero radii — point-based cell coincidence check.
  static bool cells_conflict(size_t r1, size_t c1, float rad1,
                              size_t r2, size_t c2, float rad2) {
    const float min_dist = rad1 + rad2;
    if (min_dist <= 0.0f) {
      return r1 == r2 && c1 == c2;
    }
    const float dr = static_cast<float>(r1) - static_cast<float>(r2);
    const float dc = static_cast<float>(c1) - static_cast<float>(c2);
    return dr * dr + dc * dc < min_dist * min_dist;
  }
};

// ---------------------------------------------------------------------------
// Space-Time A*
// ---------------------------------------------------------------------------

class SpaceTimeAStarPlanner {
 public:
  SpaceTimeAStarPlanner() = default;

  void reset_map(const GridMap* map) { map_ = map; }
  const GridMap* current_map() const { return map_; }
  void set_goal_dists(const std::vector<int>* d) { goal_dists_ = d; }
  size_t last_expansions() const { return last_expansions_; }

  // my_footprint_cells — physical (hard) radius of this agent in cells.
  // my_soft_cells — footprint + inflation in cells (outer penalty boundary).
  // cost_curve — gradient shape (Linear, Quadratic, Cubic).
  // proximity_penalty — cost per step at the hard boundary.
  // max_astar_expansions — per-call expansion limit (0 = unlimited).
  bool find_path(const Cell& start, const Cell& goal,
                 const ReservationTable& reservations,
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
    if (is_blocked(start.row, start.col) || is_blocked(goal.row, goal.col)) return false;

    const size_t spatial = map_->rows * map_->cols;
    const size_t state_count = spatial * (max_time + 1);

    ensure_capacity(state_count);
    next_generation();

    const size_t start_idx = idx(start.row, start.col);
    const size_t goal_idx  = idx(goal.row, goal.col);
    const size_t start_state = state_idx(start_idx, 0, spatial);

    // NOTE: we intentionally do NOT reject starts that overlap with
    // reservations at t=0.  During replanning agents are physically at
    // their current positions and must be allowed to start there even if
    // another agent's reservation covers the same cell.  A* will
    // naturally find a path that diverges at t=1+.

    set_g(start_state, 0);
    parent_[start_state] = INVALID;

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
    open.push({start_idx, 0, 0, heuristic(start, goal)});

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

      const Cell c  = cell(cur.idx);
      const size_t r = c.row, col_ = c.col;
      const size_t nt = cur.t + 1;

      // wait in place
      relax(cur.idx, cur.t, r, col_, nt, goal, reservations, open, spatial);
      // neighbours
      if (r > 0)               relax(cur.idx, cur.t, r-1, col_,   nt, goal, reservations, open, spatial);
      if (r+1 < map_->rows)    relax(cur.idx, cur.t, r+1, col_,   nt, goal, reservations, open, spatial);
      if (col_ > 0)            relax(cur.idx, cur.t, r,   col_-1, nt, goal, reservations, open, spatial);
      if (col_+1 < map_->cols) relax(cur.idx, cur.t, r,   col_+1, nt, goal, reservations, open, spatial);
    }
    return false;
  }

 private:
  static constexpr int    INF     = std::numeric_limits<int>::max() / 4;
  static constexpr size_t INVALID = std::numeric_limits<size_t>::max();

  struct Node { size_t idx, t; int g, f; };
  struct NodeCmp {
    bool operator()(const Node& a, const Node& b) const {
      return a.f != b.f ? a.f > b.f : a.g < b.g;
    }
  };

  const GridMap*  map_           = nullptr;
  const std::vector<int>* goal_dists_ = nullptr;  // precomputed Dijkstra from goal
  float           my_footprint_  = 0.0f;   // current agent's hard radius in cells
  float           my_soft_       = 0.0f;   // current agent's soft radius in cells
  CostCurve       cost_curve_    = CostCurve::Quadratic;
  int             proximity_penalty_ = 50; // cost at hard boundary
  size_t          last_expansions_ = 0;    // expansions in last find_path call
  std::vector<int>      g_;
  std::vector<size_t>   parent_;
  std::vector<uint8_t>  closed_;
  std::vector<uint32_t> gen_;
  uint32_t current_gen_ = 1;

  size_t idx(size_t r, size_t c) const { return r * map_->cols + c; }
  Cell   cell(size_t i)          const { return {i / map_->cols, i % map_->cols}; }

  static size_t state_idx(size_t ci, size_t t, size_t sp) { return t * sp + ci; }
  static size_t state_cell(size_t si, size_t sp)           { return si % sp; }

  bool is_blocked(size_t r, size_t c) const { return map_->blocked[idx(r,c)] != 0; }

  int wall_penalty(size_t r, size_t c) const {
    return map_->wall_cost.empty() ? 0 : map_->wall_cost[idx(r,c)];
  }

  // Heuristic for A*: precomputed Dijkstra step-distance from goal.
  // Accounts for wall topology (blocked cells) but excludes penalties.
  // Admissible — always <= true cost.  Falls back to Manhattan if
  // goal_dists_ is not set (e.g. during testing).
  int heuristic(const Cell& a, const Cell& b) const {
    if (goal_dists_) {
      const int d = (*goal_dists_)[idx(a.row, a.col)];
      return d < INF ? d : INF;
    }
    const int dr = a.row > b.row ? a.row - b.row : b.row - a.row;
    const int dc = a.col > b.col ? a.col - b.col : b.col - a.col;
    return dr + dc;
  }

  void ensure_capacity(size_t n) {
    if (g_.size() >= n) return;
    g_.resize(n, INF); parent_.resize(n, INVALID);
    closed_.resize(n, 0); gen_.resize(n, 0);
  }

  void next_generation() {
    if (++current_gen_ == 0) { current_gen_ = 1; std::fill(gen_.begin(), gen_.end(), 0); }
  }

  bool   is_alive(size_t s)  const { return gen_[s] == current_gen_; }
  int    get_g(size_t s)     const { return is_alive(s) ? g_[s] : INF; }
  void   set_g(size_t s, int v)    { gen_[s] = current_gen_; g_[s] = v; closed_[s] = 0; }
  bool   is_closed(size_t s) const { return is_alive(s) && closed_[s]; }
  void   mark_closed(size_t s)     { closed_[s] = 1; }

  void relax(size_t ci, size_t ct, size_t nr, size_t nc, size_t nt,
             const Cell& goal, const ReservationTable& res,
             std::priority_queue<Node, std::vector<Node>, NodeCmp>& open,
             size_t sp) {
    if (is_blocked(nr, nc)) return;

    const size_t ni = idx(nr, nc);
    const bool moving = (ni != ci);

    // Wall penalty only when entering a new cell (not waiting).
    // Agent penalty every timestep (occupying space near them).
    int penalty = moving ? wall_penalty(nr, nc) : 0;

    const int agent_pen = res.vertex_penalty(
        nr, nc, nt, my_footprint_, my_soft_,
        proximity_penalty_, cost_curve_);
    if (agent_pen < 0) return;  // physical agent overlap forbidden
    penalty += agent_pen;

    // Edge conflict: check opposing movement (uses footprint/hard radius)
    const Cell from_cell = cell(ci);
    if (res.is_edge_blocked(from_cell.row, from_cell.col, nr, nc, ct, my_footprint_))
      return;

    const size_t cs = state_idx(ci, ct, sp);
    const size_t ns = state_idx(ni, nt, sp);
    if (is_closed(ns)) return;

    // Every timestep costs 1.  Movement adds 1 more — agents prefer
    // waiting over unnecessary walking.
    const int tg = get_g(cs) + 1 + (moving ? 1 : 0) + penalty;
    if (tg < get_g(ns)) {
      set_g(ns, tg);
      parent_[ns] = cs;
      open.push({ni, nt, tg, tg + heuristic({nr, nc}, goal)});
    }
  }

  void restore(size_t goal_state, size_t sp, Path& path) const {
    for (size_t cur = goal_state; cur != INVALID; cur = parent_[cur])
      path.push_back(cell(state_cell(cur, sp)));
    std::reverse(path.begin(), path.end());
  }
};

// ---------------------------------------------------------------------------
// Conflict Detector
// ---------------------------------------------------------------------------

class ConflictDetector {
 public:
  // Detect conflicts using footprint (hard/physical) radii only.
  // Soft-zone proximity is handled by A* penalties, not PBS branching.
  // Euclidean distance between centres vs (r_a + r_b).
  // For point agents (radius 0) — equivalent to cell coincidence check.
  // TODO Level 2: for orientation-dependent polygons —
  //   rasterise and check cell-set intersection.
  static Conflict find_first(const std::vector<Path>& paths,
                              const std::vector<float>& footprint_cells) {
    const size_t n = paths.size();
    const size_t T = max_len(paths);

    // Precompute grace periods for overlapping-start pairs.
    // Agents whose starts overlap (at hard radius) get
    // ceil(combined_radius - distance) + 1 timesteps to separate
    // before conflicts are enforced.
    std::vector<std::vector<size_t>> grace(n, std::vector<size_t>(n, 0));
    for (size_t i = 0; i < n; ++i)
      for (size_t j = i + 1; j < n; ++j) {
        const size_t g = start_grace(paths[i][0], footprint_cells[i],
                                     paths[j][0], footprint_cells[j]);
        grace[i][j] = grace[j][i] = g;
      }

    for (size_t t = 0; t < T; ++t) {
      for (size_t i = 0; i < n; ++i) {
        const Cell ci = at(paths[i], t);
        for (size_t j = i + 1; j < n; ++j) {
          // Skip all conflicts during grace period for overlapping starts
          if (t < grace[i][j]) continue;

          const Cell cj = at(paths[j], t);

          // Vertex conflict: physical overlap (hard radii)
          if (cells_conflict(ci, footprint_cells[i], cj, footprint_cells[j])) {
            return {ConflictType::Vertex, i, j, t, ci, cj};
          }

          // Edge conflict: opposing movement
          if (t > 0) {
            const Cell pi = at(paths[i], t - 1);
            const Cell pj = at(paths[j], t - 1);
            // i moved pi->ci, j moved pj->cj
            // Conflict if after swapping positions both are too close
            if (cells_conflict(pi, footprint_cells[i], cj, footprint_cells[j]) &&
                cells_conflict(pj, footprint_cells[j], ci, footprint_cells[i]))
              return {ConflictType::Edge, i, j, t - 1, pi, ci};
          }
        }
      }
    }
    return {};
  }

  // How many timesteps two overlapping agents need to separate.
  // Returns 0 if starts don't overlap.
  static size_t start_grace(const Cell& a, float ra, const Cell& b, float rb) {
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
  // Two-circle conflict check.
  // With zero radii — cell coincidence.
  static bool cells_conflict(const Cell& a, float ra, const Cell& b, float rb) {
    const float min_dist = ra + rb;
    if (min_dist <= 0.0f) {
      return a.row == b.row && a.col == b.col;
    }
    const float dr = static_cast<float>(a.row) - static_cast<float>(b.row);
    const float dc = static_cast<float>(a.col) - static_cast<float>(b.col);
    return dr * dr + dc * dc < min_dist * min_dist;
  }
};

// ---------------------------------------------------------------------------
// Priority Graph
// ---------------------------------------------------------------------------

class PriorityGraph {
 public:
  explicit PriorityGraph(size_t n = 0) : n_(n), m_(n * n, 0) {}

  size_t size() const { return n_; }

  bool has(size_t hi, size_t lo) const { return at(hi, lo); }

  bool add(size_t hi, size_t lo) {
    if (hi == lo) return false;
    if (has(lo, hi)) return false;
    if (has(hi, lo)) return true;

    std::vector<size_t> preds, succs;
    for (size_t i = 0; i < n_; ++i) if (i == hi || has(i, hi)) preds.push_back(i);
    for (size_t j = 0; j < n_; ++j) if (j == lo || has(lo, j)) succs.push_back(j);
    for (size_t p : preds) for (size_t s : succs) if (p == s) return false;
    for (size_t p : preds) for (size_t s : succs) set(p, s, 1);
    return true;
  }

  std::vector<size_t> higher_than(size_t a) const {
    std::vector<size_t> r;
    for (size_t i = 0; i < n_; ++i) if (has(i, a)) r.push_back(i);
    return r;
  }

  std::vector<size_t> topo_from(size_t root) const {
    std::vector<uint8_t> sel(n_, 0);
    sel[root] = 1;
    for (size_t v = 0; v < n_; ++v) if (has(root, v)) sel[v] = 1;

    std::vector<int> indeg(n_, 0);
    for (size_t u = 0; u < n_; ++u) {
      if (!sel[u]) continue;
      for (size_t v = 0; v < n_; ++v) if (sel[v] && has(u, v)) ++indeg[v];
    }

    std::deque<size_t> q;
    for (size_t v = 0; v < n_; ++v) if (sel[v] && indeg[v] == 0) q.push_back(v);

    std::vector<size_t> order;
    while (!q.empty()) {
      size_t v = q.front(); q.pop_front();
      order.push_back(v);
      for (size_t to = 0; to < n_; ++to) {
        if (sel[to] && has(v, to) && --indeg[to] == 0) q.push_back(to);
      }
    }
    return order;
  }

  size_t hash() const {
    size_t h = 0;
    for (size_t i = 0; i < m_.size(); i += 8) {
      uint64_t b = 0;
      for (size_t j = 0; j < 8 && i+j < m_.size(); ++j)
        b |= static_cast<uint64_t>(m_[i+j] != 0) << j;
      h ^= std::hash<uint64_t>{}(b) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
  }

  bool operator==(const PriorityGraph& o) const { return n_ == o.n_ && m_ == o.m_; }

 private:
  size_t n_;
  std::vector<uint8_t> m_;

  uint8_t at(size_t i, size_t j) const { return m_[i * n_ + j]; }
  void    set(size_t i, size_t j, uint8_t v) { m_[i * n_ + j] = v; }
};

// ---------------------------------------------------------------------------
// PBS Solver (public API)
// ---------------------------------------------------------------------------

class PBSSolver {
 public:
  PBSSolver() = default;

  void set_map(const GridMap* map) {
    map_ = map;
    inflated_cache_.clear();
    dist_cache_.clear();
  }

  // Main entry point. Returns true if a solution is found.
  // solution[i] — path for agents[i], each element is a Cell in grid coordinates.
  // resolution — PBS grid cell size in metres (for radius conversion).
  // stats — if non-null, filled with solve statistics.
  // cost_curve — gradient shape (Linear, Quadratic, Cubic).
  // proximity_penalty — cost per step at the hard boundary (0 at soft boundary).
  // max_astar_expansions — per-A* call expansion limit (0 = unlimited).
  bool solve(const std::vector<Agent>& agents, std::vector<Path>& solution,
             float resolution = 0.0f, SolveStats* stats = nullptr,
             size_t max_expansions = 5000,
             CostCurve cost_curve = CostCurve::Quadratic,
             int proximity_penalty = 50,
             size_t max_astar_expansions = 200000) {
    solution.clear();
    if (!map_ || agents.empty()) return false;

    const size_t n = agents.size();
    stats_ = stats;
    cost_curve_ = cost_curve;
    proximity_penalty_ = proximity_penalty;
    max_astar_expansions_ = max_astar_expansions;

    // Precompute radii in cells: footprint (hard) and soft (footprint+inflation)
    footprint_cells_.resize(n);
    soft_cells_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      footprint_cells_[i] = (resolution > 0.0f && agents[i].footprint_radius > 0.0f)
          ? agents[i].footprint_radius / resolution
          : 0.0f;
      soft_cells_[i] = (resolution > 0.0f)
          ? (agents[i].footprint_radius + agents[i].inflation) / resolution
          : 0.0f;
    }

    // Cache gradient-inflated maps for each unique soft_radius.
    // Smaller robots fit through gaps that larger formations cannot.
    // Hard boundary = footprint, soft boundary = footprint + inflation.
    for (size_t i = 0; i < n; ++i) {
      const float soft = agents[i].footprint_radius + agents[i].inflation;
      if (soft > 0.0f && inflated_cache_.find(soft) == inflated_cache_.end()) {
        inflated_cache_[soft] = map_->inflate_gradient(
            agents[i].footprint_radius, soft, resolution,
            proximity_penalty, cost_curve);
      }
    }

    // Precompute Dijkstra distances from each agent's goal on its
    // gradient map.  Used as A* heuristic and for agent ordering.
    // Cached by goal cell index (agents sharing a goal reuse the map).
    agent_goal_dists_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      set_agent_map(agents[i]);
      const size_t key = agents[i].goal.row * map_->cols + agents[i].goal.col;
      if (dist_cache_.find(key) == dist_cache_.end()) {
        dist_cache_[key] = bfs_from(*low_level_.current_map(),
                                          agents[i].goal);
      }
      agent_goal_dists_[i] = &dist_cache_[key];
    }

    // Per-agent true distances (Dijkstra cost from start to goal).
    // Used for branch ordering and root path ordering.
    agent_dists_.resize(n);
    size_t max_dist = 0;
    for (size_t i = 0; i < n; ++i) {
      const size_t si = agents[i].start.row * map_->cols + agents[i].start.col;
      const int d = (*agent_goal_dists_[i])[si];
      agent_dists_[i] = d < std::numeric_limits<int>::max() / 4
          ? static_cast<size_t>(d) : 0;
      max_dist = std::max(max_dist, agent_dists_[i]);
    }

    // max_t: time horizon for Space-Time A*.
    // Critical: a 600x600 map = 360k cells; at max_t=1000
    // Space-Time A* allocates 360M states and hangs.
    const size_t max_t = std::min<size_t>(max_dist * 4 + 64, 600);

    // Root node: sequential root planning.
    // Plan agents by decreasing distance to goal — longest paths get
    // the most freedom, stationary agents (dist=0) plan last and must
    // yield to moving agents.
    PBSNode root;
    root.pg = PriorityGraph(n);
    root.paths.resize(n);

    if (stats) stats->diag.max_t_used = max_t;

    std::vector<size_t> root_order(n);
    for (size_t i = 0; i < n; ++i) root_order[i] = i;
    std::sort(root_order.begin(), root_order.end(), [&](size_t a, size_t b) {
      return agent_dists_[a] > agent_dists_[b];
    });

    ReservationTable root_res;
    for (size_t idx : root_order) {
      set_agent_map(agents[idx]);
      low_level_.set_goal_dists(agent_goal_dists_[idx]);
      // Root planning: uncapped A* — quality root paths prevent PBS explosion.
      if (!low_level_.find_path(agents[idx].start, agents[idx].goal,
                                 root_res, footprint_cells_[idx], soft_cells_[idx],
                                 max_t, root.paths[idx],
                                 cost_curve, proximity_penalty)) {
        record_astar(false);
        // Fallback: A* without reservations. PBS resolves conflicts.
        ReservationTable empty;
        if (!low_level_.find_path(agents[idx].start, agents[idx].goal,
                                   empty, footprint_cells_[idx], soft_cells_[idx],
                                   max_t, root.paths[idx],
                                   cost_curve, proximity_penalty)) {
          record_astar(false);
          if (stats) {
            stats->diag.fail_reason = FailReason::RootPathFailed;
            stats->diag.fail_agent = idx;
          }
          return false;
        }
        record_astar(true);
      } else {
        record_astar(true);
      }
      // Reserve this agent's path so subsequent agents avoid it
      root_res.reserve_path(root.paths[idx], max_t,
                            footprint_cells_[idx], soft_cells_[idx]);
    }
    root.cost = cost(root.paths);

    std::priority_queue<PBSNode, std::vector<PBSNode>, PBSNodeCmp> open;
    std::unordered_set<NodeKey, NodeKeyHash> visited;

    open.push(root);
    visited.insert(make_key(root));

    size_t expansions = 0;

    while (!open.empty()) {
      if (++expansions > max_expansions) {
        if (stats) {
          stats->expansions = expansions;
          stats->diag.fail_reason = FailReason::MaxExpansions;
        }
        return false;
      }

      PBSNode node = open.top(); open.pop();

      const Conflict c = ConflictDetector::find_first(
          node.paths, footprint_cells_);
      if (c.type == ConflictType::None) {
        solution = node.paths;
        if (stats) {
          stats->expansions = expansions;
          stats->max_path_length = 0;
          for (const auto& p : solution)
            stats->max_path_length = std::max(stats->max_path_length, p.size());
        }
        return true;
      }

      // Branch ordering heuristic: try giving priority to the agent
      // with the further goal first — it needs more space/freedom.
      size_t hi = c.agent1, lo = c.agent2;
      if (agent_dists_[lo] > agent_dists_[hi]) std::swap(hi, lo);

      // branch 1: further-goal agent gets priority
      PBSNode ch1 = node;
      if (branch(agents, hi, lo, max_t, ch1)) {
        auto k = make_key(ch1);
        if (visited.insert(k).second) { ch1.cost = cost(ch1.paths); open.push(std::move(ch1)); }
        if (stats) ++stats->diag.branches_tried;
      } else {
        if (stats) ++stats->diag.branches_failed;
      }

      // branch 2: closer-goal agent gets priority
      PBSNode ch2 = std::move(node);
      if (branch(agents, lo, hi, max_t, ch2)) {
        auto k = make_key(ch2);
        if (visited.insert(k).second) { ch2.cost = cost(ch2.paths); open.push(std::move(ch2)); }
        if (stats) ++stats->diag.branches_tried;
      } else {
        if (stats) ++stats->diag.branches_failed;
      }
    }
    if (stats) {
      stats->expansions = expansions;
      stats->diag.fail_reason = FailReason::BranchExhausted;
    }
    return false;
  }

 private:
  struct PBSNode {
    PriorityGraph    pg;
    std::vector<Path> paths;
    size_t           cost = 0;
  };
  struct PBSNodeCmp {
    bool operator()(const PBSNode& a, const PBSNode& b) const { return a.cost > b.cost; }
  };
  struct NodeKey {
    size_t gh;
    std::vector<size_t> lens;
    bool operator==(const NodeKey& o) const { return gh == o.gh && lens == o.lens; }
  };
  struct NodeKeyHash {
    size_t operator()(const NodeKey& k) const {
      size_t h = k.gh;
      for (size_t x : k.lens) h ^= std::hash<size_t>{}(x) + 0x9e3779b9 + (h<<6) + (h>>2);
      return h;
    }
  };

  const GridMap*         map_ = nullptr;
  SpaceTimeAStarPlanner  low_level_;
  SolveStats*            stats_ = nullptr;  // set during solve(), nullable
  CostCurve              cost_curve_ = CostCurve::Quadratic;
  int                    proximity_penalty_ = 50;
  size_t                 max_astar_expansions_ = 200000;

  // Call after each find_path() to accumulate A* stats.
  void record_astar(bool ok) {
    if (!stats_) return;
    const size_t e = low_level_.last_expansions();
    if (ok) {
      stats_->astar.ok_total_exp += e;
      stats_->astar.ok_max_exp = std::max(stats_->astar.ok_max_exp, e);
      ++stats_->astar.ok_count;
    } else {
      ++stats_->astar.fail_count;
    }
  }
  std::vector<float>     footprint_cells_;  // per-agent hard radius in cells
  std::vector<float>     soft_cells_;       // per-agent soft radius in cells
  std::vector<size_t>    agent_dists_;      // per-agent true distance to goal
  std::vector<const std::vector<int>*> agent_goal_dists_;  // per-agent Dijkstra map

  // Cache of gradient-inflated maps: soft_radius (in metres) -> GridMap
  std::unordered_map<float, GridMap> inflated_cache_;

  // Dijkstra distance cache: goal_cell_index -> distance map.
  // Precomputed backward from each goal on the agent's gradient map.
  std::unordered_map<size_t, std::vector<int>> dist_cache_;

  // Compute shortest-path distances (in steps) from every cell to the
  // goal via BFS.  All edges have uniform cost 1, so BFS is optimal
  // (O(V) vs O(V log V) for Dijkstra with a priority queue).
  // Accounts for wall topology (blocked cells) but excludes penalties.
  // Used as A* heuristic and for agent ordering / max_t computation.
  static std::vector<int> bfs_from(const GridMap& map, const Cell& goal) {
    const size_t N = map.rows * map.cols;
    static constexpr int INF = std::numeric_limits<int>::max() / 4;
    std::vector<int> dist(N, INF);

    std::deque<size_t> q;
    const size_t gi = goal.row * map.cols + goal.col;
    dist[gi] = 0;
    q.push_back(gi);

    while (!q.empty()) {
      const size_t ci = q.front(); q.pop_front();
      const int d = dist[ci];
      const size_t r = ci / map.cols;
      const size_t c = ci % map.cols;

      auto relax = [&](size_t nr, size_t nc) {
        if (nr >= map.rows || nc >= map.cols) return;
        const size_t ni = nr * map.cols + nc;
        if (map.blocked[ni]) return;
        if (d + 1 < dist[ni]) {
          dist[ni] = d + 1;
          q.push_back(ni);
        }
      };

      if (r > 0)            relax(r - 1, c);
      if (r + 1 < map.rows) relax(r + 1, c);
      if (c > 0)            relax(r, c - 1);
      if (c + 1 < map.cols) relax(r, c + 1);
    }
    return dist;
  }

  // Set the map for a specific agent's A* (gradient-inflated).
  // Smaller robots get a less inflated map and can pass through
  // narrow gaps inaccessible to larger formations.
  void set_agent_map(const Agent& a) {
    const float soft = a.footprint_radius + a.inflation;
    if (soft > 0.0f) {
      auto it = inflated_cache_.find(soft);
      if (it != inflated_cache_.end()) {
        low_level_.reset_map(&it->second);
        return;
      }
    }
    low_level_.reset_map(map_);
  }

  static size_t cost(const std::vector<Path>& ps) {
    size_t c = 0;
    for (const auto& p : ps) if (!p.empty()) c += p.size() - 1;
    return c;
  }

  static NodeKey make_key(const PBSNode& n) {
    NodeKey k; k.gh = n.pg.hash();
    for (const auto& p : n.paths) k.lens.push_back(p.size());
    return k;
  }

  bool branch(const std::vector<Agent>& agents, size_t hi, size_t lo,
              size_t max_t, PBSNode& node) {
    if (!node.pg.add(hi, lo)) return false;
    return replan(agents, lo, max_t, node);
  }

  bool replan(const std::vector<Agent>& agents, size_t changed,
              size_t max_t, PBSNode& node) {
    const auto order = node.pg.topo_from(changed);
    for (size_t ai : order) {
      // Reserve paths of all higher-priority agents with their radii.
      // For agents whose starts overlap with ours, skip reservation
      // entries during the grace period so A* can plan an escape path.
      ReservationTable res;
      for (size_t bi : node.pg.higher_than(ai)) {
        const size_t grace = ConflictDetector::start_grace(
            agents[ai].start, footprint_cells_[ai],
            agents[bi].start, footprint_cells_[bi]);
        res.reserve_path(node.paths[bi], max_t,
                         footprint_cells_[bi], soft_cells_[bi], grace);
      }

      // Use the gradient-inflated map for this agent
      set_agent_map(agents[ai]);
      low_level_.set_goal_dists(agent_goal_dists_[ai]);
      Path p;
      if (!low_level_.find_path(agents[ai].start, agents[ai].goal,
                                 res, footprint_cells_[ai], soft_cells_[ai],
                                 max_t, p,
                                 cost_curve_, proximity_penalty_,
                                 max_astar_expansions_)) {
        record_astar(false);
        return false;
      }
      record_astar(true);
      node.paths[ai] = std::move(p);
    }
    return true;
  }
};