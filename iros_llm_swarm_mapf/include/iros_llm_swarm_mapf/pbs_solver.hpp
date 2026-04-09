#pragma once

#include "iros_llm_swarm_mapf/euclidean_astar.hpp"

#include <cassert>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ---------------------------------------------------------------------------
// Conflict
// ---------------------------------------------------------------------------

struct Conflict {
  size_t agent1 = 0;
  size_t agent2 = 0;
  size_t time   = 0;
};

// SolveDiagnostics, AStarStats, SolveStats defined in mapf_types.hpp

// ---------------------------------------------------------------------------
// Conflict Detector
// ---------------------------------------------------------------------------

class ConflictDetector {
 public:
  // Detect conflicts using capsule overlap of movement segments.
  // Each agent sweeps a capsule (circle along a line segment) per timestep.
  // Capsule overlap catches both positional overlap AND mid-step crossing
  // conflicts that point-based checks miss with diagonal/multi-cell moves.
  // Soft-zone proximity is handled by A* penalties, not PBS branching.
  static std::optional<Conflict> find_first(
      const std::vector<Path>& paths,
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
        for (size_t j = i + 1; j < n; ++j) {
          if (t < grace[i][j]) continue;

          // Build movement segments for this timestep
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

  // NOTE: does not clear dist_cache_ / inflated_cache_ — params are
  // expected to stay constant across calls.  Call set_map() to reset.
  void set_movement_params(float max_speed, float time_step_sec,
                            float resolution, float urgency = 1.0f) {
    move_set_ = MoveSet::generate(max_speed, time_step_sec, resolution);
    time_cost_ = urgency * max_speed * time_step_sec;
    resolution_ = resolution;
    low_level_.set_move_set(move_set_);
    low_level_.set_cost_params(time_cost_, resolution_);
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

    // Ensure footprint radius is large enough for the PBS grid resolution
    // to prevent wall clipping with diagonal/multi-cell moves.
    for (size_t i = 0; i < n; ++i) {
      if (resolution > 0.0f && agents[i].footprint_radius > 0.0f) {
        assert(agents[i].footprint_radius >= 0.7f * resolution &&
               "footprint_radius must be >= 0.7 * pbs_resolution to prevent wall clipping");
      }
    }

    // Cache gradient-inflated maps for each unique footprint_radius.
    // Smaller robots fit through gaps that larger formations cannot.
    // Hard boundary = footprint (determines blocked cells),
    // soft boundary = footprint + inflation (gradient penalty zone).
    for (size_t i = 0; i < n; ++i) {
      const float hard = agents[i].footprint_radius;
      const float soft = hard + agents[i].inflation;
      if (hard > 0.0f && inflated_cache_.find(hard) == inflated_cache_.end()) {
        inflated_cache_[hard] = map_->inflate_gradient(
            hard, soft, resolution,
            proximity_penalty, cost_curve);
      }
    }

    // Precompute Dijkstra distances from each agent's goal on its
    // gradient map.  Used as A* heuristic and for agent ordering.
    // Cached by (goal_cell, footprint_radius) — agents with different
    // hard radii have different blocked cells and different distances.
    agent_goal_dists_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      set_agent_map(agents[i]);
      const size_t goal_idx = agents[i].goal.row * map_->cols + agents[i].goal.col;
      const size_t key = goal_idx
          ^ (std::hash<float>{}(agents[i].footprint_radius) * 2654435761u);
      if (dist_cache_.find(key) == dist_cache_.end()) {
        dist_cache_[key] = dijkstra_from(*low_level_.current_map(),
                                               agents[i].goal, move_set_,
                                               time_cost_, resolution_);
      }
      agent_goal_dists_[i] = &dist_cache_[key];
    }

    // Per-agent true distances (Dijkstra cost from start to goal).
    // Used for branch ordering and root path ordering.
    agent_dists_.resize(n);
    size_t max_dist = 0;
    for (size_t i = 0; i < n; ++i) {
      const size_t si = agents[i].start.row * map_->cols + agents[i].start.col;
      const float d = (*agent_goal_dists_[i])[si];
      agent_dists_[i] = d < std::numeric_limits<float>::max() / 4.0f
          ? static_cast<size_t>(d) : 0;
      max_dist = std::max(max_dist, agent_dists_[i]);
    }

    // max_t: time horizon for Space-Time A*.
    // Based on Chebyshev cell distance (min steps in 8-connected grid),
    // not cost units. Multiplier accounts for obstacle detours.
    // TODO: per-agent time cap based on individual cell distance
    //       instead of shared max — short-path agents don't need the
    //       full horizon, reducing their state space.
    // Critical: a 600x600 map = 360k cells; at max_t=1000
    // Space-Time A* allocates 360M states and hangs.
    size_t max_cell_dist = 0;
    for (size_t i = 0; i < n; ++i) {
      const size_t dr = agents[i].start.row > agents[i].goal.row
          ? agents[i].start.row - agents[i].goal.row
          : agents[i].goal.row - agents[i].start.row;
      const size_t dc = agents[i].start.col > agents[i].goal.col
          ? agents[i].start.col - agents[i].goal.col
          : agents[i].goal.col - agents[i].start.col;
      max_cell_dist = std::max(max_cell_dist, std::max(dr, dc));
    }
    const size_t max_t = std::min<size_t>(max_cell_dist * 2 + 32, 600);

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

    SegmentReservationTable root_res;
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
        SegmentReservationTable empty;
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

      const auto c = ConflictDetector::find_first(
          node.paths, footprint_cells_);
      if (!c) {
        solution = node.paths;
        if (stats) {
          stats->expansions = expansions;
          stats->max_path_length = 0;
          for (const auto& p : solution)
            stats->max_path_length = std::max(stats->max_path_length, p.size());
        }
        return true;
      }

      // TODO: PBS conflict resolution is brute force — just tries both
      // priority orderings. Needs smarter strategy (CBS, ECBS, or
      // improved heuristics) to scale beyond simple scenarios.
      // Branch ordering heuristic: try giving priority to the agent
      // with the further goal first — it needs more space/freedom.
      size_t hi = c->agent1, lo = c->agent2;
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
  EuclideanAStarPlanner  low_level_;
  MoveSet                move_set_;
  float                  time_cost_   = 1.0f;
  float                  resolution_  = 1.0f;
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
  std::vector<const std::vector<float>*> agent_goal_dists_;  // per-agent Dijkstra map

  // Cache of gradient-inflated maps: footprint_radius (in metres) -> GridMap
  std::unordered_map<float, GridMap> inflated_cache_;

  // Dijkstra distance cache: hash(goal_cell, footprint_radius) -> distance map.
  // Precomputed backward from each goal on the agent's gradient map.
  std::unordered_map<size_t, std::vector<float>> dist_cache_;

  // Set the map for a specific agent's A* (gradient-inflated).
  // Smaller robots get a less inflated map and can pass through
  // narrow gaps inaccessible to larger formations.
  void set_agent_map(const Agent& a) {
    if (a.footprint_radius > 0.0f) {
      auto it = inflated_cache_.find(a.footprint_radius);
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
      SegmentReservationTable res;
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