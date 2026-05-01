// iros_llm_swarm_mapf/lns2/src/soft_astar.cpp

#include "iros_llm_swarm_mapf/lns2/soft_astar.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <queue>
#include <unordered_map>

namespace lns2 {

// ---------------------------------------------------------------------------
// Backward BFS on the static grid to build h-values.
// ---------------------------------------------------------------------------

static std::vector<int> build_h(const GridMap& grid, const Cell& goal,
                                  bool diagonal)
{
  const std::size_t N = grid.rows * grid.cols;
  std::vector<int> h(N, std::numeric_limits<int>::max());
  if (!grid.in_bounds(goal) || grid.is_blocked(goal)) return h;
  h[grid.index_of(goal)] = 0;

  std::queue<Cell> q;
  q.push(goal);

  static const CellOffset step4[] = {{1,0}, {-1,0}, {0,1}, {0,-1}};
  static const CellOffset step8[] = {{1,0}, {-1,0}, {0,1}, {0,-1},
                                       {1,1}, {1,-1}, {-1,1}, {-1,-1}};
  const CellOffset* steps = diagonal ? step8 : step4;
  const std::size_t  num_steps = diagonal ? 8u : 4u;

  while (!q.empty()) {
    Cell c = q.front(); q.pop();
    const int d = h[grid.index_of(c)];
    for (std::size_t i = 0; i < num_steps; ++i) {
      Cell n = c + steps[i];
      if (!grid.in_bounds(n) || grid.is_blocked(n)) continue;
      const CellIdx nidx = grid.index_of(n);
      if (h[nidx] > d + 1) {
        h[nidx] = d + 1;
        q.push(n);
      }
    }
  }
  return h;
}

const std::vector<int>& HeuristicCache::get(const GridMap& grid,
                                              const Cell& goal,
                                              bool diagonal)
{
  Key k{goal.row, goal.col, grid.rows, grid.cols, diagonal};
  auto it = cache_.find(k);
  if (it != cache_.end()) return it->second;
  return cache_.emplace(k, build_h(grid, goal, diagonal)).first->second;
}

// ---------------------------------------------------------------------------
// soft_astar
// ---------------------------------------------------------------------------

struct Node {
  CellIdx  cell   = 0;
  Timestep t      = 0;
  Cost     g      = 0;
  Cost     f      = 0;
  std::int32_t parent = -1;   // index into closed_nodes
};

struct OpenEntry {
  Cost     f;
  std::int32_t node_idx;
  // Greater-than for min-heap.
  bool operator<(const OpenEntry& o) const {
    if (f != o.f) return f > o.f;
    return node_idx > o.node_idx;
  }
};

SoftAStarResult soft_astar(
    const Agent& agent,
    const GridMap& grid,
    const CollisionTable& table,
    const SoftAStarParams& params,
    HeuristicCache& h_cache)
{
  SoftAStarResult out;

  // Degenerate: start == goal and footprint fits.
  if (!footprint_fits(agent.start, agent.footprint, grid)) {
    return out;  // impossible to place; failure
  }

  const std::vector<int>& h = h_cache.get(grid, agent.goal,
                                           params.diagonal_moves);
  if (!grid.in_bounds(agent.goal) || grid.is_blocked(agent.goal)) return out;
  const CellIdx goal_idx = grid.index_of(agent.goal);

  // Precompute tail-cost table for this goal.
  const std::vector<std::size_t> tail_cnt =
      table.build_tail_counts(goal_idx, params.horizon, agent.id);

  // Actions: 4-conn or 8-conn, always plus WAIT.
  static const CellOffset moves4[] = {
      {0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  static const CellOffset moves8[] = {
      {0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const CellOffset* moves = params.diagonal_moves ? moves8 : moves4;
  const std::size_t  num_moves = params.diagonal_moves ? 9u : 5u;

  // Closed list: best g per (cell, t).
  struct CT {
    CellIdx cell; Timestep t;
    bool operator==(const CT& o) const { return cell==o.cell && t==o.t; }
  };
  struct CTHash {
    std::size_t operator()(const CT& k) const noexcept {
      return (static_cast<std::size_t>(k.cell) << 20)
           ^ static_cast<std::size_t>(k.t);
    }
  };
  std::unordered_map<CT, Cost, CTHash> best_g;
  std::vector<Node> closed_nodes;
  closed_nodes.reserve(4096);

  std::priority_queue<OpenEntry> open;

  // Seed with start.
  const CellIdx start_idx = grid.index_of(agent.start);
  {
    Node s;
    s.cell = start_idx;
    s.t    = 0;
    // Vertex collisions at (start, 0)
    std::size_t v = 0;
    for (const auto& off : agent.footprint.offsets) {
      const Cell c = agent.start + off;
      if (!grid.in_bounds(c)) continue;
      v += table.vertex_count_at(grid.index_of(c), 0, agent.id);
    }
    s.g = static_cast<Cost>(v) * params.collision_penalty;
    const int hv = h[start_idx];
    if (hv == std::numeric_limits<int>::max()) return out;  // unreachable
    s.f = s.g + static_cast<Cost>(hv) * params.step_cost;
    s.parent = -1;
    closed_nodes.push_back(s);
    best_g[CT{s.cell, s.t}] = s.g;
    open.push(OpenEntry{s.f, 0});
  }

  Cost     best_total = kInfCost;
  std::int32_t best_goal_idx = -1;

  std::size_t expansions = 0;

  while (!open.empty()) {
    OpenEntry top = open.top(); open.pop();
    // Admissible-pruning: if this state's f already >= best_total, stop.
    if (top.f >= best_total) break;

    const Node cur = closed_nodes[top.node_idx];

    // Stale entry check
    auto git = best_g.find(CT{cur.cell, cur.t});
    if (git != best_g.end() && cur.g > git->second) continue;

    // Goal check with tail-cost bookkeeping
    if (cur.cell == goal_idx) {
      const Cost tail_cost =
          static_cast<Cost>(tail_cnt[std::min<Timestep>(cur.t, params.horizon)])
              * params.collision_penalty;
      const Cost total = cur.g + tail_cost;
      if (total < best_total) {
        best_total    = total;
        best_goal_idx = top.node_idx;
      }
      // Do NOT break: earlier arrival may have higher tail; keep exploring
      // while f < best_total.
    }

    if (++expansions > params.max_expansions) break;
    if (cur.t >= params.horizon) continue;

    const Cell cur_cell = grid.cell_of(cur.cell);
    const Timestep nt = cur.t + 1;

    for (std::size_t mi = 0; mi < num_moves; ++mi) {
      const CellOffset& m = moves[mi];
      const Cell next = cur_cell + m;
      if (!grid.in_bounds(next)) continue;
      const bool is_wait = (m.drow == 0 && m.dcol == 0);

      // Footprint must fit statically at next (for wait same as cur, still
      // check — defensive).
      if (!footprint_fits(next, agent.footprint, grid)) continue;

      // Heuristic check: unreachable cells pruned.
      const CellIdx nidx = grid.index_of(next);
      if (h[nidx] == std::numeric_limits<int>::max()) continue;

      // Edge cost
      Cost edge_c = params.step_cost;

      // Vertex collision penalty at (next, nt)
      std::size_t vcoll = 0;
      for (const auto& off : agent.footprint.offsets) {
        const Cell c = next + off;
        if (!grid.in_bounds(c)) continue;
        vcoll += table.vertex_count_at(grid.index_of(c), nt, agent.id);
      }
      edge_c += static_cast<Cost>(vcoll) * params.collision_penalty;

      // Edge-swap penalty (only for real movement)
      if (!is_wait) {
        const std::size_t ecoll = table.edge_swap_count(
            cur.cell, nidx, cur.t, agent.id);
        edge_c += static_cast<Cost>(ecoll) * params.collision_penalty;
      }

      const Cost new_g = cur.g + edge_c;
      const Cost new_f = new_g + static_cast<Cost>(h[nidx]) * params.step_cost;
      if (new_f >= best_total) continue;

      const CT key{nidx, nt};
      auto bit = best_g.find(key);
      if (bit != best_g.end() && new_g >= bit->second) continue;
      best_g[key] = new_g;

      Node nxt;
      nxt.cell = nidx;
      nxt.t    = nt;
      nxt.g    = new_g;
      nxt.f    = new_f;
      nxt.parent = top.node_idx;

      closed_nodes.push_back(nxt);
      open.push(OpenEntry{new_f, static_cast<std::int32_t>(closed_nodes.size() - 1)});
    }
  }

  out.expansions = expansions;

  if (best_goal_idx < 0) return out;   // no path found

  // Reconstruct path
  std::vector<Cell> rev;
  std::int32_t idx = best_goal_idx;
  while (idx >= 0) {
    const Node& n = closed_nodes[idx];
    rev.push_back(grid.cell_of(n.cell));
    idx = n.parent;
  }
  out.path.assign(rev.rbegin(), rev.rend());
  out.success  = true;
  out.arrive_t = static_cast<Timestep>(out.path.size() - 1);
  out.cost     = best_total;

  // Rough # of collision instances (not strictly needed):
  out.collisions = static_cast<std::size_t>(
      best_total / params.collision_penalty);
  return out;
}

}  // namespace lns2
