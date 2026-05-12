// iros_llm_swarm_mapf_lns/lns2/collision_table.hpp
//
// Footprint-aware vertex + edge occupancy tracker with incremental updates
// and O(1) count lookups for the soft A*.
//
// Invariants:
//   * total_collisions() is the sum over all unordered agent pairs (i, j),
//     i != j, of the number of (cell, t) entries where both i and j occupy
//     the same footprint cell at time t (vertex), plus the number of
//     (edge, t) swaps where i goes c->c' while j goes c'->c.
//   * per_agent_collisions[i] is the sum over j of pair collisions (i, j).
//     Thus sum(per_agent) == 2 * total_collisions().
//   * touches_[i] is a superset of the set of agents j that currently
//     share at least one (cell, t) or swap edge with i.
//     (It may over-approximate after removals; rebuild_touches() resets.)

#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "iros_llm_swarm_mapf_lns/lns2/types.hpp"

namespace lns2 {

struct CellTime {
  CellIdx cell = 0;
  Timestep t   = 0;
  bool operator==(const CellTime& o) const {
    return cell == o.cell && t == o.t;
  }
};

struct EdgeKey {
  CellIdx from = 0;
  CellIdx to   = 0;
  Timestep t   = 0;  // the step starting at t, ending at t+1
  bool operator==(const EdgeKey& o) const {
    return from == o.from && to == o.to && t == o.t;
  }
};

}  // namespace lns2

namespace std {

template <>
struct hash<lns2::CellTime> {
  std::size_t operator()(const lns2::CellTime& k) const noexcept {
    std::size_t h = static_cast<std::size_t>(k.cell);
    h ^= static_cast<std::size_t>(k.t) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

template <>
struct hash<lns2::EdgeKey> {
  std::size_t operator()(const lns2::EdgeKey& k) const noexcept {
    std::size_t h = static_cast<std::size_t>(k.from);
    h ^= static_cast<std::size_t>(k.to)  + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(k.t)   + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

}  // namespace std

namespace lns2 {

class CollisionTable {
 public:
  CollisionTable() = default;

  void reset(std::size_t num_agents, const GridMap* grid) {
    grid_ = grid;
    per_agent_.assign(num_agents, 0);
    touches_.assign(num_agents, {});
    vertex_occ_.clear();
    edge_occ_.clear();
    total_ = 0;
  }

  std::size_t num_agents() const { return per_agent_.size(); }
  std::size_t total_collisions() const { return total_; }
  std::size_t collisions_of(AgentId id) const {
    return id < per_agent_.size() ? per_agent_[id] : 0;
  }
  const std::unordered_set<AgentId>& agents_touching(AgentId id) const {
    static const std::unordered_set<AgentId> kEmpty;
    return id < touches_.size() ? touches_[id] : kEmpty;
  }

  // ---- Incremental path ops --------------------------------------------

  // add_path assumes the agent is currently NOT in the table.
  void add_path(AgentId id, const Path& path,
                const FootprintModel& footprint,
                Timestep hold_until);   // hold agent at path.back() up to hold_until

  // remove_path must be called with the EXACT same path/footprint/hold_until
  // that add_path was called with. The Solution wrapper guarantees this.
  void remove_path(AgentId id, const Path& path,
                   const FootprintModel& footprint,
                   Timestep hold_until);

  // ---- O(1) lookups used by soft A* ------------------------------------

  // Number of OTHER agents occupying `cell` at time `t`, excluding `requester`.
  std::size_t vertex_count_at(CellIdx cell, Timestep t,
                              AgentId requester) const {
    auto it = vertex_occ_.find(CellTime{cell, t});
    if (it == vertex_occ_.end()) return 0;
    std::size_t n = it->second.size();
    // subtract requester if it is listed (shouldn't be during repair, but
    // safe to check)
    for (AgentId a : it->second) if (a == requester) { --n; break; }
    return n;
  }

  // Number of OTHER agents traversing edge (from -> to) at time t in the
  // opposite direction (to -> from). This detects swap collisions.
  std::size_t edge_swap_count(CellIdx from, CellIdx to, Timestep t,
                              AgentId requester) const {
    auto it = edge_occ_.find(EdgeKey{to, from, t});  // opposite edge
    if (it == edge_occ_.end()) return 0;
    std::size_t n = it->second.size();
    for (AgentId a : it->second) if (a == requester) { --n; break; }
    return n;
  }

  // ---- Tail-cost precompute for soft A* --------------------------------

  // Returns tail[t] = number of OTHER agents occupying goal_cell at any
  // time >= t, summed. Used as the hold-at-goal contribution.
  std::vector<std::size_t> build_tail_counts(CellIdx goal_cell,
                                              Timestep horizon,
                                              AgentId requester) const;

  // ---- Sampling for destroy operators ----------------------------------

  // Draw a random collision. Returns nullopt if total_collisions()==0.
  // Requires agents' current paths to scan (provided by Solution).
  std::optional<Collision> sample_random_collision(
      std::mt19937& rng,
      const std::vector<Path>& paths,
      const std::vector<FootprintModel>& footprints,
      const std::vector<Timestep>& hold_until) const;

  // Rebuild the touches_ sets from scratch (called lazily by Solution).
  void rebuild_touches();

  // ---- Debug helper ----------------------------------------------------
  std::size_t debug_full_recount() const;

 private:
  const GridMap* grid_ = nullptr;

  std::unordered_map<CellTime, std::vector<AgentId>> vertex_occ_;
  std::unordered_map<EdgeKey,  std::vector<AgentId>> edge_occ_;

  std::size_t total_ = 0;
  std::vector<std::size_t> per_agent_;
  std::vector<std::unordered_set<AgentId>> touches_;

  // Helpers
  void add_vertex(CellIdx cell, Timestep t, AgentId id);
  void remove_vertex(CellIdx cell, Timestep t, AgentId id);
  void add_edge(CellIdx from, CellIdx to, Timestep t, AgentId id);
  void remove_edge(CellIdx from, CellIdx to, Timestep t, AgentId id);

  // Enumerate (cell_idx) for each footprint position at base_cell.
  template <typename F>
  void for_each_footprint_cell(const Cell& base,
                                const FootprintModel& fp, F&& f) const {
    for (const auto& off : fp.offsets) {
      const Cell c = base + off;
      if (!grid_->in_bounds(c)) continue;
      f(grid_->index_of(c));
    }
  }
};

}  // namespace lns2
