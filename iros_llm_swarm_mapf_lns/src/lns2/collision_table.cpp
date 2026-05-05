// iros_llm_swarm_mapf/lns2/src/collision_table.cpp

#include "iros_llm_swarm_mapf/lns2/collision_table.hpp"

#include <algorithm>
#include <cassert>

namespace lns2 {

// ---------------------------------------------------------------------------
// Primitive add/remove for vertex and edge occupancy entries.
// Each primitive updates total_, per_agent_, and touches_ consistently.
// ---------------------------------------------------------------------------

void CollisionTable::add_vertex(CellIdx cell, Timestep t, AgentId id)
{
  auto& bucket = vertex_occ_[CellTime{cell, t}];
  for (AgentId other : bucket) {
    if (other == id) continue;  // shouldn't happen, but guard
    total_ += 1;
    per_agent_[id]    += 1;
    per_agent_[other] += 1;
    touches_[id].insert(other);
    touches_[other].insert(id);
  }
  bucket.push_back(id);
}

void CollisionTable::remove_vertex(CellIdx cell, Timestep t, AgentId id)
{
  auto it = vertex_occ_.find(CellTime{cell, t});
  if (it == vertex_occ_.end()) return;
  auto& bucket = it->second;

  auto pos = std::find(bucket.begin(), bucket.end(), id);
  if (pos == bucket.end()) return;

  // Decrement collision count for every other agent in the bucket.
  for (AgentId other : bucket) {
    if (other == id) continue;
    assert(total_ >= 1);
    total_ -= 1;
    assert(per_agent_[id]    >= 1);
    assert(per_agent_[other] >= 1);
    per_agent_[id]    -= 1;
    per_agent_[other] -= 1;
    // touches_ is left over-approximating; rebuild_touches() fixes it lazily.
  }

  // Swap-remove.
  *pos = bucket.back();
  bucket.pop_back();
  if (bucket.empty()) vertex_occ_.erase(it);
}

void CollisionTable::add_edge(CellIdx from, CellIdx to, Timestep t, AgentId id)
{
  // Register this traversal.
  auto& bucket = edge_occ_[EdgeKey{from, to, t}];
  bucket.push_back(id);

  // Count swap conflict with any agent doing the reverse edge at the same t.
  auto it = edge_occ_.find(EdgeKey{to, from, t});
  if (it == edge_occ_.end()) return;
  for (AgentId other : it->second) {
    if (other == id) continue;
    total_ += 1;
    per_agent_[id]    += 1;
    per_agent_[other] += 1;
    touches_[id].insert(other);
    touches_[other].insert(id);
  }
}

void CollisionTable::remove_edge(CellIdx from, CellIdx to, Timestep t, AgentId id)
{
  // Undo swap-conflict bookkeeping first (before removing this agent from
  // the edge bucket, since we need to know who else is on the reverse edge).
  auto rev = edge_occ_.find(EdgeKey{to, from, t});
  if (rev != edge_occ_.end()) {
    for (AgentId other : rev->second) {
      if (other == id) continue;
      assert(total_ >= 1);
      total_ -= 1;
      per_agent_[id]    -= 1;
      per_agent_[other] -= 1;
    }
  }

  auto it = edge_occ_.find(EdgeKey{from, to, t});
  if (it == edge_occ_.end()) return;
  auto& bucket = it->second;
  auto pos = std::find(bucket.begin(), bucket.end(), id);
  if (pos == bucket.end()) return;
  *pos = bucket.back();
  bucket.pop_back();
  if (bucket.empty()) edge_occ_.erase(it);
}

// ---------------------------------------------------------------------------
// Path-level add / remove
// ---------------------------------------------------------------------------

void CollisionTable::add_path(AgentId id, const Path& path,
                               const FootprintModel& footprint,
                               Timestep hold_until)
{
  if (path.empty()) return;

  const Timestep L = static_cast<Timestep>(path.size());
  const Timestep last = std::max<Timestep>(L, hold_until + 1);

  // Vertex occupancy per timestep, with goal-hold extension.
  for (Timestep t = 0; t < last; ++t) {
    const Cell base = (t < L) ? path[t] : path.back();
    for_each_footprint_cell(base, footprint, [&](CellIdx idx) {
      add_vertex(idx, t, id);
    });
  }

  // Edges — only real movement, only within path length.
  for (Timestep t = 0; t + 1 < L; ++t) {
    if (path[t] == path[t + 1]) continue;  // wait
    add_edge(grid_->index_of(path[t]),
             grid_->index_of(path[t + 1]), t, id);
  }
}

void CollisionTable::remove_path(AgentId id, const Path& path,
                                  const FootprintModel& footprint,
                                  Timestep hold_until)
{
  if (path.empty()) return;

  const Timestep L = static_cast<Timestep>(path.size());
  const Timestep last = std::max<Timestep>(L, hold_until + 1);

  for (Timestep t = 0; t < last; ++t) {
    const Cell base = (t < L) ? path[t] : path.back();
    for_each_footprint_cell(base, footprint, [&](CellIdx idx) {
      remove_vertex(idx, t, id);
    });
  }

  for (Timestep t = 0; t + 1 < L; ++t) {
    if (path[t] == path[t + 1]) continue;
    remove_edge(grid_->index_of(path[t]),
                grid_->index_of(path[t + 1]), t, id);
  }
}

// ---------------------------------------------------------------------------
// Tail-cost table for a given goal cell
// ---------------------------------------------------------------------------

std::vector<std::size_t> CollisionTable::build_tail_counts(
    CellIdx goal_cell, Timestep horizon, AgentId requester) const
{
  std::vector<std::size_t> tail(horizon + 2, 0);

  // tail[t] = sum over t' in [t, horizon] of vertex_count_at(goal_cell, t')
  // Built back-to-front.
  std::size_t running = 0;
  for (std::int64_t t = static_cast<std::int64_t>(horizon); t >= 0; --t) {
    running += vertex_count_at(goal_cell, static_cast<Timestep>(t), requester);
    tail[static_cast<std::size_t>(t)] = running;
  }
  return tail;
}

// ---------------------------------------------------------------------------
// Random collision sampling
// ---------------------------------------------------------------------------

std::optional<Collision> CollisionTable::sample_random_collision(
    std::mt19937& rng,
    const std::vector<Path>& paths,
    const std::vector<FootprintModel>& footprints,
    const std::vector<Timestep>& hold_until) const
{
  if (total_ == 0) return std::nullopt;

  // Weighted pick of agent a by per_agent_ counts.
  std::size_t total_weight = 0;
  for (std::size_t c : per_agent_) total_weight += c;
  if (total_weight == 0) return std::nullopt;

  std::uniform_int_distribution<std::size_t> dw(1, total_weight);
  std::size_t pick = dw(rng);
  AgentId a = 0;
  for (std::size_t i = 0; i < per_agent_.size(); ++i) {
    if (pick <= per_agent_[i]) { a = static_cast<AgentId>(i); break; }
    pick -= per_agent_[i];
  }

  // Pick any agent b from touches_[a] that has nonzero shared occupancy now.
  if (touches_[a].empty()) return std::nullopt;
  std::vector<AgentId> candidates(touches_[a].begin(), touches_[a].end());
  std::shuffle(candidates.begin(), candidates.end(), rng);

  // Walk a's path, find any (cell, t) where b also appears.
  const Path& pa = paths[a];
  if (pa.empty()) return std::nullopt;
  const FootprintModel& fa = footprints[a];
  const Timestep La = static_cast<Timestep>(pa.size());
  const Timestep last_a = std::max<Timestep>(La, hold_until[a] + 1);

  for (AgentId b : candidates) {
    // Try vertex collisions first.
    for (Timestep t = 0; t < last_a; ++t) {
      const Cell base = (t < La) ? pa[t] : pa.back();
      for (const auto& off : fa.offsets) {
        const Cell c = base + off;
        if (!grid_->in_bounds(c)) continue;
        const CellIdx idx = grid_->index_of(c);
        auto it = vertex_occ_.find(CellTime{idx, t});
        if (it == vertex_occ_.end()) continue;
        const auto& bucket = it->second;
        bool has_a = false, has_b = false;
        for (AgentId x : bucket) {
          if (x == a) has_a = true;
          else if (x == b) has_b = true;
        }
        if (has_a && has_b) {
          Collision col;
          col.a = a; col.b = b;
          col.cell_idx = idx; col.t = t; col.kind = Collision::Vertex;
          return col;
        }
      }
    }
    // Edge (swap) collisions.
    for (Timestep t = 0; t + 1 < La; ++t) {
      if (pa[t] == pa[t + 1]) continue;
      const CellIdx from = grid_->index_of(pa[t]);
      const CellIdx to   = grid_->index_of(pa[t + 1]);
      auto rev = edge_occ_.find(EdgeKey{to, from, t});
      if (rev == edge_occ_.end()) continue;
      for (AgentId x : rev->second) {
        if (x == b) {
          Collision col;
          col.a = a; col.b = b;
          col.cell_idx = from; col.t = t; col.kind = Collision::Edge;
          return col;
        }
      }
    }
  }

  // touches_[a] was over-approximating: none of the candidates actually
  // collide now. Return nullopt; caller will re-sample or bail.
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// Full rebuild of touches_ (lazy cleanup)
// ---------------------------------------------------------------------------

void CollisionTable::rebuild_touches()
{
  for (auto& s : touches_) s.clear();

  for (const auto& [key, bucket] : vertex_occ_) {
    if (bucket.size() < 2) continue;
    for (std::size_t i = 0; i < bucket.size(); ++i) {
      for (std::size_t j = i + 1; j < bucket.size(); ++j) {
        touches_[bucket[i]].insert(bucket[j]);
        touches_[bucket[j]].insert(bucket[i]);
      }
    }
  }

  for (const auto& [key, bucket] : edge_occ_) {
    auto rev = edge_occ_.find(EdgeKey{key.to, key.from, key.t});
    if (rev == edge_occ_.end()) continue;
    for (AgentId i : bucket) {
      for (AgentId j : rev->second) {
        if (i == j) continue;
        touches_[i].insert(j);
        touches_[j].insert(i);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Debug: recount from scratch and compare.
// ---------------------------------------------------------------------------

std::size_t CollisionTable::debug_full_recount() const
{
  std::size_t n = 0;
  for (const auto& [key, bucket] : vertex_occ_) {
    const std::size_t k = bucket.size();
    if (k >= 2) n += k * (k - 1) / 2;
  }
  for (const auto& [key, bucket] : edge_occ_) {
    auto rev = edge_occ_.find(EdgeKey{key.to, key.from, key.t});
    if (rev == edge_occ_.end()) continue;
    // Count each unordered pair once: only do it when key < rev-key by some
    // total order, e.g. from < to.
    if (!(key.from < key.to)) continue;
    n += bucket.size() * rev->second.size();
  }
  return n;
}

}  // namespace lns2
