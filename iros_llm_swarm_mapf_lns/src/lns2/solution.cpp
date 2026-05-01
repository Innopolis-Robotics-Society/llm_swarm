// iros_llm_swarm_mapf/lns2/src/solution.cpp

#include "iros_llm_swarm_mapf/lns2/solution.hpp"

#include <algorithm>
#include <cassert>

namespace lns2 {

void Solution::init(const std::vector<Agent>& agents, const GridMap* grid)
{
  grid_ = grid;
  paths_.assign(agents.size(), Path{});
  footprints_.clear();
  footprints_.reserve(agents.size());
  hold_until_.assign(agents.size(), 0);
  for (std::size_t i = 0; i < agents.size(); ++i) {
    // The solver indexes agents by a DENSE internal id equal to their
    // position in this vector. Callers must remap external ids accordingly.
    assert(agents[i].id == static_cast<AgentId>(i) &&
           "Agent::id must equal its index in the agents vector. "
           "Remap external ids to dense [0, N) before calling the solver.");
    footprints_.push_back(agents[i].footprint);
  }
  table_.reset(agents.size(), grid);
}

void Solution::set_path(AgentId id, Path new_path, Timestep new_hold_until)
{
  assert(id < paths_.size());
  if (!paths_[id].empty()) {
    table_.remove_path(id, paths_[id], footprints_[id], hold_until_[id]);
  }
  paths_[id]     = std::move(new_path);
  hold_until_[id] = new_hold_until;
  if (!paths_[id].empty()) {
    table_.add_path(id, paths_[id], footprints_[id], hold_until_[id]);
  }
}

void Solution::clear_path(AgentId id)
{
  assert(id < paths_.size());
  if (!paths_[id].empty()) {
    table_.remove_path(id, paths_[id], footprints_[id], hold_until_[id]);
  }
  paths_[id].clear();
  hold_until_[id] = 0;
}

void Solution::set_global_hold(Timestep h)
{
  for (AgentId id = 0; id < paths_.size(); ++id) {
    if (paths_[id].empty()) continue;
    if (hold_until_[id] == h) continue;
    // Re-emit the path with updated hold. Cheapest is remove+add.
    table_.remove_path(id, paths_[id], footprints_[id], hold_until_[id]);
    hold_until_[id] = h;
    table_.add_path(id, paths_[id], footprints_[id], h);
  }
}

Snapshot Solution::take_and_clear(const std::vector<AgentId>& ids)
{
  Snapshot snap;
  snap.ids = ids;
  snap.old_paths.reserve(ids.size());
  snap.old_holds.reserve(ids.size());
  snap.collisions_before = table_.total_collisions();

  for (AgentId id : ids) {
    snap.old_paths.push_back(paths_[id]);   // copy
    snap.old_holds.push_back(hold_until_[id]);
    clear_path(id);
  }
  return snap;
}

void Solution::restore(Snapshot&& snap)
{
  for (std::size_t k = 0; k < snap.ids.size(); ++k) {
    AgentId id = snap.ids[k];
    set_path(id, std::move(snap.old_paths[k]), snap.old_holds[k]);
  }
}

bool Solution::debug_consistent() const
{
  const std::size_t full = table_.debug_full_recount();
  return full == table_.total_collisions();
}

}  // namespace lns2