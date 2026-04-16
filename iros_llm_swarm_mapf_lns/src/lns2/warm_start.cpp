// iros_llm_swarm_mapf/lns2/src/warm_start.cpp

#include "iros_llm_swarm_mapf/lns2/warm_start.hpp"

#include <algorithm>
#include <cstdint>

namespace lns2 {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::int32_t chebyshev(const Cell& a, const Cell& b) {
  return std::max(std::abs(a.row - b.row), std::abs(a.col - b.col));
}

static bool path_statically_valid(const Path& p,
                                    const GridMap& grid,
                                    const FootprintModel& fp)
{
  for (const Cell& base : p) {
    for (const auto& off : fp.offsets) {
      const Cell c = base + off;
      if (!grid.in_bounds(c)) return false;
      if (grid.is_blocked(c)) return false;
    }
  }
  return true;
}

// Find index `k` such that prev_path[k] == actual and k >= delta_steps
// (the agent may have moved ahead or behind the expected step). Returns -1
// if not found.
static int find_actual_on_path(const Path& prev_path,
                                 const Cell& actual,
                                 Timestep delta_steps)
{
  // Search in a window around delta_steps — the robot is usually close.
  const int L = static_cast<int>(prev_path.size());
  if (L == 0) return -1;

  const int d  = static_cast<int>(delta_steps);
  const int W  = 5;  // +/- window
  const int lo = std::max(0, d - W);
  const int hi = std::min(L - 1, d + W);
  for (int k = lo; k <= hi; ++k) {
    if (prev_path[k] == actual) return k;
  }
  // Full scan as fallback
  for (int k = 0; k < L; ++k) {
    if (prev_path[k] == actual) return k;
  }
  return -1;
}

// ---------------------------------------------------------------------------
// build_warm_seed
// ---------------------------------------------------------------------------

void build_warm_seed(const std::vector<Agent>& agents,
                      const GridMap& grid,
                      const PreviousPlan& prev,
                      const ActualState& actual,
                      const WarmStartParams& params,
                      Timestep horizon,
                      Solution* out_solution,
                      WarmStartReport* out_report)
{
  WarmStartReport rep{};
  out_solution->init(agents, &grid);

  for (const Agent& a : agents) {
    const AgentId id = a.id;

    if (id >= actual.have_odom.size() || !actual.have_odom[id]) {
      rep.agents_missing += 1;
      continue;   // leave empty
    }

    const Cell actual_cell = actual.current_cells[id];

    // Goal change?
    bool goal_changed = false;
    if (id < prev.paths.size() && !prev.paths[id].empty()) {
      goal_changed = (prev.paths[id].back() != a.goal);
    }
    if (goal_changed) rep.agents_goal_changed += 1;

    // Agent new to the plan (no prev path)?
    const bool is_new = (id >= prev.paths.size() || prev.paths[id].empty());
    if (is_new) {
      rep.agents_new += 1;
      // Stub: one-cell path at actual position.
      out_solution->set_path(id, Path{actual_cell}, horizon);
      continue;
    }

    const Path& prev_path = prev.paths[id];

    // If goal changed or the prev path is statically invalid under current
    // map, stub.
    if (goal_changed ||
        !path_statically_valid(prev_path, grid, a.footprint)) {
      rep.agents_stubbed += 1;
      out_solution->set_path(id, Path{actual_cell}, horizon);
      continue;
    }

    // Time shift: drop first delta_steps of prev_path.
    const Timestep delta = prev.delta_steps;
    const int L = static_cast<int>(prev_path.size());

    if (static_cast<int>(delta) >= L) {
      // Whole path is in the past — agent should be at goal by now.
      // If actual is at goal, one-cell path at goal; else stub at actual.
      if (actual_cell == a.goal) {
        rep.agents_shifted_exact += 1;
        out_solution->set_path(id, Path{a.goal}, horizon);
      } else {
        rep.agents_stubbed += 1;
        out_solution->set_path(id, Path{actual_cell}, horizon);
      }
      continue;
    }

    // Expected cell at t=0 (relative to now) is prev_path[delta].
    const Cell expected = prev_path[static_cast<std::size_t>(delta)];

    if (actual_cell == expected) {
      rep.agents_shifted_exact += 1;
      Path shifted(prev_path.begin() + delta, prev_path.end());
      out_solution->set_path(id, std::move(shifted), horizon);
      continue;
    }

    // See if actual is on the prev_path at a different index.
    int k = find_actual_on_path(prev_path, actual_cell, delta);
    if (k >= 0) {
      rep.agents_shifted_exact += 1;
      Path shifted(prev_path.begin() + k, prev_path.end());
      out_solution->set_path(id, std::move(shifted), horizon);
      continue;
    }

    // Close-by patch: prepend actual_cell to the shifted path.
    if (chebyshev(actual_cell, expected) <= params.patch_radius_cells) {
      rep.agents_patched += 1;
      Path patched;
      patched.reserve(prev_path.size() - delta + 1);
      patched.push_back(actual_cell);
      patched.insert(patched.end(),
                     prev_path.begin() + delta + 1, prev_path.end());
      out_solution->set_path(id, std::move(patched), horizon);
      continue;
    }

    // Far from plan — stub.
    rep.agents_stubbed += 1;
    out_solution->set_path(id, Path{actual_cell}, horizon);
  }

  rep.seed_collisions = out_solution->num_collisions();
  if (out_report) *out_report = rep;
}

// ---------------------------------------------------------------------------
// Heuristic decision
// ---------------------------------------------------------------------------

bool should_warm_start(const WarmStartReport& rep,
                        std::size_t num_agents,
                        std::size_t max_collisions_per_agent,
                        double max_stubs_ratio)
{
  if (num_agents == 0) return false;

  if (rep.seed_collisions > num_agents * max_collisions_per_agent) return false;

  const double stubs_ratio =
      static_cast<double>(rep.agents_stubbed + rep.agents_goal_changed +
                           rep.agents_new) /
      static_cast<double>(num_agents);
  if (stubs_ratio > max_stubs_ratio) return false;

  return true;
}

}  // namespace lns2
