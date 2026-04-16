// iros_llm_swarm_mapf/lns2/warm_start.hpp
//
// Build a warm-start Solution for LNS2Solver::solve_from() from the paths
// of a previous plan, shifted forward in time and patched to match current
// robot positions.
//
// This is stateless; the caller provides everything. Typical use:
//   Solution seed;
//   WarmStartReport rep;
//   build_warm_seed(prev_plan, actuals, ..., &seed, &rep);
//   if (should_warm_start(rep, params)) solver.solve_from(std::move(seed), ...)
//   else solver.solve(agents, ...);

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "iros_llm_swarm_mapf/lns2/solution.hpp"
#include "iros_llm_swarm_mapf/lns2/types.hpp"

namespace lns2 {

// Previous plan: what each active agent was executing.
struct PreviousPlan {
  // For each agent id (full range [0, num_agents)), the previously planned
  // grid path. Empty vector for agents that were not in the plan.
  std::vector<Path> paths;
  // Timesteps to which the plan's path extends with hold-at-goal. Typically
  // equal to params.astar.horizon from the time the plan was built.
  std::vector<Timestep> hold_until;
  // How many discrete timesteps elapsed since the plan was constructed.
  // delta_steps = floor((t_now - plan_origin_time) / time_step_sec).
  Timestep delta_steps = 0;
};

struct ActualState {
  // current_cells[i] = where the robot actually is right now (grid cell).
  std::vector<Cell> current_cells;
  // have_odom[i] = whether current_cells[i] is valid.
  std::vector<std::uint8_t> have_odom;
  // goals[i] = the target cell for this agent right now (may have changed
  // since the previous plan).
  std::vector<Cell> goals;
};

struct WarmStartParams {
  // Max L-infty distance (in cells) between expected cell and actual cell
  // for "on-track" classification (patch rather than stub).
  std::int32_t patch_radius_cells = 2;
};

struct WarmStartReport {
  std::size_t agents_shifted_exact = 0;  // actual == expected
  std::size_t agents_patched       = 0;  // actual within patch_radius
  std::size_t agents_stubbed       = 0;  // actual far from plan -> one-cell path
  std::size_t agents_goal_changed  = 0;
  std::size_t agents_new           = 0;  // agent was not in prev plan
  std::size_t agents_missing       = 0;  // no odom -> skipped, left empty
  std::size_t seed_collisions      = 0;
};

// Build a warm-start Solution from the given inputs.
// `agents` is the full list for the upcoming plan (with their goals).
// Returns filled `out_solution` (already init'ed internally) and `out_report`.
void build_warm_seed(const std::vector<Agent>& agents,
                      const GridMap& grid,
                      const PreviousPlan& prev,
                      const ActualState& actual,
                      const WarmStartParams& params,
                      Timestep horizon,
                      Solution* out_solution,
                      WarmStartReport* out_report);

// Decide whether the seed is worth using versus cold start.
bool should_warm_start(const WarmStartReport& rep,
                        std::size_t num_agents,
                        std::size_t max_collisions_per_agent = 2,
                        double max_stubs_ratio = 0.5);

}  // namespace lns2
