// iros_llm_swarm_mapf/lns2/warm_start.hpp
//
// Build a warm-start Solution for LNS2Solver::solve_from() from the paths
// of a previous plan, shifted forward in time and patched to match current
// robot positions.
//
// Design (v2 — fixes the "per-agent k-shift" bug):
//   * A COMMON time shift = delta_steps is applied to ALL agents.
//     Previously, each agent was shifted by the index at which its actual
//     cell was found on its own prev_path ("find_actual_on_path"). That
//     destroyed the relative timing of the schedule: a robot that fell
//     behind got its path indexed earlier than a robot that kept pace,
//     and the resulting seed appeared to have many "new" collisions that
//     did not exist in the original plan.
//   * For agents still on-plan at t=delta: the tail of the prev_path is
//     used directly as the seed.
//   * For agents near-but-off the expected cell: a short BFS splice is
//     produced that joins actual_cell back onto the tail within a few
//     steps. Falls back to a one-cell hold if the splice can't be found.
//   * For agents that diverged far from the plan, have a changed goal,
//     are newly introduced, or whose prev_path is no longer valid on the
//     current map — a stub (one-cell hold) is emitted with an explicit
//     reason.
//
// This is stateless; the caller provides everything.

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
  // This is applied UNIFORMLY to every agent — see header comment.
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
  // for "on-track" classification (splice rather than stub).
  std::int32_t patch_radius_cells = 10;

  // Max BFS expansions when trying to splice actual -> expected. 0 disables
  // splice (forces stub for any off-track agent).
  std::size_t  splice_max_expansions = 2000;
};

// Per-agent classification of how the seed was produced. Useful both for
// diagnostics and for deciding which agents to freeze/hard-block in repair.
enum class SeedStatus : std::uint8_t {
  OnTrack      = 0,  // actual == expected; tail of prev_path used directly
  Spliced      = 1,  // actual near expected; short BFS splice prepended
  StubFar      = 2,  // actual far from expected; one-cell hold ("frozen")
  StubNew      = 3,  // agent was not in prev plan; one-cell hold at actual
  StubGoal     = 4,  // goal changed since prev plan; one-cell hold at actual
  StubInvalid  = 5,  // prev_path no longer statically valid; one-cell hold
  StubArrived  = 6,  // prev plan fully elapsed and robot is at goal
  Missing      = 7,  // no odometry; left empty (not in repair set)
};

struct WarmStartReport {
  // Per-agent seed status, indexed by agent's position in the `agents`
  // vector (dense internal id). Size == agents.size() after build.
  std::vector<SeedStatus> status;

  // Aggregate counts for logging / heuristic decisions.
  std::size_t agents_on_track     = 0;
  std::size_t agents_spliced      = 0;
  std::size_t agents_stubbed_far  = 0;
  std::size_t agents_new          = 0;
  std::size_t agents_goal_changed = 0;
  std::size_t agents_invalid      = 0;
  std::size_t agents_arrived      = 0;
  std::size_t agents_missing      = 0;

  // Seed collision count (after all paths placed in the CollisionTable).
  std::size_t seed_collisions     = 0;

  // Convenience: total stubs that the repair must actively resolve (i.e.
  // excludes OnTrack/Spliced/Arrived/Missing).
  std::size_t active_stubs() const {
    return agents_stubbed_far + agents_new +
           agents_goal_changed + agents_invalid;
  }
};

// Build a warm-start Solution from the given inputs.
// `agents` is the full list for the upcoming plan (with their goals).
// The agents' ids MUST be dense indices [0, agents.size()) — same contract
// as Solution::init.
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
