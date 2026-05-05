// iros_llm_swarm_mapf/lns2/lns2_solver.hpp
//
// Main MAPF solver: Large Neighborhood Search v2 (Li, Chan, Ma, Koenig 2022).
//
// Two entry points:
//   * solve()       — builds an initial solution (prioritized greedy) and
//                     runs destroy/repair loop.
//   * solve_from()  — takes a pre-built warm-start Solution and runs the
//                     loop without the initial construction phase.

#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <random>
#include <vector>

#include "iros_llm_swarm_mapf/lns2/destroy_operators.hpp"
#include "iros_llm_swarm_mapf/lns2/soft_astar.hpp"
#include "iros_llm_swarm_mapf/lns2/solution.hpp"
#include "iros_llm_swarm_mapf/lns2/types.hpp"

namespace lns2 {

struct LNS2Params {
  // A* / horizon
  SoftAStarParams astar;

  // Destroy / repair
  std::size_t neighborhood_size    = 8;
  std::size_t time_budget_ms       = 500;
  std::size_t plateau_limit        = 200;  // iters with no improvement -> stop
  std::size_t segment_size         = 50;   // ALNS weight update period
  double      alns_reaction        = 0.1;
  double      alns_w_min           = 0.1;

  // Wall-clock cap for build_initial_solution. When the budget is exhausted
  // the loop stops early; remaining agents get no initial path and the repair
  // loop handles them. 0 = no limit.
  std::size_t initial_time_budget_ms = 0;

  // RNG seed (0 => use random_device)
  std::uint64_t seed = 0;
};

struct LNS2Stats {
  bool        warm_started        = false;
  std::size_t iterations          = 0;
  std::size_t accepted_iterations = 0;
  std::size_t initial_collisions  = 0;
  std::size_t final_collisions    = 0;
  std::size_t total_astar_calls   = 0;
  std::size_t total_astar_expansions = 0;
  std::size_t total_astar_fails   = 0;
  double      wall_time_ms        = 0.0;

  // Per-operator stats
  std::array<std::size_t, static_cast<std::size_t>(DestroyOp::COUNT)> op_used{};
  std::array<std::size_t, static_cast<std::size_t>(DestroyOp::COUNT)> op_improved{};
  std::array<double, static_cast<std::size_t>(DestroyOp::COUNT)> op_final_weight{};
};

class LNS2Solver {
 public:
  LNS2Solver();

  // Non-warm: construct initial solution from scratch.
  bool solve(const std::vector<Agent>& agents,
             const GridMap& grid,
             const LNS2Params& params,
             std::vector<Path>* out_paths,
             LNS2Stats* stats = nullptr);

  // Warm: use provided seed Solution directly.
  bool solve_from(Solution&& seed,
                  const std::vector<Agent>& agents,
                  const GridMap& grid,
                  const LNS2Params& params,
                  std::vector<Path>* out_paths,
                  LNS2Stats* stats = nullptr);

 private:
  // Greedy prioritized initial construction: sort agents by h-distance to
  // goal, plan each one treating previously planned ones as fixed.
  void build_initial_solution(const std::vector<Agent>& agents,
                                const GridMap& grid,
                                const LNS2Params& params,
                                Solution& sol,
                                LNS2Stats& stats);

  // Main repair loop.
  void repair_loop(const std::vector<Agent>& agents,
                    const GridMap& grid,
                    const LNS2Params& params,
                    Solution& sol,
                    LNS2Stats& stats);

  // One LNS iteration: returns true if the solution was accepted (improved).
  bool iterate(const std::vector<Agent>& agents,
                const GridMap& grid,
                const LNS2Params& params,
                Solution& sol,
                LNS2Stats& stats,
                DestroyOp& op_used_out);

  std::unique_ptr<DestroyOperator> ops_[static_cast<std::size_t>(DestroyOp::COUNT)];
  ALNSWeights weights_;
  std::mt19937 rng_;
  HeuristicCache h_cache_;
};

}  // namespace lns2
