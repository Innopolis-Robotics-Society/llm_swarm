// iros_llm_swarm_mapf/lns2/src/lns2_solver.cpp

#include "iros_llm_swarm_mapf/lns2/lns2_solver.hpp"

#include <algorithm>
#include <chrono>
#include <random>

namespace lns2 {

LNS2Solver::LNS2Solver()
{
  ops_[static_cast<std::size_t>(DestroyOp::CollisionBased)] = make_collision_based();
  ops_[static_cast<std::size_t>(DestroyOp::RandomWalk)]     = make_random_walk();
  ops_[static_cast<std::size_t>(DestroyOp::Random)]         = make_random();
  ops_[static_cast<std::size_t>(DestroyOp::Bottleneck)]     = make_bottleneck();
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

bool LNS2Solver::solve(const std::vector<Agent>& agents,
                        const GridMap& grid,
                        const LNS2Params& params,
                        std::vector<Path>* out_paths,
                        LNS2Stats* out_stats)
{
  LNS2Stats stats{};
  stats.warm_started = false;

  if (params.seed == 0) {
    std::random_device rd;
    rng_.seed(rd());
  } else {
    rng_.seed(params.seed);
  }
  h_cache_.clear();
  weights_ = ALNSWeights{};

  Solution sol;
  sol.init(agents, &grid);

  const auto t0 = std::chrono::steady_clock::now();

  build_initial_solution(agents, grid, params, sol, stats);
  stats.initial_collisions = sol.num_collisions();

  if (sol.num_collisions() > 0) {
    repair_loop(agents, grid, params, sol, stats);
  }

  stats.final_collisions = sol.num_collisions();
  stats.wall_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  for (std::size_t i = 0; i < weights_.weights().size(); ++i) {
    stats.op_final_weight[i] = weights_.weights()[i];
  }

  if (out_paths) {
    out_paths->clear();
    out_paths->reserve(agents.size());
    for (AgentId id = 0; id < agents.size(); ++id) {
      out_paths->push_back(sol.path(id));
    }
  }
  if (out_stats) *out_stats = stats;
  return sol.is_collision_free();
}

bool LNS2Solver::solve_from(Solution&& seed,
                             const std::vector<Agent>& agents,
                             const GridMap& grid,
                             const LNS2Params& params,
                             std::vector<Path>* out_paths,
                             LNS2Stats* out_stats)
{
  LNS2Stats stats{};
  stats.warm_started = true;

  if (params.seed == 0) {
    std::random_device rd;
    rng_.seed(rd());
  } else {
    rng_.seed(params.seed);
  }
  h_cache_.clear();
  weights_ = ALNSWeights{};

  const auto t0 = std::chrono::steady_clock::now();

  Solution sol = std::move(seed);
  stats.initial_collisions = sol.num_collisions();

  if (sol.num_collisions() > 0) {
    repair_loop(agents, grid, params, sol, stats);
  }

  stats.final_collisions = sol.num_collisions();
  stats.wall_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  for (std::size_t i = 0; i < weights_.weights().size(); ++i) {
    stats.op_final_weight[i] = weights_.weights()[i];
  }

  if (out_paths) {
    out_paths->clear();
    out_paths->reserve(agents.size());
    for (AgentId id = 0; id < agents.size(); ++id) {
      out_paths->push_back(sol.path(id));
    }
  }
  if (out_stats) *out_stats = stats;
  return sol.is_collision_free();
}

// ---------------------------------------------------------------------------
// Initial solution: greedy prioritized planning.
// Order by h-distance to goal ascending; first agent plans on an empty
// table, each subsequent one sees prior ones as obstacles (via soft cost).
// ---------------------------------------------------------------------------

void LNS2Solver::build_initial_solution(const std::vector<Agent>& agents,
                                         const GridMap& grid,
                                         const LNS2Params& params,
                                         Solution& sol,
                                         LNS2Stats& stats)
{
  std::vector<AgentId> order(agents.size());
  for (AgentId i = 0; i < agents.size(); ++i) order[i] = i;

  // Sort by true distance to goal
  std::vector<int> dist(agents.size(), 0);
  for (AgentId i = 0; i < agents.size(); ++i) {
    const auto& h = h_cache_.get(grid, agents[i].goal, params.astar.diagonal_moves);
    if (!grid.in_bounds(agents[i].start)) continue;
    const int d = h[grid.index_of(agents[i].start)];
    dist[i] = d == std::numeric_limits<int>::max() ? -1 : d;
  }
  std::sort(order.begin(), order.end(), [&](AgentId a, AgentId b) {
    if (dist[a] < 0) return false;   // unreachable goes last
    if (dist[b] < 0) return true;
    return dist[a] < dist[b];
  });

  for (AgentId id : order) {
    if (dist[id] < 0) continue;   // skip unreachable (leaves path empty)
    auto res = soft_astar(agents[id], grid, sol.table(), params.astar, h_cache_);
    stats.total_astar_calls += 1;
    stats.total_astar_expansions += res.expansions;
    if (!res.success) {
      stats.total_astar_fails += 1;
      continue;
    }
    sol.set_path(id, std::move(res.path), params.astar.horizon);
  }
}

// ---------------------------------------------------------------------------
// Repair loop
// ---------------------------------------------------------------------------

void LNS2Solver::repair_loop(const std::vector<Agent>& agents,
                              const GridMap& grid,
                              const LNS2Params& params,
                              Solution& sol,
                              LNS2Stats& stats)
{
  const auto t0 = std::chrono::steady_clock::now();
  const auto budget = std::chrono::milliseconds(params.time_budget_ms);
  std::size_t plateau = 0;
  std::size_t iter_in_segment = 0;

  std::size_t best = sol.num_collisions();

  while (sol.num_collisions() > 0 && plateau < params.plateau_limit) {
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    if (elapsed >= budget) break;

    DestroyOp op_id;
    const bool improved = iterate(agents, grid, params, sol, stats, op_id);
    stats.iterations += 1;

    if (improved) {
      stats.accepted_iterations += 1;
      stats.op_improved[static_cast<std::size_t>(op_id)] += 1;
      if (sol.num_collisions() < best) {
        best = sol.num_collisions();
        plateau = 0;
      } else {
        plateau += 1;
      }
    } else {
      plateau += 1;
    }

    iter_in_segment += 1;
    if (iter_in_segment >= params.segment_size) {
      weights_.flush_segment(params.alns_reaction, params.alns_w_min);
      iter_in_segment = 0;
    }
  }

  // Rebuild touches_ once at the end — cheaper than doing it during the loop.
  // (Not strictly needed now but reasonable if anyone calls agents_touching
  //  after solve.)
  sol.mutable_table().rebuild_touches(sol.paths(), sol.footprints(), sol.holds());
}

// ---------------------------------------------------------------------------
// One iteration
// ---------------------------------------------------------------------------

bool LNS2Solver::iterate(const std::vector<Agent>& agents,
                          const GridMap& grid,
                          const LNS2Params& params,
                          Solution& sol,
                          LNS2Stats& stats,
                          DestroyOp& op_used_out)
{
  const DestroyOp op = weights_.sample(rng_);
  op_used_out = op;
  stats.op_used[static_cast<std::size_t>(op)] += 1;

  auto subset = ops_[static_cast<std::size_t>(op)]->select(
      sol, params.neighborhood_size, rng_);
  if (subset.empty()) {
    weights_.record(op, ALNSWeights::R_NO_CHANGE);
    return false;
  }

  const std::size_t collisions_before = sol.num_collisions();
  Snapshot snap = sol.take_and_clear(subset);

  // Randomize repair order.
  std::shuffle(subset.begin(), subset.end(), rng_);

  for (AgentId id : subset) {
    auto res = soft_astar(agents[id], grid, sol.table(),
                           params.astar, h_cache_);
    stats.total_astar_calls      += 1;
    stats.total_astar_expansions += res.expansions;
    if (!res.success) {
      stats.total_astar_fails += 1;
      // Leave this agent cleared; next iteration will try again.
      continue;
    }
    sol.set_path(id, std::move(res.path), params.astar.horizon);
  }

  if (sol.num_collisions() < collisions_before) {
    // Accept: reward (local improvement; best-reward tracked outside)
    const double reward = ALNSWeights::R_IMPROVE_LOCAL;
    weights_.record(op, reward);
    return true;
  } else {
    // Reject & rollback.
    sol.restore(std::move(snap));
    weights_.record(op, ALNSWeights::R_NO_CHANGE);
    return false;
  }
}

}  // namespace lns2
