// Stress test: narrow corridor forces conflicts, repair loop must fix them.

#include <cstdio>
#include <vector>

#include "iros_llm_swarm_mapf_lns/lns2/lns2_solver.hpp"

using namespace lns2;

int main()
{
  // 30x30 grid with a corridor: everything blocked except rows 14-15.
  // Agents on the left must reach goals on the right and vice versa.
  GridMap grid;
  grid.rows = 30; grid.cols = 30;
  grid.blocked.assign(grid.rows * grid.cols, 1);  // start all blocked
  // Open the corridor (rows 13..16 x all cols)
  for (std::size_t r = 13; r <= 16; ++r)
    for (std::size_t c = 0; c < grid.cols; ++c)
      grid.blocked[r * grid.cols + c] = 0;
  // Open "rooms" at each end
  for (std::size_t r = 5; r <= 24; ++r) {
    for (std::size_t c = 0; c < 6; ++c) grid.blocked[r * grid.cols + c] = 0;
    for (std::size_t c = 24; c < 30; ++c) grid.blocked[r * grid.cols + c] = 0;
  }

  FootprintModel fp = FootprintModel::from_radius(0.22, 0.2);

  // 4 agents left->right, 4 agents right->left — forced to share corridor
  std::vector<Agent> agents;
  auto add = [&](AgentId id, Cell s, Cell g) {
    Agent a;
    a.id = id; a.start = s; a.goal = g; a.footprint = fp;
    agents.push_back(a);
  };
  add(0, {7,  2}, {7,  27});
  add(1, {10, 2}, {10, 27});
  add(2, {19, 2}, {19, 27});
  add(3, {22, 2}, {22, 27});
  add(4, {7,  27}, {7,  2});
  add(5, {10, 27}, {10, 2});
  add(6, {19, 27}, {19, 2});
  add(7, {22, 27}, {22, 2});

  LNS2Params params;
  params.astar.horizon           = 200;
  params.astar.collision_penalty = 10000;
  params.astar.max_expansions    = 500000;
  params.astar.diagonal_moves    = false;
  params.neighborhood_size = 4;
  params.time_budget_ms    = 3000;
  params.plateau_limit     = 500;
  params.seed              = 1234;

  LNS2Solver solver;
  std::vector<Path> paths;
  LNS2Stats stats;

  bool ok = solver.solve(agents, grid, params, &paths, &stats);
  std::printf("=== RESULT ===\n");
  std::printf("ok=%d iters=%zu accepted=%zu "
              "init_coll=%zu final_coll=%zu t=%.2fms\n",
              ok, stats.iterations, stats.accepted_iterations,
              stats.initial_collisions, stats.final_collisions,
              stats.wall_time_ms);
  std::printf("astar calls=%zu exp=%zu fails=%zu avg_exp/call=%.0f\n",
              stats.total_astar_calls, stats.total_astar_expansions,
              stats.total_astar_fails,
              stats.total_astar_calls
                ? static_cast<double>(stats.total_astar_expansions) /
                    stats.total_astar_calls : 0.0);
  std::printf("ops used: coll=%zu walk=%zu rand=%zu bot=%zu\n",
              stats.op_used[0], stats.op_used[1],
              stats.op_used[2], stats.op_used[3]);
  std::printf("ops improved: coll=%zu walk=%zu rand=%zu bot=%zu\n",
              stats.op_improved[0], stats.op_improved[1],
              stats.op_improved[2], stats.op_improved[3]);
  std::printf("final weights: coll=%.2f walk=%.2f rand=%.2f bot=%.2f\n",
              stats.op_final_weight[0], stats.op_final_weight[1],
              stats.op_final_weight[2], stats.op_final_weight[3]);
  for (std::size_t i = 0; i < paths.size(); ++i) {
    std::printf("agent %zu: %zu wp\n", i, paths[i].size());
  }
  return ok ? 0 : 1;
}
