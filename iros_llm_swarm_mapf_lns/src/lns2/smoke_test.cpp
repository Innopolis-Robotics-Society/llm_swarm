// Smoke test: small warehouse with 3 agents whose straight paths cross.

#include <cstdio>
#include <vector>

#include "iros_llm_swarm_mapf_lns/lns2/lns2_solver.hpp"
#include "iros_llm_swarm_mapf_lns/lns2/warm_start.hpp"

using namespace lns2;

int main()
{
  // 20x20 empty grid at 0.2m/cell.
  GridMap grid;
  grid.rows = 20; grid.cols = 20;
  grid.blocked.assign(grid.rows * grid.cols, 0);

  // Three agents with straight crossing paths.
  FootprintModel fp = FootprintModel::from_radius(0.22, 0.2);
  std::printf("footprint cells: %zu\n", fp.size());

  std::vector<Agent> agents(3);
  agents[0].id = 0; agents[0].start = {2, 2};   agents[0].goal = {17, 17};
  agents[1].id = 1; agents[1].start = {2, 17};  agents[1].goal = {17, 2};
  agents[2].id = 2; agents[2].start = {10, 2};  agents[2].goal = {10, 17};
  for (auto& a : agents) a.footprint = fp;

  LNS2Params params;
  params.astar.horizon = 80;
  params.astar.collision_penalty = 10000;
  params.astar.max_expansions = 100000;
  params.astar.diagonal_moves = false;
  params.neighborhood_size = 3;
  params.time_budget_ms = 1000;
  params.plateau_limit  = 200;
  params.seed           = 42;

  LNS2Solver solver;
  std::vector<Path> paths;
  LNS2Stats stats;

  bool ok = solver.solve(agents, grid, params, &paths, &stats);
  std::printf("solve ok=%d iters=%zu accepted=%zu "
              "init_coll=%zu final_coll=%zu t=%.2fms\n",
              ok, stats.iterations, stats.accepted_iterations,
              stats.initial_collisions, stats.final_collisions,
              stats.wall_time_ms);
  std::printf("astar calls=%zu exp=%zu fails=%zu\n",
              stats.total_astar_calls, stats.total_astar_expansions,
              stats.total_astar_fails);
  for (std::size_t i = 0; i < paths.size(); ++i) {
    std::printf("agent %zu: %zu waypoints", i, paths[i].size());
    if (!paths[i].empty()) {
      std::printf(" [%d,%d]->[%d,%d]",
                  paths[i].front().row, paths[i].front().col,
                  paths[i].back().row,  paths[i].back().col);
    }
    std::printf("\n");
  }
  std::printf("op weights: coll=%.2f walk=%.2f rand=%.2f bottleneck=%.2f\n",
              stats.op_final_weight[0], stats.op_final_weight[1],
              stats.op_final_weight[2], stats.op_final_weight[3]);

  // Smoke-test warm start: pretend 5 steps have elapsed, build seed.
  PreviousPlan prev;
  prev.paths = paths;
  prev.hold_until.assign(paths.size(), params.astar.horizon);
  prev.delta_steps = 5;

  ActualState actual;
  actual.current_cells.resize(paths.size());
  actual.have_odom.assign(paths.size(), 1);
  actual.goals.resize(paths.size());
  for (std::size_t i = 0; i < paths.size(); ++i) {
    actual.current_cells[i] = paths[i].size() > 5 ? paths[i][5] : paths[i].back();
    actual.goals[i] = agents[i].goal;
  }

  Solution seed;
  WarmStartReport rep;
  build_warm_seed(agents, grid, prev, actual, {}, params.astar.horizon,
                   &seed, &rep);
  std::printf("warm: on_track=%zu spliced=%zu stubbed_far=%zu new=%zu "
              "goal_changed=%zu invalid=%zu arrived=%zu missing=%zu coll=%zu\n",
              rep.agents_on_track, rep.agents_spliced,
              rep.agents_stubbed_far, rep.agents_new,
              rep.agents_goal_changed, rep.agents_invalid,
              rep.agents_arrived, rep.agents_missing,
              rep.seed_collisions);

  bool use_warm = should_warm_start(rep, agents.size());
  std::printf("use_warm=%d\n", use_warm);

  LNS2Stats stats2;
  std::vector<Path> paths2;
  bool ok2 = use_warm
      ? solver.solve_from(std::move(seed), agents, grid, params, &paths2, &stats2)
      : solver.solve(agents, grid, params, &paths2, &stats2);
  std::printf("replan ok=%d iters=%zu init_coll=%zu final_coll=%zu t=%.2fms warm=%d\n",
              ok2, stats2.iterations, stats2.initial_collisions,
              stats2.final_collisions, stats2.wall_time_ms, stats2.warm_started);

  return ok ? 0 : 1;
}
