// iros_llm_swarm_mapf_lns/lns2/src/gridmap_utils.cpp

#include "iros_llm_swarm_mapf_lns/lns2/gridmap_utils.hpp"

#include <queue>
#include <unordered_set>
#include <utility>

namespace lns2 {

bool static_path_exists(const GridMap& grid,
                        const Cell& start,
                        const Cell& goal,
                        const FootprintModel& fp)
{
  if (!footprint_fits(start, fp, grid)) return false;
  if (!footprint_fits(goal, fp, grid)) return false;
  if (start == goal) return true;

  std::unordered_set<CellIdx> visited;
  visited.reserve(1024);
  std::queue<Cell> q;
  q.push(start);
  visited.insert(grid.index_of(start));

  static const CellOffset step4[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  const CellIdx goal_idx = grid.index_of(goal);

  while (!q.empty()) {
    const Cell cur = q.front();
    q.pop();
    for (const auto& s : step4) {
      const Cell nxt = cur + s;
      if (!grid.in_bounds(nxt)) continue;
      if (!footprint_fits(nxt, fp, grid)) continue;
      const CellIdx nidx = grid.index_of(nxt);
      if (!visited.insert(nidx).second) continue;
      if (nidx == goal_idx) return true;
      q.push(nxt);
    }
  }
  return false;
}

Cell find_nearest_free_cell(const GridMap& grid,
                            const Cell& wanted,
                            const FootprintModel& fp,
                            int max_radius_cells)
{
  if (grid.in_bounds(wanted) && footprint_fits(wanted, fp, grid)) {
    return wanted;
  }

  std::unordered_set<CellIdx> visited;
  visited.reserve(static_cast<std::size_t>(
      (2 * max_radius_cells + 1) * (2 * max_radius_cells + 1)));
  std::queue<std::pair<Cell, int>> q;
  if (grid.in_bounds(wanted)) {
    q.push({wanted, 0});
    visited.insert(grid.index_of(wanted));
  }

  static const CellOffset step8[] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!q.empty()) {
    auto [cur, dist] = q.front();
    q.pop();
    if (dist > max_radius_cells) continue;
    if (footprint_fits(cur, fp, grid)) {
      return cur;
    }
    for (const auto& s : step8) {
      const Cell nxt = cur + s;
      if (!grid.in_bounds(nxt)) continue;
      const CellIdx nidx = grid.index_of(nxt);
      if (!visited.insert(nidx).second) continue;
      q.push({nxt, dist + 1});
    }
  }
  return wanted;
}

}  // namespace lns2
