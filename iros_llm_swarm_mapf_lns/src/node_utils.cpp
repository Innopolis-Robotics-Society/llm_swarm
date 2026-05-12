// iros_llm_swarm_mapf_lns/src/node_utils.cpp

#include "iros_llm_swarm_mapf_lns/node_utils.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "rclcpp/logging.hpp"

namespace lns2_node {

lns2::Cell world_to_cell(double wx, double wy,
                         double origin_x, double origin_y,
                         double resolution,
                         std::size_t rows, std::size_t cols)
{
  const double cx = (wx - origin_x) / resolution;
  const double cy = (wy - origin_y) / resolution;
  const int col = std::max(0, std::min(static_cast<int>(cols) - 1,
                                         static_cast<int>(std::floor(cx))));
  const int row = std::max(0, std::min(static_cast<int>(rows) - 1,
                                         static_cast<int>(std::floor(cy))));
  return {row, col};
}

void cell_to_world(const lns2::Cell& c,
                   double origin_x, double origin_y,
                   double resolution,
                   double& wx, double& wy)
{
  wx = origin_x + (c.col + 0.5) * resolution;
  wy = origin_y + (c.row + 0.5) * resolution;
}

std::vector<MAPFStepMsg> grid_path_to_steps(
    const lns2::Path& path,
    double origin_x, double origin_y, double resolution,
    double time_step_sec)
{
  std::vector<MAPFStepMsg> steps;
  if (path.empty()) return steps;

  std::size_t i = 0;
  while (i < path.size()) {
    std::size_t j = i;
    while (j + 1 < path.size() && path[j + 1] == path[i]) ++j;
    const std::size_t run_len = j - i + 1;
    const std::size_t hold_steps = (run_len > 0) ? (run_len - 1) : 0;

    MAPFStepMsg step;
    double wx, wy;
    cell_to_world(path[i], origin_x, origin_y, resolution, wx, wy);
    step.target.x = wx;
    step.target.y = wy;
    step.target.z = 0.0;
    step.hold_sec = static_cast<float>(
        static_cast<double>(hold_steps) * time_step_sec);
    steps.push_back(std::move(step));

    i = j + 1;
  }
  return steps;
}

void block_arrived_robots(
    lns2::GridMap& grid,
    const std::vector<uint32_t>& arrived_ids,
    const std::vector<std::pair<double, double>>& positions,
    const std::vector<double>& fp_radii,
    double default_robot_radius,
    double origin_x, double origin_y, double resolution,
    int num_robots)
{
  for (const uint32_t rid : arrived_ids) {
    if (rid >= static_cast<uint32_t>(num_robots)) continue;
    const auto& [sx, sy] = positions[rid];
    const lns2::Cell c = world_to_cell(sx, sy, origin_x, origin_y, resolution,
                                        grid.rows, grid.cols);
    const double radius = fp_radii[rid] > 0 ? fp_radii[rid]
                                             : default_robot_radius;
    const auto fp = lns2::FootprintModel::from_radius(radius, resolution);
    for (const auto& off : fp.offsets) {
      const lns2::Cell fc = c + off;
      if (grid.in_bounds(fc)) {
        grid.blocked[grid.index_of(fc)] = 1;
      }
    }
  }
}

void block_skipped_robots(
    lns2::GridMap& grid,
    const std::vector<uint32_t>& planned_ext_ids,
    const std::vector<std::pair<double, double>>& positions,
    const std::vector<bool>& have_odom,
    const std::vector<double>& fp_radii,
    double default_robot_radius,
    double origin_x, double origin_y, double resolution,
    int num_robots,
    const rclcpp::Logger* logger)
{
  std::unordered_set<uint32_t> planned_set(
      planned_ext_ids.begin(), planned_ext_ids.end());

  int blocked_count = 0;
  for (int rid = 0; rid < num_robots; ++rid) {
    if (planned_set.count(static_cast<uint32_t>(rid))) continue;
    if (!have_odom[rid]) {
      if (logger) {
        RCLCPP_WARN(*logger,
            "robot_%d not in plan and NO ODOM — cannot block! "
            "It may be a phantom obstacle.", rid);
      }
      continue;
    }

    const auto& [sx, sy] = positions[rid];
    lns2::Cell c = world_to_cell(sx, sy, origin_x, origin_y, resolution,
                                  grid.rows, grid.cols);
    const double radius = fp_radii[rid] > 0 ? fp_radii[rid]
                                             : default_robot_radius;
    auto fp = lns2::FootprintModel::from_radius(radius, resolution);
    for (const auto& off : fp.offsets) {
      lns2::Cell fc = c + off;
      if (grid.in_bounds(fc)) {
        grid.blocked[grid.index_of(fc)] = 1;
      }
    }
    if (logger) {
      RCLCPP_WARN(*logger,
          "Blocked cells for non-participating robot_%d at grid(%d,%d) "
          "world(%.2f,%.2f)", rid, c.row, c.col, sx, sy);
    }
    ++blocked_count;
  }
  if (blocked_count > 0 && logger) {
    RCLCPP_INFO(*logger,
        "block_skipped_robots: %d robots blocked, %zu robots planned",
        blocked_count, planned_ext_ids.size());
  }
}

}  // namespace lns2_node
