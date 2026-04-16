// iros_llm_swarm_mapf/lns2/src/types.cpp

#include "iros_llm_swarm_mapf/lns2/types.hpp"

#include <cmath>

namespace lns2 {

FootprintModel FootprintModel::from_radius(double radius_m, double resolution_m)
{
  FootprintModel fp;
  if (radius_m <= 0.0 || resolution_m <= 0.0) {
    fp.offsets.push_back(CellOffset{0, 0});
    return fp;
  }
  const int r_cells = static_cast<int>(std::ceil(radius_m / resolution_m));
  const double r2 = (radius_m / resolution_m) * (radius_m / resolution_m);
  for (int dr = -r_cells; dr <= r_cells; ++dr) {
    for (int dc = -r_cells; dc <= r_cells; ++dc) {
      // Use the cell-center distance. A cell is "covered" if its center is
      // within the radius.
      if (static_cast<double>(dr) * dr + static_cast<double>(dc) * dc <= r2) {
        fp.offsets.push_back(CellOffset{dr, dc});
      }
    }
  }
  if (fp.offsets.empty()) fp.offsets.push_back(CellOffset{0, 0});
  return fp;
}

}  // namespace lns2
