// iros_llm_swarm_mapf_lns/lns2/gridmap_utils.hpp
//
// Footprint-aware utilities over a static GridMap. Pure functions, no state.

#pragma once

#include "iros_llm_swarm_mapf_lns/lns2/types.hpp"

namespace lns2 {

// Static reachability check (footprint-aware BFS).
// Returns true iff there exists a sequence of 4-connected moves from
// `start` to `goal` such that every intermediate cell is footprint-valid
// (entire footprint lies in free space). Ignores other agents entirely.
bool static_path_exists(const GridMap& grid,
                        const Cell& start,
                        const Cell& goal,
                        const FootprintModel& fp);

// Find the nearest cell to `wanted` where the robot's footprint fits on
// the static map. BFS with Chebyshev-radius limit `max_radius_cells`.
// Returns `wanted` unchanged if it already fits, the closest free cell
// found, or `wanted` if nothing fits within the radius (caller must
// re-check with footprint_fits).
Cell find_nearest_free_cell(const GridMap& grid,
                            const Cell& wanted,
                            const FootprintModel& fp,
                            int max_radius_cells = 10);

}  // namespace lns2
