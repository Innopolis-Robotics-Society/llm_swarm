#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

// ---------------------------------------------------------------------------
// Basic structures
// ---------------------------------------------------------------------------

enum class CostCurve { Linear, Quadratic, Cubic };

// Fast pow replacement: avoids std::pow() in hot paths.
inline float apply_curve(float ratio, CostCurve curve) {
  switch (curve) {
    case CostCurve::Linear:    return ratio;
    case CostCurve::Quadratic: return ratio * ratio;
    case CostCurve::Cubic:     return ratio * ratio * ratio;
  }
  return ratio;
}

struct Cell {
  size_t row = 0;
  size_t col = 0;
};

struct Agent {
  size_t id = 0;   // sequential index (0..N-1)
  Cell start;
  Cell goal;
  float footprint_radius = 0.0f;  // physical robot radius in metres (hard boundary)
  float inflation = 0.0f;         // soft zone width beyond footprint in metres
};

struct GridMap {
  size_t rows = 0;
  size_t cols = 0;
  std::vector<uint8_t> blocked;  // 1 = hard blocked (wall or within footprint radius)
  std::vector<int>     wall_cost; // gradient penalty near walls (0 = free, >0 = penalty)

  // Gradient inflation: cells within hard_r are blocked, cells between
  // hard_r and soft_r get a gradient penalty (max_penalty at hard boundary,
  // 0 at soft boundary).  cost_curve controls the gradient shape.
  GridMap inflate_gradient(float hard_m, float soft_m, float resolution_m,
                           int max_penalty = 10,
                           CostCurve cost_curve = CostCurve::Quadratic) const {
    if (soft_m <= 0.0f) return *this;
    const float hard_r = hard_m / resolution_m;
    const float soft_r = soft_m / resolution_m;
    const int scan_r = static_cast<int>(std::ceil(soft_r));

    GridMap out;
    out.rows = rows;
    out.cols = cols;
    out.blocked.assign(rows * cols, 0);
    out.wall_cost.assign(rows * cols, 0);

    const float hard_r_sq = hard_r * hard_r;
    const float soft_r_sq = soft_r * soft_r;

    for (size_t row = 0; row < rows; ++row) {
      for (size_t col = 0; col < cols; ++col) {
        if (blocked[row * cols + col] == 0) continue;
        for (int dr = -scan_r; dr <= scan_r; ++dr) {
          for (int dc = -scan_r; dc <= scan_r; ++dc) {
            const float dist_sq = static_cast<float>(dr * dr + dc * dc);
            if (dist_sq > soft_r_sq) continue;
            const int nr = static_cast<int>(row) + dr;
            const int nc = static_cast<int>(col) + dc;
            if (nr < 0 || nr >= static_cast<int>(rows)) continue;
            if (nc < 0 || nc >= static_cast<int>(cols)) continue;
            const size_t idx = nr * cols + nc;
            if (dist_sq <= hard_r_sq) {
              out.blocked[idx] = 1;
            } else {
              const float dist = std::sqrt(dist_sq);
              const float ratio = (soft_r - dist) / (soft_r - hard_r);
              const int cost = static_cast<int>(
                  apply_curve(ratio, cost_curve) * max_penalty);
              out.wall_cost[idx] = std::max(out.wall_cost[idx], cost);
            }
          }
        }
      }
    }
    return out;
  }
};

using Path = std::vector<Cell>;

// ---------------------------------------------------------------------------
// Solve statistics
// ---------------------------------------------------------------------------

enum class FailReason {
  None,
  RootPathFailed,    // A* failed for an agent in root (no reservations)
  BranchExhausted,   // all branches explored, no conflict-free solution
  MaxExpansions,      // hit MAX_EXP limit
};

struct SolveDiagnostics {
  FailReason fail_reason = FailReason::None;
  size_t     fail_agent = 0;        // agent index that caused root failure
  size_t     max_t_used = 0;
  size_t     branches_tried = 0;
  size_t     branches_failed = 0;
};

struct AStarStats {
  size_t ok_total_exp = 0;   // sum of expansions across successful calls
  size_t ok_max_exp = 0;     // worst single successful call
  size_t ok_count = 0;
  size_t fail_count = 0;
};

struct SolveStats {
  size_t expansions = 0;
  size_t max_path_length = 0;
  AStarStats astar;
  SolveDiagnostics diag;
};
