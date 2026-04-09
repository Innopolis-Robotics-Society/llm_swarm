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

// ---------------------------------------------------------------------------
// Segment geometry (capsule collision)
// ---------------------------------------------------------------------------

struct Segment {
  float r0, c0;  // start point (row, col) as float
  float r1, c1;  // end point

  static Segment from_cells(const Cell& a, const Cell& b) {
    return {static_cast<float>(a.row), static_cast<float>(a.col),
            static_cast<float>(b.row), static_cast<float>(b.col)};
  }

  static Segment from_cell(const Cell& a) {
    return from_cells(a, a);
  }
};

// Minimum squared distance between two line segments.
// Each segment is parameterized as P(s) = A + s*(B-A), s in [0,1].
inline float segment_segment_dist_sq(const Segment& s1, const Segment& s2) {
  const float d1r = s1.r1 - s1.r0, d1c = s1.c1 - s1.c0;
  const float d2r = s2.r1 - s2.r0, d2c = s2.c1 - s2.c0;
  const float rr = s1.r0 - s2.r0,  rc = s1.c0 - s2.c0;

  const float a = d1r * d1r + d1c * d1c;  // |d1|^2
  const float e = d2r * d2r + d2c * d2c;  // |d2|^2
  const float b = d1r * d2r + d1c * d2c;  // d1 . d2
  const float c = d1r * rr  + d1c * rc;   // d1 . r
  const float f = d2r * rr  + d2c * rc;   // d2 . r

  const float denom = a * e - b * b;
  float s, t;

  if (denom < 1e-8f) {
    s = 0.0f;
    t = (e > 1e-8f) ? f / e : 0.0f;
  } else {
    s = (b * f - c * e) / denom;
    t = (a * f - b * c) / denom;
  }

  s = std::max(0.0f, std::min(1.0f, s));
  t = (e > 1e-8f) ? (b * s + f) / e : 0.0f;

  t = std::max(0.0f, std::min(1.0f, t));
  if (a > 1e-8f) {
    s = (b * t - c) / a;
    s = std::max(0.0f, std::min(1.0f, s));
  }

  const float pr = s1.r0 + s * d1r - (s2.r0 + t * d2r);
  const float pc = s1.c0 + s * d1c - (s2.c0 + t * d2c);
  return pr * pr + pc * pc;
}

// Check if two capsules overlap (swept circles along segments).
inline bool capsule_overlap(const Segment& a, float radius_a,
                            const Segment& b, float radius_b) {
  const float combined = radius_a + radius_b;
  if (combined <= 0.0f) {
    return segment_segment_dist_sq(a, b) < 1e-6f;
  }
  return segment_segment_dist_sq(a, b) < combined * combined;
}

// Gradient penalty between two capsules.
// Returns: -1 = hard overlap (forbidden), 0 = free, >0 = soft penalty.
inline int capsule_penalty(const Segment& a, float hard_a, float soft_a,
                           const Segment& b, float hard_b, float soft_b,
                           int max_penalty = 50,
                           CostCurve cost_curve = CostCurve::Quadratic) {
  const float dist_sq = segment_segment_dist_sq(a, b);
  const float hard_dist = hard_a + hard_b;
  const float soft_dist = soft_a + soft_b;

  if (hard_dist > 0.0f && dist_sq < hard_dist * hard_dist) return -1;
  if (soft_dist <= 0.0f || dist_sq >= soft_dist * soft_dist) return 0;

  const float dist = std::sqrt(dist_sq);
  const float ratio = (soft_dist - dist) / (soft_dist - hard_dist);
  return static_cast<int>(apply_curve(ratio, cost_curve) * max_penalty);
}
