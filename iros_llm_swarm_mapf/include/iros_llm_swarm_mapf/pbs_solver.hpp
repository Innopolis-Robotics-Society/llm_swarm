#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ---------------------------------------------------------------------------
// Базовые структуры
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
  size_t id = 0;   // порядковый номер (0..N-1)
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
// Enum-ы
// ---------------------------------------------------------------------------

enum class ConflictType { None, Vertex, Edge };

struct Conflict {
  ConflictType type = ConflictType::None;
  size_t agent1 = 0;
  size_t agent2 = 0;
  size_t time   = 0;
  Cell   cell1{};
  Cell   cell2{};
};

// ---------------------------------------------------------------------------
// Статистика решения
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
  Conflict   first_conflict{};      // first unresolved conflict
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
// ReservationTable
// ---------------------------------------------------------------------------
// Хранит центры и радиусы высокоприоритетных агентов.
// Проверка конфликтов через евклидово расстояние между центрами:
// если расстояние < (r_a + r_b), агенты считаются в конфликте.
// При нулевых радиусах поведение идентично старой точечной проверке.

class ReservationTable {
 public:
  void clear() {
    entries_.clear();
    held_goals_.clear();
  }

  // Зарезервировать путь агента с данным радиусом (в клетках).
  // footprint_cells: physical (hard) radius in cells.
  // soft_radius_cells: footprint + inflation in cells (outer penalty boundary).
  // skip_until: don't reserve entries for t < skip_until (grace period
  // for agents whose starts overlap — lets them separate first).
  void reserve_path(const Path& path,
                    size_t hold_goal_until_time,
                    float footprint_cells, float soft_radius_cells,
                    size_t skip_until = 0) {
    if (path.empty()) return;
    for (size_t t = skip_until; t < path.size(); ++t) {
      entries_[t].push_back({path[t], footprint_cells, soft_radius_cells});
    }
    // Удержание цели: агент остаётся на месте после прибытия
    if (path.size() <= hold_goal_until_time) {
      held_goals_.push_back({path.back(), footprint_cells, soft_radius_cells,
                             std::max(path.size(), skip_until)});
    }
  }

  // Gradient penalty for placing an agent at (row,col) at time t.
  // Returns: 0 = free, >0 = soft penalty, -1 = FORBIDDEN (physical overlap).
  // my_footprint/my_soft: agent's own radii in cells.
  // max_penalty: cost at the hard boundary (decreasing to 0 at soft boundary).
  // Multiple nearby agents: takes the greatest penalty, not sum.
  int vertex_penalty(size_t row, size_t col, size_t time,
                     float my_footprint, float my_soft,
                     int max_penalty = 50,
                     CostCurve cost_curve = CostCurve::Quadratic) const {
    int total = 0;
    auto check = [&](const Cell& c, float fp, float sr) {
      const float dr = static_cast<float>(row) - static_cast<float>(c.row);
      const float dc = static_cast<float>(col) - static_cast<float>(c.col);
      const float dist_sq = dr * dr + dc * dc;
      const float hard = my_footprint + fp;
      const float soft = my_soft + sr;
      if (hard > 0.0f && dist_sq < hard * hard) { total = -1; return; }
      if (soft > 0.0f && dist_sq < soft * soft) {
        const float dist = std::sqrt(dist_sq);
        const float ratio = (soft - dist) / (soft - hard);
        const int cost = static_cast<int>(
            apply_curve(ratio, cost_curve) * max_penalty);
        total = std::max(total, cost);
      }
    };

    // Проверяем удержанные цели
    for (const auto& hg : held_goals_) {
      if (time >= hg.from_time) {
        check(hg.cell, hg.footprint_cells, hg.soft_radius_cells);
        if (total < 0) return -1;
      }
    }
    // Проверяем позиции на конкретном шаге
    auto it = entries_.find(time);
    if (it != entries_.end()) {
      for (const auto& e : it->second) {
        check(e.cell, e.footprint_cells, e.soft_radius_cells);
        if (total < 0) return -1;
      }
    }
    return total;
  }

  // Edge conflict: проверяет встречное движение между шагами time и time+1.
  // Uses footprint (hard) radius only — physical swap prevention.
  bool is_edge_blocked(size_t fr, size_t fc, size_t tr, size_t tc,
                        size_t time, float my_footprint_cells) const {
    auto it = entries_.find(time);
    auto it_next = entries_.find(time + 1);
    if (it == entries_.end() || it_next == entries_.end()) return false;

    // Для каждого агента: если он двигается навстречу.
    // NOTE: pairing by vector index assumes reserve_path() is called
    // once per agent in a fixed order.  With skip_until, an agent's
    // entries start later, so indices at the grace boundary may differ —
    // the min(size, size) guard safely skips unmatched trailing entries.
    // TODO: edge conflicts are silently missed at the grace boundary
    // timestep. Fix by pairing entries by agent ID, not vector index.
    for (size_t i = 0; i < it->second.size() && i < it_next->second.size(); ++i) {
      const auto& cur  = it->second[i];
      const auto& next = it_next->second[i];
      // Edge conflict: после обмена оба агента оказываются слишком близко
      // к предыдущей позиции другого
      if (cells_conflict(fr, fc, my_footprint_cells,
                          next.cell.row, next.cell.col, cur.footprint_cells) &&
          cells_conflict(tr, tc, my_footprint_cells,
                          cur.cell.row, cur.cell.col, cur.footprint_cells))
        return true;
    }
    return false;
  }

  // Можно ли агенту удерживать цель начиная с from_time?
  // Uses footprint (hard) radius — agent can hold goal as long as no
  // physical overlap with other agents' paths or held goals.
  bool can_hold_goal(size_t row, size_t col, size_t from_time,
                      size_t max_time, float my_footprint_cells) const {
    for (const auto& hg : held_goals_) {
      if (cells_conflict(row, col, my_footprint_cells,
                          hg.cell.row, hg.cell.col, hg.footprint_cells))
        return false;
    }
    for (size_t t = from_time; t <= max_time; ++t) {
      auto it = entries_.find(t);
      if (it == entries_.end()) continue;
      for (const auto& e : it->second) {
        if (cells_conflict(row, col, my_footprint_cells,
                            e.cell.row, e.cell.col, e.footprint_cells))
          return false;
      }
    }
    return true;
  }

 private:
  struct Entry {
    Cell cell;
    float footprint_cells;    // physical (hard) radius in cells
    float soft_radius_cells;  // footprint + inflation in cells
  };
  struct HeldGoal {
    Cell cell;
    float footprint_cells;
    float soft_radius_cells;
    size_t from_time;
  };

  std::unordered_map<size_t, std::vector<Entry>> entries_;  // time -> entries
  std::vector<HeldGoal> held_goals_;

  // Проверка конфликта двух кругов: расстояние между центрами < сумма радиусов.
  // При нулевых радиусах — точечная проверка совпадения клеток.
  static bool cells_conflict(size_t r1, size_t c1, float rad1,
                              size_t r2, size_t c2, float rad2) {
    const float min_dist = rad1 + rad2;
    if (min_dist <= 0.0f) {
      return r1 == r2 && c1 == c2;
    }
    const float dr = static_cast<float>(r1) - static_cast<float>(r2);
    const float dc = static_cast<float>(c1) - static_cast<float>(c2);
    return dr * dr + dc * dc < min_dist * min_dist;
  }
};

// ---------------------------------------------------------------------------
// Space-Time A*
// ---------------------------------------------------------------------------

class SpaceTimeAStarPlanner {
 public:
  SpaceTimeAStarPlanner() = default;

  void reset_map(const GridMap* map) { map_ = map; }
  const GridMap* current_map() const { return map_; }
  void set_goal_dists(const std::vector<int>* d) { goal_dists_ = d; }
  size_t last_expansions() const { return last_expansions_; }

  // my_footprint_cells — physical (hard) radius of this agent in cells.
  // my_soft_cells — footprint + inflation in cells (outer penalty boundary).
  // cost_curve — gradient shape (Linear, Quadratic, Cubic).
  // proximity_penalty — cost per step at the hard boundary.
  // max_astar_expansions — per-call expansion limit (0 = unlimited).
  bool find_path(const Cell& start, const Cell& goal,
                 const ReservationTable& reservations,
                 float my_footprint_cells, float my_soft_cells,
                 size_t max_time, Path& path,
                 CostCurve cost_curve = CostCurve::Quadratic,
                 int proximity_penalty = 50,
                 size_t max_astar_expansions = 0) {
    my_footprint_ = my_footprint_cells;
    my_soft_ = my_soft_cells;
    cost_curve_ = cost_curve;
    proximity_penalty_ = proximity_penalty;

    path.clear();
    if (!map_) return false;
    if (is_blocked(start.row, start.col) || is_blocked(goal.row, goal.col)) return false;

    const size_t spatial = map_->rows * map_->cols;
    const size_t state_count = spatial * (max_time + 1);

    ensure_capacity(state_count);
    next_generation();

    const size_t start_idx = idx(start.row, start.col);
    const size_t goal_idx  = idx(goal.row, goal.col);
    const size_t start_state = state_idx(start_idx, 0, spatial);

    // NOTE: we intentionally do NOT reject starts that overlap with
    // reservations at t=0.  During replanning agents are physically at
    // their current positions and must be allowed to start there even if
    // another agent's reservation covers the same cell.  A* will
    // naturally find a path that diverges at t=1+.

    set_g(start_state, 0);
    parent_[start_state] = INVALID;

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
    open.push({start_idx, 0, 0, heuristic(start, goal)});

    last_expansions_ = 0;
    while (!open.empty()) {
      ++last_expansions_;
      if (max_astar_expansions > 0 && last_expansions_ > max_astar_expansions)
        return false;

      const Node cur = open.top(); open.pop();
      const size_t cs = state_idx(cur.idx, cur.t, spatial);

      if (is_closed(cs)) continue;
      if (!is_alive(cs) || cur.g != g_[cs]) continue;
      mark_closed(cs);

      if (cur.idx == goal_idx) {
        if (reservations.can_hold_goal(goal.row, goal.col, cur.t,
                                        max_time, my_footprint_cells)) {
          restore(cs, spatial, path);
          return true;
        }
      }

      if (cur.t >= max_time) continue;

      const Cell c  = cell(cur.idx);
      const size_t r = c.row, col_ = c.col;
      const size_t nt = cur.t + 1;

      // ждать на месте
      relax(cur.idx, cur.t, r, col_, nt, goal, reservations, open, spatial);
      // соседи
      if (r > 0)               relax(cur.idx, cur.t, r-1, col_,   nt, goal, reservations, open, spatial);
      if (r+1 < map_->rows)    relax(cur.idx, cur.t, r+1, col_,   nt, goal, reservations, open, spatial);
      if (col_ > 0)            relax(cur.idx, cur.t, r,   col_-1, nt, goal, reservations, open, spatial);
      if (col_+1 < map_->cols) relax(cur.idx, cur.t, r,   col_+1, nt, goal, reservations, open, spatial);
    }
    return false;
  }

 private:
  static constexpr int    INF     = std::numeric_limits<int>::max() / 4;
  static constexpr size_t INVALID = std::numeric_limits<size_t>::max();

  struct Node { size_t idx, t; int g, f; };
  struct NodeCmp {
    bool operator()(const Node& a, const Node& b) const {
      return a.f != b.f ? a.f > b.f : a.g < b.g;
    }
  };

  const GridMap*  map_           = nullptr;
  const std::vector<int>* goal_dists_ = nullptr;  // precomputed Dijkstra from goal
  float           my_footprint_  = 0.0f;   // current agent's hard radius in cells
  float           my_soft_       = 0.0f;   // current agent's soft radius in cells
  CostCurve       cost_curve_    = CostCurve::Quadratic;
  int             proximity_penalty_ = 50; // cost at hard boundary
  size_t          last_expansions_ = 0;    // expansions in last find_path call
  std::vector<int>      g_;
  std::vector<size_t>   parent_;
  std::vector<uint8_t>  closed_;
  std::vector<uint32_t> gen_;
  uint32_t current_gen_ = 1;

  size_t idx(size_t r, size_t c) const { return r * map_->cols + c; }
  Cell   cell(size_t i)          const { return {i / map_->cols, i % map_->cols}; }

  static size_t state_idx(size_t ci, size_t t, size_t sp) { return t * sp + ci; }
  static size_t state_cell(size_t si, size_t sp)           { return si % sp; }

  bool is_blocked(size_t r, size_t c) const { return map_->blocked[idx(r,c)] != 0; }

  int wall_penalty(size_t r, size_t c) const {
    return map_->wall_cost.empty() ? 0 : map_->wall_cost[idx(r,c)];
  }

  // Heuristic for A*: precomputed Dijkstra step-distance from goal.
  // Accounts for wall topology (blocked cells) but excludes penalties.
  // Admissible — always <= true cost.  Falls back to Manhattan if
  // goal_dists_ is not set (e.g. during testing).
  int heuristic(const Cell& a, const Cell& b) const {
    if (goal_dists_) {
      const int d = (*goal_dists_)[idx(a.row, a.col)];
      return d < INF ? d : INF;
    }
    const int dr = a.row > b.row ? a.row - b.row : b.row - a.row;
    const int dc = a.col > b.col ? a.col - b.col : b.col - a.col;
    return dr + dc;
  }

  void ensure_capacity(size_t n) {
    if (g_.size() >= n) return;
    g_.resize(n, INF); parent_.resize(n, INVALID);
    closed_.resize(n, 0); gen_.resize(n, 0);
  }

  void next_generation() {
    if (++current_gen_ == 0) { current_gen_ = 1; std::fill(gen_.begin(), gen_.end(), 0); }
  }

  bool   is_alive(size_t s)  const { return gen_[s] == current_gen_; }
  int    get_g(size_t s)     const { return is_alive(s) ? g_[s] : INF; }
  void   set_g(size_t s, int v)    { gen_[s] = current_gen_; g_[s] = v; closed_[s] = 0; }
  bool   is_closed(size_t s) const { return is_alive(s) && closed_[s]; }
  void   mark_closed(size_t s)     { closed_[s] = 1; }

  void relax(size_t ci, size_t ct, size_t nr, size_t nc, size_t nt,
             const Cell& goal, const ReservationTable& res,
             std::priority_queue<Node, std::vector<Node>, NodeCmp>& open,
             size_t sp) {
    if (is_blocked(nr, nc)) return;

    const size_t ni = idx(nr, nc);
    const bool moving = (ni != ci);

    // Wall penalty only when entering a new cell (not waiting).
    // Agent penalty every timestep (occupying space near them).
    int penalty = moving ? wall_penalty(nr, nc) : 0;

    const int agent_pen = res.vertex_penalty(
        nr, nc, nt, my_footprint_, my_soft_,
        proximity_penalty_, cost_curve_);
    if (agent_pen < 0) return;  // physical agent overlap forbidden
    penalty += agent_pen;

    // Edge conflict: проверяем встречное движение (uses footprint/hard radius)
    const Cell from_cell = cell(ci);
    if (res.is_edge_blocked(from_cell.row, from_cell.col, nr, nc, ct, my_footprint_))
      return;

    const size_t cs = state_idx(ci, ct, sp);
    const size_t ns = state_idx(ni, nt, sp);
    if (is_closed(ns)) return;

    // Every timestep costs 1.  Movement adds 1 more — agents prefer
    // waiting over unnecessary walking.
    const int tg = get_g(cs) + 1 + (moving ? 1 : 0) + penalty;
    if (tg < get_g(ns)) {
      set_g(ns, tg);
      parent_[ns] = cs;
      open.push({ni, nt, tg, tg + heuristic({nr, nc}, goal)});
    }
  }

  void restore(size_t goal_state, size_t sp, Path& path) const {
    for (size_t cur = goal_state; cur != INVALID; cur = parent_[cur])
      path.push_back(cell(state_cell(cur, sp)));
    std::reverse(path.begin(), path.end());
  }
};

// ---------------------------------------------------------------------------
// Conflict Detector
// ---------------------------------------------------------------------------

class ConflictDetector {
 public:
  // Detect conflicts using footprint (hard/physical) radii only.
  // Soft-zone proximity is handled by A* penalties, not PBS branching.
  // Евклидово расстояние между центрами vs (r_a + r_b).
  // Для точечных агентов (радиус 0) — эквивалентно проверке совпадения клеток.
  // TODO Level 2: при появлении ориентационно-зависимых полигонов —
  //   растеризация и проверка пересечения клеточных множеств.
  static Conflict find_first(const std::vector<Path>& paths,
                              const std::vector<float>& footprint_cells) {
    const size_t n = paths.size();
    const size_t T = max_len(paths);

    // Precompute grace periods for overlapping-start pairs.
    // Agents whose starts overlap (at hard radius) get
    // ceil(combined_radius - distance) + 1 timesteps to separate
    // before conflicts are enforced.
    std::vector<std::vector<size_t>> grace(n, std::vector<size_t>(n, 0));
    for (size_t i = 0; i < n; ++i)
      for (size_t j = i + 1; j < n; ++j) {
        const size_t g = start_grace(paths[i][0], footprint_cells[i],
                                     paths[j][0], footprint_cells[j]);
        grace[i][j] = grace[j][i] = g;
      }

    for (size_t t = 0; t < T; ++t) {
      for (size_t i = 0; i < n; ++i) {
        const Cell ci = at(paths[i], t);
        for (size_t j = i + 1; j < n; ++j) {
          // Skip all conflicts during grace period for overlapping starts
          if (t < grace[i][j]) continue;

          const Cell cj = at(paths[j], t);

          // Vertex conflict: физическое пересечение (hard radii)
          if (cells_conflict(ci, footprint_cells[i], cj, footprint_cells[j])) {
            return {ConflictType::Vertex, i, j, t, ci, cj};
          }

          // Edge conflict: встречное движение
          if (t > 0) {
            const Cell pi = at(paths[i], t - 1);
            const Cell pj = at(paths[j], t - 1);
            // i двигался pi→ci, j двигался pj→cj
            // Конфликт если после обмена позициями оба слишком близко
            if (cells_conflict(pi, footprint_cells[i], cj, footprint_cells[j]) &&
                cells_conflict(pj, footprint_cells[j], ci, footprint_cells[i]))
              return {ConflictType::Edge, i, j, t - 1, pi, ci};
          }
        }
      }
    }
    return {};
  }

  // How many timesteps two overlapping agents need to separate.
  // Returns 0 if starts don't overlap.
  static size_t start_grace(const Cell& a, float ra, const Cell& b, float rb) {
    const float combined = ra + rb;
    if (combined <= 0.0f) return 0;
    const float dr = static_cast<float>(a.row) - static_cast<float>(b.row);
    const float dc = static_cast<float>(a.col) - static_cast<float>(b.col);
    const float dist = std::sqrt(dr * dr + dc * dc);
    if (dist >= combined) return 0;
    return static_cast<size_t>(std::ceil(combined - dist)) + 1;
  }

 private:
  static Cell at(const Path& p, size_t t) {
    if (p.empty()) return {};
    return t < p.size() ? p[t] : p.back();
  }
  static size_t max_len(const std::vector<Path>& ps) {
    size_t r = 0;
    for (const auto& p : ps) r = std::max(r, p.size());
    return r;
  }
  // Проверка конфликта двух кругов.
  // При нулевых радиусах — совпадение клеток.
  static bool cells_conflict(const Cell& a, float ra, const Cell& b, float rb) {
    const float min_dist = ra + rb;
    if (min_dist <= 0.0f) {
      return a.row == b.row && a.col == b.col;
    }
    const float dr = static_cast<float>(a.row) - static_cast<float>(b.row);
    const float dc = static_cast<float>(a.col) - static_cast<float>(b.col);
    return dr * dr + dc * dc < min_dist * min_dist;
  }
};

// ---------------------------------------------------------------------------
// Priority Graph
// ---------------------------------------------------------------------------

class PriorityGraph {
 public:
  explicit PriorityGraph(size_t n = 0) : n_(n), m_(n * n, 0) {}

  size_t size() const { return n_; }

  bool has(size_t hi, size_t lo) const { return at(hi, lo); }

  bool add(size_t hi, size_t lo) {
    if (hi == lo) return false;
    if (has(lo, hi)) return false;
    if (has(hi, lo)) return true;

    std::vector<size_t> preds, succs;
    for (size_t i = 0; i < n_; ++i) if (i == hi || has(i, hi)) preds.push_back(i);
    for (size_t j = 0; j < n_; ++j) if (j == lo || has(lo, j)) succs.push_back(j);
    for (size_t p : preds) for (size_t s : succs) if (p == s) return false;
    for (size_t p : preds) for (size_t s : succs) set(p, s, 1);
    return true;
  }

  std::vector<size_t> higher_than(size_t a) const {
    std::vector<size_t> r;
    for (size_t i = 0; i < n_; ++i) if (has(i, a)) r.push_back(i);
    return r;
  }

  std::vector<size_t> topo_from(size_t root) const {
    std::vector<uint8_t> sel(n_, 0);
    sel[root] = 1;
    for (size_t v = 0; v < n_; ++v) if (has(root, v)) sel[v] = 1;

    std::vector<int> indeg(n_, 0);
    for (size_t u = 0; u < n_; ++u) {
      if (!sel[u]) continue;
      for (size_t v = 0; v < n_; ++v) if (sel[v] && has(u, v)) ++indeg[v];
    }

    std::deque<size_t> q;
    for (size_t v = 0; v < n_; ++v) if (sel[v] && indeg[v] == 0) q.push_back(v);

    std::vector<size_t> order;
    while (!q.empty()) {
      size_t v = q.front(); q.pop_front();
      order.push_back(v);
      for (size_t to = 0; to < n_; ++to) {
        if (sel[to] && has(v, to) && --indeg[to] == 0) q.push_back(to);
      }
    }
    return order;
  }

  size_t hash() const {
    size_t h = 0;
    for (size_t i = 0; i < m_.size(); i += 8) {
      uint64_t b = 0;
      for (size_t j = 0; j < 8 && i+j < m_.size(); ++j)
        b |= static_cast<uint64_t>(m_[i+j] != 0) << j;
      h ^= std::hash<uint64_t>{}(b) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
  }

  bool operator==(const PriorityGraph& o) const { return n_ == o.n_ && m_ == o.m_; }

 private:
  size_t n_;
  std::vector<uint8_t> m_;

  uint8_t at(size_t i, size_t j) const { return m_[i * n_ + j]; }
  void    set(size_t i, size_t j, uint8_t v) { m_[i * n_ + j] = v; }
};

// ---------------------------------------------------------------------------
// PBS Solver (публичный API)
// ---------------------------------------------------------------------------

class PBSSolver {
 public:
  PBSSolver() = default;

  void set_map(const GridMap* map) {
    map_ = map;
    inflated_cache_.clear();
    dist_cache_.clear();
  }

  // Основной вызов. Возвращает true если решение найдено.
  // solution[i] — путь для agents[i], каждый элемент — Cell в grid-координатах.
  // resolution — размер клетки PBS-сетки в метрах (для конвертации радиуса).
  // stats — если не nullptr, заполняется статистикой решения.
  // cost_curve — gradient shape (Linear, Quadratic, Cubic).
  // proximity_penalty — cost per step at the hard boundary (0 at soft boundary).
  // max_astar_expansions — per-A* call expansion limit (0 = unlimited).
  bool solve(const std::vector<Agent>& agents, std::vector<Path>& solution,
             float resolution = 0.0f, SolveStats* stats = nullptr,
             size_t max_expansions = 5000,
             CostCurve cost_curve = CostCurve::Quadratic,
             int proximity_penalty = 50,
             size_t max_astar_expansions = 200000) {
    solution.clear();
    if (!map_ || agents.empty()) return false;

    const size_t n = agents.size();
    stats_ = stats;
    cost_curve_ = cost_curve;
    proximity_penalty_ = proximity_penalty;
    max_astar_expansions_ = max_astar_expansions;

    // Предвычислить радиусы в клетках: footprint (hard) и soft (footprint+inflation)
    footprint_cells_.resize(n);
    soft_cells_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      footprint_cells_[i] = (resolution > 0.0f && agents[i].footprint_radius > 0.0f)
          ? agents[i].footprint_radius / resolution
          : 0.0f;
      soft_cells_[i] = (resolution > 0.0f)
          ? (agents[i].footprint_radius + agents[i].inflation) / resolution
          : 0.0f;
    }

    // Кэшировать gradient-inflated карты для каждого уникального soft_radius.
    // Маленькие роботы пролезут в щели, большие формации — нет.
    // Hard boundary = footprint, soft boundary = footprint + inflation.
    for (size_t i = 0; i < n; ++i) {
      const float soft = agents[i].footprint_radius + agents[i].inflation;
      if (soft > 0.0f && inflated_cache_.find(soft) == inflated_cache_.end()) {
        inflated_cache_[soft] = map_->inflate_gradient(
            agents[i].footprint_radius, soft, resolution,
            proximity_penalty, cost_curve);
      }
    }

    // Precompute Dijkstra distances from each agent's goal on its
    // gradient map.  Used as A* heuristic and for agent ordering.
    // Cached by goal cell index (agents sharing a goal reuse the map).
    agent_goal_dists_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      set_agent_map(agents[i]);
      const size_t key = agents[i].goal.row * map_->cols + agents[i].goal.col;
      if (dist_cache_.find(key) == dist_cache_.end()) {
        dist_cache_[key] = bfs_from(*low_level_.current_map(),
                                          agents[i].goal);
      }
      agent_goal_dists_[i] = &dist_cache_[key];
    }

    // Per-agent true distances (Dijkstra cost from start to goal).
    // Used for branch ordering and root path ordering.
    agent_dists_.resize(n);
    size_t max_dist = 0;
    for (size_t i = 0; i < n; ++i) {
      const size_t si = agents[i].start.row * map_->cols + agents[i].start.col;
      const int d = (*agent_goal_dists_[i])[si];
      agent_dists_[i] = d < std::numeric_limits<int>::max() / 4
          ? static_cast<size_t>(d) : 0;
      max_dist = std::max(max_dist, agent_dists_[i]);
    }

    // max_t: time horizon for Space-Time A*.
    // Критично: карта 600x600 = 360k клеток, при max_t=1000
    // Space-Time A* выделяет 360M состояний и зависает.
    const size_t max_t = std::min<size_t>(max_dist * 4 + 64, 600);

    // Корневой узел: sequential root planning.
    // Plan agents by decreasing distance to goal — longest paths get
    // the most freedom, stationary agents (dist=0) plan last and must
    // yield to moving agents.
    PBSNode root;
    root.pg = PriorityGraph(n);
    root.paths.resize(n);

    if (stats) stats->diag.max_t_used = max_t;

    std::vector<size_t> root_order(n);
    for (size_t i = 0; i < n; ++i) root_order[i] = i;
    std::sort(root_order.begin(), root_order.end(), [&](size_t a, size_t b) {
      return agent_dists_[a] > agent_dists_[b];
    });

    ReservationTable root_res;
    for (size_t idx : root_order) {
      set_agent_map(agents[idx]);
      low_level_.set_goal_dists(agent_goal_dists_[idx]);
      // Root planning: uncapped A* — quality root paths prevent PBS explosion.
      if (!low_level_.find_path(agents[idx].start, agents[idx].goal,
                                 root_res, footprint_cells_[idx], soft_cells_[idx],
                                 max_t, root.paths[idx],
                                 cost_curve, proximity_penalty)) {
        record_astar(false);
        // Fallback: A* without reservations. PBS resolves conflicts.
        ReservationTable empty;
        if (!low_level_.find_path(agents[idx].start, agents[idx].goal,
                                   empty, footprint_cells_[idx], soft_cells_[idx],
                                   max_t, root.paths[idx],
                                   cost_curve, proximity_penalty)) {
          record_astar(false);
          if (stats) {
            stats->diag.fail_reason = FailReason::RootPathFailed;
            stats->diag.fail_agent = idx;
          }
          return false;
        }
        record_astar(true);
      } else {
        record_astar(true);
      }
      // Reserve this agent's path so subsequent agents avoid it
      root_res.reserve_path(root.paths[idx], max_t,
                            footprint_cells_[idx], soft_cells_[idx]);
    }
    root.cost = cost(root.paths);

    std::priority_queue<PBSNode, std::vector<PBSNode>, PBSNodeCmp> open;
    std::unordered_set<NodeKey, NodeKeyHash> visited;

    open.push(root);
    visited.insert(make_key(root));

    size_t expansions = 0;

    while (!open.empty()) {
      if (++expansions > max_expansions) {
        if (stats) {
          stats->expansions = expansions;
          stats->diag.fail_reason = FailReason::MaxExpansions;
        }
        return false;
      }

      PBSNode node = open.top(); open.pop();

      const Conflict c = ConflictDetector::find_first(
          node.paths, footprint_cells_);
      if (c.type == ConflictType::None) {
        solution = node.paths;
        if (stats) {
          stats->expansions = expansions;
          stats->max_path_length = 0;
          for (const auto& p : solution)
            stats->max_path_length = std::max(stats->max_path_length, p.size());
        }
        return true;
      }

      if (stats && expansions == 1) stats->diag.first_conflict = c;

      // Branch ordering heuristic: try giving priority to the agent
      // with the further goal first — it needs more space/freedom.
      size_t hi = c.agent1, lo = c.agent2;
      if (agent_dists_[lo] > agent_dists_[hi]) std::swap(hi, lo);

      // ветка 1: further-goal agent gets priority
      PBSNode ch1 = node;
      if (branch(agents, hi, lo, max_t, ch1)) {
        auto k = make_key(ch1);
        if (visited.insert(k).second) { ch1.cost = cost(ch1.paths); open.push(std::move(ch1)); }
        if (stats) ++stats->diag.branches_tried;
      } else {
        if (stats) ++stats->diag.branches_failed;
      }

      // ветка 2: closer-goal agent gets priority
      PBSNode ch2 = std::move(node);
      if (branch(agents, lo, hi, max_t, ch2)) {
        auto k = make_key(ch2);
        if (visited.insert(k).second) { ch2.cost = cost(ch2.paths); open.push(std::move(ch2)); }
        if (stats) ++stats->diag.branches_tried;
      } else {
        if (stats) ++stats->diag.branches_failed;
      }
    }
    if (stats) {
      stats->expansions = expansions;
      stats->diag.fail_reason = FailReason::BranchExhausted;
    }
    return false;
  }

 private:
  struct PBSNode {
    PriorityGraph    pg;
    std::vector<Path> paths;
    size_t           cost = 0;
  };
  struct PBSNodeCmp {
    bool operator()(const PBSNode& a, const PBSNode& b) const { return a.cost > b.cost; }
  };
  struct NodeKey {
    size_t gh;
    std::vector<size_t> lens;
    bool operator==(const NodeKey& o) const { return gh == o.gh && lens == o.lens; }
  };
  struct NodeKeyHash {
    size_t operator()(const NodeKey& k) const {
      size_t h = k.gh;
      for (size_t x : k.lens) h ^= std::hash<size_t>{}(x) + 0x9e3779b9 + (h<<6) + (h>>2);
      return h;
    }
  };

  const GridMap*         map_ = nullptr;
  SpaceTimeAStarPlanner  low_level_;
  SolveStats*            stats_ = nullptr;  // set during solve(), nullable
  CostCurve              cost_curve_ = CostCurve::Quadratic;
  int                    proximity_penalty_ = 50;
  size_t                 max_astar_expansions_ = 200000;

  // Call after each find_path() to accumulate A* stats.
  void record_astar(bool ok) {
    if (!stats_) return;
    const size_t e = low_level_.last_expansions();
    if (ok) {
      stats_->astar.ok_total_exp += e;
      stats_->astar.ok_max_exp = std::max(stats_->astar.ok_max_exp, e);
      ++stats_->astar.ok_count;
    } else {
      ++stats_->astar.fail_count;
    }
  }
  std::vector<float>     footprint_cells_;  // per-agent hard radius in cells
  std::vector<float>     soft_cells_;       // per-agent soft radius in cells
  std::vector<size_t>    agent_dists_;      // per-agent true distance to goal
  std::vector<const std::vector<int>*> agent_goal_dists_;  // per-agent Dijkstra map

  // Кэш gradient-inflated карт: soft_radius (в метрах) -> GridMap
  std::unordered_map<float, GridMap> inflated_cache_;

  // Dijkstra distance cache: goal_cell_index -> distance map.
  // Precomputed backward from each goal on the agent's gradient map.
  std::unordered_map<size_t, std::vector<int>> dist_cache_;

  // Compute shortest-path distances (in steps) from every cell to the
  // goal via BFS.  All edges have uniform cost 1, so BFS is optimal
  // (O(V) vs O(V log V) for Dijkstra with a priority queue).
  // Accounts for wall topology (blocked cells) but excludes penalties.
  // Used as A* heuristic and for agent ordering / max_t computation.
  static std::vector<int> bfs_from(const GridMap& map, const Cell& goal) {
    const size_t N = map.rows * map.cols;
    static constexpr int INF = std::numeric_limits<int>::max() / 4;
    std::vector<int> dist(N, INF);

    std::deque<size_t> q;
    const size_t gi = goal.row * map.cols + goal.col;
    dist[gi] = 0;
    q.push_back(gi);

    while (!q.empty()) {
      const size_t ci = q.front(); q.pop_front();
      const int d = dist[ci];
      const size_t r = ci / map.cols;
      const size_t c = ci % map.cols;

      auto relax = [&](size_t nr, size_t nc) {
        if (nr >= map.rows || nc >= map.cols) return;
        const size_t ni = nr * map.cols + nc;
        if (map.blocked[ni]) return;
        if (d + 1 < dist[ni]) {
          dist[ni] = d + 1;
          q.push_back(ni);
        }
      };

      if (r > 0)            relax(r - 1, c);
      if (r + 1 < map.rows) relax(r + 1, c);
      if (c > 0)            relax(r, c - 1);
      if (c + 1 < map.cols) relax(r, c + 1);
    }
    return dist;
  }

  // Установить карту для A* конкретного агента (gradient-inflated).
  // Маленькие роботы получают менее раздутую карту и могут проходить
  // через узкие проходы, недоступные большим формациям.
  void set_agent_map(const Agent& a) {
    const float soft = a.footprint_radius + a.inflation;
    if (soft > 0.0f) {
      auto it = inflated_cache_.find(soft);
      if (it != inflated_cache_.end()) {
        low_level_.reset_map(&it->second);
        return;
      }
    }
    low_level_.reset_map(map_);
  }

  static size_t cost(const std::vector<Path>& ps) {
    size_t c = 0;
    for (const auto& p : ps) if (!p.empty()) c += p.size() - 1;
    return c;
  }

  static NodeKey make_key(const PBSNode& n) {
    NodeKey k; k.gh = n.pg.hash();
    for (const auto& p : n.paths) k.lens.push_back(p.size());
    return k;
  }

  bool branch(const std::vector<Agent>& agents, size_t hi, size_t lo,
              size_t max_t, PBSNode& node) {
    if (!node.pg.add(hi, lo)) return false;
    return replan(agents, lo, max_t, node);
  }

  bool replan(const std::vector<Agent>& agents, size_t changed,
              size_t max_t, PBSNode& node) {
    const auto order = node.pg.topo_from(changed);
    for (size_t ai : order) {
      // Резервируем пути всех высокоприоритетных агентов с их радиусами.
      // For agents whose starts overlap with ours, skip reservation
      // entries during the grace period so A* can plan an escape path.
      ReservationTable res;
      for (size_t bi : node.pg.higher_than(ai)) {
        const size_t grace = ConflictDetector::start_grace(
            agents[ai].start, footprint_cells_[ai],
            agents[bi].start, footprint_cells_[bi]);
        res.reserve_path(node.paths[bi], max_t,
                         footprint_cells_[bi], soft_cells_[bi], grace);
      }

      // Используем gradient-inflated карту для данного агента
      set_agent_map(agents[ai]);
      low_level_.set_goal_dists(agent_goal_dists_[ai]);
      Path p;
      if (!low_level_.find_path(agents[ai].start, agents[ai].goal,
                                 res, footprint_cells_[ai], soft_cells_[ai],
                                 max_t, p,
                                 cost_curve_, proximity_penalty_,
                                 max_astar_expansions_)) {
        record_astar(false);
        return false;
      }
      record_astar(true);
      node.paths[ai] = std::move(p);
    }
    return true;
  }
};