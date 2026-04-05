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

struct Cell {
  size_t row = 0;
  size_t col = 0;
};

struct Agent {
  size_t id = 0;   // порядковый номер (0..N-1)
  Cell start;
  Cell goal;
  float footprint_radius = 0.0f;  // радиус в метрах (0 = точечный агент)
};

struct GridMap {
  size_t rows = 0;
  size_t cols = 0;
  std::vector<uint8_t> blocked;  // 1 = стена, 0 = свободно

  // Возвращает новую карту, в которой все клетки в пределах
  // ceil(radius_m / resolution_m) от любой стены тоже заблокированы.
  GridMap inflate(float radius_m, float resolution_m) const {
    if (radius_m <= 0.0f) return *this;
    const int r = static_cast<int>(std::ceil(radius_m / resolution_m));
    GridMap out;
    out.rows = rows;
    out.cols = cols;
    out.blocked.assign(rows * cols, 0);
    for (size_t row = 0; row < rows; ++row) {
      for (size_t col = 0; col < cols; ++col) {
        if (blocked[row * cols + col] == 0) continue;
        for (int dr = -r; dr <= r; ++dr) {
          for (int dc = -r; dc <= r; ++dc) {
            if (dr * dr + dc * dc > r * r) continue;
            const int nr = static_cast<int>(row) + dr;
            const int nc = static_cast<int>(col) + dc;
            if (nr < 0 || nr >= static_cast<int>(rows)) continue;
            if (nc < 0 || nc >= static_cast<int>(cols)) continue;
            out.blocked[nr * cols + nc] = 1;
          }
        }
      }
    }
    return out;
  }
};

using Path = std::vector<Cell>;

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

struct SolveStats {
  size_t expansions = 0;
  size_t max_path_length = 0;
  SolveDiagnostics diag;
};

// ---------------------------------------------------------------------------
// Enum-ы
// ---------------------------------------------------------------------------

enum class HeuristicType { Manhattan };

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
  void reserve_path(const Path& path,
                    size_t hold_goal_until_time, float radius_cells) {
    if (path.empty()) return;
    for (size_t t = 0; t < path.size(); ++t) {
      entries_[t].push_back({path[t], radius_cells});
    }
    // Удержание цели: агент остаётся на месте после прибытия
    if (path.size() <= hold_goal_until_time) {
      held_goals_.push_back({path.back(), radius_cells, path.size()});
    }
  }

  // Можно ли агенту с радиусом my_radius_cells встать в клетку (row,col) в момент t?
  bool is_vertex_blocked(size_t row, size_t col, size_t time,
                          float my_radius_cells) const {
    // Проверяем удержанные цели
    for (const auto& hg : held_goals_) {
      if (time >= hg.from_time && cells_conflict(row, col, my_radius_cells,
                                                   hg.cell.row, hg.cell.col, hg.radius_cells))
        return true;
    }
    // Проверяем позиции на конкретном шаге
    auto it = entries_.find(time);
    if (it == entries_.end()) return false;
    for (const auto& e : it->second) {
      if (cells_conflict(row, col, my_radius_cells,
                          e.cell.row, e.cell.col, e.radius_cells))
        return true;
    }
    return false;
  }

  // Можно ли агенту перейти из (fr, fc) в (tr, tc) между шагами time и time+1?
  // Проверяет встречное движение (edge conflict).
  bool is_edge_blocked(size_t fr, size_t fc, size_t tr, size_t tc,
                        size_t time, float my_radius_cells) const {
    auto it = entries_.find(time);
    auto it_next = entries_.find(time + 1);
    if (it == entries_.end() || it_next == entries_.end()) return false;

    // Для каждого агента: если он двигается навстречу
    for (size_t i = 0; i < it->second.size() && i < it_next->second.size(); ++i) {
      const auto& cur  = it->second[i];
      const auto& next = it_next->second[i];
      // Edge conflict: после обмена оба агента оказываются слишком близко
      // к предыдущей позиции другого
      if (cells_conflict(fr, fc, my_radius_cells,
                          next.cell.row, next.cell.col, cur.radius_cells) &&
          cells_conflict(tr, tc, my_radius_cells,
                          cur.cell.row, cur.cell.col, cur.radius_cells))
        return true;
    }
    return false;
  }

  // Можно ли агенту удерживать цель начиная с from_time?
  bool can_hold_goal(size_t row, size_t col, size_t from_time,
                      size_t max_time, float my_radius_cells) const {
    for (const auto& hg : held_goals_) {
      if (cells_conflict(row, col, my_radius_cells,
                          hg.cell.row, hg.cell.col, hg.radius_cells))
        return false;
    }
    for (size_t t = from_time; t <= max_time; ++t) {
      auto it = entries_.find(t);
      if (it == entries_.end()) continue;
      for (const auto& e : it->second) {
        if (cells_conflict(row, col, my_radius_cells,
                            e.cell.row, e.cell.col, e.radius_cells))
          return false;
      }
    }
    return true;
  }

 private:
  struct Entry {
    Cell cell;
    float radius_cells;
  };
  struct HeldGoal {
    Cell cell;
    float radius_cells;
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
  explicit SpaceTimeAStarPlanner(HeuristicType h = HeuristicType::Manhattan)
      : heuristic_type_(h) {}

  void reset_map(const GridMap* map) { map_ = map; }

  // my_radius_cells — радиус данного агента в клетках (0 = точечный).
  // Используется для проверки дистанции до зарезервированных агентов
  // и для проверки удержания цели.
  bool find_path(const Cell& start, const Cell& goal,
                 const ReservationTable& reservations,
                 float my_radius_cells,
                 size_t max_time, Path& path) {
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

    if (reservations.is_vertex_blocked(start.row, start.col, 0, my_radius_cells))
      return false;

    set_g(start_state, 0);
    parent_[start_state] = INVALID;

    std::priority_queue<Node, std::vector<Node>, NodeCmp> open;
    open.push({start_idx, 0, 0, heuristic(start, goal)});

    while (!open.empty()) {
      const Node cur = open.top(); open.pop();
      const size_t cs = state_idx(cur.idx, cur.t, spatial);

      if (is_closed(cs)) continue;
      if (!is_alive(cs) || cur.g != g_[cs]) continue;
      mark_closed(cs);

      if (cur.idx == goal_idx) {
        if (reservations.can_hold_goal(goal.row, goal.col, cur.t,
                                        max_time, my_radius_cells)) {
          restore(cs, spatial, path);
          return true;
        }
      }

      if (cur.t >= max_time) continue;

      const Cell c  = cell(cur.idx);
      const size_t r = c.row, col_ = c.col;
      const size_t nt = cur.t + 1;

      // ждать на месте
      relax(cur.idx, cur.t, r, col_, nt, goal, reservations, my_radius_cells, open, spatial);
      // соседи
      if (r > 0)               relax(cur.idx, cur.t, r-1, col_,   nt, goal, reservations, my_radius_cells, open, spatial);
      if (r+1 < map_->rows)    relax(cur.idx, cur.t, r+1, col_,   nt, goal, reservations, my_radius_cells, open, spatial);
      if (col_ > 0)            relax(cur.idx, cur.t, r,   col_-1, nt, goal, reservations, my_radius_cells, open, spatial);
      if (col_+1 < map_->cols) relax(cur.idx, cur.t, r,   col_+1, nt, goal, reservations, my_radius_cells, open, spatial);
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
  HeuristicType   heuristic_type_;
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

  int heuristic(const Cell& a, const Cell& b) const {
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
             float my_radius_cells,
             std::priority_queue<Node, std::vector<Node>, NodeCmp>& open,
             size_t sp) {
    if (is_blocked(nr, nc)) return;
    if (res.is_vertex_blocked(nr, nc, nt, my_radius_cells)) return;

    // Edge conflict: проверяем встречное движение
    const Cell from_cell = cell(ci);
    if (res.is_edge_blocked(from_cell.row, from_cell.col, nr, nc, ct, my_radius_cells))
      return;

    const size_t ni = idx(nr, nc);
    const size_t cs = state_idx(ci, ct, sp);
    const size_t ns = state_idx(ni, nt, sp);
    if (is_closed(ns)) return;

    const int tg = get_g(cs) + 1;
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
  // Версия с учётом радиусов агентов (в клетках).
  // Level 1: евклидово расстояние между центрами vs (r_a + r_b).
  // Для точечных агентов (радиус 0) — эквивалентно проверке совпадения клеток.
  // TODO Level 2: при появлении ориентационно-зависимых полигонов —
  //   растеризация и проверка пересечения клеточных множеств.
  static Conflict find_first(const std::vector<Path>& paths,
                              const std::vector<float>& radii_cells) {
    const size_t n = paths.size();
    const size_t T = max_len(paths);

    for (size_t t = 0; t < T; ++t) {
      for (size_t i = 0; i < n; ++i) {
        const Cell ci = at(paths[i], t);
        for (size_t j = i + 1; j < n; ++j) {
          const Cell cj = at(paths[j], t);

          // Vertex conflict: центры слишком близко
          if (cells_conflict(ci, radii_cells[i], cj, radii_cells[j]))
            return {ConflictType::Vertex, i, j, t, ci, cj};

          // Edge conflict: встречное движение
          if (t > 0) {
            const Cell pi = at(paths[i], t - 1);
            const Cell pj = at(paths[j], t - 1);
            // i двигался pi→ci, j двигался pj→cj
            // Конфликт если после обмена позициями оба слишком близко
            if (cells_conflict(pi, radii_cells[i], cj, radii_cells[j]) &&
                cells_conflict(pj, radii_cells[j], ci, radii_cells[i]))
              return {ConflictType::Edge, i, j, t - 1, pi, ci};
          }
        }
      }
    }
    return {};
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
  explicit PBSSolver(HeuristicType h = HeuristicType::Manhattan)
      : low_level_(h) {}

  void set_map(const GridMap* map) {
    map_ = map;
    inflated_cache_.clear();
  }

  // Основной вызов. Возвращает true если решение найдено.
  // solution[i] — путь для agents[i], каждый элемент — Cell в grid-координатах.
  // resolution — размер клетки PBS-сетки в метрах (для конвертации радиуса).
  // stats — если не nullptr, заполняется статистикой решения.
  bool solve(const std::vector<Agent>& agents, std::vector<Path>& solution,
             float resolution = 0.0f, SolveStats* stats = nullptr) {
    solution.clear();
    if (!map_ || agents.empty()) return false;

    const size_t n = agents.size();
    resolution_ = resolution;

    // Предвычислить радиусы в клетках
    radii_cells_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      radii_cells_[i] = (resolution > 0.0f && agents[i].footprint_radius > 0.0f)
          ? agents[i].footprint_radius / resolution
          : 0.0f;
    }

    // Кэшировать inflated-карты для каждого уникального радиуса.
    // Маленькие роботы пролезут в щели, большие формации — нет.
    for (size_t i = 0; i < n; ++i) {
      const float r = agents[i].footprint_radius;
      if (r > 0.0f && inflated_cache_.find(r) == inflated_cache_.end()) {
        inflated_cache_[r] = map_->inflate(r, resolution);
      }
    }

    // max_t = максимальное манхэттенское расстояние среди всех агентов * 4
    // (запас на обходы и ожидания), но не меньше 64 и не больше 600.
    // Критично: карта 600x600 = 360k клеток, при max_t=1000
    // Space-Time A* выделяет 360M состояний и зависает.
    size_t max_dist = 0;
    for (const auto& a : agents) {
      const size_t dr = a.start.row > a.goal.row
          ? a.start.row - a.goal.row : a.goal.row - a.start.row;
      const size_t dc = a.start.col > a.goal.col
          ? a.start.col - a.goal.col : a.goal.col - a.start.col;
      max_dist = std::max(max_dist, dr + dc);
    }
    const size_t max_t = std::min<size_t>(max_dist * 4 + 64, 600);

    // Корневой узел: план без ограничений
    PBSNode root;
    root.pg = PriorityGraph(n);
    root.paths.resize(n);

    if (stats) stats->diag.max_t_used = max_t;

    ReservationTable empty;
    for (size_t i = 0; i < n; ++i) {
      set_agent_map(agents[i]);
      if (!low_level_.find_path(agents[i].start, agents[i].goal,
                                 empty, radii_cells_[i], max_t, root.paths[i])) {
        if (stats) {
          stats->diag.fail_reason = FailReason::RootPathFailed;
          stats->diag.fail_agent = i;
        }
        return false;
      }
    }
    root.cost = cost(root.paths);

    std::priority_queue<PBSNode, std::vector<PBSNode>, PBSNodeCmp> open;
    std::unordered_set<NodeKey, NodeKeyHash> visited;

    open.push(root);
    visited.insert(make_key(root));

    size_t expansions = 0;
    static constexpr size_t MAX_EXP = 200000;

    while (!open.empty()) {
      if (++expansions > MAX_EXP) {
        if (stats) {
          stats->expansions = expansions;
          stats->diag.fail_reason = FailReason::MaxExpansions;
        }
        return false;
      }

      PBSNode node = open.top(); open.pop();

      const Conflict c = ConflictDetector::find_first(
          node.paths, radii_cells_);
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

      // ветка 1: agent1 имеет приоритет над agent2
      PBSNode ch1 = node;
      if (branch(agents, c.agent1, c.agent2, max_t, ch1)) {
        auto k = make_key(ch1);
        if (visited.insert(k).second) { ch1.cost = cost(ch1.paths); open.push(std::move(ch1)); }
        if (stats) ++stats->diag.branches_tried;
      } else {
        if (stats) ++stats->diag.branches_failed;
      }

      // ветка 2: agent2 имеет приоритет над agent1
      PBSNode ch2 = std::move(node);
      if (branch(agents, c.agent2, c.agent1, max_t, ch2)) {
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
  float                  resolution_ = 0.0f;
  std::vector<float>     radii_cells_;

  // Кэш inflated карт: footprint_radius (в метрах) -> inflated GridMap
  std::unordered_map<float, GridMap> inflated_cache_;

  // Установить карту для A* конкретного агента (inflated если есть радиус).
  // Маленькие роботы получают менее раздутую карту и могут проходить
  // через узкие проходы, недоступные большим формациям.
  void set_agent_map(const Agent& a) {
    if (a.footprint_radius > 0.0f) {
      auto it = inflated_cache_.find(a.footprint_radius);
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
      // Резервируем пути всех высокоприоритетных агентов с их радиусами
      ReservationTable res;
      for (size_t bi : node.pg.higher_than(ai))
        res.reserve_path(node.paths[bi], max_t, radii_cells_[bi]);

      // Используем inflated-карту для данного агента
      set_agent_map(agents[ai]);
      Path p;
      if (!low_level_.find_path(agents[ai].start, agents[ai].goal,
                                 res, radii_cells_[ai], max_t, p))
        return false;
      node.paths[ai] = std::move(p);
    }
    return true;
  }
};