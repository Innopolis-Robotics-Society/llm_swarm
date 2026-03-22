#pragma once

#include <algorithm>
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
};

struct GridMap {
  size_t rows = 0;
  size_t cols = 0;
  std::vector<uint8_t> blocked;  // 1 = стена, 0 = свободно
};

using Path = std::vector<Cell>;

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

class ReservationTable {
 public:
  void clear() {
    edge_reserved_.clear();
    reserved_times_.clear();
    held_from_.clear();
  }

  void reserve_vertex(size_t idx, size_t time) {
    auto& times = reserved_times_[idx];
    if (!times.empty() && times.back() < time) {
      times.push_back(time);
      return;
    }
    auto pos = std::lower_bound(times.begin(), times.end(), time);
    if (pos == times.end() || *pos != time) times.insert(pos, time);
  }

  void hold_goal_from(size_t idx, size_t from_time) {
    auto it = held_from_.find(idx);
    if (it == held_from_.end() || from_time < it->second) held_from_[idx] = from_time;
  }

  void reserve_edge(size_t from_idx, size_t to_idx, size_t time) {
    edge_reserved_.insert(pack_edge(from_idx, to_idx, time));
  }

  bool is_vertex_reserved(size_t idx, size_t time) const {
    auto hit = held_from_.find(idx);
    if (hit != held_from_.end() && time >= hit->second) return true;
    auto it = reserved_times_.find(idx);
    if (it == reserved_times_.end()) return false;
    return std::binary_search(it->second.begin(), it->second.end(), time);
  }

  bool is_edge_reserved(size_t from_idx, size_t to_idx, size_t time) const {
    return edge_reserved_.count(pack_edge(from_idx, to_idx, time)) > 0;
  }

  void reserve_path(const Path& path, size_t cols, size_t hold_goal_until_time) {
    if (path.empty()) return;
    for (size_t t = 0; t < path.size(); ++t) {
      const size_t idx = path[t].row * cols + path[t].col;
      reserve_vertex(idx, t);
      if (t > 0) {
        const size_t prev = path[t - 1].row * cols + path[t - 1].col;
        reserve_edge(prev, idx, t - 1);
      }
    }
    const size_t goal_idx = path.back().row * cols + path.back().col;
    if (path.size() <= hold_goal_until_time) hold_goal_from(goal_idx, path.size());
  }

  bool can_hold_goal(size_t idx, size_t from_time, size_t max_time) const {
    auto hit = held_from_.find(idx);
    if (hit != held_from_.end() && hit->second <= max_time) return false;
    auto it = reserved_times_.find(idx);
    if (it == reserved_times_.end()) return true;
    auto lb = std::lower_bound(it->second.begin(), it->second.end(), from_time);
    return lb == it->second.end() || *lb > max_time;
  }

 private:
  std::unordered_set<uint64_t>                   edge_reserved_;
  std::unordered_map<size_t, std::vector<size_t>> reserved_times_;
  std::unordered_map<size_t, size_t>              held_from_;

  static uint64_t pack_edge(size_t from, size_t to, size_t t) {
    return (static_cast<uint64_t>(t) << 42) |
           (static_cast<uint64_t>(from) << 21) |
           static_cast<uint64_t>(to);
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

  bool find_path(const Cell& start, const Cell& goal,
                 const ReservationTable& reservations, size_t max_time,
                 Path& path) {
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

    if (reservations.is_vertex_reserved(start_idx, 0)) return false;

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
        if (reservations.can_hold_goal(goal_idx, cur.t, max_time)) {
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
             std::priority_queue<Node, std::vector<Node>, NodeCmp>& open,
             size_t sp) {
    if (is_blocked(nr, nc)) return;
    const size_t ni = idx(nr, nc);
    if (res.is_vertex_reserved(ni, nt)) return;
    if (res.is_edge_reserved(ni, ci, ct)) return;

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
  static Conflict find_first(const std::vector<Path>& paths, size_t cols) {
    const size_t n = paths.size();
    const size_t T = max_len(paths);

    std::unordered_map<uint64_t, size_t> vowner, eowner;
    vowner.reserve(n * 8);
    eowner.reserve(n * 8);

    for (size_t t = 0; t < T; ++t) {
      vowner.clear(); eowner.clear();
      for (size_t i = 0; i < n; ++i) {
        const Cell cur = at(paths[i], t);
        const size_t ci = cur.row * cols + cur.col;

        auto vit = vowner.find(pack_vt(ci, t));
        if (vit != vowner.end())
          return {ConflictType::Vertex, vit->second, i, t, cur, {}};
        vowner.emplace(pack_vt(ci, t), i);

        if (t > 0) {
          const Cell prev = at(paths[i], t - 1);
          const size_t pi = prev.row * cols + prev.col;
          auto eit = eowner.find(pack_et(ci, pi, t - 1));
          if (eit != eowner.end())
            return {ConflictType::Edge, eit->second, i, t - 1, prev, cur};
          eowner.emplace(pack_et(pi, ci, t - 1), i);
        }
      }
    }
    return {};
  }

 private:
  static Cell   at(const Path& p, size_t t) {
    if (p.empty()) return {};
    return t < p.size() ? p[t] : p.back();
  }
  static size_t max_len(const std::vector<Path>& ps) {
    size_t r = 0;
    for (const auto& p : ps) r = std::max(r, p.size());
    return r;
  }
  static uint64_t pack_vt(size_t i, size_t t) {
    return (static_cast<uint64_t>(t) << 32) | i;
  }
  static uint64_t pack_et(size_t f, size_t to, size_t t) {
    return (static_cast<uint64_t>(t) << 42) | (static_cast<uint64_t>(f) << 21) | to;
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
    low_level_.reset_map(map);
  }

  // Основной вызов. Возвращает true если решение найдено.
  // solution[i] — путь для agents[i], каждый элемент — Cell в grid-координатах.
  bool solve(const std::vector<Agent>& agents, std::vector<Path>& solution) {
    solution.clear();
    if (!map_ || agents.empty()) return false;

    const size_t n = agents.size();

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

    ReservationTable empty;
    for (size_t i = 0; i < n; ++i) {
      if (!low_level_.find_path(agents[i].start, agents[i].goal, empty, max_t, root.paths[i]))
        return false;
    }
    root.cost = cost(root.paths);

    std::priority_queue<PBSNode, std::vector<PBSNode>, PBSNodeCmp> open;
    std::unordered_set<NodeKey, NodeKeyHash> visited;

    open.push(root);
    visited.insert(make_key(root));

    size_t expansions = 0;
    static constexpr size_t MAX_EXP = 200000;

    while (!open.empty()) {
      if (++expansions > MAX_EXP) return false;

      PBSNode node = open.top(); open.pop();

      const Conflict c = ConflictDetector::find_first(node.paths, map_->cols);
      if (c.type == ConflictType::None) {
        solution = node.paths;
        return true;
      }

      // ветка 1: agent1 имеет приоритет над agent2
      PBSNode ch1 = node;
      if (branch(agents, c.agent1, c.agent2, max_t, ch1)) {
        auto k = make_key(ch1);
        if (visited.insert(k).second) { ch1.cost = cost(ch1.paths); open.push(std::move(ch1)); }
      }

      // ветка 2: agent2 имеет приоритет над agent1
      PBSNode ch2 = std::move(node);
      if (branch(agents, c.agent2, c.agent1, max_t, ch2)) {
        auto k = make_key(ch2);
        if (visited.insert(k).second) { ch2.cost = cost(ch2.paths); open.push(std::move(ch2)); }
      }
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
      ReservationTable res;
      for (size_t bi : node.pg.higher_than(ai))
        res.reserve_path(node.paths[bi], map_->cols, max_t);

      Path p;
      if (!low_level_.find_path(agents[ai].start, agents[ai].goal, res, max_t, p))
        return false;
      node.paths[ai] = std::move(p);
    }
    return true;
  }
};