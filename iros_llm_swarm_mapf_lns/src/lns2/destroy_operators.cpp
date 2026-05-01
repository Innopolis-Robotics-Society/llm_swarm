// iros_llm_swarm_mapf/lns2/src/destroy_operators.cpp

#include "iros_llm_swarm_mapf/lns2/destroy_operators.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace lns2 {

// ---------------------------------------------------------------------------
// Utility: pick one AgentId from a non-empty container uniformly.
// ---------------------------------------------------------------------------

template <typename Container>
static AgentId pick_one(const Container& c, std::mt19937& rng) {
  std::vector<AgentId> v(c.begin(), c.end());
  std::uniform_int_distribution<std::size_t> U(0, v.size() - 1);
  return v[U(rng)];
}

// ---------------------------------------------------------------------------
// Collision-based
// ---------------------------------------------------------------------------

class CollisionBased : public DestroyOperator {
 public:
  DestroyOp id() const override { return DestroyOp::CollisionBased; }
  std::vector<AgentId> select(const Solution& sol, std::size_t N,
                                std::mt19937& rng) override
  {
    auto coll = sol.table().sample_random_collision(
        rng, sol.paths(), sol.footprints(), sol.holds());
    if (!coll) return {};

    std::unordered_set<AgentId> set{coll->a, coll->b};
    std::vector<AgentId> frontier{coll->a, coll->b};

    while (set.size() < N && !frontier.empty()) {
      std::uniform_int_distribution<std::size_t> Uf(0, frontier.size() - 1);
      const std::size_t pi = Uf(rng);
      AgentId seed = frontier[pi];
      frontier[pi] = frontier.back();
      frontier.pop_back();

      const auto& neigh = sol.table().agents_touching(seed);

      // Weight candidates by their own collision counts (heavier = more
      // likely to be picked).
      std::vector<AgentId> cands;
      std::vector<std::size_t> w;
      cands.reserve(neigh.size());
      w.reserve(neigh.size());
      std::size_t total_w = 0;
      for (AgentId n : neigh) {
        if (set.count(n)) continue;
        const std::size_t cw = std::max<std::size_t>(
            1, sol.table().collisions_of(n));
        cands.push_back(n);
        w.push_back(cw);
        total_w += cw;
      }
      if (cands.empty()) continue;

      std::uniform_int_distribution<std::size_t> Uw(1, total_w);
      std::size_t r = Uw(rng);
      AgentId chosen = cands[0];
      for (std::size_t i = 0; i < cands.size(); ++i) {
        if (r <= w[i]) { chosen = cands[i]; break; }
        r -= w[i];
      }
      set.insert(chosen);
      frontier.push_back(chosen);
    }

    return {set.begin(), set.end()};
  }
};

// ---------------------------------------------------------------------------
// Random walk (spatially close agents, via vertex-occupancy co-residency)
// ---------------------------------------------------------------------------

class RandomWalk : public DestroyOperator {
 public:
  DestroyOp id() const override { return DestroyOp::RandomWalk; }
  std::vector<AgentId> select(const Solution& sol, std::size_t N,
                                std::mt19937& rng) override
  {
    const std::size_t A = sol.num_agents();
    if (A == 0) return {};

    // Pick a seed agent with non-empty path, uniformly at random.
    std::vector<AgentId> nonempty;
    nonempty.reserve(A);
    for (AgentId i = 0; i < A; ++i) {
      if (!sol.path(i).empty()) nonempty.push_back(i);
    }
    if (nonempty.empty()) return {};
    std::uniform_int_distribution<std::size_t> U(0, nonempty.size() - 1);
    AgentId seed = nonempty[U(rng)];

    std::unordered_set<AgentId> set{seed};
    std::vector<AgentId> frontier{seed};

    while (set.size() < N && !frontier.empty()) {
      std::uniform_int_distribution<std::size_t> Uf(0, frontier.size() - 1);
      const std::size_t pi = Uf(rng);
      AgentId cur = frontier[pi];
      frontier[pi] = frontier.back();
      frontier.pop_back();

      // Spatially close = touches_ (which includes actual collisions; for
      // a collision-free solution this will be empty, in which case we fall
      // through to an empty result — RandomWalk is mostly useful when some
      // conflicts exist).
      const auto& neigh = sol.table().agents_touching(cur);
      std::vector<AgentId> cands;
      for (AgentId n : neigh) if (!set.count(n)) cands.push_back(n);
      if (cands.empty()) continue;
      std::uniform_int_distribution<std::size_t> Uc(0, cands.size() - 1);
      AgentId chosen = cands[Uc(rng)];
      set.insert(chosen);
      frontier.push_back(chosen);
    }

    return {set.begin(), set.end()};
  }
};

// ---------------------------------------------------------------------------
// Random
// ---------------------------------------------------------------------------

class RandomDestroy : public DestroyOperator {
 public:
  DestroyOp id() const override { return DestroyOp::Random; }
  std::vector<AgentId> select(const Solution& sol, std::size_t N,
                                std::mt19937& rng) override
  {
    const std::size_t A = sol.num_agents();
    if (A == 0) return {};
    std::vector<AgentId> all(A);
    std::iota(all.begin(), all.end(), 0);
    std::shuffle(all.begin(), all.end(), rng);
    if (all.size() > N) all.resize(N);
    return all;
  }
};

// ---------------------------------------------------------------------------
// Bottleneck — pick agents traversing the hottest grid cell.
// ---------------------------------------------------------------------------

class Bottleneck : public DestroyOperator {
 public:
  DestroyOp id() const override { return DestroyOp::Bottleneck; }
  std::vector<AgentId> select(const Solution& sol, std::size_t N,
                                std::mt19937& rng) override
  {
    // Build traffic map: cell -> set of agent ids (distinct traversals).
    const GridMap* grid = sol.grid();
    if (!grid) return {};

    std::unordered_map<CellIdx, std::unordered_set<AgentId>> traffic;
    traffic.reserve(1024);

    for (AgentId id = 0; id < sol.num_agents(); ++id) {
      const Path& p = sol.path(id);
      if (p.empty()) continue;
      // Cheap approximation: only count base cells, not full footprint.
      for (const Cell& c : p) {
        if (!grid->in_bounds(c)) continue;
        traffic[grid->index_of(c)].insert(id);
      }
    }
    if (traffic.empty()) return {};

    // Top-K hottest cells.
    constexpr std::size_t K = 5;
    std::vector<std::pair<std::size_t, CellIdx>> heap;  // (size, cell)
    heap.reserve(traffic.size());
    for (const auto& [cell, s] : traffic) {
      heap.emplace_back(s.size(), cell);
    }
    std::partial_sort(heap.begin(),
                      heap.begin() + std::min(K, heap.size()),
                      heap.end(),
                      [](const auto& a, const auto& b) {
                        return a.first > b.first;
                      });
    const std::size_t top_n = std::min(K, heap.size());

    // Weighted pick by heat.
    std::size_t total_w = 0;
    for (std::size_t i = 0; i < top_n; ++i) total_w += heap[i].first;
    if (total_w == 0) return {};
    std::uniform_int_distribution<std::size_t> U(1, total_w);
    std::size_t r = U(rng);
    CellIdx chosen = heap[0].second;
    for (std::size_t i = 0; i < top_n; ++i) {
      if (r <= heap[i].first) { chosen = heap[i].second; break; }
      r -= heap[i].first;
    }

    std::vector<AgentId> through(traffic[chosen].begin(),
                                   traffic[chosen].end());
    std::shuffle(through.begin(), through.end(), rng);
    if (through.size() > N) through.resize(N);
    return through;
  }
};

// ---------------------------------------------------------------------------
// Factories
// ---------------------------------------------------------------------------

std::unique_ptr<DestroyOperator> make_collision_based() {
  return std::make_unique<CollisionBased>();
}
std::unique_ptr<DestroyOperator> make_random_walk() {
  return std::make_unique<RandomWalk>();
}
std::unique_ptr<DestroyOperator> make_random() {
  return std::make_unique<RandomDestroy>();
}
std::unique_ptr<DestroyOperator> make_bottleneck() {
  return std::make_unique<Bottleneck>();
}

}  // namespace lns2
