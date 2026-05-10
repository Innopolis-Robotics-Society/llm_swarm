// iros_llm_swarm_mapf_lns/lns2/src/warm_start.cpp

#include "iros_llm_swarm_mapf_lns/lns2/warm_start.hpp"

#include <algorithm>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace lns2 {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::int32_t chebyshev(const Cell& a, const Cell& b) {
  return std::max(std::abs(a.row - b.row), std::abs(a.col - b.col));
}

// Check that the tail of prev_path (from `from` inclusive to the end) is
// statically valid under the current map + this agent's footprint.
// We don't validate the head (already in the past).
static bool tail_statically_valid(const Path& prev_path,
                                   std::size_t from,
                                   const GridMap& grid,
                                   const FootprintModel& fp) {
  for (std::size_t i = from; i < prev_path.size(); ++i) {
    if (!footprint_fits(prev_path[i], fp, grid)) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// BFS splice: find a footprint-valid path from `start` to `target` on the
// static grid, length <= max_steps. Returns the sequence of cells including
// both endpoints, or empty if not found / not reachable.
//
// We use 4-connected moves regardless of params.astar.diagonal_moves — the
// splice is a short repair only (a handful of cells), and 4-connected keeps
// it MAPF-safe: no diagonal clipping past swept obstacles during execution.
// ---------------------------------------------------------------------------
static Path bfs_splice(const Cell& start, const Cell& target,
                        std::int32_t max_steps,
                        std::size_t max_expansions,
                        const GridMap& grid,
                        const FootprintModel& fp) {
  if (start == target) return Path{start};
  if (max_steps <= 0 || max_expansions == 0) return {};
  if (!footprint_fits(start, fp, grid)) return {};
  if (!footprint_fits(target, fp, grid)) return {};

  // Chebyshev lower bound — we use 4-conn here so Manhattan is correct,
  // but Chebyshev <= Manhattan so it is a valid early-exit check.
  const std::int32_t min_needed =
      std::abs(start.row - target.row) + std::abs(start.col - target.col);
  if (min_needed > max_steps) return {};

  // parent map: cell_idx -> (parent_cell_idx, depth)
  struct Node {
    CellIdx parent;
    std::int32_t depth;
  };
  std::unordered_map<CellIdx, Node> visited;
  visited.reserve(static_cast<std::size_t>(max_steps) * 4u + 16u);

  const CellIdx start_idx  = grid.index_of(start);
  const CellIdx target_idx = grid.index_of(target);

  std::queue<CellIdx> q;
  visited.emplace(start_idx, Node{start_idx, 0});
  q.push(start_idx);

  static const CellOffset step4[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  std::size_t expansions = 0;

  while (!q.empty()) {
    const CellIdx cur_idx = q.front();
    q.pop();
    if (++expansions > max_expansions) return {};

    const auto it = visited.find(cur_idx);
    const std::int32_t cur_depth = it->second.depth;
    if (cur_depth >= max_steps) continue;

    const Cell cur = grid.cell_of(cur_idx);
    for (const auto& s : step4) {
      const Cell nxt = cur + s;
      if (!grid.in_bounds(nxt)) continue;
      if (!footprint_fits(nxt, fp, grid)) continue;
      const CellIdx nxt_idx = grid.index_of(nxt);
      if (visited.count(nxt_idx)) continue;
      visited.emplace(nxt_idx, Node{cur_idx, cur_depth + 1});
      if (nxt_idx == target_idx) {
        // Reconstruct path
        Path rev;
        rev.push_back(nxt);
        CellIdx cursor = cur_idx;
        while (cursor != start_idx) {
          rev.push_back(grid.cell_of(cursor));
          cursor = visited[cursor].parent;
        }
        rev.push_back(start);
        return Path(rev.rbegin(), rev.rend());
      }
      q.push(nxt_idx);
    }
  }
  return {};
}

// ---------------------------------------------------------------------------
// Per-agent seed construction
// ---------------------------------------------------------------------------
//
// Given that a COMMON time shift `delta` is applied, each agent produces one
// of these seed shapes:
//
//   OnTrack  : shifted_tail = prev_path[delta .. end]
//              where actual == prev_path[delta].
//
//   Spliced  : seed = bfs_splice(actual, prev_path[delta+splice_len])
//              followed by prev_path[delta+splice_len+1 .. end]
//              (we splice to the earliest cell on the tail we can reach
//               from actual within patch_radius_cells).
//
//   StubFar  : seed = { actual } (held for horizon).
//
//   StubGoal : seed = { actual }; prev_path.back() != a.goal.
//
//   StubNew  : seed = { actual }; no prev_path for this agent.
//
//   StubInvalid : seed = { actual }; the tail of prev_path is statically
//                 invalid (wall appeared, etc.).
//
//   StubArrived : delta >= prev_path.size() and actual == a.goal:
//                 seed = { a.goal }. If actual != a.goal, we can't simply
//                 sit — classify as StubFar so repair tries to reach goal.
//
// For every stub, hold_until = horizon so the CollisionTable reserves that
// cell for the full time horizon (other agents see it as an obstacle in
// space-time). For OnTrack / Spliced, hold_until = horizon as well — short
// paths at their goal continue to block it.
// ---------------------------------------------------------------------------

struct SeedResult {
  Path path;
  SeedStatus status;
};

static SeedResult make_seed_for_agent(const Agent& a,
                                       const GridMap& grid,
                                       const Path& prev_path,
                                       Timestep delta,
                                       const Cell& actual_cell_in,
                                       const WarmStartParams& params) {
  // The actual (odom-derived) cell may not be footprint-valid at the
  // current map resolution — e.g. a 0.22 m robot at 0.20 m resolution whose
  // 4-conn footprint clips a wall. Using such a cell as the seed's start
  // breaks every downstream check (soft_astar::footprint_fits fails, agent
  // ends up with an empty path). Try to recover by snapping to the nearest
  // footprint-valid neighbour within a 2-cell Chebyshev box. This mirrors
  // the PBS-side convention of requiring footprint >= 0.7 * resolution but
  // is more forgiving at runtime.
  Cell actual_cell = actual_cell_in;
  if (!footprint_fits(actual_cell, a.footprint, grid)) {
    Cell best = actual_cell;
    bool found = false;
    for (std::int32_t dr = -2; dr <= 2 && !found; ++dr) {
      for (std::int32_t dc = -2; dc <= 2 && !found; ++dc) {
        const Cell c{actual_cell.row + dr, actual_cell.col + dc};
        if (footprint_fits(c, a.footprint, grid)) {
          best = c;
          found = true;
        }
      }
    }
    if (!found) {
      // Truly stuck: fall back to the agent's goal if it fits, else mark
      // invalid so the caller / solver can skip this agent gracefully
      // rather than silently producing an empty path downstream.
      if (footprint_fits(a.goal, a.footprint, grid)) {
        return SeedResult{Path{a.goal}, SeedStatus::StubInvalid};
      }
      return SeedResult{Path{}, SeedStatus::StubInvalid};
    }
    actual_cell = best;
  }

  // Agent new to the plan.
  if (prev_path.empty()) {
    return SeedResult{Path{actual_cell}, SeedStatus::StubNew};
  }

  // Goal changed since previous plan — tail leads to the wrong place.
  if (prev_path.back() != a.goal) {
    return SeedResult{Path{actual_cell}, SeedStatus::StubGoal};
  }

  const std::size_t L = prev_path.size();
  const std::size_t d = static_cast<std::size_t>(delta);

  // Whole prev plan is in the past.
  if (d >= L) {
    if (actual_cell == a.goal) {
      return SeedResult{Path{a.goal}, SeedStatus::StubArrived};
    }
    // Plan elapsed but robot hasn't arrived — repair must replan from here.
    return SeedResult{Path{actual_cell}, SeedStatus::StubFar};
  }

  // The "future" portion of prev_path.
  // Validate statically under the current map + footprint.
  if (!tail_statically_valid(prev_path, d, grid, a.footprint)) {
    return SeedResult{Path{actual_cell}, SeedStatus::StubInvalid};
  }

  const Cell expected = prev_path[d];

  // On track: actual matches the expected cell at t=delta.
  if (actual_cell == expected) {
    Path tail(prev_path.begin() + d, prev_path.end());
    return SeedResult{std::move(tail), SeedStatus::OnTrack};
  }

  // Too far: can't splice.
  const std::int32_t offset_cheb = chebyshev(actual_cell, expected);
  if (offset_cheb > params.patch_radius_cells) {
    return SeedResult{Path{actual_cell}, SeedStatus::StubFar};
  }

  // Near-but-off: try to BFS-splice from `actual` to the earliest cell on
  // the tail we can reach. Trying later cells lets us "catch up" to a
  // robot that has fallen slightly behind its schedule.
  //
  // We scan tail indices j = d, d+1, ..., d + patch_radius and pick the
  // first for which a splice of length <= patch_radius_cells exists.
  // This biases toward the earliest tail index, which preserves the
  // schedule best.
  const std::size_t max_j =
      std::min<std::size_t>(L - 1,
                             d + static_cast<std::size_t>(params.patch_radius_cells));
  for (std::size_t j = d; j <= max_j; ++j) {
    const Cell tail_cell = prev_path[j];
    const std::int32_t budget =
        params.patch_radius_cells - static_cast<std::int32_t>(j - d);
    if (budget < 0) break;
    if (!footprint_fits(tail_cell, a.footprint, grid)) continue;

    Path splice = bfs_splice(actual_cell, tail_cell, budget,
                              params.splice_max_expansions,
                              grid, a.footprint);
    if (splice.empty()) continue;

    // Stitch: splice (includes both endpoints) + tail from (j+1) onward.
    Path stitched;
    stitched.reserve(splice.size() + (L - j - 1));
    stitched.insert(stitched.end(), splice.begin(), splice.end());
    for (std::size_t k = j + 1; k < L; ++k) {
      stitched.push_back(prev_path[k]);
    }
    return SeedResult{std::move(stitched), SeedStatus::Spliced};
  }

  // Close to the plan but couldn't find a footprint-valid splice (likely
  // a tight corridor blocked by the robot's footprint). Stub.
  return SeedResult{Path{actual_cell}, SeedStatus::StubFar};
}

// ---------------------------------------------------------------------------
// build_warm_seed
// ---------------------------------------------------------------------------

void build_warm_seed(const std::vector<Agent>& agents,
                     const GridMap& grid,
                     const PreviousPlan& prev,
                     const ActualState& actual,
                     const WarmStartParams& params,
                     Timestep horizon,
                     Solution* out_solution,
                     WarmStartReport* out_report) {
  WarmStartReport rep{};
  rep.status.assign(agents.size(), SeedStatus::Missing);

  out_solution->init(agents, &grid);

  for (const Agent& a : agents) {
    const AgentId id = a.id;

    // Missing odom — leave empty. The repair loop will not touch this
    // agent; the caller decides what to do (warn, drop from plan, etc.).
    if (id >= actual.have_odom.size() || !actual.have_odom[id]) {
      rep.status[id] = SeedStatus::Missing;
      rep.agents_missing += 1;
      continue;
    }

    const Cell actual_cell = actual.current_cells[id];

    // prev_path is indexed by EXTERNAL id in the caller's convention; but
    // here PreviousPlan::paths is indexed by the agent's internal dense
    // id (caller maps external -> internal before filling prev).
    Path empty_path;
    const Path& prev_path =
        (id < prev.paths.size()) ? prev.paths[id] : empty_path;

    SeedResult sr = make_seed_for_agent(a, grid, prev_path,
                                         prev.delta_steps,
                                         actual_cell, params);

    rep.status[id] = sr.status;
    switch (sr.status) {
      case SeedStatus::OnTrack:     rep.agents_on_track     += 1; break;
      case SeedStatus::Spliced:     rep.agents_spliced      += 1; break;
      case SeedStatus::StubFar:     rep.agents_stubbed_far  += 1; break;
      case SeedStatus::StubNew:     rep.agents_new          += 1; break;
      case SeedStatus::StubGoal:    rep.agents_goal_changed += 1; break;
      case SeedStatus::StubInvalid: rep.agents_invalid      += 1; break;
      case SeedStatus::StubArrived: rep.agents_arrived      += 1; break;
      case SeedStatus::Missing:     rep.agents_missing      += 1; break;
    }

    out_solution->set_path(id, std::move(sr.path), horizon);
  }

  rep.seed_collisions = out_solution->num_collisions();
  if (out_report) *out_report = std::move(rep);
}

// ---------------------------------------------------------------------------
// Heuristic decision
// ---------------------------------------------------------------------------

bool should_warm_start(const WarmStartReport& rep,
                       std::size_t num_agents,
                       std::size_t max_collisions_per_agent,
                       double max_stubs_ratio) {
  if (num_agents == 0) return false;

  // Too many collisions in the seed means repair is unlikely to converge
  // in the warm-start's (usually tight) time budget.
  if (rep.seed_collisions > num_agents * max_collisions_per_agent) return false;

  // Only count stubs that the repair must resolve (ignore arrived / missing
  // since those are not active in the repair set).
  const double stubs_ratio =
      static_cast<double>(rep.active_stubs()) /
      static_cast<double>(num_agents);
  if (stubs_ratio > max_stubs_ratio) return false;

  return true;
}

}  // namespace lns2
