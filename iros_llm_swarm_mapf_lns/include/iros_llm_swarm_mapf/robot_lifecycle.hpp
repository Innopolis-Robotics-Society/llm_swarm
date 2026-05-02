// iros_llm_swarm_mapf/robot_lifecycle.hpp
//
// Bundles the per-robot mission-scoped state previously spread across nine
// separate arrays/sets in MapfLns2Node:
//   * arrival latch
//   * last-movement baseline (pos + time)
//   * three "consecutive failure" counters
//   * lives remaining
//   * unplanable membership and bench timestamp
//
// Read access goes through the public fields. The intricate mutation
// clusters (fresh-mission reset, bench-on-stall, revive-after-cooldown)
// are exposed as methods so the callers don't have to keep the field
// invariants straight by hand.
//
// Thread-safety is the caller's responsibility — every public method must
// be called under the same mutex (state_mutex_) that protects the rest of
// the mission state.

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/time.hpp"

namespace lns2_node {

struct RobotLifecycle {
  using Pos = std::pair<double, double>;

  // ---- Per-robot arrays (size N == num_robots; indexed by external id) ---
  std::vector<bool>            arrived;          // latched once, used for one-shot logs
  std::vector<Pos>             last_move_pos;    // last position past the stall threshold
  std::vector<rclcpp::Time>    last_move_time;   // time of that last position
  std::vector<int>             empty_fails;      // consecutive empty-path solves
  std::vector<int>             stall_iters;      // consecutive stalled check_schedule ticks
  std::vector<int>             failed_ticks;     // consecutive STATE_FAILED ticks
  std::vector<int>             lives;            // unplanable events left before permanent

  // ---- Cross-cutting sets keyed by external robot_id --------------------
  std::unordered_set<std::uint32_t>            unplanable;
  std::unordered_map<std::uint32_t, rclcpp::Time> unplanable_since;

  int max_lives = 0;
  int num_robots() const { return static_cast<int>(lives.size()); }

  // ---- Lifecycle clusters -----------------------------------------------

  // One-time setup; called from the node constructor.
  void init(int num_robots_arg, int max_lives_arg);

  // Reset everything that's mission-scoped: counters → 0, lives → max,
  // arrival latches dropped, unplanable bookkeeping cleared.
  void reset_for_new_mission();

  enum class BenchResult { Benched, Permanent, AlreadyUnplanable };

  // Bench a robot whose stall / empty-fail counter has crossed the
  // threshold. Decrements `lives[rid]`; if any are left, marks the robot
  // benched (it will be revived after cooldown), else marks it permanent.
  // Returns AlreadyUnplanable when the robot was already in the set
  // (idempotent — no-op).
  BenchResult bench(std::uint32_t rid, const rclcpp::Time& t);

  // Restore a benched robot to the planning pool: clear its unplanable
  // mark, zero its stall / failure counters, refresh movement baseline.
  // Caller must have already verified rid was benched.
  void revive(std::uint32_t rid, const Pos& current_pos,
              const rclcpp::Time& t);

  // Refresh movement baseline. Called when the robot moves further than
  // stall_move_thresh_m_ since the last sample, and on arrival.
  void mark_movement(std::uint32_t rid, const Pos& pos,
                     const rclcpp::Time& t);
};

}  // namespace lns2_node
