// iros_llm_swarm_mapf/src/robot_lifecycle.cpp

#include "iros_llm_swarm_mapf/robot_lifecycle.hpp"

#include <algorithm>

namespace lns2_node {

void RobotLifecycle::init(int num_robots_arg, int max_lives_arg)
{
  max_lives = max_lives_arg;
  arrived.assign(num_robots_arg, false);
  last_move_pos.assign(num_robots_arg, Pos{0.0, 0.0});
  last_move_time.assign(num_robots_arg, rclcpp::Time{0, 0, RCL_ROS_TIME});
  empty_fails.assign(num_robots_arg, 0);
  stall_iters.assign(num_robots_arg, 0);
  failed_ticks.assign(num_robots_arg, 0);
  lives.assign(num_robots_arg, max_lives_arg);
  unplanable.clear();
  unplanable_since.clear();
}

void RobotLifecycle::reset_for_new_mission()
{
  std::fill(arrived.begin(),       arrived.end(),       false);
  std::fill(empty_fails.begin(),   empty_fails.end(),   0);
  std::fill(stall_iters.begin(),   stall_iters.end(),   0);
  std::fill(failed_ticks.begin(),  failed_ticks.end(),  0);
  std::fill(lives.begin(),         lives.end(),         max_lives);
  unplanable.clear();
  unplanable_since.clear();
}

RobotLifecycle::BenchResult RobotLifecycle::bench(
    std::uint32_t rid, const rclcpp::Time& t)
{
  if (!unplanable.insert(rid).second) {
    return BenchResult::AlreadyUnplanable;
  }
  if (rid < lives.size()) {
    lives[rid] -= 1;
  }
  if (rid < lives.size() && lives[rid] > 0) {
    unplanable_since[rid] = t;
    return BenchResult::Benched;
  }
  return BenchResult::Permanent;
}

void RobotLifecycle::revive(std::uint32_t rid, const Pos& current_pos,
                             const rclcpp::Time& t)
{
  unplanable.erase(rid);
  unplanable_since.erase(rid);
  if (rid < stall_iters.size())  stall_iters[rid]  = 0;
  if (rid < empty_fails.size())  empty_fails[rid]  = 0;
  if (rid < failed_ticks.size()) failed_ticks[rid] = 0;
  if (rid < last_move_pos.size())  last_move_pos[rid]  = current_pos;
  if (rid < last_move_time.size()) last_move_time[rid] = t;
}

void RobotLifecycle::mark_movement(std::uint32_t rid, const Pos& pos,
                                    const rclcpp::Time& t)
{
  if (rid < last_move_pos.size())  last_move_pos[rid]  = pos;
  if (rid < last_move_time.size()) last_move_time[rid] = t;
  if (rid < stall_iters.size())   stall_iters[rid]   = 0;
  if (rid < failed_ticks.size())  failed_ticks[rid]  = 0;
}

}  // namespace lns2_node
