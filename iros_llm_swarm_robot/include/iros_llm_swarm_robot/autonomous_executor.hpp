#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "robot_pose.hpp"

namespace iros_llm_swarm_robot
{

/**
 * AutonomousExecutor
 * ==================
 * Handles MAPF path execution via Nav2's FollowPath action.
 * Implements PBS schedule compliance by splitting the path into chunks
 * at waypoints where the robot must wait for a scheduled arrival time.
 *
 * Lifecycle
 * ---------
 *   activate(path)  — start executing a new path (cancels any in-flight goal)
 *   deactivate()    — cancel in-flight goal and all timers, safe to call anytime
 */
class AutonomousExecutor
{
  using FollowPath  = nav2_msgs::action::FollowPath;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<FollowPath>;

public:
  /**
   * @param node         Parent node — used to create action client and timers.
   * @param ns           Robot namespace e.g. "robot_3".
   * @param frame        Path frame id, typically "map".
   * @param own_pose     Shared robot pose — read at chunk-send time for start point.
   * @param schedule_tol PBS schedule tolerance [s]: hold if ahead by more than this.
   */
  AutonomousExecutor(rclcpp::Node * node,
                     const std::string & ns,
                     const std::string & frame,
                     const RobotPose & own_pose,
                     double schedule_tol);

  // Non-copyable
  AutonomousExecutor(const AutonomousExecutor &) = delete;
  AutonomousExecutor & operator=(const AutonomousExecutor &) = delete;

  /** Start executing path. Cancels any currently running goal first. */
  void activate(nav_msgs::msg::Path::SharedPtr path);

  /** Cancel in-flight goal and stop all timers. Robot stops at current position. */
  void deactivate();

  bool is_active() const { return active_; }

private:
  void send_next_chunk();
  void on_chunk_done(size_t stop_idx, const rclcpp::Time & stop_sched);
  void cancel_goal();

  static void fix_orientations(nav_msgs::msg::Path & path);

  // ---- owned resources ----
  rclcpp::Node *                               node_;
  std::string                                  ns_;
  std::string                                  frame_;
  const RobotPose &                            own_pose_;
  double                                       schedule_tol_;

  rclcpp_action::Client<FollowPath>::SharedPtr ac_;
  GoalHandle::SharedPtr                        current_gh_;

  rclcpp::TimerBase::SharedPtr                 hold_timer_;
  rclcpp::TimerBase::SharedPtr                 start_timer_;

  // ---- execution state ----
  bool                                         active_  = false;
  size_t                                       wp_idx_  = 0;
  nav_msgs::msg::Path::SharedPtr               path_;
};

}  // namespace iros_llm_swarm_robot
