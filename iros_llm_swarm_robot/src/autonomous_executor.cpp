#include "iros_llm_swarm_robot/autonomous_executor.hpp"

#include <cmath>

namespace iros_llm_swarm_robot
{

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

AutonomousExecutor::AutonomousExecutor(rclcpp::Node * node,
                                       const std::string & ns,
                                       const std::string & frame,
                                       const RobotPose & own_pose,
                                       double schedule_tol)
: node_(node),
  ns_(ns),
  frame_(frame),
  own_pose_(own_pose),
  schedule_tol_(schedule_tol)
{
  ac_ = rclcpp_action::create_client<FollowPath>(node_, "/" + ns_ + "/follow_path");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void AutonomousExecutor::activate(nav_msgs::msg::Path::SharedPtr path)
{
  if (!path || path->poses.empty()) return;

  deactivate();   // cancel whatever is running

  RCLCPP_INFO(node_->get_logger(),
    "[%s][autonomous] new path: %zu waypoints", ns_.c_str(), path->poses.size());

  path_    = path;
  wp_idx_  = 0;
  active_  = true;

  if (!own_pose_.valid) {
    // Wait for first odometry before sending
    start_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (!own_pose_.valid) return;
        start_timer_->cancel();
        start_timer_.reset();
        send_next_chunk();
      });
  } else {
    send_next_chunk();
  }
}

void AutonomousExecutor::deactivate()
{
  active_ = false;
  cancel_goal();
  path_.reset();
  wp_idx_ = 0;
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void AutonomousExecutor::send_next_chunk()
{
  if (!active_ || !path_ || wp_idx_ >= path_->poses.size()) {
    if (active_) {
      RCLCPP_INFO(node_->get_logger(), "[%s][autonomous] path complete", ns_.c_str());
    }
    active_ = false;
    path_.reset();
    return;
  }

  if (!ac_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[%s][autonomous] follow_path server not available", ns_.c_str());
    return;
  }

  const rclcpp::Time now_t  = node_->now();
  const auto & poses        = path_->poses;

  // Find the first waypoint we need to hold at (PBS schedule compliance).
  // A waypoint requires a hold if its scheduled time is more than
  // schedule_tol_ seconds in the future.
  size_t       stop_idx   = poses.size() - 1;   // default: go to end
  rclcpp::Time stop_sched {0, 0, RCL_ROS_TIME};

  for (size_t i = wp_idx_; i < poses.size(); ++i) {
    const rclcpp::Time sched(poses[i].header.stamp);
    if ((sched - now_t).seconds() > schedule_tol_) {
      stop_idx   = i;
      stop_sched = sched;
      break;
    }
  }

  // Build sub-path: current position → waypoints [wp_idx_..stop_idx]
  nav_msgs::msg::Path chunk;
  chunk.header.frame_id = frame_;
  chunk.header.stamp    = now_t;

  // Prepend current position so DWB doesn't jerk from a stale start
  geometry_msgs::msg::PoseStamped cur;
  cur.header.frame_id    = frame_;
  cur.header.stamp       = now_t;
  cur.pose.position.x    = own_pose_.x;
  cur.pose.position.y    = own_pose_.y;
  cur.pose.orientation.w = 1.0;
  chunk.poses.push_back(cur);

  for (size_t i = wp_idx_; i <= stop_idx; ++i) {
    chunk.poses.push_back(poses[i]);
  }

  fix_orientations(chunk);

  RCLCPP_INFO(node_->get_logger(),
    "[%s][autonomous] chunk wp %zu..%zu/%zu → (%.2f, %.2f)%s",
    ns_.c_str(), wp_idx_, stop_idx, poses.size() - 1,
    poses[stop_idx].pose.position.x,
    poses[stop_idx].pose.position.y,
    stop_idx < poses.size() - 1 ? "  [hold ahead]" : "");

  FollowPath::Goal goal;
  goal.path = chunk;

  auto opts = rclcpp_action::Client<FollowPath>::SendGoalOptions{};

  opts.goal_response_callback =
    [this](const GoalHandle::SharedPtr & gh) {
      if (!gh) {
        RCLCPP_WARN(node_->get_logger(),
          "[%s][autonomous] Nav2 goal rejected", ns_.c_str());
      } else {
        current_gh_ = gh;
      }
    };

  opts.result_callback =
    [this, stop_idx, stop_sched](const GoalHandle::WrappedResult &) {
      current_gh_.reset();
      on_chunk_done(stop_idx, stop_sched);
    };

  ac_->async_send_goal(goal, opts);
  wp_idx_ = stop_idx + 1;
}

void AutonomousExecutor::on_chunk_done(size_t stop_idx,
                                       const rclcpp::Time & stop_sched)
{
  if (!active_) return;

  if (!path_ || stop_idx >= path_->poses.size() - 1) {
    RCLCPP_INFO(node_->get_logger(), "[%s][autonomous] path complete", ns_.c_str());
    active_ = false;
    path_.reset();
    return;
  }

  const double wait = (stop_sched - node_->now()).seconds();
  if (wait > 0.02) {
    RCLCPP_INFO(node_->get_logger(),
      "[%s][autonomous] holding %.2fs at wp %zu per PBS schedule",
      ns_.c_str(), wait, stop_idx);
    hold_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(wait),
      [this]() {
        hold_timer_->cancel();
        hold_timer_.reset();
        send_next_chunk();
      });
  } else {
    send_next_chunk();
  }
}

void AutonomousExecutor::cancel_goal()
{
  if (hold_timer_)  { hold_timer_->cancel();  hold_timer_.reset();  }
  if (start_timer_) { start_timer_->cancel(); start_timer_.reset(); }
  if (current_gh_) {
    ac_->async_cancel_goal(current_gh_);
    current_gh_.reset();
  }
}

void AutonomousExecutor::fix_orientations(nav_msgs::msg::Path & path)
{
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    if (std::abs(dx) > 1e-4 || std::abs(dy) > 1e-4) {
      const double yaw = std::atan2(dy, dx);
      path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
      path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
    }
  }
  if (path.poses.size() >= 2) {
    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2].pose.orientation;
  }
}

}  // namespace iros_llm_swarm_robot
