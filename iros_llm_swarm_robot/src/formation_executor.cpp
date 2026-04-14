#include "iros_llm_swarm_robot/formation_executor.hpp"

#include <cmath>

namespace iros_llm_swarm_robot
{

// ---------------------------------------------------------------------------
// Helpers (private static)
// ---------------------------------------------------------------------------

double FormationExecutor::quat_to_yaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double FormationExecutor::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

FormationExecutor::FormationExecutor(rclcpp::Node * node,
                                     const std::string & ns,
                                     const RobotPose & own_pose,
                                     double kp,
                                     double kd,
                                     double max_v,
                                     double max_omega,
                                     double control_hz)
: node_(node),
  ns_(ns),
  own_pose_(own_pose),
  kp_(kp),
  kd_(kd),
  max_v_(max_v),
  max_omega_(max_omega),
  pd_period_(std::chrono::duration<double>(1.0 / control_hz))
{
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/" + ns_ + "/cmd_vel", 10);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void FormationExecutor::activate(const std::string & leader_ns,
                                 double ox, double oy)
{
  // If already active with the same leader, just update offset
  if (active_ && leader_ns == leader_ns_) {
    update_offset(ox, oy);
    return;
  }

  // Full (re-)activation
  deactivate();

  leader_ns_ = leader_ns;
  offset_x_  = ox;
  offset_y_  = oy;
  prev_ex_   = 0.0;
  prev_ey_   = 0.0;
  active_    = true;

  RCLCPP_INFO(node_->get_logger(),
    "[%s][formation] activating  leader=%s  offset=(%.2f, %.2f)",
    ns_.c_str(), leader_ns_.c_str(), offset_x_, offset_y_);

  // Subscribe to leader odometry
  leader_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/" + leader_ns_ + "/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      leader_pose_.x   = msg->pose.pose.position.x;
      leader_pose_.y   = msg->pose.pose.position.y;
      leader_pose_.yaw = quat_to_yaw(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      leader_pose_.valid = true;
    });

  // Start PD timer
  pd_timer_ = node_->create_wall_timer(pd_period_, [this]() { pd_step(); });
}

void FormationExecutor::update_offset(double ox, double oy)
{
  if (offset_x_ == ox && offset_y_ == oy) return;

  RCLCPP_INFO(node_->get_logger(),
    "[%s][formation] offset updated (%.2f, %.2f) → (%.2f, %.2f)",
    ns_.c_str(), offset_x_, offset_y_, ox, oy);

  offset_x_ = ox;
  offset_y_ = oy;
  // Reset derivative to avoid a spike from the offset jump
  prev_ex_ = 0.0;
  prev_ey_ = 0.0;
}

void FormationExecutor::deactivate()
{
  if (!active_) return;

  active_ = false;
  stop_timer();
  leader_sub_.reset();
  leader_pose_ = RobotPose{};
  leader_ns_.clear();

  // Zero velocity so robot doesn't coast
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

  RCLCPP_INFO(node_->get_logger(), "[%s][formation] deactivated", ns_.c_str());
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void FormationExecutor::pd_step()
{
  if (!own_pose_.valid || !leader_pose_.valid) return;

  // --- 1. Target position in world frame -----------------------------------
  //   T = leader_pos + R(leader_yaw) * offset
  const double c  = std::cos(leader_pose_.yaw);
  const double s  = std::sin(leader_pose_.yaw);
  const double tx = leader_pose_.x + c * offset_x_ - s * offset_y_;
  const double ty = leader_pose_.y + s * offset_x_ + c * offset_y_;

  // --- 2. World-frame error ------------------------------------------------
  const double ex = tx - own_pose_.x;
  const double ey = ty - own_pose_.y;

  // --- 3. Derivative (finite difference over one control tick) -------------
  const double dex = ex - prev_ex_;
  const double dey = ey - prev_ey_;
  prev_ex_ = ex;
  prev_ey_ = ey;

  // --- 4. PD output in world frame -----------------------------------------
  const double vx_w = kp_ * ex + kd_ * dex;
  const double vy_w = kp_ * ey + kd_ * dey;

  // --- 5. Project into robot body frame ------------------------------------
  //   v_body = R(own_yaw)^T * v_world
  const double cr = std::cos(own_pose_.yaw);
  const double sr = std::sin(own_pose_.yaw);

  // Forward component → linear velocity along heading
  // Lateral component → angular velocity to steer toward target
  const double v     = clamp( cr * vx_w + sr * vy_w, -max_v_,     max_v_);
  const double omega = clamp(-sr * vx_w + cr * vy_w, -max_omega_, max_omega_);

  // --- 6. Publish ----------------------------------------------------------
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = v;
  cmd.angular.z = omega;
  cmd_vel_pub_->publish(cmd);
}

void FormationExecutor::stop_timer()
{
  if (pd_timer_) {
    pd_timer_->cancel();
    pd_timer_.reset();
  }
  prev_ex_ = 0.0;
  prev_ey_ = 0.0;
}

}  // namespace iros_llm_swarm_robot
