/**
 * formation_controller_node
 * =========================
 * Per-follower PD controller that tracks the leader and maintains
 * a fixed offset in the leader's body frame.
 *
 * Subscriptions
 * -------------
 *   /<leader_ns>/odom          (nav_msgs/Odometry)  — leader pose
 *   /<follower_ns>/odom        (nav_msgs/Odometry)  — own pose
 *   /formations/<id>/config    (FormationConfig)    — offset & active flag
 *
 * Publications
 * ------------
 *   /<follower_ns>/cmd_vel     (geometry_msgs/Twist) — direct velocity command
 *
 * Parameters
 * ----------
 *   follower_ns   : string  — e.g. "robot_1"
 *   leader_ns     : string  — e.g. "robot_0"  (overridden by FormationConfig)
 *   formation_id  : string  — subscribe to this formation's config topic
 *   offset_x      : double  — initial offset in leader body frame (+x = forward)
 *   offset_y      : double  — initial offset in leader body frame (+y = left)
 *   kp            : double  — proportional gain  (default 1.2)
 *   kd            : double  — derivative gain    (default 0.3)
 *   max_v         : double  — max linear speed   (default 0.5 m/s)
 *   max_omega     : double  — max angular speed  (default 1.0 rad/s)
 *   control_hz    : double  — control loop rate  (default 20.0 Hz)
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "iros_llm_swarm_interfaces/msg/formation_config.hpp"

using FormationConfig = iros_llm_swarm_interfaces::msg::FormationConfig;

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

static double quat_to_yaw(double x, double y, double z, double w)
{
  // yaw (z-axis rotation)
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

static double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

// ---------------------------------------------------------------------------

class FormationControllerNode : public rclcpp::Node
{
public:
  FormationControllerNode() : Node("formation_controller")
  {
    // ---- parameters -------------------------------------------------------
    declare_parameter("follower_ns",  std::string("robot_1"));
    declare_parameter("leader_ns",    std::string("robot_0"));
    declare_parameter("formation_id", std::string(""));
    declare_parameter("offset_x",     -1.0);
    declare_parameter("offset_y",      0.0);
    declare_parameter("kp",            1.2);
    declare_parameter("kd",            0.3);
    declare_parameter("max_v",         0.5);
    declare_parameter("max_omega",     1.0);
    declare_parameter("control_hz",   20.0);

    follower_ns_  = get_parameter("follower_ns").as_string();
    leader_ns_    = get_parameter("leader_ns").as_string();
    formation_id_ = get_parameter("formation_id").as_string();
    offset_x_     = get_parameter("offset_x").as_double();
    offset_y_     = get_parameter("offset_y").as_double();
    kp_           = get_parameter("kp").as_double();
    kd_           = get_parameter("kd").as_double();
    max_v_        = get_parameter("max_v").as_double();
    max_omega_    = get_parameter("max_omega").as_double();

    const double hz = get_parameter("control_hz").as_double();

    // ---- subscriptions ----------------------------------------------------
    auto latched = rclcpp::QoS(1).transient_local().reliable();

    leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + leader_ns_ + "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_leader_odom(msg); });

    follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + follower_ns_ + "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_follower_odom(msg); });

    if (!formation_id_.empty()) {
      config_sub_ = create_subscription<FormationConfig>(
          "/formations/" + formation_id_ + "/config", latched,
          [this](const FormationConfig::SharedPtr msg) { on_config(msg); });
    }

    // ---- publisher --------------------------------------------------------
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/" + follower_ns_ + "/cmd_vel", 10);

    // ---- control timer ----------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_wall_timer(period, [this]() { control_step(); });

    RCLCPP_INFO(get_logger(),
                "[%s] formation_controller ready | leader=%s offset=(%.2f, %.2f)",
                follower_ns_.c_str(), leader_ns_.c_str(), offset_x_, offset_y_);
  }

private:
  // ------------------------------------------------------------------
  // Callbacks
  // ------------------------------------------------------------------

  void on_leader_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    leader_x_   = msg->pose.pose.position.x;
    leader_y_   = msg->pose.pose.position.y;
    leader_yaw_ = quat_to_yaw(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_leader_ = true;
  }

  void on_follower_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_   = msg->pose.pose.position.x;
    robot_y_   = msg->pose.pose.position.y;
    robot_yaw_ = quat_to_yaw(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_robot_ = true;
  }

  void on_config(const FormationConfig::SharedPtr msg)
  {
    // Update active flag
    active_ = msg->active;

    if (!active_) {
      stop();
      RCLCPP_INFO(get_logger(), "[%s] formation disbanded — stopping", follower_ns_.c_str());
      return;
    }

    // Find our slot in the follower list and update offset
    for (size_t i = 0; i < msg->follower_ns.size(); ++i) {
      if (msg->follower_ns[i] == follower_ns_) {
        if (i < msg->offsets.size()) {
          offset_x_ = msg->offsets[i].x;
          offset_y_ = msg->offsets[i].y;
          RCLCPP_INFO(get_logger(),
                      "[%s] offset updated to (%.2f, %.2f)",
                      follower_ns_.c_str(), offset_x_, offset_y_);
        }
        break;
      }
    }

    // Update leader_ns if config carries a different one
    if (!msg->leader_ns.empty() && msg->leader_ns != leader_ns_) {
      leader_ns_ = msg->leader_ns;
      // Re-subscribe to new leader odom
      leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "/" + leader_ns_ + "/odom", 10,
          [this](const nav_msgs::msg::Odometry::SharedPtr m) { on_leader_odom(m); });
      have_leader_ = false;
      RCLCPP_INFO(get_logger(), "[%s] leader changed to %s",
                  follower_ns_.c_str(), leader_ns_.c_str());
    }
  }

  // ------------------------------------------------------------------
  // Control step (runs at control_hz)
  // ------------------------------------------------------------------

  void control_step()
  {
    if (!active_ || !have_leader_ || !have_robot_) {
      return;
    }

    // --- 1. Compute target position in world frame -----------------------
    //
    //  target = leader_pos + R(leader_yaw) * offset
    //
    const double c = std::cos(leader_yaw_);
    const double s = std::sin(leader_yaw_);
    const double target_x = leader_x_ + c * offset_x_ - s * offset_y_;
    const double target_y = leader_y_ + s * offset_x_ + c * offset_y_;

    // --- 2. Position error in world frame --------------------------------
    const double ex = target_x - robot_x_;
    const double ey = target_y - robot_y_;

    // --- 3. Derivative of error ------------------------------------------
    const double dex = ex - prev_ex_;
    const double dey = ey - prev_ey_;
    prev_ex_ = ex;
    prev_ey_ = ey;

    // --- 4. PD output in world frame -------------------------------------
    const double vx_world = kp_ * ex + kd_ * dex;
    const double vy_world = kp_ * ey + kd_ * dey;

    // --- 5. Project into robot body frame --------------------------------
    const double cr = std::cos(robot_yaw_);
    const double sr = std::sin(robot_yaw_);

    const double v_forward  =  cr * vx_world + sr * vy_world;  // along heading
    const double v_lateral  = -sr * vx_world + cr * vy_world;  // perpendicular

    // For a differential-drive robot we cannot move laterally directly.
    // Use lateral error to generate a yaw correction (omega).
    const double v     = clamp(v_forward, -max_v_,     max_v_);
    const double omega = clamp(v_lateral,  -max_omega_, max_omega_);

    // --- 6. Publish cmd_vel -----------------------------------------------
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = omega;
    cmd_pub_->publish(cmd);
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------

  std::string follower_ns_, leader_ns_, formation_id_;

  double offset_x_, offset_y_;
  double kp_, kd_;
  double max_v_, max_omega_;

  bool active_      = true;   // formation active by default (set via param)
  bool have_leader_ = false;
  bool have_robot_  = false;

  double leader_x_ = 0.0, leader_y_ = 0.0, leader_yaw_ = 0.0;
  double robot_x_  = 0.0, robot_y_  = 0.0, robot_yaw_  = 0.0;

  double prev_ex_ = 0.0, prev_ey_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_sub_;
  rclcpp::Subscription<FormationConfig>::SharedPtr         config_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationControllerNode>());
  rclcpp::shutdown();
  return 0;
}