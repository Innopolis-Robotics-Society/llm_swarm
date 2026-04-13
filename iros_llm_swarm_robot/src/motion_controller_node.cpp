/**
 * motion_controller_node
 * ======================
 * Per-robot orchestrator. Owns all subscriptions that drive mode switching
 * and delegates execution to AutonomousExecutor or FormationExecutor.
 *
 * Mode FSM
 * --------
 * AUTONOMOUS -(formation active, self in follower_ns)-> FORMATION_FOLLOWER
 * FORMATION_FOLLOWER -(formation disbanded / self removed)-> AUTONOMOUS
 *
 * Subscriptions
 * -------------
 *   /<ns>/odom              (nav_msgs/Odometry)       — own pose, always
 *   /<ns>/mapf_path         (nav_msgs/Path)           — MAPF path, always subscribed
 *                                                       but forwarded only in AUTONOMOUS
 *   /formations/config    (FormationsState, latched) — full formation snapshot
 *
 * Parameters
 * ----------
 *   robot_id              : int    — ns = "robot_<id>"
 *   path_frame            : string — MAPF path frame   (default: "map")
 *   schedule_tolerance_sec: double — PBS hold tolerance (default: 0.5)
 *   kp                    : double — PD proportional gain (default: 1.2)
 *   kd                    : double — PD derivative gain   (default: 0.3)
 *   max_v                 : double — max linear speed     (default: 0.5)
 *   max_omega             : double — max angular speed    (default: 1.0)
 *   control_hz            : double — PD loop rate         (default: 20.0)
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "iros_llm_swarm_interfaces/msg/formations_state.hpp"
#include "iros_llm_swarm_interfaces/msg/formation_config.hpp"

#include "iros_llm_swarm_robot/robot_pose.hpp"
#include "iros_llm_swarm_robot/autonomous_executor.hpp"
#include "iros_llm_swarm_robot/formation_executor.hpp"

using FormationsState = iros_llm_swarm_interfaces::msg::FormationsState;
using FormationConfig = iros_llm_swarm_interfaces::msg::FormationConfig;

namespace iros_llm_swarm_robot
{

class MotionControllerNode : public rclcpp::Node
{
public:
  MotionControllerNode() : Node("motion_controller")
  {
    // Parameters
    declare_parameter("robot_id",               0);
    declare_parameter("path_frame",             std::string("map"));
    declare_parameter("schedule_tolerance_sec", 0.5);
    declare_parameter("kp",                     1.2);
    declare_parameter("kd",                     0.3);
    declare_parameter("max_v",                  0.5);
    declare_parameter("max_omega",              1.0);
    declare_parameter("control_hz",            20.0);

    const int robot_id = get_parameter("robot_id").as_int();
    ns_ = "robot_" + std::to_string(robot_id);

    const std::string frame = get_parameter("path_frame").as_string();
    const double tol        = get_parameter("schedule_tolerance_sec").as_double();
    const double kp         = get_parameter("kp").as_double();
    const double kd         = get_parameter("kd").as_double();
    const double max_v      = get_parameter("max_v").as_double();
    const double max_omega  = get_parameter("max_omega").as_double();
    const double hz         = get_parameter("control_hz").as_double();

    // Executors
    autonomous_ = std::make_unique<AutonomousExecutor>(
      this, ns_, frame, own_pose_, tol);

    formation_ = std::make_unique<FormationExecutor>(
      this, ns_, own_pose_, kp, kd, max_v, max_omega, hz);

    // Subscriptions

    // Own odometry — always active, updates shared pose struct
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + ns_ + "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

    // MAPF path — always subscribed, forwarded only in AUTONOMOUS mode
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/" + ns_ + "/mapf_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { on_mapf_path(msg); });

    // Formation registry — latched, single topic carries all formations
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    formation_sub_ = create_subscription<FormationsState>(
      "/formations/config", latched,
      [this](const FormationsState::SharedPtr msg) { on_formations_state(msg); });

    RCLCPP_INFO(get_logger(),
      "[%s] motion_controller ready  mode=AUTONOMOUS", ns_.c_str());
  }

private:
  // Mode FSM

  enum class Mode { AUTONOMOUS, FORMATION_FOLLOWER };

  void enter_autonomous()
  {
    if (mode_ == Mode::AUTONOMOUS) return;
    RCLCPP_INFO(get_logger(), "[%s] → AUTONOMOUS", ns_.c_str());
    mode_ = Mode::AUTONOMOUS;
    formation_->deactivate();
  }

  void enter_formation(const std::string & leader_ns, double ox, double oy)
  {
    if (mode_ == Mode::FORMATION_FOLLOWER) {
      // Already in formation — executor handles the no-op / offset update case
      formation_->activate(leader_ns, ox, oy);
      return;
    }
    RCLCPP_INFO(get_logger(),
      "[%s] → FORMATION_FOLLOWER  leader=%s  offset=(%.2f, %.2f)",
      ns_.c_str(), leader_ns.c_str(), ox, oy);
    mode_ = Mode::FORMATION_FOLLOWER;
    autonomous_->deactivate();
    formation_->activate(leader_ns, ox, oy);
  }


  // Callbacks

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    own_pose_.x   = msg->pose.pose.position.x;
    own_pose_.y   = msg->pose.pose.position.y;
    own_pose_.yaw = quat_to_yaw(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    own_pose_.valid = true;
  }

  void on_mapf_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    if (mode_ == Mode::FORMATION_FOLLOWER) {
      RCLCPP_DEBUG(get_logger(),
        "[%s] FORMATION mode — ignoring MAPF path (%zu wp)",
        ns_.c_str(), msg->poses.size());
      return;
    }

    autonomous_->activate(msg);
  }

  void on_formations_state(const FormationsState::SharedPtr msg)
  {
    // Scan the full snapshot for a formation where we are a follower.
    // At most one formation should claim the same robot.
    for (const auto & f : msg->formations) {
      int my_slot = find_my_slot(f);
      if (my_slot < 0) continue;

      // Found our formation
      if (!f.active) {
        if (active_formation_id_ == f.formation_id) {
          RCLCPP_INFO(get_logger(),
            "[%s] formation '%s' disbanded → AUTONOMOUS",
            ns_.c_str(), f.formation_id.c_str());
          active_formation_id_.clear();
          enter_autonomous();
        }
        return;
      }

      // Active — switch or update
      active_formation_id_ = f.formation_id;
      enter_formation(f.leader_ns,
                      f.offsets[my_slot].x,
                      f.offsets[my_slot].y);
      return;
    }

    // Not found in any formation — return to autonomous if we were following one
    if (!active_formation_id_.empty()) {
      RCLCPP_INFO(get_logger(),
        "[%s] formation '%s' gone from registry → AUTONOMOUS",
        ns_.c_str(), active_formation_id_.c_str());
      active_formation_id_.clear();
      enter_autonomous();
    }
  }


  // Utilities

  int find_my_slot(const FormationConfig & f) const
  {
    for (size_t i = 0; i < f.follower_ns.size(); ++i) {
      if (f.follower_ns[i] == ns_) return static_cast<int>(i);
    }
    return -1;
  }

  static double quat_to_yaw(double x, double y, double z, double w)
  {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }


  // State

  std::string ns_;
  Mode        mode_                = Mode::AUTONOMOUS;
  std::string active_formation_id_;

  // Shared pose — written by on_odom(), read by both executors via const ref
  RobotPose own_pose_;

  std::unique_ptr<AutonomousExecutor> autonomous_;
  std::unique_ptr<FormationExecutor>  formation_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      path_sub_;
  rclcpp::Subscription<FormationsState>::SharedPtr          formation_sub_;
};

}  // namespace iros_llm_swarm_robot

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<iros_llm_swarm_robot::MotionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
