#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robot_pose.hpp"

namespace iros_llm_swarm_robot
{

/**
 * FormationExecutor
 * =================
 * Runs a PD control loop that keeps this robot at a fixed offset
 * in the leader's body frame, publishing directly to cmd_vel.
 *
 * The derivative term operates on the discrete position error
 * between consecutive control ticks (dt = 1/control_hz).
 *
 * Lifecycle
 * ---------
 *   activate(leader_ns, ox, oy)  — subscribe to leader odom, start PD timer
 *   update_offset(ox, oy)        — hot-update offset without restarting timer
 *   deactivate()                 — stop timer, publish zero cmd_vel, unsub leader
 */
class FormationExecutor
{
public:
  /**
   * @param node        Parent node — used to create sub/pub/timer.
   * @param ns          Own robot namespace e.g. "robot_1".
   * @param own_pose    Shared own pose (kept current by MotionControllerNode).
   * @param kp          Proportional gain.
   * @param kd          Derivative gain.
   * @param max_v       Max linear speed [m/s].
   * @param max_omega   Max angular speed [rad/s].
   * @param control_hz  PD loop frequency [Hz].
   */
  FormationExecutor(rclcpp::Node * node,
                    const std::string & ns,
                    const RobotPose & own_pose,
                    double kp,
                    double kd,
                    double max_v,
                    double max_omega,
                    double control_hz);

  FormationExecutor(const FormationExecutor &) = delete;
  FormationExecutor & operator=(const FormationExecutor &) = delete;

  /** Start PD loop tracking leader_ns at body-frame offset (ox, oy). */
  void activate(const std::string & leader_ns, double ox, double oy);

  /** Update offset in-place without restarting the timer or re-subscribing. */
  void update_offset(double ox, double oy);

  /** Stop PD timer, zero cmd_vel, drop leader subscription. */
  void deactivate();

  bool is_active() const { return active_; }

private:
  void pd_step();
  void stop_timer();

  static double quat_to_yaw(double x, double y, double z, double w);
  static double clamp(double v, double lo, double hi);

  // ---- owned resources ----
  rclcpp::Node *   node_;
  std::string      ns_;
  const RobotPose & own_pose_;

  double kp_, kd_, max_v_, max_omega_;
  std::chrono::duration<double> pd_period_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  leader_sub_;
  rclcpp::TimerBase::SharedPtr                              pd_timer_;

  // ---- execution state ----
  bool        active_      = false;
  std::string leader_ns_;
  RobotPose   leader_pose_;

  double      offset_x_    = 0.0;
  double      offset_y_    = 0.0;
  double      prev_ex_     = 0.0;
  double      prev_ey_     = 0.0;
};

}  // namespace iros_llm_swarm_robot
