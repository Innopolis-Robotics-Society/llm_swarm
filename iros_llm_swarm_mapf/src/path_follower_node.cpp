/**
 * path_follower
 * ==========
 * Subscriptions
 * -------------
 *   /<ns>/mapf_path        (nav_msgs/Path)         — MAPF planned path (AUTONOMOUS only)
 *   /<ns>/odom             (nav_msgs/Odometry)      — own odometry (both modes)
 *   /<leader_ns>/odom      (nav_msgs/Odometry)      — leader odometry (FORMATION only, dynamic)
 *   /formations/config     (FormationConfig)        — mode switch + offset updates
 *
 * Publications
 * ------------
 *   /<ns>/cmd_vel          (geometry_msgs/Twist)    — FORMATION mode direct control
 *
 * Actions (client)
 * ----------------
 *   /<ns>/follow_path      (nav2_msgs/FollowPath)   — AUTONOMOUS mode Nav2 execution
 *
 * Parameters
 * ----------
 *   robot_id              : int    — robot index; ns = "robot_<id>"
 *   path_frame            : string — frame for MAPF paths (default: "map")
 *   schedule_tolerance_sec: double — PBS schedule hold tolerance (default: 0.5)
 *   kp                    : double — PD proportional gain (default: 1.2)
 *   kd                    : double — PD derivative gain   (default: 0.3)
 *   max_v                 : double — max linear speed [m/s]  (default: 0.5)
 *   max_omega             : double — max angular speed [rad/s] (default: 1.0)
 *   control_hz            : double — formation PD loop rate (default: 20.0)
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "iros_llm_swarm_interfaces/msg/formation_config.hpp"


// Aliases

using FollowPath     = nav2_msgs::action::FollowPath;
using GoalHandle     = rclcpp_action::ClientGoalHandle<FollowPath>;
using FormationConfig = iros_llm_swarm_interfaces::msg::FormationConfig;


// Utilities

static double quat_to_yaw(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

static double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static void fix_orientations(nav_msgs::msg::Path & path)
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


// Node

class PathFollowerNode : public rclcpp::Node
{
public:

  PathFollowerNode() : Node("path_follower")
  {
    declare_parameter("robot_id",               0);
    declare_parameter("path_frame",             std::string("map"));
    declare_parameter("schedule_tolerance_sec", 0.5);
    declare_parameter("kp",                     1.2);
    declare_parameter("kd",                     0.3);
    declare_parameter("max_v",                  0.5);
    declare_parameter("max_omega",              1.0);
    declare_parameter("control_hz",            20.0);

    const int robot_id = get_parameter("robot_id").as_int();
    ns_     = "robot_" + std::to_string(robot_id);
    frame_  = get_parameter("path_frame").as_string();
    tol_    = get_parameter("schedule_tolerance_sec").as_double();
    kp_     = get_parameter("kp").as_double();
    kd_     = get_parameter("kd").as_double();
    max_v_  = get_parameter("max_v").as_double();
    max_omega_ = get_parameter("max_omega").as_double();

    const double hz = get_parameter("control_hz").as_double();
    pd_period_ = std::chrono::duration<double>(1.0 / hz);

    // Own odometry (always active)
    own_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + ns_ + "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_own_odom(msg); });

    // Formation config (always active, single shared topic)
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    formation_sub_ = create_subscription<FormationConfig>(
      "/formations/config", latched,
      [this](const FormationConfig::SharedPtr msg) { on_formation_config(msg); });

    // MAPF path (always subscribed, only processed in AUTONOMOUS)
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/" + ns_ + "/mapf_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { on_mapf_path(msg); });

    // Nav2 action client (used in AUTONOMOUS mode)
    nav2_ac_ = rclcpp_action::create_client<FollowPath>(
      this, "/" + ns_ + "/follow_path");

    //  cmd_vel publisher (used in FORMATION_FOLLOWER mode)
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/" + ns_ + "/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "[%s] path_follower ready  mode=AUTONOMOUS", ns_.c_str());
  }

private:

  enum class Mode { AUTONOMOUS, FORMATION_FOLLOWER };

  void enter_autonomous()
  {
    if (mode_ == Mode::AUTONOMOUS) return;

    RCLCPP_INFO(get_logger(), "[%s] → AUTONOMOUS", ns_.c_str());
    mode_ = Mode::AUTONOMOUS;

    // Stop PD timer
    stop_pd_timer();

    // Zero out cmd_vel so robot doesn't drift
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

    // Drop leader subscription — no longer needed
    leader_odom_sub_.reset();
    leader_ns_.clear();
    have_leader_ = false;
  }

  void enter_formation_follower(const std::string & leader_ns,
                                double offset_x, double offset_y)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] → FORMATION_FOLLOWER  leader=%s  offset=(%.2f, %.2f)",
      ns_.c_str(), leader_ns.c_str(), offset_x, offset_y);

    mode_     = Mode::FORMATION_FOLLOWER;
    offset_x_ = offset_x;
    offset_y_ = offset_y;

    // Cancel any in-flight Nav2 goal
    cancel_nav2();

    // Subscribe to leader odom (re-subscribe if leader changed)
    if (leader_ns != leader_ns_) {
      leader_ns_ = leader_ns;
      have_leader_ = false;
      prev_ex_ = 0.0;
      prev_ey_ = 0.0;

      leader_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + leader_ns_ + "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_leader_odom(msg); });
    }

    // Start PD timer
    start_pd_timer();
  }


  // Callbacks — odometry

  void on_own_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    own_x_   = msg->pose.pose.position.x;
    own_y_   = msg->pose.pose.position.y;
    own_yaw_ = quat_to_yaw(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_own_odom_ = true;
  }

  void on_leader_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    leader_x_   = msg->pose.pose.position.x;
    leader_y_   = msg->pose.pose.position.y;
    leader_yaw_ = quat_to_yaw(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_leader_ = true;
  }


  // Callbacks — formation config

  void on_formation_config(const FormationConfig::SharedPtr msg)
  {
    int my_slot = -1;
    for (size_t i = 0; i < msg->follower_ns.size(); ++i) {
      if (msg->follower_ns[i] == ns_) {
        my_slot = static_cast<int>(i);
        break;
      }
    }

    if (my_slot >= 0) {
      if (!msg->active) {
        // Formation disbanded — go autonomous
        if (active_formation_id_ == msg->formation_id) {
          RCLCPP_INFO(get_logger(),
            "[%s] formation '%s' disbanded — returning to AUTONOMOUS",
            ns_.c_str(), msg->formation_id.c_str());
          active_formation_id_.clear();
          enter_autonomous();
        }
        return;
      }

      // Active formation — enter/update follower mode
      const double ox = msg->offsets[my_slot].x;
      const double oy = msg->offsets[my_slot].y;
      active_formation_id_ = msg->formation_id;

      if (mode_ == Mode::FORMATION_FOLLOWER &&
          leader_ns_ == msg->leader_ns &&
          offset_x_ == ox && offset_y_ == oy)
      {
        // Nothing changed — no-op
        return;
      }

      enter_formation_follower(msg->leader_ns, ox, oy);
      return;
    }

    // If previously followed THIS formation, return to autonomous
    if (active_formation_id_ == msg->formation_id) {
      RCLCPP_INFO(get_logger(),
        "[%s] removed from formation '%s' — returning to AUTONOMOUS",
        ns_.c_str(), msg->formation_id.c_str());
      active_formation_id_.clear();
      enter_autonomous();
    }
    // Otherwise this formation update is irrelevant — ignore
  }


  // AUTONOMOUS mode — MAPF path execution

  void on_mapf_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    if (mode_ == Mode::FORMATION_FOLLOWER) {
      RCLCPP_DEBUG(get_logger(),
        "[%s] in FORMATION mode — ignoring MAPF path (%zu wp)",
        ns_.c_str(), msg->poses.size());
      return;
    }

    start_path_execution(msg);
  }

  void start_path_execution(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "[%s] new MAPF path: %zu waypoints",
                ns_.c_str(), msg->poses.size());

    cancel_nav2();

    active_path_ = msg;
    wp_idx_      = 0;

    if (!have_own_odom_) {
      start_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          if (!have_own_odom_) return;
          start_timer_->cancel();
          start_timer_.reset();
          send_next_chunk();
        });
    } else {
      send_next_chunk();
    }
  }

  void send_next_chunk()
  {
    if (!active_path_ || wp_idx_ >= active_path_->poses.size()) {
      RCLCPP_INFO(get_logger(), "[%s] path complete", ns_.c_str());
      active_path_.reset();
      return;
    }

    if (!nav2_ac_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "[%s] follow_path server not available", ns_.c_str());
      return;
    }

    const rclcpp::Time now_t  = now();
    const auto & poses        = active_path_->poses;

    // Find first waypoint we need to hold at (PBS schedule compliance)
    size_t       stop_idx   = poses.size() - 1;
    rclcpp::Time stop_sched {0, 0, RCL_ROS_TIME};

    for (size_t i = wp_idx_; i < poses.size(); ++i) {
      const rclcpp::Time sched(poses[i].header.stamp);
      if ((sched - now_t).seconds() > tol_) {
        stop_idx   = i;
        stop_sched = sched;
        break;
      }
    }

    // Build sub-path starting at current position
    nav_msgs::msg::Path chunk;
    chunk.header.frame_id = frame_;
    chunk.header.stamp    = now_t;

    geometry_msgs::msg::PoseStamped cur;
    cur.header.frame_id       = frame_;
    cur.header.stamp          = now_t;
    cur.pose.position.x       = own_x_;
    cur.pose.position.y       = own_y_;
    cur.pose.orientation.w    = 1.0;
    chunk.poses.push_back(cur);

    for (size_t i = wp_idx_; i <= stop_idx; ++i) {
      chunk.poses.push_back(poses[i]);
    }

    fix_orientations(chunk);

    RCLCPP_INFO(get_logger(),
      "[%s] sending chunk wp %zu..%zu/%zu → (%.2f, %.2f)%s",
      ns_.c_str(), wp_idx_, stop_idx, poses.size() - 1,
      poses[stop_idx].pose.position.x, poses[stop_idx].pose.position.y,
      stop_idx < poses.size() - 1 ? "  [hold ahead]" : "");

    FollowPath::Goal goal;
    goal.path = chunk;

    auto opts = rclcpp_action::Client<FollowPath>::SendGoalOptions{};

    opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_WARN(get_logger(), "[%s] Nav2 goal rejected", ns_.c_str());
        } else {
          current_gh_ = gh;
        }
      };

    opts.result_callback =
      [this, stop_idx, stop_sched](const GoalHandle::WrappedResult &) {
        current_gh_.reset();
        on_chunk_done(stop_idx, stop_sched);
      };

    nav2_ac_->async_send_goal(goal, opts);
    wp_idx_ = stop_idx + 1;
  }

  void on_chunk_done(size_t stop_idx, const rclcpp::Time & stop_sched)
  {
    if (!active_path_ || stop_idx >= active_path_->poses.size() - 1) {
      RCLCPP_INFO(get_logger(), "[%s] path complete", ns_.c_str());
      active_path_.reset();
      return;
    }

    const double wait = (stop_sched - now()).seconds();
    if (wait > 0.02) {
      RCLCPP_INFO(get_logger(),
        "[%s] holding %.2fs at wp %zu per PBS schedule",
        ns_.c_str(), wait, stop_idx);
      hold_timer_ = create_wall_timer(
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


  // FORMATION_FOLLOWER mode — PD controller

  void start_pd_timer()
  {
    if (pd_timer_) return;   // already running
    pd_timer_ = create_wall_timer(pd_period_,
      [this]() { pd_step(); });
  }

  void stop_pd_timer()
  {
    if (pd_timer_) {
      pd_timer_->cancel();
      pd_timer_.reset();
    }
    prev_ex_ = 0.0;
    prev_ey_ = 0.0;
  }

  void pd_step()
  {
    if (!have_own_odom_ || !have_leader_) return;

    // Target position in world frame:  T = leader_pos + R(leader_yaw) * offset
    const double c = std::cos(leader_yaw_);
    const double s = std::sin(leader_yaw_);
    const double tx = leader_x_ + c * offset_x_ - s * offset_y_;
    const double ty = leader_y_ + s * offset_x_ + c * offset_y_;

    // World-frame error
    const double ex = tx - own_x_;
    const double ey = ty - own_y_;

    // Derivative
    const double dex = ex - prev_ex_;
    const double dey = ey - prev_ey_;
    prev_ex_ = ex;
    prev_ey_ = ey;

    // PD in world frame → project into robot body frame
    const double vx_w = kp_ * ex + kd_ * dex;
    const double vy_w = kp_ * ey + kd_ * dey;

    const double cr = std::cos(own_yaw_);
    const double sr = std::sin(own_yaw_);

    // Forward component → linear velocity
    // Lateral component → angular velocity (steers robot toward target)
    const double v     = clamp( cr * vx_w + sr * vy_w, -max_v_,     max_v_);
    const double omega = clamp(-sr * vx_w + cr * vy_w, -max_omega_, max_omega_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = omega;
    cmd_vel_pub_->publish(cmd);
  }


  // Nav2 cancellation helpers

  void cancel_nav2()
  {
    if (hold_timer_)  { hold_timer_->cancel();  hold_timer_.reset();  }
    if (start_timer_) { start_timer_->cancel(); start_timer_.reset(); }
    if (current_gh_) {
      nav2_ac_->async_cancel_goal(current_gh_);
      current_gh_.reset();
    }
    active_path_.reset();
    wp_idx_ = 0;
  }


  // State

  // --- identity ---
  std::string ns_, frame_;

  // --- mode ---
  Mode        mode_               = Mode::AUTONOMOUS;
  std::string active_formation_id_;           // which formation we're following

  // --- own pose ---
  bool   have_own_odom_ = false;
  double own_x_ = 0.0, own_y_ = 0.0, own_yaw_ = 0.0;

  // --- leader pose (FORMATION mode) ---
  std::string leader_ns_;
  bool   have_leader_ = false;
  double leader_x_ = 0.0, leader_y_ = 0.0, leader_yaw_ = 0.0;

  // --- formation offset & PD gains ---
  double offset_x_ = 0.0, offset_y_ = 0.0;
  double kp_, kd_, max_v_, max_omega_;
  double prev_ex_ = 0.0, prev_ey_ = 0.0;
  std::chrono::duration<double> pd_period_;

  // --- AUTONOMOUS path execution ---
  double tol_;
  size_t wp_idx_ = 0;
  std::shared_ptr<const nav_msgs::msg::Path> active_path_;
  GoalHandle::SharedPtr                      current_gh_;

  // --- subscribers ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  own_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  leader_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      path_sub_;
  rclcpp::Subscription<FormationConfig>::SharedPtr          formation_sub_;

  // --- publisher ---
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_pub_;

  // --- action client ---
  rclcpp_action::Client<FollowPath>::SharedPtr              nav2_ac_;

  // --- timers ---
  rclcpp::TimerBase::SharedPtr pd_timer_;
  rclcpp::TimerBase::SharedPtr hold_timer_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}