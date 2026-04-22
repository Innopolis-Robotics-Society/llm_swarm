/**
 * path_follower (contract-based, v2)
 * ==================================
 *
 * This version replaces the old chunked-nav_msgs/Path interface with an
 * explicit contract between the MAPF planner and the follower:
 *
 *  * The planner publishes a MAPFPlan: a sequence of `MAPFStep`s
 *    (target_position, hold_sec). Holds are first-class; they're not
 *    encoded implicitly via repeated waypoints + timestamps as before.
 *
 *  * The follower publishes a FollowerStatus continuously: its current
 *    state (IDLE/NAVIGATING/HOLDING/COMPLETE/FAILED), which step of
 *    which plan it's on, hold countdown, and Nav2 failure count.
 *
 * The follower's AUTONOMOUS behaviour is a straight-line state machine:
 *
 *     IDLE  ← new plan →  NAVIGATING(seg)  → seg done + hold > 0 → HOLDING
 *                               │                                      │
 *                        seg done + last step                   hold expires
 *                               │                                      │
 *                               ▼                                      ▼
 *                         PLAN_COMPLETE                        NAVIGATING(next seg)
 *
 * A "segment" is the longest run of steps from current_step_index where
 * all but the final step have hold_sec == 0. Segments are dispatched to
 * Nav2 as a single FollowPath action — no more arbitrary 11-waypoint
 * chunking.
 *
 * FORMATION_FOLLOWER mode (PD-based leader following) is unchanged from
 * the previous implementation; it still uses /formations/config to
 * switch modes and leader odom to compute cmd_vel.
 *
 *
 * Subscriptions
 * -------------
 *   /<ns>/mapf_plan        (iros_llm_swarm_interfaces/MAPFPlan)
 *   /<ns>/odom             (nav_msgs/Odometry)
 *   /<leader_ns>/odom      (nav_msgs/Odometry)  [FORMATION only, dynamic]
 *   /formations/config     (FormationsState)
 *
 * Publications
 * ------------
 *   /<ns>/follower_status  (iros_llm_swarm_interfaces/FollowerStatus)
 *   /<ns>/cmd_vel          (geometry_msgs/Twist)  [FORMATION mode]
 *
 * Actions (client)
 * ----------------
 *   /<ns>/follow_path      (nav2_msgs/FollowPath)
 *
 * Parameters
 * ----------
 *   robot_id              : int    — robot index; ns = "robot_<id>"
 *   path_frame            : string — MAPF plan frame (default: "map")
 *   status_pub_hz         : double — heartbeat rate for FollowerStatus (2.0)
 *   kp, kd, max_v, max_omega, control_hz : PD tuning for FORMATION mode
 */

#include <algorithm>
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

#include "iros_llm_swarm_interfaces/msg/formations_state.hpp"
#include "iros_llm_swarm_interfaces/msg/formation_config.hpp"
#include "iros_llm_swarm_interfaces/msg/mapf_plan.hpp"
#include "iros_llm_swarm_interfaces/msg/mapf_step.hpp"
#include "iros_llm_swarm_interfaces/msg/follower_status.hpp"


// Aliases

using FollowPath      = nav2_msgs::action::FollowPath;
using GoalHandle      = rclcpp_action::ClientGoalHandle<FollowPath>;
using FormationConfig = iros_llm_swarm_interfaces::msg::FormationConfig;
using FormationsState = iros_llm_swarm_interfaces::msg::FormationsState;
using MAPFPlanMsg     = iros_llm_swarm_interfaces::msg::MAPFPlan;
using MAPFStepMsg     = iros_llm_swarm_interfaces::msg::MAPFStep;
using FollowerStatus  = iros_llm_swarm_interfaces::msg::FollowerStatus;


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
}


// Node

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode() : Node("path_follower")
  {
    declare_parameter("robot_id",       0);
    declare_parameter("path_frame",     std::string("map"));
    declare_parameter("status_pub_hz",  2.0);
    declare_parameter("kp",             1.2);
    declare_parameter("kd",             0.3);
    declare_parameter("max_v",          0.5);
    declare_parameter("max_omega",      1.0);
    declare_parameter("control_hz",    20.0);

    const int robot_id = get_parameter("robot_id").as_int();
    ns_      = "robot_" + std::to_string(robot_id);
    frame_   = get_parameter("path_frame").as_string();
    kp_      = get_parameter("kp").as_double();
    kd_      = get_parameter("kd").as_double();
    max_v_   = get_parameter("max_v").as_double();
    max_omega_ = get_parameter("max_omega").as_double();

    const double hz = get_parameter("control_hz").as_double();
    pd_period_ = std::chrono::duration<double>(1.0 / hz);
    const double status_hz = get_parameter("status_pub_hz").as_double();
    const auto   status_period = std::chrono::duration<double>(
        1.0 / std::max(status_hz, 0.1));

    // Own odometry (always active)
    own_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + ns_ + "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_own_odom(msg); });

    // Formation config (always active, single shared latched topic)
    auto latched = rclcpp::QoS(1).transient_local().reliable();
    formation_sub_ = create_subscription<FormationsState>(
      "/formations/config", latched,
      [this](const FormationsState::SharedPtr msg) { on_formations_state(msg); });

    // MAPF plan (always subscribed, only acted on in AUTONOMOUS mode)
    plan_sub_ = create_subscription<MAPFPlanMsg>(
      "/" + ns_ + "/mapf_plan", 10,
      [this](const MAPFPlanMsg::SharedPtr msg) { on_mapf_plan(msg); });

    // Status publisher (always publishing)
    status_pub_ = create_publisher<FollowerStatus>(
      "/" + ns_ + "/follower_status", 10);

    // Heartbeat timer — republish status at a steady rate so MAPF can
    // distinguish "follower silent/dead" from "follower legitimately
    // holding". Also updates hold_remaining_sec countdown.
    status_heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      [this]() { publish_status_heartbeat(); });

    // Nav2 action client
    nav2_ac_ = rclcpp_action::create_client<FollowPath>(
      this, "/" + ns_ + "/follow_path");

    // cmd_vel publisher (FORMATION mode)
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/" + ns_ + "/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "[%s] path_follower ready  mode=AUTONOMOUS", ns_.c_str());
    publish_status();  // first status announcement
  }

private:
  enum class Mode { AUTONOMOUS, FORMATION_FOLLOWER };

  // ---------------------------------------------------------------
  // Mode transitions
  // ---------------------------------------------------------------

  void enter_autonomous()
  {
    if (mode_ == Mode::AUTONOMOUS) return;

    RCLCPP_INFO(get_logger(), "[%s] - AUTONOMOUS", ns_.c_str());
    mode_ = Mode::AUTONOMOUS;

    stop_pd_timer();
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    leader_odom_sub_.reset();
    leader_ns_.clear();
    have_leader_ = false;

    // Drop any active plan — wait for MAPF to publish a fresh one.
    drop_plan();
    publish_status();
  }

  void enter_formation_follower(const std::string & leader_ns,
                                double offset_x, double offset_y)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] - FORMATION_FOLLOWER  leader=%s  offset=(%.2f, %.2f)",
      ns_.c_str(), leader_ns.c_str(), offset_x, offset_y);

    mode_     = Mode::FORMATION_FOLLOWER;
    offset_x_ = offset_x;
    offset_y_ = offset_y;

    cancel_nav2();
    drop_plan();

    if (leader_ns != leader_ns_) {
      leader_ns_   = leader_ns;
      have_leader_ = false;
      prev_ex_ = prev_ey_ = 0.0;
      leader_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + leader_ns_ + "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_leader_odom(msg); });
    }

    start_pd_timer();
    publish_status();
  }

  // ---------------------------------------------------------------
  // Odometry callbacks
  // ---------------------------------------------------------------

  void on_own_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    own_x_   = msg->pose.pose.position.x;
    own_y_   = msg->pose.pose.position.y;
    own_yaw_ = quat_to_yaw(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    have_own_odom_ = true;

    // If we got the very first odom *after* a plan arrived, kick off
    // execution now.
    if (mode_ == Mode::AUTONOMOUS && plan_ && state_ == FollowerStatus::STATE_IDLE
        && plan_id_ != 0 && !plan_->steps.empty()) {
      begin_plan();
    }
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

  // ---------------------------------------------------------------
  // Formation mode switching
  // ---------------------------------------------------------------

  void on_formations_state(const FormationsState::SharedPtr msg)
  {
    bool found_my_formation = false;

    for (const auto & f : msg->formations) {
      int my_slot = -1;
      for (size_t i = 0; i < f.follower_ns.size(); ++i) {
        if (f.follower_ns[i] == ns_) { my_slot = static_cast<int>(i); break; }
      }
      if (my_slot < 0) continue;

      found_my_formation = true;

      if (!f.active) {
        if (active_formation_id_ == f.formation_id) {
          RCLCPP_INFO(get_logger(),
            "[%s] formation '%s' disbanded — returning to AUTONOMOUS",
            ns_.c_str(), f.formation_id.c_str());
          active_formation_id_.clear();
          enter_autonomous();
        }
        return;
      }

      const double ox = f.offsets[my_slot].x;
      const double oy = f.offsets[my_slot].y;
      active_formation_id_ = f.formation_id;

      if (mode_ == Mode::FORMATION_FOLLOWER && leader_ns_ == f.leader_ns &&
          offset_x_ == ox && offset_y_ == oy) {
        return;
      }
      enter_formation_follower(f.leader_ns, ox, oy);
      return;
    }

    if (!found_my_formation && !active_formation_id_.empty()) {
      RCLCPP_INFO(get_logger(),
        "[%s] removed from formation '%s' — returning to AUTONOMOUS",
        ns_.c_str(), active_formation_id_.c_str());
      active_formation_id_.clear();
      enter_autonomous();
    }
  }

  // ---------------------------------------------------------------
  // AUTONOMOUS plan execution — MAPFPlan inbox
  // ---------------------------------------------------------------

  void on_mapf_plan(const MAPFPlanMsg::SharedPtr msg)
  {
    if (mode_ == Mode::FORMATION_FOLLOWER) {
      RCLCPP_DEBUG(get_logger(),
        "[%s] in FORMATION mode — ignoring MAPFPlan (id=%u, %zu steps)",
        ns_.c_str(), msg->plan_id, msg->steps.size());
      return;
    }

    // Empty steps = cancel signal. Drop any in-flight work and go idle.
    if (msg->steps.empty()) {
      RCLCPP_INFO(get_logger(),
        "[%s] MAPFPlan id=%u is empty — cancelling", ns_.c_str(), msg->plan_id);
      cancel_nav2();
      plan_.reset();
      plan_id_       = msg->plan_id;  // record plan_id so status echoes it
      current_step_  = 0;
      nav2_failures_ = 0;
      last_fail_     = FollowerStatus::FAIL_NONE;
      set_state(FollowerStatus::STATE_IDLE);
      return;
    }

    RCLCPP_INFO(get_logger(), "[%s] new MAPFPlan id=%u  %zu steps",
                ns_.c_str(), msg->plan_id, msg->steps.size());

    // Adopt the new plan wholesale — any previous plan and Nav2 goal
    // are cancelled; counters reset.
    cancel_nav2();
    plan_          = msg;
    plan_id_       = msg->plan_id;
    current_step_  = 0;
    nav2_failures_ = 0;
    last_fail_     = FollowerStatus::FAIL_NONE;

    // If odom hasn't arrived yet, we'll start from on_own_odom.
    if (!have_own_odom_) {
      set_state(FollowerStatus::STATE_IDLE);
      return;
    }
    begin_plan();
  }

  void begin_plan()
  {
    if (!plan_ || plan_->steps.empty()) {
      set_state(FollowerStatus::STATE_IDLE);
      return;
    }
    current_step_ = 0;
    start_next_segment();
  }

  // Compute the end of the current segment: inclusive index of the step
  // at which navigation should stop. The boundary is either
  //   (a) a step with a non-zero hold_sec, or
  //   (b) the last step of the plan.
  // Everything from current_step_ up to and including the boundary step
  // is sent to Nav2 in one FollowPath action.
  size_t segment_end_index() const
  {
    if (!plan_ || plan_->steps.empty()) return 0;
    const size_t n = plan_->steps.size();
    for (size_t i = current_step_; i < n; ++i) {
      if (plan_->steps[i].hold_sec > kHoldEps || i + 1 == n) return i;
    }
    return n - 1;  // unreachable — but keeps the compiler happy
  }

  void start_next_segment()
  {
    if (!plan_ || current_step_ >= plan_->steps.size()) {
      set_state(FollowerStatus::STATE_PLAN_COMPLETE);
      return;
    }
    if (!nav2_ac_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(),
        "[%s] follow_path server not available — FAILED", ns_.c_str());
      last_fail_ = FollowerStatus::FAIL_NAV2_SERVER_DOWN;
      set_state(FollowerStatus::STATE_FAILED);
      return;
    }

    // Skip leading steps the robot has effectively already passed.
    //
    // Context: when a plan arrives a few seconds after the replan
    // request (cold replan can take 2-4s), the robot may have driven
    // past the first waypoint in the plan by the time we receive it.
    // If we naively hand Nav2 a path starting at those stale waypoints,
    // Nav2 rotates back to reach them before moving forward — wasting
    // time and often confusing the local planner into "No valid
    // trajectories". Skip any leading steps that are either behind us
    // or very close to our current pose.
    //
    // We advance current_step_ past those stale steps so the follower
    // status correctly reflects progress. If the last step gets
    // skipped (unlikely but possible), the segment is empty and we
    // treat the plan as already complete.
    size_t skipped = 0;
    while (current_step_ <= segment_end_index()) {
      const auto& tgt = plan_->steps[current_step_].target;
      const double dx = tgt.x - own_x_;
      const double dy = tgt.y - own_y_;
      const double dist = std::hypot(dx, dy);

      // Too close: we're basically there. Skip.
      if (dist < kSkipRadiusNear) { ++current_step_; ++skipped; continue; }

      // Far enough: check if the step is "ahead" (in the direction of
      // whatever step follows). If there's no next step, keep it.
      if (current_step_ + 1 >= plan_->steps.size()) break;

      const auto& next_tgt = plan_->steps[current_step_ + 1].target;
      const double fwd_dx = next_tgt.x - tgt.x;
      const double fwd_dy = next_tgt.y - tgt.y;
      const double fwd_len = std::hypot(fwd_dx, fwd_dy);
      if (fwd_len < 1e-3) break;  // hold step: keep it

      // If the vector (robot -> tgt) is opposite to (tgt -> next_tgt),
      // the robot is already past tgt along the plan's direction. Skip.
      const double dot = dx * fwd_dx + dy * fwd_dy;
      if (dot < 0.0) { ++current_step_; ++skipped; continue; }

      break;  // step is ahead and not too close — it's the start.
    }
    if (skipped > 0) {
      RCLCPP_INFO(get_logger(),
        "[%s] skipped %zu leading stale steps (now at step %u)",
        ns_.c_str(), skipped, static_cast<unsigned>(current_step_));
    }

    if (!plan_ || current_step_ >= plan_->steps.size()) {
      set_state(FollowerStatus::STATE_PLAN_COMPLETE);
      return;
    }

    const size_t seg_end = segment_end_index();

    // Build Nav2 path: current odom pose, then each segment step's target.
    nav_msgs::msg::Path nav_path;
    nav_path.header.frame_id = frame_;
    nav_path.header.stamp    = now();

    geometry_msgs::msg::PoseStamped cur;
    cur.header.frame_id    = frame_;
    cur.header.stamp       = nav_path.header.stamp;
    cur.pose.position.x    = own_x_;
    cur.pose.position.y    = own_y_;
    cur.pose.orientation.w = 1.0;
    nav_path.poses.push_back(cur);

    for (size_t i = current_step_; i <= seg_end; ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id    = frame_;
      ps.header.stamp       = nav_path.header.stamp;
      ps.pose.position.x    = plan_->steps[i].target.x;
      ps.pose.position.y    = plan_->steps[i].target.y;
      ps.pose.orientation.w = 1.0;
      nav_path.poses.push_back(ps);
    }
    fix_orientations(nav_path);

    RCLCPP_INFO(get_logger(),
      "[%s] nav segment steps %u..%u/%zu  end=(%.2f, %.2f)%s",
      ns_.c_str(), static_cast<unsigned>(current_step_),
      static_cast<unsigned>(seg_end), plan_->steps.size() - 1,
      plan_->steps[seg_end].target.x, plan_->steps[seg_end].target.y,
      (plan_->steps[seg_end].hold_sec > kHoldEps) ? "  [hold ahead]" : "");

    FollowPath::Goal goal;
    goal.path            = nav_path;
    goal.controller_id   = "FollowPath";
    goal.goal_checker_id = "goal_checker";

    set_state(FollowerStatus::STATE_NAVIGATING);

    auto opts = rclcpp_action::Client<FollowPath>::SendGoalOptions{};
    const uint32_t plan_at_dispatch = plan_id_;
    const size_t   seg_end_at_dispatch = seg_end;

    opts.goal_response_callback =
      [this, plan_at_dispatch](const GoalHandle::SharedPtr & gh) {
        // If a new plan came in between send and response, ignore.
        if (plan_id_ != plan_at_dispatch) return;
        if (!gh) {
          RCLCPP_WARN(get_logger(), "[%s] Nav2 goal rejected", ns_.c_str());
          ++nav2_failures_;
          last_fail_ = FollowerStatus::FAIL_NAV2_REJECTED;
          set_state(FollowerStatus::STATE_FAILED);
          return;
        }
        current_gh_ = gh;
      };

    opts.result_callback =
      [this, plan_at_dispatch, seg_end_at_dispatch]
      (const GoalHandle::WrappedResult & result) {
        if (plan_id_ != plan_at_dispatch) {
          // Stale callback from a cancelled plan. Ignore.
          return;
        }
        current_gh_.reset();
        on_nav2_result(result.code, seg_end_at_dispatch);
      };

    nav2_ac_->async_send_goal(goal, opts);
  }

  void on_nav2_result(rclcpp_action::ResultCode code, size_t seg_end)
  {
    if (code != rclcpp_action::ResultCode::SUCCEEDED) {
      // Any non-success (ABORTED, CANCELED) from Nav2 means the segment
      // didn't complete. Report FAILED and wait for MAPF to replan.
      ++nav2_failures_;
      last_fail_ = FollowerStatus::FAIL_NAV2_ABORTED;
      RCLCPP_WARN(get_logger(),
        "[%s] Nav2 segment aborted (result code=%d, failures=%u) — FAILED",
        ns_.c_str(), static_cast<int>(code), nav2_failures_);
      set_state(FollowerStatus::STATE_FAILED);
      return;
    }

    // Segment succeeded. Advance to the last step of the segment.
    current_step_ = seg_end;

    if (!plan_ || current_step_ >= plan_->steps.size()) {
      set_state(FollowerStatus::STATE_PLAN_COMPLETE);
      return;
    }

    const float hold_sec = plan_->steps[current_step_].hold_sec;
    if (hold_sec > kHoldEps) {
      hold_remaining_sec_ = hold_sec;
      hold_started_at_    = now();
      set_state(FollowerStatus::STATE_HOLDING);
      const auto dur = std::chrono::duration<double>(hold_sec);
      hold_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(dur),
        [this]() { on_hold_expired(); });
      return;
    }

    // No hold — this can only mean this was the last segment
    // (segment_end_index returned the final step with hold_sec == 0).
    set_state(FollowerStatus::STATE_PLAN_COMPLETE);
  }

  void on_hold_expired()
  {
    if (hold_timer_) { hold_timer_->cancel(); hold_timer_.reset(); }
    hold_remaining_sec_ = 0.0f;

    if (!plan_) { set_state(FollowerStatus::STATE_IDLE); return; }

    // Advance past the hold step. If that was the last step, done.
    if (current_step_ + 1 >= plan_->steps.size()) {
      set_state(FollowerStatus::STATE_PLAN_COMPLETE);
      return;
    }
    current_step_ += 1;
    start_next_segment();
  }

  // ---------------------------------------------------------------
  // Nav2 + plan cancellation
  // ---------------------------------------------------------------

  void cancel_nav2()
  {
    if (hold_timer_) { hold_timer_->cancel(); hold_timer_.reset(); }
    if (current_gh_) {
      nav2_ac_->async_cancel_goal(current_gh_);
      current_gh_.reset();
    }
  }

  void drop_plan()
  {
    plan_.reset();
    plan_id_            = 0;
    current_step_       = 0;
    nav2_failures_      = 0;
    last_fail_          = FollowerStatus::FAIL_NONE;
    hold_remaining_sec_ = 0.0f;
    set_state(FollowerStatus::STATE_IDLE);
  }

  // ---------------------------------------------------------------
  // Status publishing
  // ---------------------------------------------------------------

  void set_state(uint8_t new_state)
  {
    state_ = new_state;
    publish_status();
  }

  void publish_status()
  {
    FollowerStatus msg;
    msg.header.stamp       = now();
    msg.header.frame_id    = frame_;
    msg.plan_id            = plan_id_;
    msg.state              = state_;
    msg.current_step_index = static_cast<uint32_t>(current_step_);
    msg.num_steps          = plan_ ? static_cast<uint32_t>(plan_->steps.size()) : 0u;
    msg.nav2_failures      = nav2_failures_;
    msg.last_failure_reason = last_fail_;

    if (state_ == FollowerStatus::STATE_HOLDING) {
      // Compute fresh remaining time from wall clock so heartbeat stays
      // honest even if the user changes hold_timer_ in unusual ways.
      if (hold_remaining_sec_ > 0.0f && hold_started_at_.nanoseconds() > 0) {
        const double elapsed = (now() - hold_started_at_).seconds();
        const double remaining = static_cast<double>(hold_remaining_sec_) - elapsed;
        msg.hold_remaining_sec = static_cast<float>(std::max(0.0, remaining));
      } else {
        msg.hold_remaining_sec = hold_remaining_sec_;
      }
    } else {
      msg.hold_remaining_sec = 0.0f;
    }

    status_pub_->publish(msg);
  }

  void publish_status_heartbeat()
  {
    // This fires at status_pub_hz regardless of state. Keeps MAPF's
    // view of follower state fresh even when nothing has changed.
    publish_status();
  }

  // ---------------------------------------------------------------
  // FORMATION_FOLLOWER PD controller — unchanged from previous version
  // ---------------------------------------------------------------

  void start_pd_timer()
  {
    if (pd_timer_) return;
    pd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(pd_period_),
      [this]() { pd_step(); });
  }

  void stop_pd_timer()
  {
    if (pd_timer_) { pd_timer_->cancel(); pd_timer_.reset(); }
    prev_ex_ = prev_ey_ = 0.0;
  }

  void pd_step()
  {
    if (!have_own_odom_ || !have_leader_) return;

    const double c = std::cos(leader_yaw_);
    const double s = std::sin(leader_yaw_);
    const double tx = leader_x_ + c * offset_x_ - s * offset_y_;
    const double ty = leader_y_ + s * offset_x_ + c * offset_y_;

    const double ex = tx - own_x_;
    const double ey = ty - own_y_;
    const double dex = ex - prev_ex_;
    const double dey = ey - prev_ey_;
    prev_ex_ = ex;
    prev_ey_ = ey;

    const double vx_w = kp_ * ex + kd_ * dex;
    const double vy_w = kp_ * ey + kd_ * dey;

    const double cr = std::cos(own_yaw_);
    const double sr = std::sin(own_yaw_);
    const double v     = clamp( cr * vx_w + sr * vy_w, -max_v_,     max_v_);
    const double omega = clamp(-sr * vx_w + cr * vy_w, -max_omega_, max_omega_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = omega;
    cmd_vel_pub_->publish(cmd);
  }

  // ---------------------------------------------------------------
  // State
  // ---------------------------------------------------------------

  static constexpr float  kHoldEps = 0.01f;  // hold_sec below this counts as 0
  // "Already there" radius for leading-step skip. Anything within this
  // distance of the robot's current pose is treated as already visited
  // and elided from the Nav2 path. 0.3 m is just above grid resolution
  // (0.2 m) so same-cell repeats get coalesced; larger values risk
  // skipping real intermediate waypoints around corners.
  static constexpr double kSkipRadiusNear = 0.3;

  // Identity
  std::string ns_, frame_;

  // Mode
  Mode        mode_ = Mode::AUTONOMOUS;
  std::string active_formation_id_;

  // Own pose
  bool   have_own_odom_ = false;
  double own_x_ = 0.0, own_y_ = 0.0, own_yaw_ = 0.0;

  // Leader pose (FORMATION mode)
  std::string leader_ns_;
  bool        have_leader_ = false;
  double      leader_x_ = 0.0, leader_y_ = 0.0, leader_yaw_ = 0.0;

  // FORMATION PD gains
  double offset_x_ = 0.0, offset_y_ = 0.0;
  double kp_ = 0.0, kd_ = 0.0, max_v_ = 0.0, max_omega_ = 0.0;
  double prev_ex_ = 0.0, prev_ey_ = 0.0;
  std::chrono::duration<double> pd_period_;

  // AUTONOMOUS plan execution
  MAPFPlanMsg::SharedPtr plan_;
  uint32_t plan_id_       = 0;
  size_t   current_step_  = 0;
  uint8_t  state_         = FollowerStatus::STATE_IDLE;
  uint32_t nav2_failures_ = 0;
  uint8_t  last_fail_     = FollowerStatus::FAIL_NONE;
  float    hold_remaining_sec_ = 0.0f;
  rclcpp::Time hold_started_at_{0, 0, RCL_ROS_TIME};

  GoalHandle::SharedPtr current_gh_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr own_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_odom_sub_;
  rclcpp::Subscription<MAPFPlanMsg>::SharedPtr             plan_sub_;
  rclcpp::Subscription<FormationsState>::SharedPtr         formation_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;
  rclcpp::Publisher<FollowerStatus>::SharedPtr             status_pub_;

  // Action client
  rclcpp_action::Client<FollowPath>::SharedPtr             nav2_ac_;

  // Timers
  rclcpp::TimerBase::SharedPtr pd_timer_;
  rclcpp::TimerBase::SharedPtr hold_timer_;
  rclcpp::TimerBase::SharedPtr status_heartbeat_timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}