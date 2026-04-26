// mapf_lns2_node.cpp
//
// ROS 2 action server wrapping the LNS2 MAPF solver. API-compatible with
// the PBS-based mapf_planner. Now fully supports the updated SetGoals.action
// with rich feedback (robots_arrived, robots_active, robots_deviated, 
// replans_done, robot_stall, info, warning) and result fields.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "iros_llm_swarm_interfaces/action/set_goals.hpp"
#include "iros_llm_swarm_interfaces/msg/mapf_plan.hpp"
#include "iros_llm_swarm_interfaces/msg/mapf_step.hpp"
#include "iros_llm_swarm_interfaces/msg/follower_status.hpp"

#include "iros_llm_swarm_mapf/lns2/lns2_solver.hpp"
#include "iros_llm_swarm_mapf/lns2/warm_start.hpp"

using namespace std::chrono_literals;
using SetGoalsAction = iros_llm_swarm_interfaces::action::SetGoals;
using GoalHandle     = rclcpp_action::ServerGoalHandle<SetGoalsAction>;
using MAPFPlanMsg    = iros_llm_swarm_interfaces::msg::MAPFPlan;
using MAPFStepMsg    = iros_llm_swarm_interfaces::msg::MAPFStep;
using FollowerStatus = iros_llm_swarm_interfaces::msg::FollowerStatus;

// ---------------------------------------------------------------------------
// World <-> grid conversion
// ---------------------------------------------------------------------------

static lns2::Cell world_to_cell(double wx, double wy,
                                 double origin_x, double origin_y,
                                 double resolution,
                                 std::size_t rows, std::size_t cols)
{
  const double cx = (wx - origin_x) / resolution;
  const double cy = (wy - origin_y) / resolution;
  const int col = std::max(0, std::min(static_cast<int>(cols) - 1,
                                         static_cast<int>(std::floor(cx))));
  const int row = std::max(0, std::min(static_cast<int>(rows) - 1,
                                         static_cast<int>(std::floor(cy))));
  return {row, col};
}

static void cell_to_world(const lns2::Cell& c,
                          double origin_x, double origin_y,
                          double resolution,
                          double& wx, double& wy)
{
  wx = origin_x + (c.col + 0.5) * resolution;
  wy = origin_y + (c.row + 0.5) * resolution;
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

class MapfLns2Node : public rclcpp::Node {
 public:
  MapfLns2Node() : Node("mapf_lns2")
  {
    // ---- parameters -----------------------------------------------------
    declare_parameter("num_robots",              20);
    declare_parameter("time_step_sec",           0.5);
    declare_parameter("map_topic",               std::string("/map"));
    declare_parameter("grid_resolution",         0.2);
    declare_parameter("default_robot_radius",    0.22);
    declare_parameter("inflation_radius",        0.0);
    declare_parameter("max_speed",               0.5);
    declare_parameter("goal_reached_m",          0.5);
    // LNS2 params
    declare_parameter("collision_penalty",       10000);
    declare_parameter("horizon_steps",           300);
    declare_parameter("max_astar_expansions",    200000);
    declare_parameter("neighborhood_size",       8);
    declare_parameter("time_budget_ms",          500);
    declare_parameter("plateau_limit",           200);
    declare_parameter("segment_size",            50);
    declare_parameter("alns_reaction",           0.1);
    declare_parameter("diagonal_moves",          false);
    // Replan / warm start
    declare_parameter("replan_check_hz",         2.0);
    declare_parameter("replan_threshold_m",      2.0);
    declare_parameter("replan_cooldown_sec",     2.0);
    declare_parameter("replan_time_budget_ms",   150);
    declare_parameter("warm_start_enabled",      true);
    declare_parameter("warm_patch_radius_cells", 10);
    // Stall detection: if a robot doesn't move at least `stall_move_thresh_m`
    // over `stall_timeout_sec` and hasn't arrived at its goal, force a replan.
    // This catches the case where ros_path's expected position equals the
    // robot's current position (e.g. robot reached the end of its plan but
    // not its goal), so the deviation check would never trigger.
    declare_parameter("stall_timeout_sec",       4.0);
    declare_parameter("stall_move_thresh_m",     0.15);
    declare_parameter("progress_log_interval_sec", 3.0);
    // Soft unplanable: robots are not marked unplanable on first failure.
    // The previous hard-marking caused cascades where one failed solve for
    // robot A -> block A's cell -> now B can't plan -> block B -> etc, even
    // when A just needed a few seconds for its neighbours to move out.
    //   empty_fails_before_unplanable: consecutive solve() calls that leave
    //     this robot with an empty path before we give up on it.
    //   stall_iters_before_unplanable: consecutive check_schedule ticks
    //     reporting this robot as stalled before we give up on it. With
    //     replan_check_hz=2 and stall_timeout_sec=4, one tick ~= 0.5 s of
    //     extra stall time past the initial 4 s, so 10 = 5 extra seconds.
    declare_parameter("empty_fails_before_unplanable", 3);
    declare_parameter("stall_iters_before_unplanable", 10);

    // "Lives" system: instead of permanently benching a robot that hits
    // the unplanable threshold, give it a retry after a cooldown. A
    // robot often gets stuck transiently because a neighbour happens
    // to be in the way; if we wait a few seconds the neighbour moves
    // on and the robot can succeed. We burn a life per unplanable
    // event; when lives hit zero the robot is permanently skipped.
    //   max_lives: total unplanable events a robot can survive. Set
    //     to 1 to recover the old "permanent-on-first-fail" behaviour.
    //   unplanable_retry_delay_sec: how long a robot stays benched
    //     before being put back in the planning pool.
    declare_parameter("max_lives", 3);
    declare_parameter("unplanable_retry_delay_sec", 8.0);
    declare_parameter("publish_debug_grid",      true); 

    num_robots_            = get_parameter("num_robots").as_int();
    time_step_sec_         = get_parameter("time_step_sec").as_double();
    grid_resolution_       = get_parameter("grid_resolution").as_double();
    default_robot_radius_  = get_parameter("default_robot_radius").as_double();
    inflation_radius_      = get_parameter("inflation_radius").as_double();
    max_speed_             = get_parameter("max_speed").as_double();
    goal_reached_m_        = get_parameter("goal_reached_m").as_double();
    collision_penalty_     = get_parameter("collision_penalty").as_int();
    horizon_steps_         = get_parameter("horizon_steps").as_int();
    max_astar_expansions_  = get_parameter("max_astar_expansions").as_int();
    neighborhood_size_     = get_parameter("neighborhood_size").as_int();
    time_budget_ms_        = get_parameter("time_budget_ms").as_int();
    plateau_limit_         = get_parameter("plateau_limit").as_int();
    segment_size_          = get_parameter("segment_size").as_int();
    alns_reaction_         = get_parameter("alns_reaction").as_double();
    diagonal_moves_        = get_parameter("diagonal_moves").as_bool();
    replan_check_hz_       = get_parameter("replan_check_hz").as_double();
    replan_threshold_m_    = get_parameter("replan_threshold_m").as_double();
    replan_cooldown_sec_   = get_parameter("replan_cooldown_sec").as_double();
    replan_time_budget_ms_ = get_parameter("replan_time_budget_ms").as_int();
    warm_start_enabled_    = get_parameter("warm_start_enabled").as_bool();
    warm_patch_radius_     = get_parameter("warm_patch_radius_cells").as_int();
    stall_timeout_sec_     = get_parameter("stall_timeout_sec").as_double();
    stall_move_thresh_m_   = get_parameter("stall_move_thresh_m").as_double();
    progress_log_interval_sec_ =
        get_parameter("progress_log_interval_sec").as_double();
    empty_fails_before_unplanable_ =
        get_parameter("empty_fails_before_unplanable").as_int();
    stall_iters_before_unplanable_ =
        get_parameter("stall_iters_before_unplanable").as_int();
    max_lives_ =
        get_parameter("max_lives").as_int();
    unplanable_retry_delay_sec_ =
        get_parameter("unplanable_retry_delay_sec").as_double();
    publish_debug_grid_ = get_parameter("publish_debug_grid").as_bool();

    // ---- state ----------------------------------------------------------
    current_positions_.resize(num_robots_, {0.0, 0.0});
    have_odom_.assign(num_robots_, false);
    footprint_radii_.assign(num_robots_, 0.0);
    prev_grid_paths_.assign(num_robots_, lns2::Path{});
    robot_arrived_.assign(num_robots_, false);
    last_movement_pos_.assign(num_robots_, {0.0, 0.0});
    last_movement_time_.assign(num_robots_, rclcpp::Time{0, 0, RCL_ROS_TIME});
    consecutive_empty_fails_.assign(num_robots_, 0);
    consecutive_stall_iters_.assign(num_robots_, 0);
    lives_remaining_.assign(num_robots_, max_lives_);

    // ---- map subscription (transient local) -----------------------------
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), map_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          on_map(msg);
        });

    // ---- per-robot odom + footprint -------------------------------------
    odom_subs_.resize(num_robots_);
    footprint_subs_.resize(num_robots_);
    status_subs_.resize(num_robots_);
    plan_pubs_.resize(num_robots_);
    last_status_.assign(num_robots_, FollowerStatus{});
    last_status_time_.assign(num_robots_, rclcpp::Time{0, 0, RCL_ROS_TIME});
    for (int i = 0; i < num_robots_; ++i) {
      odom_subs_[i] = create_subscription<nav_msgs::msg::Odometry>(
          "/robot_" + std::to_string(i) + "/odom", 10,
          [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            current_positions_[i] = {msg->pose.pose.position.x,
                                      msg->pose.pose.position.y};
            have_odom_[i] = true;
          });

      footprint_subs_[i] = create_subscription<geometry_msgs::msg::PolygonStamped>(
          "/robot_" + std::to_string(i) +
              "/local_costmap/published_footprint",
          rclcpp::QoS(1),
          [this, i](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
            if (msg->polygon.points.empty()) return;
            double cx = 0, cy = 0;
            for (const auto& p : msg->polygon.points) { cx += p.x; cy += p.y; }
            const double n = static_cast<double>(msg->polygon.points.size());
            cx /= n; cy /= n;
            double max_r = 0;
            for (const auto& p : msg->polygon.points) {
              const double r = std::hypot(p.x - cx, p.y - cy);
              if (r > max_r) max_r = r;
            }
            std::lock_guard<std::mutex> lk(state_mutex_);
            footprint_radii_[i] = max_r;
          });

      // Follower status: the follower publishes its state (IDLE /
      // NAVIGATING / HOLDING / PLAN_COMPLETE / FAILED) plus the plan_id
      // it's currently executing. We cache the most recent value so
      // check_schedule() can make decisions without heuristics.
      status_subs_[i] = create_subscription<FollowerStatus>(
          "/robot_" + std::to_string(i) + "/follower_status", 10,
          [this, i](const FollowerStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            last_status_[i] = *msg;
            last_status_time_[i] = now();
          });

      plan_pubs_[i] = create_publisher<MAPFPlanMsg>(
          "/robot_" + std::to_string(i) + "/mapf_plan", 10);
    }

    // ---- action server --------------------------------------------------
    plan_action_server_ = rclcpp_action::create_server<SetGoalsAction>(
        this, "/swarm/set_goals",
        [this](const rclcpp_action::GoalUUID&,
                std::shared_ptr<const SetGoalsAction::Goal> goal) {
          return handle_goal(goal);
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          return handle_cancel(gh);
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          std::thread([this, gh]() { execute_goal(gh); }).detach();
        });

    RCLCPP_INFO(get_logger(),
                "mapf_lns2 ready: %d robots, time_step=%.2fs, "
                "grid_res=%.2fm, default_radius=%.2fm, warm_start=%d",
                num_robots_, time_step_sec_, grid_resolution_,
                default_robot_radius_, warm_start_enabled_ ? 1 : 0);

    // ---- debug grid publisher -------------------------------------------
    if (publish_debug_grid_) {
      debug_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/mapf_grid", rclcpp::QoS(1).transient_local().reliable());
    }
  }

 private:
  // --------------------------------------------------------------------
  // Rich feedback helper
  // --------------------------------------------------------------------
  void publish_rich_feedback(const std::shared_ptr<GoalHandle>& gh,
                             const std::string& status,
                             std::size_t arrived = 0,
                             std::size_t active = 0,
                             std::size_t deviated = 0,
                             std::size_t stalled = 0,
                             const std::string& info = "",
                             const std::string& warning = "")
  {
    if (!gh) return;
    auto fb = std::make_shared<SetGoalsAction::Feedback>();
    fb->status          = status;
    fb->elapsed_ms      = static_cast<uint32_t>((now() - mission_start_time_).seconds() * 1000.0);
    fb->robots_arrived  = static_cast<uint32_t>(arrived);
    fb->robots_active   = static_cast<uint32_t>(active);
    fb->robots_deviated = static_cast<uint32_t>(deviated);
    fb->replans_done    = total_replans_;
    fb->robot_stall    = static_cast<uint32_t>(stalled);
    fb->info            = info;
    fb->warning         = warning;
    gh->publish_feedback(fb);
  }

  // --------------------------------------------------------------------
  // Debug grid publisher
  // --------------------------------------------------------------------
  void publish_debug_grid(const lns2::GridMap& g)
  {
    if (!publish_debug_grid_ || !debug_grid_pub_ || !map_ready_) return;

    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = now();
    msg->header.frame_id = "map";

    msg->info.resolution = map_resolution_;
    msg->info.width      = static_cast<uint32_t>(g.cols);
    msg->info.height     = static_cast<uint32_t>(g.rows);
    msg->info.origin.position.x = map_origin_x_;
    msg->info.origin.position.y = map_origin_y_;
    msg->info.origin.orientation.w = 1.0;

    msg->data.resize(g.rows * g.cols);
    for (std::size_t i = 0; i < g.blocked.size(); ++i) {
      msg->data[i] = g.blocked[i] ? 100 : 0;   // 100 = occupied, 0 = free
    }

    debug_grid_pub_->publish(std::move(msg));
  }

  void publish_debug_grid() { publish_debug_grid(grid_); }

  // --------------------------------------------------------------------
  // Map callback
  // --------------------------------------------------------------------
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const double src_res = msg->info.resolution;
    const int ratio = std::max(1, static_cast<int>(
        std::round(grid_resolution_ / src_res)));
    const double actual_res = src_res * ratio;
    const int src_w = static_cast<int>(msg->info.width);
    const int src_h = static_cast<int>(msg->info.height);
    const int dst_w = (src_w + ratio - 1) / ratio;
    const int dst_h = (src_h + ratio - 1) / ratio;

    RCLCPP_INFO_ONCE(get_logger(),
        "Map received: %dx%d @ %.3fm -> LNS2 grid: %dx%d @ %.3fm (ratio=%d)",
        src_w, src_h, src_res, dst_w, dst_h, actual_res, ratio);

    grid_.rows = static_cast<std::size_t>(dst_h);
    grid_.cols = static_cast<std::size_t>(dst_w);
    grid_.blocked.assign(grid_.rows * grid_.cols, 0);

    for (int sr = 0; sr < src_h; ++sr) {
      for (int sc = 0; sc < src_w; ++sc) {
        const int8_t v = msg->data[static_cast<std::size_t>(sr * src_w + sc)];
        if (v > 50 || v < 0) {
          grid_.blocked[(sr / ratio) * dst_w + (sc / ratio)] = 1;
        }
      }
    }
    map_origin_x_  = msg->info.origin.position.x;
    map_origin_y_  = msg->info.origin.position.y;
    map_resolution_ = actual_res;
    map_ready_ = true;
    publish_debug_grid();
  }

  // --------------------------------------------------------------------
  // Action callbacks
  // --------------------------------------------------------------------
  rclcpp_action::GoalResponse handle_goal(
      std::shared_ptr<const SetGoalsAction::Goal> goal)
  {
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "Goal rejected: map not ready");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->robot_ids.size() != goal->goals.size() ||
        goal->robot_ids.empty()) {
      RCLCPP_WARN(get_logger(), "Goal rejected: invalid robot_ids/goals");
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!is_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Goal rejected: mission in progress");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Fresh mission — forget agents marked unplanable by the previous one.
    // A new goal with a reachable target should give a previously-stuck
    // robot another chance.
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (!unplanable_robots_.empty()) {
        RCLCPP_INFO(get_logger(),
            "New goal accepted — clearing %zu unplanable robots from prior mission",
            unplanable_robots_.size());
        unplanable_robots_.clear();
      }
      // Reset per-robot arrival flags so we emit "arrived" logs again.
      std::fill(robot_arrived_.begin(), robot_arrived_.end(), false);
      std::fill(consecutive_empty_fails_.begin(),
                consecutive_empty_fails_.end(), 0);
      std::fill(consecutive_stall_iters_.begin(),
                consecutive_stall_iters_.end(), 0);
      std::fill(lives_remaining_.begin(), lives_remaining_.end(), max_lives_);
      unplanable_since_.clear();
      last_progress_log_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};

      mission_start_time_ = now();
      total_replans_ = 0;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle>)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    stop_monitoring();
    publish_stop_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // --------------------------------------------------------------------
  // Initial plan execution
  // --------------------------------------------------------------------
  void execute_goal(const std::shared_ptr<GoalHandle> gh)
  {
    const auto& req = gh->get_goal();
    auto result = std::make_shared<SetGoalsAction::Result>();

    // Snapshot state briefly
    lns2::GridMap snap_grid;
    double snap_ox, snap_oy, snap_res;
    std::vector<std::pair<double, double>> snap_pos;
    std::vector<bool> snap_have_odom;
    std::vector<double> snap_fp_radii;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      stop_monitoring();
      snap_grid       = grid_;
      snap_ox         = map_origin_x_;
      snap_oy         = map_origin_y_;
      snap_res        = map_resolution_;
      snap_pos        = current_positions_;
      snap_have_odom  = have_odom_;
      snap_fp_radii   = footprint_radii_;
    }

    publish_rich_feedback(gh, "validating");

    // Build agent list with warnings
    // IMPORTANT: lns2::Agent::id must be a DENSE internal index
    // (0..agents.size()-1). The external robot_id is kept in plan_ids_ext.
    std::vector<lns2::Agent> agents;
    std::vector<uint32_t> plan_ids_ext;      // external robot_id per agent
    std::vector<geometry_msgs::msg::Point> plan_goals_world;
    std::string validation_warnings;

    for (std::size_t i = 0; i < req->robot_ids.size(); ++i) {
      const uint32_t rid = req->robot_ids[i];

      if (rid >= static_cast<uint32_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "robot id %u >= num_robots (%d), skip", rid, num_robots_);
        if (!validation_warnings.empty()) validation_warnings += "; ";
        validation_warnings += "robot id " + std::to_string(rid) + " >= num_robots";
        continue;
      }
      if (!snap_have_odom[rid]) {
        RCLCPP_WARN(get_logger(), "no odom for robot_%u, skip", rid);
        if (!validation_warnings.empty()) validation_warnings += "; ";
        validation_warnings += "no odom for robot_" + std::to_string(rid);
        continue;
      }
      // Previously-marked unplanable — solver already established that no
      // collision-free path exists for this robot under current conditions.
      // Keep skipping until a fresh /swarm/set_goals action resets the set.
      if (unplanable_robots_.count(rid)) {
        RCLCPP_WARN(get_logger(), "robot_%u was marked unplanable earlier, skip", rid);
        if (!validation_warnings.empty()) validation_warnings += "; ";
        validation_warnings += "robot_" + std::to_string(rid) + " unplanable";
        continue;
      }
      lns2::Agent a;
      a.id = static_cast<lns2::AgentId>(agents.size());  // dense internal id
      const auto& [sx, sy] = snap_pos[rid];
      a.start = world_to_cell(sx, sy, snap_ox, snap_oy, snap_res,
                               snap_grid.rows, snap_grid.cols);
      a.goal  = world_to_cell(req->goals[i].x, req->goals[i].y,
                               snap_ox, snap_oy, snap_res,
                               snap_grid.rows, snap_grid.cols);
      const double radius = snap_fp_radii[rid] > 0 ? snap_fp_radii[rid]
                                                    : default_robot_radius_;
      a.footprint = lns2::FootprintModel::from_radius(
          radius + inflation_radius_, snap_res);
      // Escape fallback: if the robot's current odom cell or its goal
      // cell is blocked (e.g. robot standing close to a wall on the
      // downsampled grid), re-home start/goal to the nearest free cell.
      // See find_nearest_free_cell's comment for full rationale.
      if (snap_grid.is_blocked(a.start)) {
        const lns2::Cell rescued = find_nearest_free_cell(
            snap_grid, a.start, a.footprint);
        if (snap_grid.is_blocked(rescued)) {
          RCLCPP_WARN(get_logger(),
              "robot_%u start grid(%d,%d) is blocked, no free cell within "
              "escape radius — skip this mission",
              rid, a.start.row, a.start.col);
          continue;
        }
        RCLCPP_INFO(get_logger(),
            "robot_%u start grid(%d,%d) was blocked, using nearest free "
            "cell grid(%d,%d) as virtual start",
            rid, a.start.row, a.start.col, rescued.row, rescued.col);
        a.start = rescued;
      }
      if (snap_grid.is_blocked(a.goal)) {
        const lns2::Cell rescued = find_nearest_free_cell(
            snap_grid, a.goal, a.footprint);
        if (snap_grid.is_blocked(rescued)) {
          RCLCPP_WARN(get_logger(),
              "robot_%u goal grid(%d,%d) is blocked, no free cell within "
              "escape radius — skip",
              rid, a.goal.row, a.goal.col);
          continue;
        }
        RCLCPP_INFO(get_logger(),
            "robot_%u goal grid(%d,%d) was blocked, using nearest free "
            "cell grid(%d,%d) as virtual goal",
            rid, a.goal.row, a.goal.col, rescued.row, rescued.col);
        a.goal = rescued;
      }
      if (a.start.row == a.goal.row && a.start.col == a.goal.col) {
        RCLCPP_WARN(get_logger(),
            "robot_%u start==goal at grid(%d,%d), skip (static obstacle)",
            rid, a.start.row, a.start.col);
        continue;
      }
      // Static reachability: is there ANY footprint-valid path from start
      // to goal on the static map alone? If not, the LNS2 solver will fail
      // silently on this agent (empty path, claimed as success) and we'll
      // be stuck in a replan loop forever. Mark unplanable up front.
      if (!static_path_exists(snap_grid, a.start, a.goal, a.footprint)) {
        RCLCPP_ERROR(get_logger(),
            "robot_%u: no static path from start grid(%d,%d) to goal grid(%d,%d) "
            "exists on the current map (footprint won't fit through). "
            "Marking unplanable.",
            rid, a.start.row, a.start.col, a.goal.row, a.goal.col);
        {
          std::lock_guard<std::mutex> lk(state_mutex_);
          unplanable_robots_.insert(rid);
        }
        continue;
      }
      agents.push_back(std::move(a));
      plan_ids_ext.push_back(rid);
      plan_goals_world.push_back(req->goals[i]);
    }

    //  Send warnings once
    if (!validation_warnings.empty()) {
      publish_rich_feedback(gh, "validating", 0, 0, 0, 0, "", validation_warnings);
    }

    if (agents.empty()) {
      result->success    = false;
      result->message    = "No valid agents";
      result->error_code = SetGoalsAction::Result::NO_VALID_AGENTS;
      result->total_replans = total_replans_;
      result->total_execution_sec = (now() - mission_start_time_).seconds();
      is_active_ = false;
      gh->succeed(result);
      return;
    }

    if (gh->is_canceling()) {
      result->success    = false;
      result->message    = "Cancelled before planning";
      result->error_code = SetGoalsAction::Result::CANCELLED;
      is_active_ = false;
      gh->canceled(result);
      return;
    }

    // Log which robots are planned vs skipped
    {
      std::string planned_str, skipped_str;
      std::unordered_set<uint32_t> planned_set(plan_ids_ext.begin(),
                                                plan_ids_ext.end());
      for (std::size_t i = 0; i < req->robot_ids.size(); ++i) {
        const uint32_t rid = req->robot_ids[i];
        if (planned_set.count(rid)) {
          planned_str += " " + std::to_string(rid);
        } else {
          skipped_str += " " + std::to_string(rid);
        }
      }
      RCLCPP_INFO(get_logger(), "Planned robots:%s", planned_str.c_str());
      if (!skipped_str.empty()) {
        RCLCPP_WARN(get_logger(), "Skipped robots:%s", skipped_str.c_str());
      }
    }

    // Block cells of robots not in the plan (physically present obstacles)
    block_skipped_robots(snap_grid, plan_ids_ext, snap_pos, snap_have_odom,
                          snap_fp_radii, snap_ox, snap_oy, snap_res);

    // Run solver (cold)
    publish_rich_feedback(gh, "planning");

    const auto t0 = std::chrono::steady_clock::now();
    lns2::LNS2Params params = make_params(/*warm=*/false);
    std::vector<lns2::Path> paths;
    lns2::LNS2Stats stats;
    const bool ok = solver_.solve(agents, snap_grid, params, &paths, &stats);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    fill_result_metrics(*result, stats, agents, paths, elapsed_ms);

    if (!ok) {
      result->success    = false;
      result->message    = "LNS2 failed to reach collision-free (" +
          std::to_string(stats.final_collisions) + " remaining)";
      result->error_code = SetGoalsAction::Result::PBS_FAILED;
      result->total_replans = total_replans_;
      result->total_execution_sec = (now() - mission_start_time_).seconds();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      is_active_ = false;
      gh->succeed(result);
      return;
    }

    if (gh->is_canceling()) {
      result->success    = false;
      result->message    = "Cancelled after planning";
      result->error_code = SetGoalsAction::Result::CANCELLED;
      is_active_ = false;
      gh->canceled(result);
      return;
    }

    // Post-solve: detect silently-failed agents (empty paths). is_collision_free
    // accepts these, but they'd cause the robots to sit forever + be routed
    // through by others. Mark them unplanable for future iterations.
    const std::size_t newly_failed = detect_and_mark_failed(paths, plan_ids_ext);
    if (newly_failed == plan_ids_ext.size()) {
      result->success    = false;
      result->message    = "All agents failed planning (empty paths)";
      result->error_code = SetGoalsAction::Result::PBS_FAILED;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      is_active_ = false;
      gh->succeed(result);
      return;
    }

    // Publish & remember plan
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      publish_mapf_plans(paths, plan_ids_ext, snap_ox, snap_oy, snap_res);
      store_active_plan(paths, plan_ids_ext, plan_goals_world, now());
      active_goal_handle_ = gh;
      start_monitoring();
    }

    publish_debug_grid(snap_grid);  // snap_grid has skipped-robot cells blocked
    publish_rich_feedback(gh, "executing", 0, agents.size(), 0, 0);

    result->success = true;
    if (result->message.empty()) result->message = "OK";
    RCLCPP_INFO(get_logger(),
                "LNS2 initial plan: %zu agents, %.1fms, iters=%zu, "
                "init_coll=%zu -> 0",
                agents.size(), stats.wall_time_ms,
                stats.iterations, stats.initial_collisions);
  }

  // --------------------------------------------------------------------
  // Monitoring + warm-start replan
  // --------------------------------------------------------------------
  void start_monitoring()
  {
    // Reset stall tracking for every active robot — they just got a fresh
    // plan so the stall clock starts now. Must be called while holding
    // state_mutex_ (caller's responsibility — this is currently invoked
    // from execute_goal() under lock).
    const rclcpp::Time t_now = now();
    for (uint32_t rid : active_robot_ids_) {
      if (rid < static_cast<uint32_t>(num_robots_)) {
        last_movement_pos_[rid]  = current_positions_[rid];
        last_movement_time_[rid] = t_now;
      }
    }

    if (monitor_timer_) return;
    const auto period = std::chrono::duration<double>(1.0 / replan_check_hz_);
    monitor_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { check_schedule(); });
  }

  void stop_monitoring()
  {
    if (monitor_timer_) {
      monitor_timer_->cancel();
      monitor_timer_.reset();
    }
  }

  void publish_stop_all()
  {
    ++plan_id_counter_;
    for (uint32_t rid : active_robot_ids_) {
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      MAPFPlanMsg empty;
      empty.header.frame_id = "map";
      empty.header.stamp    = now();
      empty.plan_id         = plan_id_counter_;
      // empty.steps left empty -> follower treats this as CANCEL
      plan_pubs_[rid]->publish(empty);
    }
  }

  // check_schedule — runs at replan_check_hz.
  //
  // Decides per-robot whether to treat the robot as "arrived",
  // "stalled", or "failed", and triggers a replan when warranted.
  //
  // With the FollowerStatus contract, the source of truth for "is this
  // robot waiting on purpose?" is the follower itself. We no longer try
  // to infer it from spatial waypoint matching or from time-based
  // expected_position interpolation. The rules are:
  //
  //   STATE_HOLDING       — legitimate PBS hold, nothing to do.
  //   STATE_NAVIGATING    — odom-based progress check: if the robot
  //                          hasn't moved for stall_timeout_sec_ while
  //                          supposedly navigating, something is wrong
  //                          (Nav2 stuck spinning, blocked by another
  //                          robot, etc.). Counts as stalled.
  //   STATE_FAILED        — Nav2 aborted the segment. Always counts as
  //                          stalled; we need a replan to give the
  //                          follower a fresh plan it CAN execute.
  //   STATE_PLAN_COMPLETE — the follower finished executing its plan.
  //                          If the robot is at its goal, that's arrival.
  //                          If it isn't, the plan was bad (solver
  //                          produced something that didn't end at the
  //                          commanded goal, or the robot's physical
  //                          position drifted); treat as stalled so the
  //                          next replan starts from the current odom.
  //   STATE_IDLE          — stale / not yet acting on this plan_id.
  //                          Skip: give the follower more time to start.
  //
  // Stall behaviour is rate-limited via consecutive_stall_iters_[rid]
  // the same way as before: the robot only gets marked unplanable after
  // stall_iters_before_unplanable_ consecutive ticks in trouble.
  void check_schedule()
  {
    if (is_planning_.load()) return;

    std::unique_lock<std::mutex> lk(state_mutex_);
    if (!active_goal_handle_ || active_robot_ids_.empty()) return;

    const rclcpp::Time t_now = now();

    std::vector<uint32_t> stalled;            // needs replan this tick
    std::vector<uint32_t> stall_reason_nav;   // NAVIGATING but odom frozen
    std::vector<uint32_t> stall_reason_fail;  // follower reported FAILED
    std::vector<uint32_t> stall_reason_done;  // PLAN_COMPLETE but not at goal
    std::vector<uint32_t> newly_arrived;
    std::size_t arrived_count     = 0;
    std::size_t in_progress_count = 0;
    std::size_t holding_count     = 0;
    std::size_t failed_count      = 0;
    std::size_t stale_count       = 0;

    // --- Retry benched (unplanable with lives remaining) robots -----
    //
    // A robot that was marked unplanable but still has lives left gets
    // put back into the planning pool after unplanable_retry_delay_sec.
    // This handles the common case where a robot is briefly boxed in
    // by a neighbour that later moves on: instead of permanently
    // benching it on the first stall, we give it another try once the
    // scene has had time to clear.
    //
    // We reset per-robot stall / empty-fail counters and refresh the
    // movement baseline. We do NOT trigger a replan here directly —
    // we put the reinstated rid into `stalled` below so the normal
    // replan trigger picks it up on this tick.
    {
      std::vector<uint32_t> reinstated;
      for (auto it = unplanable_since_.begin();
           it != unplanable_since_.end();) {
        const uint32_t rid = it->first;
        if (rid >= static_cast<uint32_t>(num_robots_)) {
          it = unplanable_since_.erase(it);
          continue;
        }
        const double since = (t_now - it->second).seconds();
        if (since >= unplanable_retry_delay_sec_) {
          unplanable_robots_.erase(rid);
          consecutive_stall_iters_[rid] = 0;
          consecutive_empty_fails_[rid] = 0;
          if (have_odom_[rid]) {
            last_movement_pos_[rid] = current_positions_[rid];
          }
          last_movement_time_[rid] = t_now;
          reinstated.push_back(rid);
          it = unplanable_since_.erase(it);
        } else {
          ++it;
        }
      }
      if (!reinstated.empty()) {
        std::string ids_str;
        for (uint32_t rid : reinstated) {
          if (!ids_str.empty()) ids_str += " ";
          ids_str += std::to_string(rid) + "(lives=" +
              std::to_string(lives_remaining_[rid]) + ")";
        }
        RCLCPP_INFO(get_logger(),
            "retrying benched robots after %.1fs cooldown: [%s]",
            unplanable_retry_delay_sec_, ids_str.c_str());
        // Force a replan this tick so reinstated robots get fresh
        // paths immediately (they'd otherwise wait for an organic
        // replan triggered by someone else's stall). stall_reason_nav
        // carries them for the log breakdown.
        for (uint32_t rid : reinstated) {
          stalled.push_back(rid);
          stall_reason_nav.push_back(rid);
        }
      }
    }

    // Fresh status = FollowerStatus whose plan_id matches the plan we
    // most recently published. Anything else is either a pre-handover
    // echo (old plan_id) or the follower has never reported since the
    // publish. Both warrant patience rather than action.
    const double status_fresh_sec = std::max(2.0, 4.0 * stall_timeout_sec_);

    for (std::size_t i = 0; i < active_robot_ids_.size(); ++i) {
      const uint32_t rid = active_robot_ids_[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !have_odom_[rid]) continue;

      const auto& [cx, cy] = current_positions_[rid];
      const double gx = active_goals_world_[i].x;
      const double gy = active_goals_world_[i].y;
      const double to_goal = std::hypot(cx - gx, cy - gy);
      const bool   at_goal = (to_goal <= goal_reached_m_);

      // Arrival is always judged on physical odom vs goal, regardless
      // of follower state — if the robot is at its goal, we don't care
      // whether the follower thinks it's navigating or holding.
      if (at_goal) {
        ++arrived_count;
        if (!robot_arrived_[rid]) {
          robot_arrived_[rid] = true;
          newly_arrived.push_back(rid);
        }
        // Reset odom stall bookkeeping so a briefly un-arrived robot
        // (goal_reached_m_ is not latched) doesn't immediately stall.
        last_movement_pos_[rid]  = {cx, cy};
        last_movement_time_[rid] = t_now;
        consecutive_stall_iters_[rid] = 0;
        continue;
      }

      ++in_progress_count;

      // Always update odom-based movement tracking. Even for robots
      // that are reporting HOLDING, we want an up-to-date baseline so
      // that when they transition back to NAVIGATING we don't
      // instantly claim "4+ seconds without movement".
      const auto& [lx, ly] = last_movement_pos_[rid];
      const double moved_since = std::hypot(cx - lx, cy - ly);
      if (moved_since >= stall_move_thresh_m_) {
        last_movement_pos_[rid]  = {cx, cy};
        last_movement_time_[rid] = t_now;
        consecutive_stall_iters_[rid] = 0;
      }

      // Look at the most recent FollowerStatus for this robot.
      const FollowerStatus& st = last_status_[rid];
      const bool have_fresh_status =
          (last_status_time_[rid].nanoseconds() != 0) &&
          ((t_now - last_status_time_[rid]).seconds() <= status_fresh_sec);
      const bool status_matches_plan =
          have_fresh_status && (st.plan_id == active_plan_id_);

      if (!status_matches_plan) {
        // The follower hasn't caught up to the plan we published yet.
        // Give it a bit of slack — if we've been waiting too long
        // (more than stall_timeout_sec_ past last replan), treat it
        // as stalled so we either republish or move on.
        ++stale_count;
        if (last_movement_time_[rid].nanoseconds() == 0) {
          last_movement_time_[rid] = t_now;
        }
        const double since_replan =
            (t_now - last_replan_time_).seconds();
        if (since_replan > stall_timeout_sec_ && moved_since < stall_move_thresh_m_) {
          stalled.push_back(rid);
          stall_reason_nav.push_back(rid);
          consecutive_stall_iters_[rid] += 1;
        }
        continue;
      }

      switch (st.state) {
        case FollowerStatus::STATE_HOLDING: {
          ++holding_count;
          // While holding is a legitimate wait, we DO reset the stall
          // clock so the transition back to NAVIGATING doesn't inherit
          // old staleness.
          last_movement_time_[rid] = t_now;
          consecutive_stall_iters_[rid] = 0;
          break;
        }
        case FollowerStatus::STATE_NAVIGATING: {
          // Odom-based: if navigating but not moving for too long,
          // something is physically stuck (Nav2 spin, obstacle, etc.).
          if (last_movement_time_[rid].nanoseconds() == 0) {
            last_movement_time_[rid] = t_now;
            break;
          }
          const double stalled_for =
              (t_now - last_movement_time_[rid]).seconds();
          if (stalled_for > stall_timeout_sec_) {
            stalled.push_back(rid);
            stall_reason_nav.push_back(rid);
            consecutive_stall_iters_[rid] += 1;
          }
          break;
        }
        case FollowerStatus::STATE_FAILED: {
          ++failed_count;
          stalled.push_back(rid);
          stall_reason_fail.push_back(rid);
          consecutive_stall_iters_[rid] += 1;
          break;
        }
        case FollowerStatus::STATE_PLAN_COMPLETE: {
          // Follower finished the plan but the robot is not at its
          // goal. This means the grid plan ended somewhere other than
          // the commanded goal cell (can happen if odom drifted during
          // plan construction, or the solver cut the path short).
          // A fresh replan from the robot's current position is the
          // right answer.
          stalled.push_back(rid);
          stall_reason_done.push_back(rid);
          consecutive_stall_iters_[rid] += 1;
          break;
        }
        case FollowerStatus::STATE_IDLE:
        default: {
          // Follower reports IDLE with a matching plan_id. That's
          // unusual — it means the follower got the plan and then
          // cancelled on its own, or hasn't started yet. Treat like
          // stale: wait-and-see, escalate only if enough time passes.
          ++stale_count;
          const double since_replan =
              (t_now - last_replan_time_).seconds();
          if (since_replan > stall_timeout_sec_) {
            stalled.push_back(rid);
            stall_reason_nav.push_back(rid);
            consecutive_stall_iters_[rid] += 1;
          }
          break;
        }
      }
    }

    // --- Per-robot arrival log (one-shot) ------------------------------
    for (uint32_t rid : newly_arrived) {
      for (std::size_t i = 0; i < active_robot_ids_.size(); ++i) {
        if (active_robot_ids_[i] != rid) continue;
        const auto& [cx, cy] = current_positions_[rid];
        const double gx = active_goals_world_[i].x;
        const double gy = active_goals_world_[i].y;
        RCLCPP_INFO(get_logger(),
            "robot_%u arrived (%.2fm from goal, tol=%.2fm)",
            rid, std::hypot(cx - gx, cy - gy), goal_reached_m_);
        break;
      }
    }

    // --- Periodic progress summary -------------------------------------
    if (progress_log_interval_sec_ > 0.0) {
      const bool first_log = (last_progress_log_time_.nanoseconds() == 0);
      const double since_last = first_log
          ? std::numeric_limits<double>::infinity()
          : (t_now - last_progress_log_time_).seconds();
      if (since_last >= progress_log_interval_sec_) {
        RCLCPP_INFO(get_logger(),
            "progress: arrived=%zu/%zu, in_progress=%zu, holding=%zu, "
            "failed=%zu, stalled=%zu, stale=%zu, unplanable=%zu",
            arrived_count, active_robot_ids_.size(),
            in_progress_count, holding_count, failed_count,
            stalled.size(), stale_count, unplanable_robots_.size());
        
  // publish rich feedback with the same counts
  publish_rich_feedback(active_goal_handle_, "executing",
                        arrived_count,
                        active_robot_ids_.size(),
                        /*deviated=*/in_progress_count,   // or 0 if you prefer
                        /*stalled=*/stalled.size(),
                        /*info=*/"",
                        /*warning=*/"");
          
        last_progress_log_time_ = t_now;
      }
    }

    // --- Mission completion --------------------------------------------
    std::size_t plan_required = 0;
    for (uint32_t rid : active_robot_ids_) {
      if (rid < static_cast<uint32_t>(num_robots_) &&
          !unplanable_robots_.count(rid)) {
        ++plan_required;
      }
    }
    std::size_t arrived_and_planable = 0;
    for (std::size_t i = 0; i < active_robot_ids_.size(); ++i) {
      const uint32_t rid = active_robot_ids_[i];
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      if (unplanable_robots_.count(rid)) continue;
      const auto& [cx, cy] = current_positions_[rid];
      const double gx = active_goals_world_[i].x;
      const double gy = active_goals_world_[i].y;
      if (std::hypot(cx - gx, cy - gy) <= goal_reached_m_) {
        ++arrived_and_planable;
      }
    }

    if (plan_required > 0 && arrived_and_planable == plan_required) {
      auto result = std::make_shared<SetGoalsAction::Result>();
      result->success = true;
      if (unplanable_robots_.empty()) {
        result->message = "All agents arrived";
      } else {
        result->message = "All planable agents arrived (" +
            std::to_string(unplanable_robots_.size()) + " unplanable skipped)";
      }
      RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
      auto gh = active_goal_handle_;
      active_goal_handle_.reset();
      active_robot_ids_.clear();
      active_goals_world_.clear();
      stop_monitoring();
      is_active_ = false;
      lk.unlock();
      gh->succeed(result);
      return;
    }

    // --- Convert persistent stalls to unplanable ----------------------
    std::vector<uint32_t> newly_benched;   // lives > 0, will retry later
    std::vector<uint32_t> newly_permanent; // out of lives, done
    for (uint32_t rid : stalled) {
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      if (consecutive_stall_iters_[rid] >= stall_iters_before_unplanable_ &&
          !unplanable_robots_.count(rid)) {
        unplanable_robots_.insert(rid);
        lives_remaining_[rid] -= 1;
        if (lives_remaining_[rid] > 0) {
          unplanable_since_[rid] = t_now;
          newly_benched.push_back(rid);
        } else {
          newly_permanent.push_back(rid);
        }
      }
    }
    if (!newly_benched.empty() || !newly_permanent.empty()) {
      // Cancel followers for both groups — they need to stop executing
      // their current (now-abandoned) plans.
      auto cancel_follower = [&](uint32_t rid) {
        MAPFPlanMsg cancel;
        cancel.header.frame_id = "map";
        cancel.header.stamp    = t_now;
        cancel.plan_id         = ++plan_id_counter_;
        plan_pubs_[rid]->publish(cancel);
      };
      for (uint32_t rid : newly_benched)   cancel_follower(rid);
      for (uint32_t rid : newly_permanent) cancel_follower(rid);

      if (!newly_benched.empty()) {
        std::string ids_str;
        for (uint32_t rid : newly_benched) {
          if (!ids_str.empty()) ids_str += " ";
          ids_str += std::to_string(rid) + "(lives=" +
              std::to_string(lives_remaining_[rid]) + ")";
        }
        RCLCPP_WARN(get_logger(),
            "robots [%s] stalled for >= %d ticks — benching, will retry "
            "after %.1fs cooldown",
            ids_str.c_str(), stall_iters_before_unplanable_,
            unplanable_retry_delay_sec_);
      }
      if (!newly_permanent.empty()) {
        std::string ids_str;
        for (uint32_t rid : newly_permanent) {
          if (!ids_str.empty()) ids_str += " ";
          ids_str += std::to_string(rid);
        }
        RCLCPP_ERROR(get_logger(),
            "robots [%s] out of lives after persistent stalls — giving "
            "up permanently. Reset with a fresh /swarm/set_goals.",
            ids_str.c_str());
        publish_rich_feedback(active_goal_handle_, "executing",
            arrived_count, in_progress_count, 0,
            unplanable_robots_.size(),
            /*info=*/"",
            /*warning=*/"robots out of lives (stalls): " + ids_str);
      }
      stalled.erase(std::remove_if(stalled.begin(), stalled.end(),
          [this](uint32_t rid) { return unplanable_robots_.count(rid) > 0; }),
          stalled.end());
    }

    // --- Replan trigger ------------------------------------------------
    const bool cooldown_passed =
        (t_now - last_replan_time_).seconds() > replan_cooldown_sec_;

    if (!stalled.empty() && cooldown_passed) {
      // Build a breakdown log so it's visible what triggered the replan.
      auto fmt_ids = [this](const std::vector<uint32_t>& v) {
        std::string s;
        for (uint32_t rid : v) {
          if (!s.empty()) s += " ";
          s += std::to_string(rid) + "(" +
              std::to_string(consecutive_stall_iters_[rid]) + "/" +
              std::to_string(stall_iters_before_unplanable_) + ")";
        }
        return s;
      };
      std::string parts;
      if (!stall_reason_nav.empty()) {
        parts += "nav[" + fmt_ids(stall_reason_nav) + "]";
      }
      if (!stall_reason_fail.empty()) {
        if (!parts.empty()) parts += " ";
        parts += "failed[" + fmt_ids(stall_reason_fail) + "]";
      }
      if (!stall_reason_done.empty()) {
        if (!parts.empty()) parts += " ";
        parts += "plan_done_off_goal[" + fmt_ids(stall_reason_done) + "]";
      }
      RCLCPP_WARN(get_logger(),
          "replan requested: %zu stalled robots  %s",
          stalled.size(), parts.c_str());

      // feedback before replan
      publish_rich_feedback(active_goal_handle_, "replanning",
                            arrived_count, in_progress_count, stalled.size(),
                            unplanable_robots_.size(), "", "replan triggered by stalls");

      lk.unlock();
      trigger_replan();
      return;
    }
  }

  // --------------------------------------------------------------------
  // trigger_replan()
  // --------------------------------------------------------------------
  void trigger_replan()
  {
    bool expected = false;
    if (!is_planning_.compare_exchange_strong(expected, true)) return;

    std::unique_lock<std::mutex> lk(state_mutex_);
    if (!active_goal_handle_ || active_robot_ids_.empty()) {
      is_planning_ = false;
      return;
    }

    // Snapshot state
    lns2::GridMap snap_grid = grid_;
    const double snap_ox  = map_origin_x_;
    const double snap_oy  = map_origin_y_;
    const double snap_res = map_resolution_;
    auto snap_pos     = current_positions_;
    auto snap_odom    = have_odom_;
    auto snap_fp      = footprint_radii_;
    auto snap_ids     = active_robot_ids_;
    auto snap_goals   = active_goals_world_;
    auto snap_prev_paths = prev_grid_paths_;
    auto snap_unplanable = unplanable_robots_;
    const rclcpp::Time snap_plan_time = plan_origin_time_;
    auto gh = active_goal_handle_;
    lk.unlock();

    // Build agents for active set. Again: lns2::Agent::id is a dense
    // internal index; snap_ids[internal_idx] maps back to external robot_id.
    std::vector<lns2::Agent> agents;
    std::vector<uint32_t> active_ext_ids;   // external robot_id per internal idx
    std::vector<std::size_t> active_slot;   // active_slot[internal_idx] =
                                            //   position in snap_ids
    agents.reserve(snap_ids.size());
    active_ext_ids.reserve(snap_ids.size());
    active_slot.reserve(snap_ids.size());

    std::vector<uint32_t> newly_unplanable;
    std::vector<uint32_t> arrived_ext_ids;
    for (std::size_t i = 0; i < snap_ids.size(); ++i) {
      const uint32_t rid = snap_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !snap_odom[rid]) continue;
      if (snap_unplanable.count(rid)) continue;  // already marked

      // Arrived robots are treated as settled static obstacles.
      if (std::hypot(snap_pos[rid].first  - snap_goals[i].x,
                     snap_pos[rid].second - snap_goals[i].y) <= goal_reached_m_) {
        arrived_ext_ids.push_back(rid);
        continue;
      }

      lns2::Agent a;
      a.id = static_cast<lns2::AgentId>(agents.size());
      const auto& [sx, sy] = snap_pos[rid];
      a.start = world_to_cell(sx, sy, snap_ox, snap_oy, snap_res,
                               snap_grid.rows, snap_grid.cols);
      a.goal  = world_to_cell(snap_goals[i].x, snap_goals[i].y,
                               snap_ox, snap_oy, snap_res,
                               snap_grid.rows, snap_grid.cols);
      const double radius = snap_fp[rid] > 0 ? snap_fp[rid]
                                              : default_robot_radius_;
      a.footprint = lns2::FootprintModel::from_radius(
          radius + inflation_radius_, snap_res);
      // Escape fallback: same logic as execute_goal. A retry'd robot
      // often lands in a blocked cell because it stalled next to a
      // wall; without escape it gets silently skipped and never moves.
      if (snap_grid.is_blocked(a.start)) {
        const lns2::Cell rescued = find_nearest_free_cell(
            snap_grid, a.start, a.footprint);
        if (snap_grid.is_blocked(rescued)) continue;
        a.start = rescued;
      }
      if (snap_grid.is_blocked(a.goal)) {
        const lns2::Cell rescued = find_nearest_free_cell(
            snap_grid, a.goal, a.footprint);
        if (snap_grid.is_blocked(rescued)) continue;
        a.goal = rescued;
      }
      if (a.start.row == a.goal.row && a.start.col == a.goal.col) continue;
      // Static reachability (same rationale as execute_goal).
      if (!static_path_exists(snap_grid, a.start, a.goal, a.footprint)) {
        RCLCPP_ERROR(get_logger(),
            "replan: robot_%u has no static path start(%d,%d) -> goal(%d,%d), "
            "marking unplanable",
            rid, a.start.row, a.start.col, a.goal.row, a.goal.col);
        newly_unplanable.push_back(rid);
        continue;
      }
      agents.push_back(std::move(a));
      active_ext_ids.push_back(rid);
      active_slot.push_back(i);
    }

    // Commit any new unplanable marks back to shared state. They must be
    // visible to block_skipped_robots below and to future iterations.
    if (!newly_unplanable.empty()) {
      lk.lock();
      for (uint32_t rid : newly_unplanable) {
        unplanable_robots_.insert(rid);
      }
      lk.unlock();

      std::string ids_str;
      for (uint32_t rid : newly_unplanable) {
        if (!ids_str.empty()) ids_str += " ";
        ids_str += std::to_string(rid);
      }
      publish_rich_feedback(gh, "executing",
          /*arrived=*/0, /*active=*/0, /*deviated=*/0,
          /*stalled=*/newly_unplanable.size(),
          /*info=*/"",
          /*warning=*/"robots have no static path, marked unplanable: " + ids_str);
    }

    // Log and block cells of arrived robots.
    if (!arrived_ext_ids.empty()) {
      std::string ids;
      for (uint32_t rid : arrived_ext_ids) {
        if (!ids.empty()) ids += " ";
        ids += std::to_string(rid);
      }
      RCLCPP_INFO(get_logger(),
          "replan: %zu robot(s) settled at goal [%s] — marking as static obstacle",
          arrived_ext_ids.size(), ids.c_str());
      block_arrived_robots(snap_grid, arrived_ext_ids, snap_pos, snap_fp,
                            snap_ox, snap_oy, snap_res);
    }

    if (agents.empty()) {
      RCLCPP_WARN(get_logger(), "replan: no valid agents");
      is_planning_ = false;
      return;
    }

    // Block non-participating robots (combine active + arrived so arrived
    // robots are not double-logged by block_skipped_robots).
    {
      std::vector<uint32_t> all_accounted = active_ext_ids;
      all_accounted.insert(all_accounted.end(),
                           arrived_ext_ids.begin(), arrived_ext_ids.end());
      block_skipped_robots(snap_grid, all_accounted, snap_pos, snap_odom,
                            snap_fp, snap_ox, snap_oy, snap_res);
    }

    // Publish planning grid now — shows blocked cells for arrived/skipped
    // robots regardless of whether the solve below succeeds.
    publish_debug_grid(snap_grid);

    // ---- Try warm start ----
    bool used_warm = false;
    std::vector<lns2::Path> paths;
    lns2::LNS2Stats stats;
    lns2::LNS2Params params;

    if (warm_start_enabled_) {
      // PreviousPlan / ActualState indexed by INTERNAL id (matches agents).
      const std::size_t N = agents.size();
      lns2::PreviousPlan prev;
      prev.paths.assign(N, lns2::Path{});
      prev.hold_until.assign(N, static_cast<lns2::Timestep>(horizon_steps_));
      for (std::size_t k = 0; k < N; ++k) {
        const uint32_t ext = active_ext_ids[k];
        if (ext < snap_prev_paths.size()) {
          prev.paths[k] = snap_prev_paths[ext];
        }
      }
      const double dt = (now() - snap_plan_time).seconds();
      prev.delta_steps = static_cast<lns2::Timestep>(
          std::max(0.0, std::floor(dt / time_step_sec_)));

      lns2::ActualState actual;
      actual.current_cells.assign(N, lns2::Cell{0, 0});
      actual.have_odom.assign(N, 0);
      actual.goals.assign(N, lns2::Cell{0, 0});
      for (std::size_t k = 0; k < N; ++k) {
        const uint32_t ext = active_ext_ids[k];
        const auto& [cx, cy] = snap_pos[ext];
        actual.current_cells[k] = world_to_cell(
            cx, cy, snap_ox, snap_oy, snap_res, snap_grid.rows, snap_grid.cols);
        actual.have_odom[k] = 1;
        // goal cell also from snap_goals, which is aligned to snap_ids.
        const std::size_t slot = active_slot[k];
        actual.goals[k] = world_to_cell(
            snap_goals[slot].x, snap_goals[slot].y,
            snap_ox, snap_oy, snap_res, snap_grid.rows, snap_grid.cols);
      }

      lns2::Solution seed;
      lns2::WarmStartParams wp;
      wp.patch_radius_cells = warm_patch_radius_;
      lns2::WarmStartReport rep;
      lns2::build_warm_seed(agents, snap_grid, prev, actual, wp,
                             static_cast<lns2::Timestep>(horizon_steps_),
                             &seed, &rep);

      // Helper to format the report compactly for logs. We always log
      // everything non-zero so it's obvious from the log WHY a seed ended
      // up the way it did.
      auto fmt_rep = [&rep]() {
        std::string s;
        s += "on_track=" + std::to_string(rep.agents_on_track);
        s += " spliced=" + std::to_string(rep.agents_spliced);
        if (rep.agents_stubbed_far)  s += " stub_far="  + std::to_string(rep.agents_stubbed_far);
        if (rep.agents_new)          s += " stub_new="  + std::to_string(rep.agents_new);
        if (rep.agents_goal_changed) s += " stub_goal=" + std::to_string(rep.agents_goal_changed);
        if (rep.agents_invalid)      s += " stub_inv="  + std::to_string(rep.agents_invalid);
        if (rep.agents_arrived)      s += " arrived="   + std::to_string(rep.agents_arrived);
        if (rep.agents_missing)      s += " missing="   + std::to_string(rep.agents_missing);
        return s;
      };

      // If the seed has agents with StubInvalid AND an empty path, those
      // agents cannot be planned for at all (the solver's soft_astar will
      // fail on them). Log it loudly so the operator can see they're being
      // dropped from this plan iteration.
      std::size_t dropped = 0;
      for (lns2::AgentId k = 0; k < agents.size(); ++k) {
        if (rep.status[k] == lns2::SeedStatus::StubInvalid &&
            seed.path(k).empty()) {
          ++dropped;
        }
      }
      if (dropped > 0) {
        RCLCPP_WARN(get_logger(),
            "warm seed: %zu agents are StubInvalid with empty path "
            "(odom cell + goal both footprint-invalid); "
            "they will be skipped this iteration", dropped);
      }

      // ── KEY: if the warm seed is already collision-free, the current
      //    paths are still valid. Do NOT republish — that would reset
      //    path_follower and stop robot motion.
      if (rep.seed_collisions == 0) {
        RCLCPP_DEBUG(get_logger(),
            "warm seed collision-free (%s) — skipping replan, "
            "current paths are valid",
            fmt_rep().c_str());
        is_planning_ = false;
        return;
      }

      if (lns2::should_warm_start(rep, agents.size())) {
        params = make_params(/*warm=*/true);
        const bool ok = solver_.solve_from(std::move(seed), agents, snap_grid,
                                            params, &paths, &stats);
        used_warm = true;
        RCLCPP_INFO(get_logger(),
          "warm replan: %s init_coll=%zu -> %zu, iters=%zu, %.1fms, ok=%d",
          fmt_rep().c_str(), rep.seed_collisions, stats.final_collisions,
          stats.iterations, stats.wall_time_ms, ok);
      } else {
        RCLCPP_INFO(get_logger(),
          "warm seed rejected (coll=%zu, %s) -> cold replan",
          rep.seed_collisions, fmt_rep().c_str());
      }
    }

    if (!used_warm) {
      params = make_params(/*warm=*/false);
      const bool ok = solver_.solve(agents, snap_grid, params, &paths, &stats);
      RCLCPP_INFO(get_logger(),
        "cold replan: iters=%zu init_coll=%zu -> %zu, %.1fms, ok=%d",
        stats.iterations, stats.initial_collisions, stats.final_collisions,
        stats.wall_time_ms, ok);
    }

    const bool succeeded = (stats.final_collisions == 0);

    if (succeeded) {
      total_replans_++;
    }

    if (!succeeded) {
      RCLCPP_WARN(get_logger(),
          "replan did not reach collision-free; keeping old paths");
      publish_rich_feedback(gh, "replanning",
          /*arrived=*/0, /*active=*/static_cast<std::size_t>(agents.size()),
          /*deviated=*/0, /*stalled=*/0,
          /*info=*/"",
          /*warning=*/"replan did not reach collision-free (" +
              std::to_string(stats.final_collisions) + " collisions remain); keeping old paths");
      // Impose the cooldown even for a failed replan — otherwise the next
      // check_schedule tick (which will still see the same stalls that
      // triggered this one) will immediately request another replan,
      // bursting solve() calls that each take seconds with no progress.
      {
        std::lock_guard<std::mutex> lg(state_mutex_);
        last_replan_time_ = now();
      }
      is_planning_ = false;
      return;
    }

    // Narrow goals list down to the active subset actually planned, in the
    // same order as `agents`.
    std::vector<geometry_msgs::msg::Point> planned_goals;
    planned_goals.reserve(agents.size());
    for (std::size_t k = 0; k < agents.size(); ++k) {
      planned_goals.push_back(snap_goals[active_slot[k]]);
    }

    // Post-solve failure detection (same as execute_goal). Empty paths here
    // mean soft_astar gave up under the shorter warm replan time budget;
    // marking them unplanable prevents a loop where next replan sees the
    // same empty prev_path -> StubNew -> same failure.
    {
      lk.lock();
      const std::size_t perm_failed = detect_and_mark_failed(paths, active_ext_ids);
      lk.unlock();
      if (perm_failed > 0) {
        publish_rich_feedback(gh, "executing",
            /*arrived=*/0, /*active=*/static_cast<std::size_t>(agents.size()),
            /*deviated=*/0, /*stalled=*/perm_failed,
            /*info=*/"",
            /*warning=*/std::to_string(perm_failed) +
                " agent(s) out of lives after empty-path failures — permanently unplanable");
      }
    }

    // Commit: publish new plans, update active plan
    lk.lock();
    publish_mapf_plans(paths, active_ext_ids, snap_ox, snap_oy, snap_res);
    store_active_plan(paths, active_ext_ids, planned_goals, now());
    last_replan_time_ = now();
    publish_rich_feedback(active_goal_handle_, "executing");
    is_planning_ = false;
    (void)gh;
  }

  // --------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------

  // Block grid cells occupied by robots that have arrived at their goal.
  // Called during replan so that active robots route around settled ones.
  void block_arrived_robots(
      lns2::GridMap& grid,
      const std::vector<uint32_t>& arrived_ids,
      const std::vector<std::pair<double, double>>& positions,
      const std::vector<double>& fp_radii,
      double ox, double oy, double res) const
  {
    for (const uint32_t rid : arrived_ids) {
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      const auto& [sx, sy] = positions[rid];
      const lns2::Cell c = world_to_cell(sx, sy, ox, oy, res,
                                          grid.rows, grid.cols);
      const double radius = fp_radii[rid] > 0 ? fp_radii[rid]
                                               : default_robot_radius_;
      // Block only the physical footprint (no inflation): the active
      // robot's own footprint+inflation provides the safety buffer,
      // so adding inflation here would double-count it.
      const auto fp = lns2::FootprintModel::from_radius(radius, res);
      for (const auto& off : fp.offsets) {
        const lns2::Cell fc = c + off;
        if (grid.in_bounds(fc)) {
          grid.blocked[grid.index_of(fc)] = 1;
        }
      }
    }
  }

  // Block grid cells occupied by robots that have odom but are NOT in the
  // plan (e.g. skipped due to blocked goal). Other robots must route around
  // them — they're physically present on the map.
  void block_skipped_robots(
      lns2::GridMap& grid,
      const std::vector<uint32_t>& planned_ext_ids,
      const std::vector<std::pair<double, double>>& positions,
      const std::vector<bool>& have_odom,
      const std::vector<double>& fp_radii,
      double ox, double oy, double res) const
  {
    std::unordered_set<uint32_t> planned_set(
        planned_ext_ids.begin(), planned_ext_ids.end());

    int blocked_count = 0;
    for (int rid = 0; rid < num_robots_; ++rid) {
      if (planned_set.count(static_cast<uint32_t>(rid))) continue;
      if (!have_odom[rid]) {
        RCLCPP_WARN(get_logger(),
            "robot_%d not in plan and NO ODOM — cannot block! "
            "It may be a phantom obstacle.", rid);
        continue;
      }

      const auto& [sx, sy] = positions[rid];
      lns2::Cell c = world_to_cell(sx, sy, ox, oy, res, grid.rows, grid.cols);
      const double radius = fp_radii[rid] > 0 ? fp_radii[rid]
                                               : default_robot_radius_;
      auto fp = lns2::FootprintModel::from_radius(radius, res);
      for (const auto& off : fp.offsets) {
        lns2::Cell fc = c + off;
        if (grid.in_bounds(fc)) {
          grid.blocked[grid.index_of(fc)] = 1;
        }
      }
      RCLCPP_WARN(get_logger(),
          "Blocked cells for non-participating robot_%d at grid(%d,%d) "
          "world(%.2f,%.2f)", rid, c.row, c.col, sx, sy);
      ++blocked_count;
    }
    if (blocked_count > 0) {
      RCLCPP_INFO(get_logger(),
          "block_skipped_robots: %d robots blocked, %zu robots planned",
          blocked_count, planned_ext_ids.size());
    }
  }

  // Static reachability check (footprint-aware BFS).
  // Returns true if there exists any sequence of 4-connected moves from
  // `start` to `goal` such that every intermediate cell is footprint-valid
  // (entire footprint lies in free space). Ignores other agents entirely
  // — this is a purely static check that complements is_blocked(start/goal).
  //
  // Purpose: detect agents whose goal sits in a region not connected to
  // their start (e.g. partitioned by walls or skipped-robot blocks), BEFORE
  // handing them to the LNS2 solver. Without this, such agents produce
  // empty paths inside build_initial_solution, which is_collision_free()
  // silently accepts — and they end up stuck forever with StubNew status.
  //
  // Cost: bounded by the free-cell count reachable from `start`. On a
  // 150x150 grid the worst case is ~22500 hash ops; measured <5 ms.
  static bool static_path_exists(const lns2::GridMap& grid,
                                  const lns2::Cell& start,
                                  const lns2::Cell& goal,
                                  const lns2::FootprintModel& fp)
  {
    auto footprint_fits = [&](const lns2::Cell& base) {
      for (const auto& off : fp.offsets) {
        const lns2::Cell c = base + off;
        if (!grid.in_bounds(c)) return false;
        if (grid.is_blocked(c)) return false;
      }
      return true;
    };

    if (!footprint_fits(start) || !footprint_fits(goal)) return false;
    if (start == goal) return true;

    std::unordered_set<lns2::CellIdx> visited;
    visited.reserve(1024);
    std::queue<lns2::Cell> q;
    q.push(start);
    visited.insert(grid.index_of(start));

    const lns2::CellOffset step4[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    const lns2::CellIdx goal_idx = grid.index_of(goal);

    while (!q.empty()) {
      const lns2::Cell cur = q.front();
      q.pop();
      for (const auto& s : step4) {
        const lns2::Cell nxt = cur + s;
        if (!grid.in_bounds(nxt)) continue;
        if (!footprint_fits(nxt)) continue;
        const lns2::CellIdx nidx = grid.index_of(nxt);
        if (!visited.insert(nidx).second) continue;
        if (nidx == goal_idx) return true;
        q.push(nxt);
      }
    }
    return false;
  }

  // Find the nearest cell to `wanted` where the robot's footprint fits
  // on the static map. Used as an "escape hatch" when a robot's current
  // odom position (or its goal) maps to a grid cell that MAPF considers
  // blocked.
  //
  // This happens because:
  //   * The MAPF grid is downsampled from the occupancy map (0.05m ->
  //     0.2m), so a grid cell is blocked if ANY of its 16 source pixels
  //     is occupied. A robot standing close to a wall can easily land
  //     in such a cell physically.
  //   * The MAPF footprint uses robot_radius + inflation_radius_, which
  //     excludes even more cells near walls.
  //
  // Without an escape, such a robot is permanently excluded from
  // planning — MAPF skips it in execute_goal / trigger_replan, follower
  // receives an empty plan, robot stays put, goal never reached.
  //
  // With escape: we plan from/to the nearest free cell instead. The
  // follower will navigate to the first MAPFStep target via Nav2; the
  // local planner (RPP) will construct a short manoeuvre out of the
  // blocked cell into the free cell, and then proceed along the plan.
  //
  // BFS with Chebyshev-radius limit `max_radius_cells` (default 10 =
  // 2m on a 0.2m grid). Returns `wanted` unchanged if it's already free,
  // or the closest free cell found. If nothing fits within the radius,
  // returns `wanted` (caller must still check footprint_fits).
  static lns2::Cell find_nearest_free_cell(
      const lns2::GridMap& grid,
      const lns2::Cell& wanted,
      const lns2::FootprintModel& fp,
      int max_radius_cells = 10)
  {
    auto footprint_fits = [&](const lns2::Cell& base) {
      for (const auto& off : fp.offsets) {
        const lns2::Cell c = base + off;
        if (!grid.in_bounds(c)) return false;
        if (grid.is_blocked(c)) return false;
      }
      return true;
    };

    if (grid.in_bounds(wanted) && footprint_fits(wanted)) {
      return wanted;
    }

    std::unordered_set<lns2::CellIdx> visited;
    visited.reserve(static_cast<size_t>(
        (2 * max_radius_cells + 1) * (2 * max_radius_cells + 1)));
    std::queue<std::pair<lns2::Cell, int>> q;
    if (grid.in_bounds(wanted)) {
      q.push({wanted, 0});
      visited.insert(grid.index_of(wanted));
    }

    const lns2::CellOffset step8[] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!q.empty()) {
      auto [cur, dist] = q.front();
      q.pop();
      if (dist > max_radius_cells) continue;
      if (footprint_fits(cur)) {
        return cur;
      }
      for (const auto& s : step8) {
        const lns2::Cell nxt = cur + s;
        if (!grid.in_bounds(nxt)) continue;
        const lns2::CellIdx nidx = grid.index_of(nxt);
        if (!visited.insert(nidx).second) continue;
        q.push({nxt, dist + 1});
      }
    }
    return wanted;  // no free cell within radius — caller must handle
  }

  // After a successful solve(), scan the produced paths. Any agent left
  // with an empty path wasn't actually planned for — soft_astar failed
  // on it inside build_initial_solution / repair, but `is_collision_free`
  // still returned true (an empty path has no collisions).
  //
  // Policy (soft unplanable): we do NOT mark a robot unplanable on the
  // first empty path. A single empty solve often means "this robot is
  // boxed in by peers that are themselves planning to move". Give it a
  // few solves to reconcile before banning it from future plans.
  //
  //  * Increment consecutive_empty_fails_[rid] for each empty path.
  //  * Reset the counter to 0 for every non-empty path (recovered).
  //  * Only insert into unplanable_robots_ once the counter crosses
  //    empty_fails_before_unplanable_.
  //
  // Side effects:
  //  * Publishes an empty nav_msgs::Path for each failed robot so its
  //    path_follower stops sending chunks from the previous plan. This is
  //    done every time, not just at the unplanable threshold — otherwise
  //    the robot keeps executing stale waypoints while we wait to see if
  //    solve recovers.
  //
  // Returns: number of robots that crossed the unplanable threshold.
  std::size_t detect_and_mark_failed(
      const std::vector<lns2::Path>& paths,
      const std::vector<uint32_t>& ext_ids)
  {
    std::size_t newly_benched_cnt   = 0;
    std::size_t newly_permanent_cnt = 0;
    std::string new_bench_ids, new_perm_ids, soft_fail_ids;
    const rclcpp::Time t_now = now();

    for (std::size_t k = 0; k < paths.size() && k < ext_ids.size(); ++k) {
      const uint32_t rid = ext_ids[k];
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;

      // Treat 1-cell "stay put" paths as empty — they mean the solver
      // couldn't route this robot and warm_start emitted a stub. They
      // should count toward the unplanable threshold just like truly
      // empty paths do; otherwise publish_mapf_plans would suppress
      // them silently and the empty-fail counter would never advance.
      const bool is_empty_like =
          paths[k].empty() || paths[k].size() == 1;

      if (!is_empty_like) {
        // Successful plan for this robot — reset the empty-fail streak.
        consecutive_empty_fails_[rid] = 0;
        continue;
      }

      // Empty or stub path — count it.
      consecutive_empty_fails_[rid] += 1;

      // Always tell the follower to cancel so it stops executing stale
      // steps from a plan it's no longer participating in. Empty steps
      // in MAPFPlan is the cancel signal.
      MAPFPlanMsg cancel;
      cancel.header.frame_id = "map";
      cancel.header.stamp    = t_now;
      cancel.plan_id         = ++plan_id_counter_;
      plan_pubs_[rid]->publish(cancel);

      if (consecutive_empty_fails_[rid] >= empty_fails_before_unplanable_) {
        if (unplanable_robots_.insert(rid).second) {
          lives_remaining_[rid] -= 1;
          if (lives_remaining_[rid] > 0) {
            unplanable_since_[rid] = t_now;
            if (!new_bench_ids.empty()) new_bench_ids += " ";
            new_bench_ids += std::to_string(rid) + "(lives=" +
                std::to_string(lives_remaining_[rid]) + ")";
            ++newly_benched_cnt;
          } else {
            if (!new_perm_ids.empty()) new_perm_ids += " ";
            new_perm_ids += std::to_string(rid);
            ++newly_permanent_cnt;
          }
        }
      } else {
        if (!soft_fail_ids.empty()) soft_fail_ids += " ";
        soft_fail_ids += std::to_string(rid) + "(" +
            std::to_string(consecutive_empty_fails_[rid]) + "/" +
            std::to_string(empty_fails_before_unplanable_) + ")";
      }
    }

    if (!soft_fail_ids.empty()) {
      RCLCPP_WARN(get_logger(),
          "solve() left agents with empty paths (soft, keep trying): [%s]",
          soft_fail_ids.c_str());
    }
    if (newly_benched_cnt > 0) {
      RCLCPP_WARN(get_logger(),
          "agents crossed empty-path threshold (%d consecutive fails) "
          "-> benching for %.1fs: [%s]",
          empty_fails_before_unplanable_,
          unplanable_retry_delay_sec_, new_bench_ids.c_str());
    }
    if (newly_permanent_cnt > 0) {
      RCLCPP_ERROR(get_logger(),
          "agents out of lives after persistent empty-path failures "
          "-> giving up permanently: [%s]. Reset with a fresh "
          "/swarm/set_goals.", new_perm_ids.c_str());
    }
    return newly_benched_cnt + newly_permanent_cnt;
  }

  lns2::LNS2Params make_params(bool warm) const
  {
    lns2::LNS2Params p;
    p.astar.step_cost          = 1;
    p.astar.collision_penalty  = static_cast<lns2::Cost>(collision_penalty_);
    p.astar.horizon            = static_cast<lns2::Timestep>(horizon_steps_);
    p.astar.max_expansions     = static_cast<std::size_t>(max_astar_expansions_);
    p.astar.diagonal_moves     = diagonal_moves_;
    p.neighborhood_size        = static_cast<std::size_t>(neighborhood_size_);
    p.time_budget_ms           = warm ? static_cast<std::size_t>(replan_time_budget_ms_)
                                        : static_cast<std::size_t>(time_budget_ms_);
    p.plateau_limit            = static_cast<std::size_t>(plateau_limit_);
    p.segment_size             = static_cast<std::size_t>(segment_size_);
    p.alns_reaction            = alns_reaction_;
    p.seed                     = 0;  // nondeterministic per run
    return p;
  }

  // Publish a MAPFPlan to each robot's /<ns>/mapf_plan topic.
  //
  // `paths[k]` is the LNS2 grid path for the k-th planned agent, whose
  // external robot_id is `plan_ids_ext[k]`. The LNS2 solver encodes
  // time via repetition: if a robot must wait at cell C for N time
  // steps, the path contains N consecutive entries at C. This function
  // compresses those runs into a single MAPFStep whose hold_sec equals
  // the extra wait time.
  //
  // Robots whose solved path is empty (StubNew / solver-failed cases)
  // get a CANCEL plan (empty steps), which tells the follower to drop
  // any in-flight Nav2 goal and go IDLE. detect_and_mark_failed()
  // decides whether to escalate them to unplanable.
  //
  // A fresh plan_id is assigned to every call; followers echo it back
  // in their FollowerStatus so check_schedule can tell which plan the
  // status refers to.
  void publish_mapf_plans(const std::vector<lns2::Path>& paths,
                          const std::vector<uint32_t>& plan_ids_ext,
                          double ox, double oy, double res)
  {
    const rclcpp::Time base = now();
    ++plan_id_counter_;
    active_plan_id_ = plan_id_counter_;

    std::size_t published = 0;
    std::size_t cancelled = 0;
    for (std::size_t k = 0; k < paths.size() && k < plan_ids_ext.size(); ++k) {
      const uint32_t rid = plan_ids_ext[k];
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;

      MAPFPlanMsg plan_msg;
      plan_msg.header.frame_id = "map";
      plan_msg.header.stamp    = base;
      plan_msg.plan_id         = active_plan_id_;

      // Paths that collapsed to a single same-cell entry are stub
      // plans: the solver couldn't route this robot and warm_start
      // emitted a "stay put" placeholder. Publishing such a plan to
      // the follower would make it navigate to its current cell and
      // immediately report PLAN_COMPLETE, masking the real failure.
      // Treat it as empty so the follower cancels and check_schedule
      // can count it as an empty-path failure.
      bool is_stub = false;
      if (paths[k].size() == 1) {
        // Single-cell path with no movement. The only case where this
        // is legitimate is when the robot is already on its goal cell
        // — but in that case we shouldn't be planning for it at all.
        is_stub = true;
      }

      // Empty grid path -> empty MAPFPlan -> follower cancels.
      if (!paths[k].empty() && !is_stub) {
        plan_msg.steps = grid_path_to_steps(paths[k], ox, oy, res);
        ++published;
      } else {
        ++cancelled;
      }
      plan_pubs_[rid]->publish(plan_msg);

      // The follower will send its first NAVIGATING status shortly.
      // Until then, invalidate any cached status so we don't act on
      // stale data referring to the previous plan.
      last_status_[rid] = FollowerStatus{};
      last_status_time_[rid] = rclcpp::Time{0, 0, RCL_ROS_TIME};
    }
    RCLCPP_INFO(get_logger(),
        "published MAPFPlan id=%u: %zu with steps, %zu cancels",
        active_plan_id_, published, cancelled);
  }

  // Compress a grid path into an ordered list of MAPFSteps. Consecutive
  // entries at the same cell become a single step whose hold_sec counts
  // the extra time beyond the arrival step.
  //
  // Example (time_step_sec=0.5):
  //   grid path  = [A, A, A, B, B, C]
  //   compressed = [A hold 1.0s, B hold 0.5s, C hold 0.0s]
  //
  // We drop the leading entry if it matches the rest of the plan's
  // start (the LNS2 convention is that path[0] = start cell; followers
  // take their current odom pose as implicit start, so the first step
  // we emit is actually path[0]'s target).
  //
  // The final step always has hold_sec == 0 unless the solver itself
  // planned a trailing hold (LNS2 normally trims trailing holds).
  std::vector<MAPFStepMsg> grid_path_to_steps(
      const lns2::Path& path, double ox, double oy, double res) const
  {
    std::vector<MAPFStepMsg> steps;
    if (path.empty()) return steps;

    // Walk the path run-length encoding same-cell runs.
    std::size_t i = 0;
    while (i < path.size()) {
      std::size_t j = i;
      while (j + 1 < path.size() && path[j + 1] == path[i]) ++j;
      // Run [i, j] has (j - i + 1) entries at cell path[i]. Arriving at
      // the cell eats 1 step; the rest is hold time.
      const std::size_t run_len = j - i + 1;
      const std::size_t hold_steps =
          (run_len > 0) ? (run_len - 1) : 0;

      MAPFStepMsg step;
      double wx, wy;
      cell_to_world(path[i], ox, oy, res, wx, wy);
      step.target.x = wx;
      step.target.y = wy;
      step.target.z = 0.0;
      step.hold_sec = static_cast<float>(
          static_cast<double>(hold_steps) * time_step_sec_);
      steps.push_back(std::move(step));

      i = j + 1;
    }
    return steps;
  }

  // Must be called with state_mutex_ held.
  // `paths` / `plan_ids_ext` / `goals` are parallel arrays of length N,
  // where plan_ids_ext[k] is the external robot_id for the k-th planned
  // agent and goals[k] is its goal in world coordinates.
  void store_active_plan(const std::vector<lns2::Path>& paths,
                          const std::vector<uint32_t>& plan_ids_ext,
                          const std::vector<geometry_msgs::msg::Point>& goals,
                          const rclcpp::Time& plan_time)
  {
    active_robot_ids_    = plan_ids_ext;
    active_goals_world_  = goals;

    // Save grid paths indexed by EXTERNAL robot_id for warm-start reuse.
    prev_grid_paths_.assign(num_robots_, lns2::Path{});
    for (std::size_t k = 0; k < paths.size() && k < plan_ids_ext.size(); ++k) {
      const uint32_t rid = plan_ids_ext[k];
      if (rid < static_cast<uint32_t>(num_robots_)) {
        prev_grid_paths_[rid] = paths[k];
      }
    }

    plan_origin_time_ = plan_time;
    last_replan_time_ = plan_time;
  }


  void fill_result_metrics(SetGoalsAction::Result& r,
                            const lns2::LNS2Stats& s,
                            const std::vector<lns2::Agent>& agents,
                            const std::vector<lns2::Path>& paths,
                            double elapsed_ms)
  {
    r.planning_time_ms   = static_cast<float>(elapsed_ms);
    r.num_agents_planned = static_cast<uint32_t>(agents.size());
    // Reuse PBS-named fields for LNS2 stats so the existing action msg works:
    r.pbs_expansions     = static_cast<uint32_t>(s.iterations);
    r.astar_ok_count     =
        static_cast<uint32_t>(s.total_astar_calls - s.total_astar_fails);
    r.astar_fail_count   = static_cast<uint32_t>(s.total_astar_fails);
    r.astar_avg_exp      = s.total_astar_calls > 0
        ? static_cast<uint32_t>(
            s.total_astar_expansions / s.total_astar_calls)
        : 0;
    r.astar_max_exp      = 0;  // not tracked
    std::size_t mx = 0;
    r.path_lengths.resize(paths.size());
    for (std::size_t i = 0; i < paths.size(); ++i) {
      r.path_lengths[i] = static_cast<uint32_t>(paths[i].size());
      if (paths[i].size() > mx) mx = paths[i].size();
    }
    r.max_path_length = static_cast<uint32_t>(mx);
  }

  // ------------------ Members ------------------
  int    num_robots_;
  double time_step_sec_;
  double grid_resolution_;
  double default_robot_radius_;
  double inflation_radius_;
  double max_speed_;
  double goal_reached_m_;
  int    collision_penalty_;
  int    horizon_steps_;
  int    max_astar_expansions_;
  int    neighborhood_size_;
  int    time_budget_ms_;
  int    plateau_limit_;
  int    segment_size_;
  double alns_reaction_;
  bool   diagonal_moves_;
  double replan_check_hz_;
  double replan_threshold_m_;
  double replan_cooldown_sec_;
  int    replan_time_budget_ms_;
  bool   warm_start_enabled_;
  int    warm_patch_radius_;
  double stall_timeout_sec_;
  double stall_move_thresh_m_;
  double progress_log_interval_sec_;
  int    empty_fails_before_unplanable_;
  int    stall_iters_before_unplanable_;
  int    max_lives_;
  double unplanable_retry_delay_sec_;

  // Map / grid
  bool   map_ready_ = false;
  double map_origin_x_ = 0.0, map_origin_y_ = 0.0, map_resolution_ = 0.2;
  bool publish_debug_grid_ = true;
  lns2::GridMap grid_;

  // Per-robot state
  std::vector<std::pair<double, double>> current_positions_;
  std::vector<bool>   have_odom_;
  std::vector<double> footprint_radii_;

  uint32_t total_replans_ = 0;
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};

  // Active plan
  std::vector<lns2::Path> prev_grid_paths_;
  std::vector<uint32_t>   active_robot_ids_;
  std::vector<geometry_msgs::msg::Point> active_goals_world_;
  rclcpp::Time plan_origin_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};

  // Robots the solver has (silently) failed to plan for. They are skipped
  // from the agent set and their cells are blocked as static obstacles so
  // that other robots route around their physical bodies rather than
  // trying to pass through them. Entries are reinstated after
  // unplanable_retry_delay_sec if the robot has lives remaining; they
  // are fully cleared on each new /swarm/set_goals accept.
  std::unordered_set<uint32_t> unplanable_robots_;

  // When a robot was most recently marked unplanable, so we can retry
  // it after a cooldown. Only populated for robots currently in
  // unplanable_robots_ whose lives_remaining_[rid] > 0.
  std::unordered_map<uint32_t, rclcpp::Time> unplanable_since_;

  // Per-robot progress tracking (indexed by external robot_id, size num_robots_)
  //  * robot_arrived_: latched once a robot reaches its goal — used to emit
  //    a per-robot "arrived" log message exactly once per mission.
  //  * last_movement_pos_, last_movement_time_: last position the robot was
  //    seen to have moved beyond stall_move_thresh_m_ from. Used by stall
  //    detection to force a replan when a robot is stuck without having
  //    deviated from its plan (e.g. path_follower finished all chunks but
  //    the plan's last waypoint wasn't actually the goal).
  std::vector<bool>                       robot_arrived_;
  std::vector<std::pair<double, double>>  last_movement_pos_;
  std::vector<rclcpp::Time>               last_movement_time_;
  rclcpp::Time last_progress_log_time_{0, 0, RCL_ROS_TIME};

  // Soft-unplanable counters (indexed by external robot_id). Reset to 0
  // when the corresponding robot appears in a successful non-empty plan
  // (or moves more than stall_move_thresh_m_). Only converted to a hard
  // unplanable mark when the counter crosses its threshold.
  std::vector<int>  consecutive_empty_fails_;
  std::vector<int>  consecutive_stall_iters_;

  // Remaining "lives" per robot. Decremented each time a robot is
  // marked unplanable. While > 0, the robot is returned to the
  // planning pool after unplanable_retry_delay_sec. Once it hits 0,
  // the robot stays unplanable until a new /swarm/set_goals resets it.
  std::vector<int>  lives_remaining_;

  // Concurrency
  std::mutex          state_mutex_;
  std::atomic<bool>   is_planning_{false};
  std::atomic<bool>   is_active_{false};

  // Plan publishing / follower status. plan_id_counter_ is incremented
  // on every publish of a MAPFPlan. Followers echo the plan_id in their
  // FollowerStatus so we can ignore stale statuses referring to the
  // previous plan while the new one is propagating.
  uint32_t plan_id_counter_ = 0;
  uint32_t active_plan_id_  = 0;
  std::vector<FollowerStatus>  last_status_;
  std::vector<rclcpp::Time>    last_status_time_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>     odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> footprint_subs_;
  std::vector<rclcpp::Subscription<FollowerStatus>::SharedPtr>               status_subs_;
  std::vector<rclcpp::Publisher<MAPFPlanMsg>::SharedPtr>                     plan_pubs_;
  rclcpp_action::Server<SetGoalsAction>::SharedPtr plan_action_server_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  std::shared_ptr<GoalHandle>  active_goal_handle_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_grid_pub_;

  // Solver
  lns2::LNS2Solver solver_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<MapfLns2Node>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}