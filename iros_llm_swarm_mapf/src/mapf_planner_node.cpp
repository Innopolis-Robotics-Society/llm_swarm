#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

// PBS solver (header-only library)
#include "iros_llm_swarm_mapf/pbs_solver.hpp"

// Action interface (replaces the old SetGoals service)
#include "iros_llm_swarm_interfaces/action/set_goals.hpp"

// ---------------------------------------------------------------------------
// The planner exposes the /swarm/set_goals action accepting robot_ids
// and goal poses.  Start positions come from odometry (/robot_N/odom).
// Footprint radii come from Nav2 (/robot_N/local_costmap/published_footprint).
//
// Action result includes:
//   success, message, planning_time_ms, num_agents_planned,
//   pbs_expansions, max_path_length, path_lengths[]
//
// On success, the node publishes paths to per-robot topics:
//   /robot_0/mapf_path, /robot_1/mapf_path, ...  (nav_msgs/Path)
//
// Each PoseStamped contains a timestamp:
//   stamp = t_start + step_index * time_step_sec
//
// The path follower should reach waypoint[k] and wait until stamp[k]
// before moving to waypoint[k+1].
//
// Threading model
// ---------------
// Executor thread (rclcpp::spin): runs all subscription/timer callbacks.
// Planning thread (detached):     runs execute_goal() for each accepted goal.
//
// Shared state is protected by state_mutex_.  Planning itself (solver.solve)
// runs without the lock (pure computation on a grid snapshot).  The lock is
// held only for brief snapshots at the start and state updates at the end.
//
// is_planning_ (atomic) prevents concurrent goal execution and makes
// check_schedule() a no-op while planning is in progress.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------

static Cell world_to_cell(double wx, double wy,
                           double origin_x, double origin_y,
                           double resolution,
                           size_t rows, size_t cols)
{
  const size_t col = static_cast<size_t>((wx - origin_x) / resolution);
  const size_t row = static_cast<size_t>((wy - origin_y) / resolution);
  return {std::min(row, rows - 1), std::min(col, cols - 1)};
}

static void cell_to_world(const Cell& c,
                           double origin_x, double origin_y,
                           double resolution,
                           double& wx, double& wy)
{
  wx = origin_x + (c.col + 0.5) * resolution;
  wy = origin_y + (c.row + 0.5) * resolution;
}

// ---------------------------------------------------------------------------
// MAPF Planner Node
// ---------------------------------------------------------------------------

class MapfPlannerNode : public rclcpp::Node {
 public:
  using SetGoalsAction = iros_llm_swarm_interfaces::action::SetGoals;
  using GoalHandle     = rclcpp_action::ServerGoalHandle<SetGoalsAction>;

  MapfPlannerNode()
      : Node("mapf_planner")
  {
    // ---------------------------------------------------------------- params
    declare_parameter("num_robots",           20);
    declare_parameter("time_step_sec",        0.1);
    declare_parameter("map_topic",            std::string("/map"));
    declare_parameter("pbs_resolution",       0.2);
    declare_parameter("default_robot_radius", 0.22);
    declare_parameter("inflation_radius",     0.5);
    declare_parameter("replan_check_hz",      2.0);
    declare_parameter("replan_threshold_m",   1.0);
    declare_parameter("replan_cooldown_sec",  10.0);
    declare_parameter("replan_predict_sec",  -1.0);
    declare_parameter("replan_stop_mode",     std::string("all"));
    declare_parameter("goal_reached_m",       0.5);
    declare_parameter("max_pbs_expansions",   5000);
    declare_parameter("max_astar_expansions", 200000);
    declare_parameter("cost_curve",           std::string("quadratic"));
    declare_parameter("proximity_penalty",    15);
    declare_parameter("max_speed",             0.5);
    declare_parameter("urgency",              1.0);

    num_robots_           = get_parameter("num_robots").as_int();
    time_step_sec_        = get_parameter("time_step_sec").as_double();
    pbs_resolution_       = get_parameter("pbs_resolution").as_double();
    default_robot_radius_ = get_parameter("default_robot_radius").as_double();
    inflation_radius_     = get_parameter("inflation_radius").as_double();
    replan_check_hz_      = get_parameter("replan_check_hz").as_double();
    replan_threshold_m_   = get_parameter("replan_threshold_m").as_double();
    replan_cooldown_sec_  = get_parameter("replan_cooldown_sec").as_double();
    replan_predict_sec_   = get_parameter("replan_predict_sec").as_double();
    replan_stop_mode_     = get_parameter("replan_stop_mode").as_string();
    if (replan_stop_mode_ != "none" && replan_stop_mode_ != "deviated" &&
        replan_stop_mode_ != "all") {
      RCLCPP_ERROR(get_logger(),
          "Invalid replan_stop_mode '%s', falling back to 'deviated'.",
          replan_stop_mode_.c_str());
      replan_stop_mode_ = "deviated";
    }
    goal_reached_m_       = get_parameter("goal_reached_m").as_double();
    max_pbs_expansions_   = static_cast<size_t>(
        get_parameter("max_pbs_expansions").as_int());
    max_astar_expansions_ = static_cast<size_t>(
        get_parameter("max_astar_expansions").as_int());
    {
      const auto s = get_parameter("cost_curve").as_string();
      if      (s == "linear")    cost_curve_ = CostCurve::Linear;
      else if (s == "cubic")     cost_curve_ = CostCurve::Cubic;
      else                       cost_curve_ = CostCurve::Quadratic;
      if (s != "linear" && s != "quadratic" && s != "cubic") {
        RCLCPP_ERROR(get_logger(),
            "Invalid cost_curve '%s', falling back to 'quadratic'.", s.c_str());
      }
    }
    proximity_penalty_ = get_parameter("proximity_penalty").as_int();
    max_speed_         = get_parameter("max_speed").as_double();
    urgency_           = get_parameter("urgency").as_double();

    // ------------------------------------------------------- map subscription
    // map_server publishes with transient_local QoS — must match.
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), map_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          on_map(msg);
        });

    // --------------------------------- per-robot odometry and footprint subs
    current_positions_.resize(num_robots_, {0.0, 0.0});
    have_odom_.resize(num_robots_, false);
    footprint_radii_.resize(num_robots_, 0.0);
    odom_subs_.resize(num_robots_);
    footprint_subs_.resize(num_robots_);

    for (int i = 0; i < num_robots_; ++i) {
      const std::string odom_topic = "/robot_" + std::to_string(i) + "/odom";
      odom_subs_[i] = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 10,
          [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            current_positions_[i] = {msg->pose.pose.position.x,
                                      msg->pose.pose.position.y};
            have_odom_[i] = true;
          });

      const std::string fp_topic =
          "/robot_" + std::to_string(i) + "/local_costmap/published_footprint";
      footprint_subs_[i] = create_subscription<geometry_msgs::msg::PolygonStamped>(
          fp_topic, rclcpp::QoS(1),
          [this, i](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
            if (msg->polygon.points.empty()) return;
            double cx = 0.0, cy = 0.0;
            for (const auto& p : msg->polygon.points) { cx += p.x; cy += p.y; }
            const double n = static_cast<double>(msg->polygon.points.size());
            cx /= n; cy /= n;
            double max_r = 0.0;
            for (const auto& p : msg->polygon.points) {
              const double r = std::hypot(p.x - cx, p.y - cy);
              if (r > max_r) max_r = r;
            }
            std::lock_guard<std::mutex> lk(state_mutex_);
            footprint_radii_[i] = max_r;
          });
    }

    // --------------------------------------------------------- action server
    plan_action_server_ = rclcpp_action::create_server<SetGoalsAction>(
        this,
        "/swarm/set_goals",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const SetGoalsAction::Goal> goal) {
          return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          return handle_cancel(gh);
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          handle_accepted(gh);
        });

    // --------------------------------------------------------- path publishers
    path_pubs_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
      const std::string topic = "/robot_" + std::to_string(i) + "/mapf_path";
      path_pubs_[i] = create_publisher<nav_msgs::msg::Path>(topic, 10);
    }

    RCLCPP_INFO(get_logger(),
        "mapf_planner ready (action): %d robots, time_step=%.3f s, "
        "default_radius=%.3f m, inflation=%.3f m (effective=%.3f m), "
        "replan: %.1f Hz, threshold=%.2f m, cooldown=%.1f s, "
        "predict=%.2f s, stop_mode=%s",
        num_robots_, time_step_sec_, default_robot_radius_,
        inflation_radius_, default_robot_radius_ + inflation_radius_,
        replan_check_hz_, replan_threshold_m_, replan_cooldown_sec_,
        replan_predict_sec_, replan_stop_mode_.c_str());
  }

 private:
  // ------------------------------------------------------------------
  // Map callback  (called under state_mutex_)
  // ------------------------------------------------------------------
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const double src_res = msg->info.resolution;
    const int ratio = std::max(1, static_cast<int>(std::round(pbs_resolution_ / src_res)));
    const double actual_res = src_res * ratio;

    const int src_w = static_cast<int>(msg->info.width);
    const int src_h = static_cast<int>(msg->info.height);
    const int dst_w = (src_w + ratio - 1) / ratio;
    const int dst_h = (src_h + ratio - 1) / ratio;

    RCLCPP_INFO_ONCE(get_logger(),
        "Map received: %dx%d @ %.3fm -> PBS grid: %dx%d @ %.3fm (ratio=%d)",
        src_w, src_h, src_res, dst_w, dst_h, actual_res, ratio);

    grid_.rows = static_cast<size_t>(dst_h);
    grid_.cols = static_cast<size_t>(dst_w);
    grid_.blocked.assign(grid_.rows * grid_.cols, 0);

    for (int sr = 0; sr < src_h; ++sr) {
      for (int sc = 0; sc < src_w; ++sc) {
        const int8_t v = msg->data[static_cast<size_t>(sr * src_w + sc)];
        if (v > 50 || v < 0) {
          grid_.blocked[static_cast<size_t>((sr / ratio) * dst_w + (sc / ratio))] = 1;
        }
      }
    }

    map_origin_x_   = msg->info.origin.position.x;
    map_origin_y_   = msg->info.origin.position.y;
    map_resolution_ = actual_res;
    map_ready_      = true;
  }

  // ------------------------------------------------------------------
  // Action server callbacks
  // ------------------------------------------------------------------

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const SetGoalsAction::Goal> goal)
  {
    // Quick checks so we can reject before touching shared state
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "Goal rejected: map not ready yet");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->robot_ids.size() != goal->goals.size()) {
      RCLCPP_WARN(get_logger(), "Goal rejected: robot_ids/goals size mismatch");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->robot_ids.empty()) {
      RCLCPP_WARN(get_logger(), "Goal rejected: empty robot_ids");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Reject concurrent goals — PBS is not re-entrant
    bool expected = false;
    if (!is_planning_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Goal rejected: planning already in progress");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Goal cancel requested — will honour at next check");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Detach immediately so the executor thread is never blocked by planning
    std::thread([this, goal_handle]() { execute_goal(goal_handle); }).detach();
  }

  // ------------------------------------------------------------------
  // Planning execution  (runs in a detached thread)
  // ------------------------------------------------------------------
  void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto& req = goal_handle->get_goal();
    auto result   = std::make_shared<SetGoalsAction::Result>();
    auto feedback = std::make_shared<SetGoalsAction::Feedback>();

    // ── 1. Snapshot shared state (brief lock) ────────────────────────────
    std::vector<std::pair<double, double>> snap_pos;
    std::vector<bool>   snap_have_odom;
    std::vector<double> snap_footprint;
    GridMap snap_grid;
    double  snap_ox, snap_oy, snap_res;

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      stop_monitoring();            // cancel schedule timer before planning
      snap_pos       = current_positions_;
      snap_have_odom = have_odom_;
      snap_footprint = footprint_radii_;
      snap_grid      = grid_;
      snap_ox        = map_origin_x_;
      snap_oy        = map_origin_y_;
      snap_res       = map_resolution_;
    }

    feedback->status = "validating";
    feedback->elapsed_ms = 0;
    goal_handle->publish_feedback(feedback);

    // ── 2. Build agents from snapshot (no lock) ───────────────────────────
    std::vector<Agent> agents;
    agents.reserve(req->robot_ids.size());
    std::vector<uint32_t> skipped_ids;
    std::vector<uint32_t> plan_robot_ids;
    std::vector<geometry_msgs::msg::Point> plan_world_goals;
    std::unordered_map<float, GridMap> inflated_cache;

    for (size_t i = 0; i < req->robot_ids.size(); ++i) {
      const uint32_t rid = req->robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "Robot id %u >= num_robots %d, skip", rid, num_robots_);
        continue;
      }
      if (!snap_have_odom[rid]) {
        RCLCPP_WARN(get_logger(), "No odom for robot_%u yet, skip", rid);
        continue;
      }
      Agent a;
      a.id = rid;
      const auto& [sx, sy] = snap_pos[rid];
      a.start = world_to_cell(sx, sy, snap_ox, snap_oy, snap_res,
                               snap_grid.rows, snap_grid.cols);
      a.goal  = world_to_cell(req->goals[i].x, req->goals[i].y,
                               snap_ox, snap_oy, snap_res,
                               snap_grid.rows, snap_grid.cols);
      if (!validate_agent(a, rid, snap_footprint, snap_grid, snap_res,
                           snap_ox, snap_oy, inflated_cache, skipped_ids))
        continue;
      agents.push_back(a);
      plan_robot_ids.push_back(rid);
      double gwx, gwy;
      cell_to_world(a.goal, snap_ox, snap_oy, snap_res, gwx, gwy);
      plan_world_goals.push_back(geometry_msgs::msg::Point());
      plan_world_goals.back().x = gwx;
      plan_world_goals.back().y = gwy;
    }

    if (agents.empty()) {
      result->success = false;
      result->message = "No valid agents after filtering (check odom and map)";
      is_planning_ = false;
      goal_handle->succeed(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled before planning started";
      is_planning_ = false;
      goal_handle->canceled(result);
      return;
    }

    const auto plan_start = std::chrono::steady_clock::now();
    feedback->status = "planning";
    goal_handle->publish_feedback(feedback);

    // ── 3. Plan (pure computation — no shared state) ──────────────────────
    do_plan(agents, plan_robot_ids, plan_world_goals, skipped_ids, result,
            snap_grid, snap_ox, snap_oy, snap_res, goal_handle, plan_start);

    if (!result->success) {
      is_planning_ = false;
      goal_handle->succeed(result);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled after planning (paths already published)";
      is_planning_ = false;
      goal_handle->canceled(result);
      return;
    }

    is_planning_ = false;
    goal_handle->succeed(result);
  }

  // ------------------------------------------------------------------
  // Core planning and path publishing logic
  // Called from execute_goal (detached thread, uses grid snapshot) and
  // from trigger_replan (executor thread, uses member vars directly).
  // ------------------------------------------------------------------
  void do_plan(const std::vector<Agent>& agents,
               const std::vector<uint32_t>& plan_robot_ids,
               const std::vector<geometry_msgs::msg::Point>& plan_world_goals,
               const std::vector<uint32_t>& skipped_ids,
               SetGoalsAction::Result::SharedPtr res,
               GridMap& grid,          // non-const: set_map() stores a raw pointer
               double origin_x, double origin_y, double resolution,
               const std::shared_ptr<GoalHandle>& goal_handle,
               const std::chrono::steady_clock::time_point& plan_start)
  {
    RCLCPP_INFO(get_logger(), "Planning for %zu agents...", agents.size());

    solver_.set_map(&grid);
    solver_.set_movement_params(
        static_cast<float>(max_speed_),
        static_cast<float>(time_step_sec_),
        static_cast<float>(resolution),
        static_cast<float>(urgency_));

    std::vector<Path> paths;
    SolveStats stats;

    // Publish elapsed-time feedback while solver runs (progress ticks)
    // The solver itself is blocking, so feedback is sent before and after.
    const bool ok = solver_.solve(agents, paths,
                                   static_cast<float>(resolution), &stats,
                                   max_pbs_expansions_,
                                   cost_curve_,
                                   proximity_penalty_,
                                   max_astar_expansions_);

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plan_start).count();

    if (goal_handle) {
      auto feedback = std::make_shared<SetGoalsAction::Feedback>();
      feedback->status     = ok ? "publishing" : "failed";
      feedback->elapsed_ms = static_cast<uint32_t>(elapsed_ms);
      goal_handle->publish_feedback(feedback);
    }

    res->planning_time_ms   = elapsed_ms;
    res->num_agents_planned = static_cast<uint32_t>(agents.size());
    res->pbs_expansions     = static_cast<uint32_t>(stats.expansions);
    res->max_path_length    = static_cast<uint32_t>(stats.max_path_length);
    res->astar_ok_count     = static_cast<uint32_t>(stats.astar.ok_count);
    res->astar_fail_count   = static_cast<uint32_t>(stats.astar.fail_count);
    res->astar_avg_exp      = stats.astar.ok_count > 0
        ? static_cast<uint32_t>(stats.astar.ok_total_exp / stats.astar.ok_count) : 0;
    res->astar_max_exp      = static_cast<uint32_t>(stats.astar.ok_max_exp);

    if (!ok) {
      res->success = false;
      res->message = "PBS failed (" +
                     std::to_string(elapsed_ms) + " ms, " +
                     std::to_string(stats.expansions) + " expansions)";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());

      const auto& d = stats.diag;
      const char* reason_str =
          d.fail_reason == FailReason::RootPathFailed  ? "root A* failed" :
          d.fail_reason == FailReason::BranchExhausted ? "branches exhausted" :
          d.fail_reason == FailReason::MaxExpansions    ? "max expansions" : "unknown";
      RCLCPP_ERROR(get_logger(),
          "  reason: %s, max_t=%zu, branches tried=%zu failed=%zu",
          reason_str, d.max_t_used, d.branches_tried, d.branches_failed);

      if (d.fail_reason == FailReason::RootPathFailed) {
        const auto& fa = agents[d.fail_agent];
        RCLCPP_ERROR(get_logger(),
            "  root A* failed for agent %zu: start=(%zu,%zu) goal=(%zu,%zu) "
            "footprint=%.3fm inflation=%.3fm",
            fa.id, fa.start.row, fa.start.col, fa.goal.row, fa.goal.col,
            fa.footprint_radius, fa.inflation);
      }
      for (const auto& a : agents) {
        RCLCPP_DEBUG(get_logger(),
            "  agent %zu: start=(%zu,%zu) goal=(%zu,%zu) footprint=%.3fm inflation=%.3fm",
            a.id, a.start.row, a.start.col, a.goal.row, a.goal.col,
            a.footprint_radius, a.inflation);
      }
      return;
    }

    RCLCPP_INFO(get_logger(),
        "PBS solved in %.1f ms (%zu PBS exp, A*: %zu ok (avg %zu / max %zu exp), "
        "%zu failed), publishing %zu paths",
        elapsed_ms, stats.expansions,
        stats.astar.ok_count,
        stats.astar.ok_count > 0 ? stats.astar.ok_total_exp / stats.astar.ok_count : 0,
        stats.astar.ok_max_exp, stats.astar.fail_count, paths.size());

    // ── Publish paths and update plan state (brief lock for state update) ──
    {
      std::lock_guard<std::mutex> pub_lk(state_mutex_);

      last_planning_ms_ = elapsed_ms;
      last_replan_time_ = now();

      const rclcpp::Time base_time = now();
      res->path_lengths.resize(agents.size());
      std::vector<nav_msgs::msg::Path> ros_paths(agents.size());

      for (size_t i = 0; i < agents.size(); ++i) {
        const size_t robot_id = agents[i].id;
        res->path_lengths[i] = static_cast<uint32_t>(paths[i].size());
        if (robot_id >= static_cast<size_t>(num_robots_)) {
          RCLCPP_WARN(get_logger(),
              "Robot id %zu >= num_robots %d, skip publish", robot_id, num_robots_);
          continue;
        }
        ros_paths[i] = make_ros_path(paths[i], base_time, origin_x, origin_y, resolution);
        path_pubs_[robot_id]->publish(ros_paths[i]);
        RCLCPP_DEBUG(get_logger(), "Published path for robot_%zu: %zu waypoints",
                     robot_id, paths[i].size());
      }

      active_plan_.robot_ids = plan_robot_ids;
      active_plan_.goals     = plan_world_goals;
      active_plan_.ros_paths = std::move(ros_paths);
      has_active_plan_       = true;
      start_monitoring();
    }

    res->success = true;
    if (skipped_ids.empty()) {
      res->message = "OK";
    } else {
      res->message = "OK, skipped agents (blocked goal, stationary):";
      for (uint32_t id : skipped_ids) res->message += " " + std::to_string(id);
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    }
  }

  // ------------------------------------------------------------------
  // Convert PBS Path -> nav_msgs::Path with timestamps
  // ------------------------------------------------------------------
  nav_msgs::msg::Path make_ros_path(const Path& pbs_path,
                                     const rclcpp::Time& base_time,
                                     double origin_x, double origin_y,
                                     double resolution) const
  {
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = "map";
    ros_path.header.stamp    = base_time;

    for (size_t step = 0; step < pbs_path.size(); ++step) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";

      const double offset_ns = step * time_step_sec_ * 1e9;
      ps.header.stamp = rclcpp::Time(
          base_time.nanoseconds() + static_cast<int64_t>(offset_ns));

      double wx, wy;
      cell_to_world(pbs_path[step], origin_x, origin_y, resolution, wx, wy);
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.position.z = 0.0;

      if (step + 1 < pbs_path.size()) {
        double nx, ny;
        cell_to_world(pbs_path[step + 1], origin_x, origin_y, resolution, nx, ny);
        const double yaw = std::atan2(ny - wy, nx - wx);
        ps.pose.orientation.z = std::sin(yaw * 0.5);
        ps.pose.orientation.w = std::cos(yaw * 0.5);
      } else if (!ros_path.poses.empty()) {
        ps.pose.orientation = ros_path.poses.back().pose.orientation;
      } else {
        ps.pose.orientation.w = 1.0;
      }

      ros_path.poses.push_back(ps);
    }

    return ros_path;
  }

  // ------------------------------------------------------------------
  // Validate a single agent's start/goal; set footprint from snapshot.
  // Returns false if start is in wall hard-radius (skip agent entirely).
  // Sets goal = start (stationary obstacle) if goal is in hard-radius.
  // ------------------------------------------------------------------
  bool validate_agent(Agent& a, uint32_t rid,
                      const std::vector<double>& footprint_radii,
                      const GridMap& grid,
                      double resolution,
                      double origin_x, double origin_y,
                      std::unordered_map<float, GridMap>& inflated_cache,
                      std::vector<uint32_t>& skipped_ids)
  {
    const double base_radius = footprint_radii[rid] > 0.0
        ? footprint_radii[rid] : default_robot_radius_;
    a.footprint_radius = static_cast<float>(base_radius);
    a.inflation        = static_cast<float>(inflation_radius_);

    const float soft = a.footprint_radius + a.inflation;
    if (soft > 0.0f && inflated_cache.find(soft) == inflated_cache.end()) {
      inflated_cache[soft] = grid.inflate_gradient(
          a.footprint_radius, soft, static_cast<float>(resolution));
    }
    const auto& check_grid = (soft > 0.0f) ? inflated_cache[soft] : grid;

    double start_wx, start_wy, goal_wx, goal_wy;
    cell_to_world(a.start, origin_x, origin_y, resolution, start_wx, start_wy);
    cell_to_world(a.goal,  origin_x, origin_y, resolution, goal_wx,  goal_wy);

    if (check_grid.blocked[a.start.row * check_grid.cols + a.start.col]) {
      RCLCPP_DEBUG(get_logger(),
          "Agent %u: start (%.2f, %.2f) in hard-radius wall (%.3fm), skipping",
          rid, start_wx, start_wy, a.footprint_radius);
      return false;
    }
    if (check_grid.blocked[a.goal.row * check_grid.cols + a.goal.col]) {
      RCLCPP_DEBUG(get_logger(),
          "Agent %u: goal (%.2f, %.2f) in hard-radius wall (%.3fm), making stationary",
          rid, goal_wx, goal_wy, a.footprint_radius);
      a.goal = a.start;
      skipped_ids.push_back(rid);
    }
    return true;
  }

  // ------------------------------------------------------------------
  // Schedule monitoring
  // Called under state_mutex_ from execute_goal / trigger_replan.
  // Also called from executor thread (single-threaded) without lock
  // when is_planning_ == false.
  // ------------------------------------------------------------------

  void start_monitoring()  // caller holds state_mutex_ or is executor-only
  {
    if (monitor_timer_) return;
    if (replan_check_hz_ <= 0.0) return;

    RCLCPP_INFO(get_logger(),
        "Starting schedule monitor at %.1f Hz "
        "(threshold=%.2fm, cooldown=%.1fs, predict=%.2fs, stop=%s)",
        replan_check_hz_, replan_threshold_m_, replan_cooldown_sec_,
        replan_predict_sec_, replan_stop_mode_.c_str());

    monitor_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / replan_check_hz_),
        [this]() { check_schedule(); });
  }

  void stop_monitoring()  // caller holds state_mutex_
  {
    if (monitor_timer_) {
      monitor_timer_->cancel();
      monitor_timer_.reset();
    }
    has_active_plan_ = false;
  }

  // ------------------------------------------------------------------
  // Interpolate expected position from a timestamped path at time t
  // ------------------------------------------------------------------
  static std::pair<double, double> expected_position(
      const nav_msgs::msg::Path& path, const rclcpp::Time& t)
  {
    if (path.poses.empty()) return {0.0, 0.0};
    const auto& first = path.poses.front();
    const auto& last  = path.poses.back();

    if (t <= rclcpp::Time(first.header.stamp))
      return {first.pose.position.x, first.pose.position.y};
    if (t >= rclcpp::Time(last.header.stamp))
      return {last.pose.position.x, last.pose.position.y};

    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const rclcpp::Time t0(path.poses[i].header.stamp);
      const rclcpp::Time t1(path.poses[i + 1].header.stamp);
      if (t >= t0 && t < t1) {
        const double dt    = (t1 - t0).seconds();
        const double alpha = dt > 1e-9 ? (t - t0).seconds() / dt : 0.0;
        return {
            path.poses[i].pose.position.x * (1.0 - alpha) +
                path.poses[i + 1].pose.position.x * alpha,
            path.poses[i].pose.position.y * (1.0 - alpha) +
                path.poses[i + 1].pose.position.y * alpha,
        };
      }
    }
    return {last.pose.position.x, last.pose.position.y};
  }

  // ------------------------------------------------------------------
  // Periodic schedule deviation check  (executor thread)
  // ------------------------------------------------------------------
  // TODO: replanning triggers too often — robots drift from schedule.
  // Needs better path-following with temporal sync (root cause) or
  // hysteresis / less-sensitive thresholds (workaround).
  void check_schedule()
  {
    // Skip entirely if an action goal is currently executing
    if (is_planning_) return;

    // Take a non-blocking snapshot of shared state
    std::unique_lock<std::mutex> lk(state_mutex_, std::try_to_lock);
    if (!lk.owns_lock()) return;  // planning thread is updating state

    if (!has_active_plan_ || !map_ready_) return;

    const rclcpp::Time now_t = now();

    // Count robots still en-route
    size_t active_count = 0;
    for (size_t i = 0; i < active_plan_.robot_ids.size(); ++i) {
      const uint32_t rid = active_plan_.robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !have_odom_[rid]) continue;
      const auto& [ax, ay] = current_positions_[rid];
      if (std::hypot(ax - active_plan_.goals[i].x,
                     ay - active_plan_.goals[i].y) >= goal_reached_m_)
        ++active_count;
    }

    if (active_count == 0) {
      RCLCPP_INFO(get_logger(),
          "==== ALL %zu ROBOTS REACHED THEIR GOALS! Awaiting new goals ====",
          active_plan_.robot_ids.size());
      stop_monitoring();
      return;
    }

    // Static cooldown from end of last replan
    if (last_replan_time_.nanoseconds() > 0 &&
        (now_t - last_replan_time_).seconds() < replan_cooldown_sec_)
      return;

    // Detect deviations
    std::vector<uint32_t> deviated_ids;
    for (size_t i = 0; i < active_plan_.robot_ids.size(); ++i) {
      const uint32_t rid = active_plan_.robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !have_odom_[rid]) continue;

      const auto& [ax, ay] = current_positions_[rid];
      if (std::hypot(ax - active_plan_.goals[i].x,
                     ay - active_plan_.goals[i].y) < goal_reached_m_) continue;

      if (i >= active_plan_.ros_paths.size()) continue;
      const auto& path = active_plan_.ros_paths[i];
      if (path.poses.empty()) continue;

      const auto [ex, ey] = expected_position(path, now_t);
      if (std::hypot(ax - ex, ay - ey) > replan_threshold_m_) {
        RCLCPP_DEBUG(get_logger(),
            "robot_%u: deviation=%.2fm (at %.2f,%.2f expected %.2f,%.2f)",
            rid, std::hypot(ax - ex, ay - ey), ax, ay, ex, ey);
        deviated_ids.push_back(rid);
      }
    }

    if (!deviated_ids.empty()) {
      std::string ids_str;
      for (uint32_t id : deviated_ids) {
        if (!ids_str.empty()) ids_str += ", ";
        ids_str += std::to_string(id);
      }
      RCLCPP_WARN(get_logger(),
          "Schedule deviation: %zu/%zu active robots off-plan [%s], triggering replan",
          deviated_ids.size(), active_count, ids_str.c_str());
      trigger_replan(deviated_ids, lk);   // lk is already held
    }
  }

  // ------------------------------------------------------------------
  // Replanning  (executor thread, state_mutex_ already held by caller)
  // ------------------------------------------------------------------
  void trigger_replan(const std::vector<uint32_t>& deviated_ids,
                      std::unique_lock<std::mutex>& lk)
  {
    const double predict_sec = (replan_predict_sec_ < 0.0)
        ? last_planning_ms_ / 1000.0
        : replan_predict_sec_;

    const rclcpp::Time predict_time =
        now() + rclcpp::Duration::from_seconds(predict_sec);

    const std::unordered_set<uint32_t> deviated_set(deviated_ids.begin(),
                                                     deviated_ids.end());
    const bool stop_deviated = (replan_stop_mode_ == "deviated" || replan_stop_mode_ == "all");
    const bool stop_all      = (replan_stop_mode_ == "all");

    size_t stopped_count = 0;
    for (size_t i = 0; i < active_plan_.robot_ids.size(); ++i) {
      const uint32_t rid = active_plan_.robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      const bool is_deviated = deviated_set.count(rid) > 0;
      if (stop_all || (stop_deviated && is_deviated)) {
        nav_msgs::msg::Path empty;
        empty.header.frame_id = "map";
        empty.header.stamp = now();
        path_pubs_[rid]->publish(empty);
        ++stopped_count;
      }
    }

    RCLCPP_INFO(get_logger(),
        "Replanning: %zu deviated, %zu stopped (mode=%s), predict_offset=%.3fs",
        deviated_ids.size(), stopped_count, replan_stop_mode_.c_str(), predict_sec);

    std::vector<Agent> agents;
    std::vector<uint32_t> plan_robot_ids;
    std::vector<geometry_msgs::msg::Point> plan_world_goals;
    std::vector<uint32_t> skipped_ids;
    std::unordered_map<float, GridMap> inflated_cache;

    for (size_t i = 0; i < active_plan_.robot_ids.size(); ++i) {
      const uint32_t rid = active_plan_.robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !have_odom_[rid]) continue;

      Agent a;
      a.id = rid;

      const auto& [ox, oy] = current_positions_[rid];
      const double gx = active_plan_.goals[i].x;
      const double gy = active_plan_.goals[i].y;
      const bool is_deviated = deviated_set.count(rid) > 0;

      double sx, sy;
      if (std::hypot(ox - gx, oy - gy) < goal_reached_m_) {
        sx = ox; sy = oy;
        a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows, grid_.cols);
        a.goal  = world_to_cell(gx, gy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows, grid_.cols);
      } else if (is_deviated || stop_all) {
        sx = ox; sy = oy;
        a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows, grid_.cols);
        a.goal  = world_to_cell(gx, gy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows, grid_.cols);
      } else {
        if (i < active_plan_.ros_paths.size() && !active_plan_.ros_paths[i].poses.empty())
          std::tie(sx, sy) = expected_position(active_plan_.ros_paths[i], predict_time);
        else
          sx = ox, sy = oy;
        a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows, grid_.cols);
        a.goal  = world_to_cell(gx, gy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows, grid_.cols);
      }

      if (!validate_agent(a, rid, footprint_radii_, grid_, map_resolution_,
                           map_origin_x_, map_origin_y_, inflated_cache, skipped_ids))
        continue;

      agents.push_back(a);
      plan_robot_ids.push_back(rid);
      double gwx, gwy;
      cell_to_world(a.goal, map_origin_x_, map_origin_y_, map_resolution_, gwx, gwy);
      plan_world_goals.push_back(geometry_msgs::msg::Point());
      plan_world_goals.back().x = gwx;
      plan_world_goals.back().y = gwy;
    }

    if (agents.empty()) {
      RCLCPP_WARN(get_logger(), "Replan: no valid agents, stopping monitor");
      stop_monitoring();
      return;
    }

    auto res = std::make_shared<SetGoalsAction::Result>();

    // Release lock during long computation, re-acquire for state update
    lk.unlock();
    do_plan(agents, plan_robot_ids, plan_world_goals, skipped_ids, res,
            grid_, map_origin_x_, map_origin_y_, map_resolution_,
            nullptr /* no goal_handle for internal replan */,
            std::chrono::steady_clock::now());
    lk.lock();

    last_replan_time_ = now();

    if (res->success) {
      RCLCPP_INFO(get_logger(), "Replan OK: %s (%.1fms, %u PBS exp)",
                  res->message.c_str(), res->planning_time_ms, res->pbs_expansions);
    } else {
      RCLCPP_ERROR(get_logger(), "Replan FAILED: %s", res->message.c_str());
    }
  }

  // ------------------------------------------------------------------
  // Member fields
  // ------------------------------------------------------------------
  int    num_robots_;
  double time_step_sec_;
  double pbs_resolution_       = 0.2;
  double default_robot_radius_ = 0.22;
  double inflation_radius_     = 0.5;

  double      replan_check_hz_      = 2.0;
  double      replan_threshold_m_   = 1.0;
  double      replan_cooldown_sec_  = 5.0;
  double      replan_predict_sec_   = -1.0;
  std::string replan_stop_mode_     = "all";
  double      goal_reached_m_       = 0.5;
  size_t      max_pbs_expansions_   = 5000;
  size_t      max_astar_expansions_ = 200000;
  CostCurve   cost_curve_           = CostCurve::Quadratic;
  int         proximity_penalty_    = 50;
  double      max_speed_            = 0.5;
  double      urgency_              = 1.0;

  bool   map_ready_       = false;
  double map_origin_x_    = 0.0;
  double map_origin_y_    = 0.0;
  double map_resolution_  = 0.2;

  GridMap   grid_;
  PBSSolver solver_;

  // Per-robot state
  std::vector<std::pair<double, double>> current_positions_;
  std::vector<bool>   have_odom_;
  std::vector<double> footprint_radii_;

  // Concurrency: protects all shared mutable state above
  std::mutex          state_mutex_;
  std::atomic<bool>   is_planning_{false};

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>        odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> footprint_subs_;

  // Action server
  rclcpp_action::Server<SetGoalsAction>::SharedPtr plan_action_server_;

  // Path publishers
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;

  // Schedule monitoring
  struct ActivePlan {
    std::vector<uint32_t>                  robot_ids;
    std::vector<geometry_msgs::msg::Point> goals;
    std::vector<nav_msgs::msg::Path>       ros_paths;
  };
  ActivePlan active_plan_;
  bool       has_active_plan_  = false;
  double     last_planning_ms_ = 0.0;
  rclcpp::Time             last_replan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

// ---------------------------------------------------------------------------
// main — uses MultiThreadedExecutor so the action server's internal
// infrastructure can run while the detached planning thread is active.
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<MapfPlannerNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
