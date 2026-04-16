// mapf_lns2_node.cpp
//
// ROS 2 action server wrapping the LNS2 MAPF solver. API-compatible with
// the PBS-based mapf_planner in iros_llm_swarm_mapf: same /swarm/set_goals
// action, same /robot_<i>/mapf_path publishers, same odometry and footprint
// subscriptions. Drop-in replacement — launch one OR the other.
//
// New behavior: warm-started replans. Each successful plan saves its grid
// paths + origin time; monitor_timer() detects deviations and calls
// trigger_replan(), which builds a warm seed from the previous plan +
// current odom and calls LNS2Solver::solve_from() when the seed is good.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "iros_llm_swarm_interfaces/action/set_goals.hpp"

#include "iros_llm_swarm_mapf/lns2/lns2_solver.hpp"
#include "iros_llm_swarm_mapf/lns2/warm_start.hpp"

using namespace std::chrono_literals;
using SetGoalsAction = iros_llm_swarm_interfaces::action::SetGoals;
using GoalHandle     = rclcpp_action::ServerGoalHandle<SetGoalsAction>;

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
    declare_parameter("warm_patch_radius_cells", 2);

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

    // ---- state ----------------------------------------------------------
    current_positions_.resize(num_robots_, {0.0, 0.0});
    have_odom_.assign(num_robots_, false);
    footprint_radii_.assign(num_robots_, 0.0);
    prev_grid_paths_.assign(num_robots_, lns2::Path{});

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
    path_pubs_.resize(num_robots_);
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

      path_pubs_[i] = create_publisher<nav_msgs::msg::Path>(
          "/robot_" + std::to_string(i) + "/mapf_path", 10);
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
  }

 private:
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
    auto fb = std::make_shared<SetGoalsAction::Feedback>();

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

    fb->status = "validating";
    gh->publish_feedback(fb);

    // Build agent list.
    // IMPORTANT: lns2::Agent::id must be a DENSE internal index
    // (0..agents.size()-1). The external robot_id is kept in plan_ids_ext.
    std::vector<lns2::Agent> agents;
    std::vector<uint32_t> plan_ids_ext;      // external robot_id per agent
    std::vector<geometry_msgs::msg::Point> plan_goals_world;

    for (std::size_t i = 0; i < req->robot_ids.size(); ++i) {
      const uint32_t rid = req->robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "robot id %u >= num_robots, skip", rid);
        continue;
      }
      if (!snap_have_odom[rid]) {
        RCLCPP_WARN(get_logger(), "no odom for robot_%u, skip", rid);
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
      if (snap_grid.is_blocked(a.start) || snap_grid.is_blocked(a.goal)) {
        RCLCPP_WARN(get_logger(),
            "robot_%u start or goal is blocked, skip", rid);
        continue;
      }
      agents.push_back(std::move(a));
      plan_ids_ext.push_back(rid);
      plan_goals_world.push_back(req->goals[i]);
    }

    if (agents.empty()) {
      result->success    = false;
      result->message    = "No valid agents";
      result->error_code = SetGoalsAction::Result::NO_VALID_AGENTS;
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

    // Block cells of robots not in the plan (physically present obstacles)
    block_skipped_robots(snap_grid, plan_ids_ext, snap_pos, snap_have_odom,
                          snap_fp_radii, snap_ox, snap_oy, snap_res);

    // Run solver (cold)
    fb->status = "planning";
    gh->publish_feedback(fb);

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

    // Publish & remember plan
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      publish_paths(paths, plan_ids_ext, snap_ox, snap_oy, snap_res);
      store_active_plan(paths, plan_ids_ext, plan_goals_world, now());
      active_goal_handle_ = gh;
      start_monitoring();
    }

    result->success = true;
    if (result->message.empty()) result->message = "OK";
    RCLCPP_INFO(get_logger(),
                "LNS2 initial plan: %zu agents, %.1fms, iters=%zu, "
                "init_coll=%zu -> 0",
                agents.size(), stats.wall_time_ms,
                stats.iterations, stats.initial_collisions);

    fb->status     = "executing";
    fb->elapsed_ms = static_cast<uint32_t>(elapsed_ms);
    fb->robots_active = static_cast<uint32_t>(agents.size());
    gh->publish_feedback(fb);
    // is_active_ stays true until check_schedule sees all arrived.
  }

  // --------------------------------------------------------------------
  // Monitoring + warm-start replan
  // --------------------------------------------------------------------
  void start_monitoring()
  {
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
    for (uint32_t rid : active_robot_ids_) {
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      nav_msgs::msg::Path empty;
      empty.header.frame_id = "map";
      empty.header.stamp    = now();
      path_pubs_[rid]->publish(empty);
    }
  }

  void check_schedule()
  {
    if (is_planning_.load()) return;

    std::unique_lock<std::mutex> lk(state_mutex_);
    if (!active_goal_handle_ || active_robot_ids_.empty()) return;

    // 1. Check arrivals
    bool all_arrived = true;
    std::vector<uint32_t> deviated;
    for (std::size_t i = 0; i < active_robot_ids_.size(); ++i) {
      const uint32_t rid = active_robot_ids_[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !have_odom_[rid]) continue;
      const auto& [cx, cy] = current_positions_[rid];
      const double gx = active_goals_world_[i].x;
      const double gy = active_goals_world_[i].y;
      if (std::hypot(cx - gx, cy - gy) > goal_reached_m_) all_arrived = false;

      // Deviation check
      const auto& ros_path = active_ros_paths_[i];
      if (!ros_path.poses.empty()) {
        double ex, ey;
        expected_position(ros_path, now(), ex, ey);
        if (std::hypot(cx - ex, cy - ey) > replan_threshold_m_) {
          deviated.push_back(rid);
        }
      }
    }

    if (all_arrived) {
      auto result = std::make_shared<SetGoalsAction::Result>();
      result->success = true;
      result->message = "All agents arrived";
      auto gh = active_goal_handle_;
      active_goal_handle_.reset();
      active_robot_ids_.clear();
      active_ros_paths_.clear();
      active_goals_world_.clear();
      stop_monitoring();
      is_active_ = false;
      lk.unlock();
      gh->succeed(result);
      return;
    }

    // 2. Replan on deviation (with cooldown)
    if (!deviated.empty() &&
        (now() - last_replan_time_).seconds() > replan_cooldown_sec_) {
      RCLCPP_INFO(get_logger(), "deviation detected on %zu robots -> replan",
                  deviated.size());
      lk.unlock();
      trigger_replan();
      return;
    }
  }

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

    for (std::size_t i = 0; i < snap_ids.size(); ++i) {
      const uint32_t rid = snap_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !snap_odom[rid]) continue;
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
      if (snap_grid.is_blocked(a.start) || snap_grid.is_blocked(a.goal)) continue;
      agents.push_back(std::move(a));
      active_ext_ids.push_back(rid);
      active_slot.push_back(i);
    }

    if (agents.empty()) {
      RCLCPP_WARN(get_logger(), "replan: no valid agents");
      is_planning_ = false;
      return;
    }

    // Block cells of robots not in the plan (physically present obstacles)
    block_skipped_robots(snap_grid, active_ext_ids, snap_pos, snap_odom,
                          snap_fp, snap_ox, snap_oy, snap_res);

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

      // ── KEY: if the warm seed is already collision-free, the current
      //    paths are still valid. Do NOT republish — that would reset
      //    path_follower and stop robot motion.
      if (rep.seed_collisions == 0) {
        RCLCPP_DEBUG(get_logger(),
            "warm seed collision-free (shifted=%zu patched=%zu stubbed=%zu) "
            "— skipping replan, current paths are valid",
            rep.agents_shifted_exact, rep.agents_patched, rep.agents_stubbed);
        is_planning_ = false;
        return;
      }

      if (lns2::should_warm_start(rep, agents.size())) {
        params = make_params(/*warm=*/true);
        const bool ok = solver_.solve_from(std::move(seed), agents, snap_grid,
                                            params, &paths, &stats);
        used_warm = true;
        RCLCPP_INFO(get_logger(),
          "warm replan: shifted=%zu patched=%zu stubbed=%zu init_coll=%zu "
          "-> %zu, iters=%zu, %.1fms, ok=%d",
          rep.agents_shifted_exact, rep.agents_patched,
          rep.agents_stubbed, rep.seed_collisions, stats.final_collisions,
          stats.iterations, stats.wall_time_ms, ok);
      } else {
        RCLCPP_INFO(get_logger(),
          "warm seed rejected (coll=%zu stubs=%zu) -> cold replan",
          rep.seed_collisions, rep.agents_stubbed);
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
    if (!succeeded) {
      RCLCPP_WARN(get_logger(),
          "replan did not reach collision-free; keeping old paths");
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

    // Commit: publish new paths, update active plan
    lk.lock();
    publish_paths(paths, active_ext_ids, snap_ox, snap_oy, snap_res);
    store_active_plan(paths, active_ext_ids, planned_goals, now());
    last_replan_time_ = now();
    is_planning_ = false;
    (void)gh;
  }

  // --------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------

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

    for (int rid = 0; rid < num_robots_; ++rid) {
      if (planned_set.count(static_cast<uint32_t>(rid))) continue;
      if (!have_odom[rid]) continue;

      const auto& [sx, sy] = positions[rid];
      lns2::Cell c = world_to_cell(sx, sy, ox, oy, res, grid.rows, grid.cols);
      const double radius = fp_radii[rid] > 0 ? fp_radii[rid]
                                               : default_robot_radius_;
      auto fp = lns2::FootprintModel::from_radius(
          radius + inflation_radius_, res);
      for (const auto& off : fp.offsets) {
        lns2::Cell fc = c + off;
        if (grid.in_bounds(fc)) {
          grid.blocked[grid.index_of(fc)] = 1;
        }
      }
      RCLCPP_DEBUG(get_logger(),
          "Blocked cells for non-participating robot_%d at (%d,%d)",
          rid, c.row, c.col);
    }
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

  // `paths[k]` is the plan for agent whose external robot_id is
  // `plan_ids_ext[k]`. The internal agent.id is k.
  void publish_paths(const std::vector<lns2::Path>& paths,
                      const std::vector<uint32_t>& plan_ids_ext,
                      double ox, double oy, double res)
  {
    const rclcpp::Time base = now();
    for (std::size_t k = 0; k < paths.size() && k < plan_ids_ext.size(); ++k) {
      const uint32_t rid = plan_ids_ext[k];
      if (rid >= static_cast<uint32_t>(num_robots_)) continue;
      nav_msgs::msg::Path rp;
      rp.header.frame_id = "map";
      rp.header.stamp    = base;
      rp.poses.reserve(paths[k].size());
      for (std::size_t t = 0; t < paths[k].size(); ++t) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = base + rclcpp::Duration::from_seconds(
            static_cast<double>(t) * time_step_sec_);
        double wx, wy;
        cell_to_world(paths[k][t], ox, oy, res, wx, wy);
        ps.pose.position.x = wx;
        ps.pose.position.y = wy;
        ps.pose.orientation.w = 1.0;
        rp.poses.push_back(ps);
      }
      path_pubs_[rid]->publish(rp);
    }
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
    active_ros_paths_.assign(paths.size(), nav_msgs::msg::Path{});

    // Save grid paths indexed by EXTERNAL robot_id for warm-start reuse.
    prev_grid_paths_.assign(num_robots_, lns2::Path{});
    for (std::size_t k = 0; k < paths.size() && k < plan_ids_ext.size(); ++k) {
      const uint32_t rid = plan_ids_ext[k];
      if (rid < static_cast<uint32_t>(num_robots_)) {
        prev_grid_paths_[rid] = paths[k];
      }

      // Rebuild ros_path for deviation checks
      nav_msgs::msg::Path rp;
      rp.header.frame_id = "map";
      rp.header.stamp    = plan_time;
      rp.poses.reserve(paths[k].size());
      for (std::size_t t = 0; t < paths[k].size(); ++t) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = plan_time + rclcpp::Duration::from_seconds(
            static_cast<double>(t) * time_step_sec_);
        double wx, wy;
        cell_to_world(paths[k][t], map_origin_x_, map_origin_y_,
                      map_resolution_, wx, wy);
        ps.pose.position.x = wx;
        ps.pose.position.y = wy;
        ps.pose.orientation.w = 1.0;
        rp.poses.push_back(ps);
      }
      active_ros_paths_[k] = std::move(rp);
    }

    plan_origin_time_ = plan_time;
    last_replan_time_ = plan_time;
  }

  // Linearly interpolate expected position at time `t`.
  void expected_position(const nav_msgs::msg::Path& p, const rclcpp::Time& t,
                          double& x, double& y) const
  {
    if (p.poses.empty()) { x = y = 0.0; return; }
    if (p.poses.size() == 1 || t < rclcpp::Time(p.poses.front().header.stamp)) {
      x = p.poses.front().pose.position.x;
      y = p.poses.front().pose.position.y;
      return;
    }
    if (t >= rclcpp::Time(p.poses.back().header.stamp)) {
      x = p.poses.back().pose.position.x;
      y = p.poses.back().pose.position.y;
      return;
    }
    for (std::size_t i = 0; i + 1 < p.poses.size(); ++i) {
      const rclcpp::Time t0(p.poses[i].header.stamp);
      const rclcpp::Time t1(p.poses[i + 1].header.stamp);
      if (t >= t0 && t <= t1) {
        const double dt = (t1 - t0).seconds();
        const double a = dt > 1e-6 ? (t - t0).seconds() / dt : 0.0;
        x = p.poses[i].pose.position.x + a *
            (p.poses[i + 1].pose.position.x - p.poses[i].pose.position.x);
        y = p.poses[i].pose.position.y + a *
            (p.poses[i + 1].pose.position.y - p.poses[i].pose.position.y);
        return;
      }
    }
    x = p.poses.back().pose.position.x;
    y = p.poses.back().pose.position.y;
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

  // Map / grid
  bool   map_ready_ = false;
  double map_origin_x_ = 0.0, map_origin_y_ = 0.0, map_resolution_ = 0.2;
  lns2::GridMap grid_;

  // Per-robot state
  std::vector<std::pair<double, double>> current_positions_;
  std::vector<bool>   have_odom_;
  std::vector<double> footprint_radii_;

  // Active plan
  std::vector<lns2::Path> prev_grid_paths_;
  std::vector<uint32_t>   active_robot_ids_;
  std::vector<geometry_msgs::msg::Point> active_goals_world_;
  std::vector<nav_msgs::msg::Path>       active_ros_paths_;
  rclcpp::Time plan_origin_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};

  // Concurrency
  std::mutex          state_mutex_;
  std::atomic<bool>   is_planning_{false};
  std::atomic<bool>   is_active_{false};

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>     odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> footprint_subs_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>             path_pubs_;
  rclcpp_action::Server<SetGoalsAction>::SharedPtr plan_action_server_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  std::shared_ptr<GoalHandle>  active_goal_handle_;

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