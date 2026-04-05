#include <chrono>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

// Наш solver (header-only)
#include "iros_llm_swarm_mapf/pbs_solver.hpp"

// Сервис планирования
#include "iros_llm_swarm_interfaces/srv/set_goals.hpp"

// ---------------------------------------------------------------------------
// Планировщик принимает сервис /swarm/set_goals с robot_ids и целями.
// Стартовые позиции берутся из одометрии (/robot_N/odom).
// Футпринты берутся из Nav2 (/robot_N/local_costmap/published_footprint).
//
// Сервис возвращает:
//   success, message, planning_time_ms, num_agents_planned,
//   pbs_expansions, max_path_length, path_lengths[]
//
// После успешного планирования нода публикует пути в топики:
//   /robot_0/mapf_path, /robot_1/mapf_path, ...  (nav_msgs/Path)
//
// Каждый PoseStamped в пути содержит временну́ю метку:
//   stamp = t_start + step_index * time_step_sec
//
// Path follower должен добраться до waypoint[k] и подождать
// до stamp[k] перед движением к waypoint[k+1].
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Конвертация координат
// ---------------------------------------------------------------------------

static Cell world_to_cell(double wx, double wy,
                           double origin_x, double origin_y,
                           double resolution,
                           size_t rows) {
  // OccupancyGrid: origin = левый нижний угол.
  // row = 0 соответствует y = origin_y (низ карты).
  const size_t col = static_cast<size_t>((wx - origin_x) / resolution);
  const size_t row = static_cast<size_t>((wy - origin_y) / resolution);
  // Ограничиваем на всякий случай
  return {std::min(row, rows - 1), col};
}

static void cell_to_world(const Cell& c,
                           double origin_x, double origin_y,
                           double resolution,
                           double& wx, double& wy) {
  wx = origin_x + (c.col + 0.5) * resolution;
  wy = origin_y + (c.row + 0.5) * resolution;
}

// ---------------------------------------------------------------------------
// Нода
// ---------------------------------------------------------------------------

class MapfPlannerNode : public rclcpp::Node {
 public:
  MapfPlannerNode()
      : Node("mapf_planner"),
        solver_(HeuristicType::Manhattan) {

    using SetGoals = iros_llm_swarm_interfaces::srv::SetGoals;

    // Параметры
    declare_parameter("num_robots",          20);
    declare_parameter("time_step_sec",       0.1);
    declare_parameter("map_topic",           std::string("/map"));
    // PBS планирует на грубой сетке. 0.2м/клетку -> 150x150 вместо 600x600.
    // Увеличь если медленно, уменьши для точности.
    declare_parameter("pbs_resolution",      0.2);
    // Радиус по умолчанию если от Nav2 ещё не пришёл footprint
    declare_parameter("default_robot_radius", 0.22);
    // Дополнительный запас поверх footprint radius для PBS.
    // Nav2 inflation_radius (0.55м) — это мягкий градиент, не жёсткий барьер.
    // Здесь достаточно небольшого буфера для безопасности.
    declare_parameter("inflation_radius", 0.2);
    // Schedule monitoring & replanning
    declare_parameter("replan_check_hz",      2.0);
    declare_parameter("replan_threshold_m",   1.0);
    declare_parameter("replan_cooldown_sec",  5.0);
    declare_parameter("replan_cooldown_factor", 3.0);
    declare_parameter("replan_predict_sec",  -1.0);
    declare_parameter("replan_stop_mode",     std::string("deviated"));
    declare_parameter("goal_reached_m",       0.5);

    num_robots_           = get_parameter("num_robots").as_int();
    time_step_sec_        = get_parameter("time_step_sec").as_double();
    pbs_resolution_       = get_parameter("pbs_resolution").as_double();
    default_robot_radius_ = get_parameter("default_robot_radius").as_double();
    inflation_radius_     = get_parameter("inflation_radius").as_double();
    replan_check_hz_      = get_parameter("replan_check_hz").as_double();
    replan_threshold_m_   = get_parameter("replan_threshold_m").as_double();
    replan_cooldown_sec_  = get_parameter("replan_cooldown_sec").as_double();
    replan_cooldown_factor_ = get_parameter("replan_cooldown_factor").as_double();
    replan_predict_sec_   = get_parameter("replan_predict_sec").as_double();
    replan_stop_mode_     = get_parameter("replan_stop_mode").as_string();
    if (replan_stop_mode_ != "none" && replan_stop_mode_ != "deviated" &&
        replan_stop_mode_ != "all") {
      RCLCPP_ERROR(get_logger(),
          "Invalid replan_stop_mode '%s', must be 'none', 'deviated', or 'all'. "
          "Falling back to 'deviated'.", replan_stop_mode_.c_str());
      replan_stop_mode_ = "deviated";
    }
    goal_reached_m_       = get_parameter("goal_reached_m").as_double();

    // Подписка на карту.
    // map_server публикует с transient_local — подписчик обязан использовать
    // тот же QoS, иначе уже опубликованная карта не придёт никогда.
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .transient_local()
        .reliable();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        get_parameter("map_topic").as_string(), map_qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          on_map(msg);
        });

    // Подписки на одометрию и футпринты для каждого робота
    current_positions_.resize(num_robots_, {0.0, 0.0});
    have_odom_.resize(num_robots_, false);
    footprint_radii_.resize(num_robots_, 0.0);

    odom_subs_.resize(num_robots_);
    footprint_subs_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
      // Одометрия — стартовые позиции
      const std::string odom_topic = "/robot_" + std::to_string(i) + "/odom";
      odom_subs_[i] = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 10,
          [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_positions_[i] = {msg->pose.pose.position.x,
                                      msg->pose.pose.position.y};
            have_odom_[i] = true;
          });

      // Футпринт от Nav2 costmap — всегда PolygonStamped (даже для robot_radius).
      // Вычисляем bounding radius: max(hypot(p.x, p.y)) по вершинам.
      const std::string fp_topic =
          "/robot_" + std::to_string(i) + "/local_costmap/published_footprint";
      footprint_subs_[i] = create_subscription<geometry_msgs::msg::PolygonStamped>(
          fp_topic, rclcpp::QoS(1).transient_local(),
          [this, i](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
            double max_r = 0.0;
            for (const auto& p : msg->polygon.points) {
              const double r = std::hypot(p.x, p.y);
              if (r > max_r) max_r = r;
            }
            footprint_radii_[i] = max_r;
          });
    }

    // Сервис планирования
    plan_srv_ = create_service<SetGoals>(
        "/swarm/set_goals",
        [this](const SetGoals::Request::SharedPtr req,
               SetGoals::Response::SharedPtr res) {
          on_set_goals(req, res);
        });

    // Паблишеры путей — по одному на каждого робота
    path_pubs_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
      const std::string topic = "/robot_" + std::to_string(i) + "/mapf_path";
      path_pubs_[i] = create_publisher<nav_msgs::msg::Path>(topic, 10);
    }

    RCLCPP_INFO(get_logger(),
        "mapf_planner ready: %d robots, time_step=%.3f s, "
        "default_radius=%.3f m, inflation=%.3f m (effective=%.3f m), "
        "replan: %.1f Hz, threshold=%.2f m, cooldown=%.1f s (factor=%.1f), "
        "predict=%.2f s, stop_mode=%s",
        num_robots_, time_step_sec_, default_robot_radius_,
        inflation_radius_, default_robot_radius_ + inflation_radius_,
        replan_check_hz_, replan_threshold_m_, replan_cooldown_sec_,
        replan_cooldown_factor_, replan_predict_sec_,
        replan_stop_mode_.c_str());
  }

 private:
  // ------------------------------------------------------------------
  // Обработка карты
  // ------------------------------------------------------------------
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    const double src_res = msg->info.resolution;
    // Сколько исходных клеток входит в одну PBS-клетку
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

    // Ячейка PBS занята если ЛЮБАЯ исходная клетка внутри неё занята
    for (int sr = 0; sr < src_h; ++sr) {
      for (int sc = 0; sc < src_w; ++sc) {
        const int8_t v = msg->data[static_cast<size_t>(sr * src_w + sc)];
        if (v > 50 || v < 0) {
          const int dr = sr / ratio;
          const int dc = sc / ratio;
          grid_.blocked[static_cast<size_t>(dr * dst_w + dc)] = 1;
        }
      }
    }

    map_origin_x_   = msg->info.origin.position.x;
    map_origin_y_   = msg->info.origin.position.y;
    map_resolution_ = actual_res;   // PBS использует огрублённый resolution
    map_ready_      = true;
  }

  // ------------------------------------------------------------------
  // Обработка сервиса /swarm/set_goals
  // ------------------------------------------------------------------
  void on_set_goals(
      const iros_llm_swarm_interfaces::srv::SetGoals::Request::SharedPtr req,
      iros_llm_swarm_interfaces::srv::SetGoals::Response::SharedPtr res) {

    // New external request cancels any active schedule monitoring
    stop_monitoring();

    // Валидация запроса
    if (!map_ready_) {
      res->success = false;
      res->message = "Map not ready yet";
      return;
    }
    if (req->robot_ids.size() != req->goals.size()) {
      res->success = false;
      res->message = "robot_ids and goals must have the same length";
      return;
    }
    if (req->robot_ids.empty()) {
      res->success = false;
      res->message = "robot_ids is empty";
      return;
    }

    // Собираем агентов из одометрии, футпринтов и целей запроса.
    // Агенты с заблокированной целью включаются как стационарные препятствия
    // (start = goal = текущая позиция), чтобы PBS обходил их.
    std::vector<Agent> agents;
    agents.reserve(req->robot_ids.size());
    std::vector<uint32_t> skipped_ids;  // агенты с заблокированными целями
    std::vector<uint32_t> plan_robot_ids;
    std::vector<geometry_msgs::msg::Point> plan_world_goals;

    // Кэш inflated-карт для валидации start/goal
    // (те же карты будут пересозданы в solver, но нам нужны уже здесь)
    std::unordered_map<float, GridMap> inflated_cache;

    for (size_t i = 0; i < req->robot_ids.size(); ++i) {
      const uint32_t rid = req->robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "Robot id %u >= num_robots %d, skip", rid, num_robots_);
        continue;
      }
      if (!have_odom_[rid]) {
        RCLCPP_WARN(get_logger(), "No odom for robot_%u yet, skip", rid);
        continue;
      }

      Agent a;
      a.id = rid;

      // Старт из одометрии
      const auto& [sx, sy] = current_positions_[rid];
      a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                               map_resolution_, grid_.rows);

      // Цель из запроса
      a.goal = world_to_cell(req->goals[i].x, req->goals[i].y,
                              map_origin_x_, map_origin_y_,
                              map_resolution_, grid_.rows);

      if (!validate_agent(a, rid, inflated_cache, skipped_ids)) continue;

      agents.push_back(a);
      plan_robot_ids.push_back(rid);
      plan_world_goals.push_back(req->goals[i]);
    }

    if (agents.empty()) {
      res->success = false;
      res->message = "No valid agents after filtering (check odom and map)";
      return;
    }

    // Планируем и публикуем пути
    do_plan(agents, plan_robot_ids, plan_world_goals, skipped_ids, res);
  }

  // ------------------------------------------------------------------
  // Общая логика планирования и публикации
  // ------------------------------------------------------------------
  void do_plan(const std::vector<Agent>& agents,
               const std::vector<uint32_t>& plan_robot_ids,
               const std::vector<geometry_msgs::msg::Point>& plan_world_goals,
               const std::vector<uint32_t>& skipped_ids,
               iros_llm_swarm_interfaces::srv::SetGoals::Response::SharedPtr res) {
    RCLCPP_INFO(get_logger(), "Planning for %zu agents...", agents.size());

    solver_.set_map(&grid_);

    std::vector<Path> paths;
    SolveStats stats;
    const auto t0 = now();

    const bool ok = solver_.solve(agents, paths,
                                   static_cast<float>(map_resolution_), &stats);

    const double elapsed_ms = (now() - t0).nanoseconds() / 1.0e6;

    // Заполняем ответ
    res->planning_time_ms    = elapsed_ms;
    res->num_agents_planned  = static_cast<uint32_t>(agents.size());
    res->pbs_expansions      = static_cast<uint32_t>(stats.expansions);
    res->max_path_length     = static_cast<uint32_t>(stats.max_path_length);

    if (!ok) {
      res->success = false;
      res->message = "PBS failed (" +
                     std::to_string(elapsed_ms) + " ms, " +
                     std::to_string(stats.expansions) + " expansions)";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      // Диагностика: логируем каждого агента для отладки
      for (const auto& a : agents) {
        RCLCPP_WARN(get_logger(),
            "  agent %zu: start=(%zu,%zu) goal=(%zu,%zu) radius=%.3fm",
            a.id, a.start.row, a.start.col, a.goal.row, a.goal.col,
            a.footprint_radius);
      }
      return;
    }

    RCLCPP_INFO(get_logger(),
        "PBS solved in %.1f ms (%zu expansions), publishing %zu paths",
        elapsed_ms, stats.expansions, paths.size());

    last_planning_ms_ = elapsed_ms;

    // Публикуем пути и собираем длины для ответа
    const rclcpp::Time base_time = now();
    res->path_lengths.resize(agents.size());
    std::vector<nav_msgs::msg::Path> ros_paths(agents.size());

    for (size_t i = 0; i < agents.size(); ++i) {
      const size_t robot_id = agents[i].id;
      res->path_lengths[i] = static_cast<uint32_t>(paths[i].size());

      if (robot_id >= static_cast<size_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "Robot id %zu >= num_robots %d, skip publish",
                    robot_id, num_robots_);
        continue;
      }

      ros_paths[i] = make_ros_path(paths[i], base_time);
      path_pubs_[robot_id]->publish(ros_paths[i]);

      RCLCPP_DEBUG(get_logger(),
          "Published path for robot_%zu: %zu waypoints", robot_id, paths[i].size());
    }

    // Store active plan for schedule monitoring
    active_plan_.robot_ids  = plan_robot_ids;
    active_plan_.goals      = plan_world_goals;
    active_plan_.ros_paths  = std::move(ros_paths);
    has_active_plan_ = true;
    start_monitoring();

    res->success = true;
    if (skipped_ids.empty()) {
      res->message = "OK";
    } else {
      res->message = "OK, skipped agents (blocked goal, stationary):";
      for (uint32_t id : skipped_ids) res->message += " " + std::to_string(id);
    }
  }

  // ------------------------------------------------------------------
  // Конвертация Path -> nav_msgs::Path с временны́ми метками
  // ------------------------------------------------------------------
  nav_msgs::msg::Path make_ros_path(const Path& pbs_path,
                                     const rclcpp::Time& base_time) const {
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = "map";
    ros_path.header.stamp    = base_time;

    for (size_t step = 0; step < pbs_path.size(); ++step) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";

      // Временна́я метка: base + step * time_step
      const double offset_ns = step * time_step_sec_ * 1e9;
      ps.header.stamp = rclcpp::Time(
          base_time.nanoseconds() + static_cast<int64_t>(offset_ns));

      // Позиция: центр ячейки
      double wx, wy;
      cell_to_world(pbs_path[step],
                    map_origin_x_, map_origin_y_, map_resolution_,
                    wx, wy);
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.position.z = 0.0;

      // Ориентация: смотрим в сторону следующей точки
      if (step + 1 < pbs_path.size()) {
        double nx, ny;
        cell_to_world(pbs_path[step + 1],
                      map_origin_x_, map_origin_y_, map_resolution_,
                      nx, ny);
        const double yaw = std::atan2(ny - wy, nx - wx);
        ps.pose.orientation.z = std::sin(yaw * 0.5);
        ps.pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        // последняя точка — ориентация как у предыдущей
        if (!ros_path.poses.empty()) {
          ps.pose.orientation = ros_path.poses.back().pose.orientation;
        } else {
          ps.pose.orientation.w = 1.0;
        }
      }

      ros_path.poses.push_back(ps);
    }

    return ros_path;
  }

  // ------------------------------------------------------------------
  // Compute footprint, inflate map, validate start/goal for a single agent.
  // Caller must set a.start and a.goal before calling.
  //
  // Footprint: uses Nav2-published bounding radius if available, otherwise
  // default_robot_radius. Adds inflation_radius as a safety buffer beyond
  // the footprint (Nav2's own inflation is a soft gradient, not a hard wall).
  //
  // If the goal cell is blocked on the inflated map, the agent is turned
  // into a stationary obstacle (start = goal = current position) so that
  // PBS routes other robots around it.
  //
  // Returns false if agent should be skipped entirely (start blocked).
  // ------------------------------------------------------------------
  bool validate_agent(Agent& a, uint32_t rid,
                      std::unordered_map<float, GridMap>& inflated_cache,
                      std::vector<uint32_t>& skipped_ids)
  {
    const double base_radius = footprint_radii_[rid] > 0.0
        ? footprint_radii_[rid] : default_robot_radius_;
    a.footprint_radius = static_cast<float>(base_radius + inflation_radius_);

    const float fr = a.footprint_radius;
    if (fr > 0.0f && inflated_cache.find(fr) == inflated_cache.end()) {
      inflated_cache[fr] = grid_.inflate(fr, static_cast<float>(map_resolution_));
    }
    const auto& check_grid = (fr > 0.0f) ? inflated_cache[fr] : grid_;

    // Convert cells back to world coords for diagnostics
    double start_wx, start_wy, goal_wx, goal_wy;
    cell_to_world(a.start, map_origin_x_, map_origin_y_, map_resolution_,
                  start_wx, start_wy);
    cell_to_world(a.goal, map_origin_x_, map_origin_y_, map_resolution_,
                  goal_wx, goal_wy);

    if (check_grid.blocked[a.start.row * check_grid.cols + a.start.col]) {
      RCLCPP_WARN(get_logger(),
          "Agent %u: start (%.2f, %.2f) -> cell (%zu, %zu) blocked on inflated map "
          "(r=%.3fm), skipping entirely",
          rid, start_wx, start_wy, a.start.row, a.start.col, fr);
      return false;
    }
    if (check_grid.blocked[a.goal.row * check_grid.cols + a.goal.col]) {
      RCLCPP_WARN(get_logger(),
          "Agent %u: goal (%.2f, %.2f) -> cell (%zu, %zu) blocked on inflated map "
          "(r=%.3fm), including as stationary obstacle",
          rid, goal_wx, goal_wy, a.goal.row, a.goal.col, fr);
      a.goal = a.start;
      skipped_ids.push_back(rid);
    }
    return true;
  }

  // ------------------------------------------------------------------
  // Schedule monitoring
  // ------------------------------------------------------------------

  void start_monitoring()
  {
    if (monitor_timer_) return;          // already running
    if (replan_check_hz_ <= 0.0) return; // monitoring disabled

    RCLCPP_INFO(get_logger(),
        "Starting schedule monitor at %.1f Hz "
        "(threshold=%.2fm, cooldown=%.1fs, factor=%.1f, predict=%.2fs, stop=%s)",
        replan_check_hz_, replan_threshold_m_, replan_cooldown_sec_,
        replan_cooldown_factor_, replan_predict_sec_, replan_stop_mode_.c_str());

    monitor_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / replan_check_hz_),
        [this]() { check_schedule(); });
  }

  void stop_monitoring()
  {
    if (monitor_timer_) {
      monitor_timer_->cancel();
      monitor_timer_.reset();
    }
    has_active_plan_ = false;
  }

  // Interpolate expected position from a timestamped path at a given time
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
        const double dt = (t1 - t0).seconds();
        const double alpha = dt > 1e-9 ? (t - t0).seconds() / dt : 0.0;
        const double x = path.poses[i].pose.position.x * (1.0 - alpha)
                       + path.poses[i + 1].pose.position.x * alpha;
        const double y = path.poses[i].pose.position.y * (1.0 - alpha)
                       + path.poses[i + 1].pose.position.y * alpha;
        return {x, y};
      }
    }

    return {last.pose.position.x, last.pose.position.y};
  }

  void check_schedule()
  {
    if (!has_active_plan_ || !map_ready_) return;

    const rclcpp::Time now_t = now();

    // Adaptive cooldown: max(configured, factor * last_planning_time)
    const double adaptive_cd = replan_cooldown_factor_ * last_planning_ms_ / 1000.0;
    const double eff_cooldown = std::max(replan_cooldown_sec_, adaptive_cd);
    if (last_replan_time_.nanoseconds() > 0 &&
        (now_t - last_replan_time_).seconds() < eff_cooldown) {
      return;
    }

    std::vector<uint32_t> deviated_ids;
    size_t active_count = 0;

    for (size_t i = 0; i < active_plan_.robot_ids.size(); ++i) {
      const uint32_t rid = active_plan_.robot_ids[i];
      if (rid >= static_cast<uint32_t>(num_robots_) || !have_odom_[rid]) continue;

      const auto& [ax, ay] = current_positions_[rid];
      const double gx = active_plan_.goals[i].x;
      const double gy = active_plan_.goals[i].y;

      // Robot reached goal?
      if (std::hypot(ax - gx, ay - gy) < goal_reached_m_) continue;

      ++active_count;

      // Check deviation from schedule
      if (i >= active_plan_.ros_paths.size()) continue;
      const auto& path = active_plan_.ros_paths[i];
      if (path.poses.empty()) continue;

      const auto [ex, ey] = expected_position(path, now_t);
      const double deviation = std::hypot(ax - ex, ay - ey);

      if (deviation > replan_threshold_m_) {
        RCLCPP_WARN(get_logger(),
            "robot_%u deviates %.2fm from schedule (threshold %.2fm)",
            rid, deviation, replan_threshold_m_);
        deviated_ids.push_back(rid);
      }
    }

    if (active_count == 0) {
      RCLCPP_INFO(get_logger(), "All robots reached goals, stopping schedule monitor");
      stop_monitoring();
      return;
    }

    if (!deviated_ids.empty()) {
      trigger_replan(deviated_ids);
    }
  }

  // ------------------------------------------------------------------
  // Replanning
  //
  // Called when check_schedule() detects robots deviating from their
  // PBS paths. Depending on replan_stop_mode:
  //   "none"     — no robots are stopped; deviated use current odom,
  //                on-schedule use predicted future positions as starts
  //   "deviated" — only deviated robots are stopped (empty path cancel);
  //                deviated use odom, on-schedule use prediction
  //   "all"      — all active robots are stopped; everyone uses odom
  //
  // Arrived robots (within goal_reached_m) are included as stationary
  // obstacles. Starts/goals are validated against the inflated map.
  // After planning, new paths are published and schedule monitoring
  // continues with the updated plan.
  // ------------------------------------------------------------------

  void trigger_replan(const std::vector<uint32_t>& deviated_ids)
  {
    using SetGoals = iros_llm_swarm_interfaces::srv::SetGoals;

    // Effective prediction offset (adaptive: use last planning time; 0 if not measured yet)
    const double predict_sec = (replan_predict_sec_ < 0.0)
        ? last_planning_ms_ / 1000.0
        : replan_predict_sec_;

    const rclcpp::Time predict_time =
        now() + rclcpp::Duration::from_seconds(predict_sec);

    // Build set of deviated robot IDs for quick lookup
    std::unordered_set<uint32_t> deviated_set(deviated_ids.begin(),
                                               deviated_ids.end());

    // Stop robots based on stop mode
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

    // Build agents for replanning
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

      // Decide start position
      double sx, sy;
      if (std::hypot(ox - gx, oy - gy) < goal_reached_m_) {
        // Arrived: stationary obstacle at current position
        sx = ox; sy = oy;
        a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows);
        a.goal = a.start;
      } else if (is_deviated || stop_all) {
        // Stopped or deviated: use current odom (exact or close to exact)
        sx = ox; sy = oy;
        a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows);
        a.goal = world_to_cell(gx, gy, map_origin_x_, map_origin_y_,
                                map_resolution_, grid_.rows);
      } else {
        // On-schedule: predict future position from plan
        if (i < active_plan_.ros_paths.size() && !active_plan_.ros_paths[i].poses.empty()) {
          std::tie(sx, sy) = expected_position(active_plan_.ros_paths[i], predict_time);
        } else {
          sx = ox; sy = oy;
        }
        a.start = world_to_cell(sx, sy, map_origin_x_, map_origin_y_,
                                 map_resolution_, grid_.rows);
        a.goal = world_to_cell(gx, gy, map_origin_x_, map_origin_y_,
                                map_resolution_, grid_.rows);
      }

      if (!validate_agent(a, rid, inflated_cache, skipped_ids)) continue;

      agents.push_back(a);
      plan_robot_ids.push_back(rid);
      plan_world_goals.push_back(active_plan_.goals[i]);
    }

    if (agents.empty()) {
      RCLCPP_WARN(get_logger(), "Replan: no valid agents, stopping monitor");
      stop_monitoring();
      return;
    }

    auto res = std::make_shared<SetGoals::Response>();
    do_plan(agents, plan_robot_ids, plan_world_goals, skipped_ids, res);

    last_replan_time_ = now();

    if (res->success) {
      RCLCPP_INFO(get_logger(), "Replan OK: %s (%.1fms, %u expansions)",
                  res->message.c_str(), res->planning_time_ms, res->pbs_expansions);
    } else {
      RCLCPP_WARN(get_logger(), "Replan FAILED: %s", res->message.c_str());
    }
  }

  // ------------------------------------------------------------------
  // Поля
  // ------------------------------------------------------------------
  int    num_robots_;
  double time_step_sec_;
  double pbs_resolution_       = 0.2;
  double default_robot_radius_ = 0.22;
  double inflation_radius_     = 0.55;

  // Replanning
  double replan_check_hz_       = 2.0;
  double replan_threshold_m_    = 1.0;
  double replan_cooldown_sec_   = 5.0;
  double replan_cooldown_factor_= 3.0;
  double replan_predict_sec_    = -1.0;
  std::string replan_stop_mode_ = "deviated";
  double goal_reached_m_        = 0.5;

  bool   map_ready_       = false;
  double map_origin_x_    = 0.0;
  double map_origin_y_    = 0.0;
  double map_resolution_  = 0.2;   // реальный resolution PBS-сетки (после downsampling)

  GridMap    grid_;
  PBSSolver  solver_;

  // Кэш одометрии и футпринтов для каждого робота
  std::vector<std::pair<double, double>> current_positions_;
  std::vector<bool>   have_odom_;
  std::vector<double> footprint_radii_;   // bounding radius из Nav2 polygon

  // Подписки
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> footprint_subs_;

  // Сервис
  rclcpp::Service<iros_llm_swarm_interfaces::srv::SetGoals>::SharedPtr plan_srv_;

  // Паблишеры путей
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;

  // Schedule monitoring
  struct ActivePlan {
    std::vector<uint32_t>                   robot_ids;
    std::vector<geometry_msgs::msg::Point>  goals;      // original goals (world coords)
    std::vector<nav_msgs::msg::Path>        ros_paths;  // published paths (for schedule checks)
  };
  ActivePlan active_plan_;
  bool       has_active_plan_  = false;
  double     last_planning_ms_ = 0.0;
  rclcpp::Time           last_replan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

// ---------------------------------------------------------------------------
// WARNING: This node relies on single-threaded execution (rclcpp::spin).
// The monitor timer, service callback, and odom/map subscriptions share
// state without locks.  Switching to MultiThreadedExecutor would require
// adding mutexes around grid_, active_plan_, current_positions_, etc.
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapfPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
