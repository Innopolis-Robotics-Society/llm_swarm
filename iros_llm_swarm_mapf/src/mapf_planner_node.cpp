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

    num_robots_           = get_parameter("num_robots").as_int();
    time_step_sec_        = get_parameter("time_step_sec").as_double();
    pbs_resolution_       = get_parameter("pbs_resolution").as_double();
    default_robot_radius_ = get_parameter("default_robot_radius").as_double();

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
        "mapf_planner ready: %d robots, time_step=%.3f s, default_radius=%.3f m",
        num_robots_, time_step_sec_, default_robot_radius_);
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

    // Собираем агентов из одометрии, футпринтов и целей запроса
    std::vector<Agent> agents;
    agents.reserve(req->robot_ids.size());

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

      // Футпринт: из Nav2 если пришёл, иначе default
      a.footprint_radius = footprint_radii_[rid] > 0.0
          ? static_cast<float>(footprint_radii_[rid])
          : static_cast<float>(default_robot_radius_);

      // Проверяем что клетки не в стене
      if (grid_.blocked[a.start.row * grid_.cols + a.start.col]) {
        RCLCPP_WARN(get_logger(),
            "Agent %u: start (%.2f, %.2f) -> cell (%zu, %zu) is blocked! Skipping.",
            rid, sx, sy, a.start.row, a.start.col);
        continue;
      }
      if (grid_.blocked[a.goal.row * grid_.cols + a.goal.col]) {
        RCLCPP_WARN(get_logger(),
            "Agent %u: goal (%.2f, %.2f) -> cell (%zu, %zu) is blocked! Skipping.",
            rid, req->goals[i].x, req->goals[i].y, a.goal.row, a.goal.col);
        continue;
      }

      agents.push_back(a);
    }

    if (agents.empty()) {
      res->success = false;
      res->message = "No valid agents after filtering (check odom and map)";
      return;
    }

    // Планируем и публикуем пути
    do_plan(agents, res);
  }

  // ------------------------------------------------------------------
  // Общая логика планирования и публикации
  // ------------------------------------------------------------------
  void do_plan(const std::vector<Agent>& agents,
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
      res->message = "PBS failed to find solution (" +
                     std::to_string(elapsed_ms) + " ms, " +
                     std::to_string(stats.expansions) + " expansions)";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(),
        "PBS solved in %.1f ms (%zu expansions), publishing %zu paths",
        elapsed_ms, stats.expansions, paths.size());

    // Публикуем пути и собираем длины для ответа
    const rclcpp::Time base_time = now();
    res->path_lengths.resize(agents.size());

    for (size_t i = 0; i < agents.size(); ++i) {
      const size_t robot_id = agents[i].id;
      res->path_lengths[i] = static_cast<uint32_t>(paths[i].size());

      if (robot_id >= static_cast<size_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "Robot id %zu >= num_robots %d, skip publish",
                    robot_id, num_robots_);
        continue;
      }

      auto ros_path = make_ros_path(paths[i], base_time);
      path_pubs_[robot_id]->publish(ros_path);

      RCLCPP_DEBUG(get_logger(),
          "Published path for robot_%zu: %zu waypoints", robot_id, paths[i].size());
    }

    res->success = true;
    res->message = "OK";
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
  // Поля
  // ------------------------------------------------------------------
  int    num_robots_;
  double time_step_sec_;
  double pbs_resolution_       = 0.2;
  double default_robot_radius_ = 0.22;

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
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapfPlannerNode>());
  rclcpp::shutdown();
  return 0;
}