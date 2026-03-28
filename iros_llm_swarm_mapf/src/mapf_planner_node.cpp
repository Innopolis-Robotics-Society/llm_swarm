#include <chrono>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

// Наш solver (header-only)
#include "iros_llm_swarm_mapf/pbs_solver.hpp"

// ---------------------------------------------------------------------------
// Формат запроса на планирование.
//
// Публикуй в топик /swarm_goals сообщение типа std_msgs/String в JSON:
//
//   {
//     "goals": [
//       {"id": 0, "sx": 2.0, "sy": 2.0, "gx": 15.0, "gy": 15.0},
//       {"id": 1, "sx": 3.5, "sy": 2.0, "gx": 20.0, "gy": 10.0},
//       ...
//     ]
//   }
//
// sx/sy — стартовая позиция в метрах (map-фрейм).
// gx/gy — целевая позиция в метрах (map-фрейм).
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
// Простой JSON-парсер (без внешних зависимостей)
// ---------------------------------------------------------------------------

struct GoalEntry {
  int    id = 0;
  double sx = 0, sy = 0;  // start в метрах
  double gx = 0, gy = 0;  // goal  в метрах
};

// Возвращает false если парсинг не удался.
// Ожидаем строго формат описанный выше.
static bool parse_goals_json(const std::string& json,
                              std::vector<GoalEntry>& out) {
  out.clear();
  // Ищем массив "goals": [...]
  const auto arr_start = json.find('[');
  const auto arr_end   = json.rfind(']');
  if (arr_start == std::string::npos || arr_end == std::string::npos) return false;

  std::string arr = json.substr(arr_start + 1, arr_end - arr_start - 1);

  // Разбиваем по объектам { ... }
  size_t pos = 0;
  while (pos < arr.size()) {
    const auto ob = arr.find('{', pos);
    const auto oe = arr.find('}', ob);
    if (ob == std::string::npos || oe == std::string::npos) break;

    const std::string obj = arr.substr(ob + 1, oe - ob - 1);

    auto get_val = [&](const std::string& key) -> double {
      const std::string search = "\"" + key + "\"";
      auto kp = obj.find(search);
      if (kp == std::string::npos) return 0.0;
      auto colon = obj.find(':', kp);
      if (colon == std::string::npos) return 0.0;
      size_t vstart = colon + 1;
      while (vstart < obj.size() && (obj[vstart] == ' ' || obj[vstart] == '\t')) ++vstart;
      return std::stod(obj.substr(vstart));
    };

    GoalEntry e;
    e.id = static_cast<int>(get_val("id"));
    e.sx = get_val("sx");
    e.sy = get_val("sy");
    e.gx = get_val("gx");
    e.gy = get_val("gy");
    out.push_back(e);

    pos = oe + 1;
  }
  return !out.empty();
}

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

    // Параметры
    declare_parameter("num_robots",     20);
    declare_parameter("time_step_sec",  0.1);
    declare_parameter("map_topic",      std::string("/map"));
    declare_parameter("goals_topic",    std::string("/swarm_goals"));
    // PBS планирует на грубой сетке. 0.2м/клетку -> 150x150 вместо 600x600.
    // Увеличь если медленно, уменьши для точности.
    declare_parameter("pbs_resolution", 0.2);

    num_robots_     = get_parameter("num_robots").as_int();
    time_step_sec_  = get_parameter("time_step_sec").as_double();
    pbs_resolution_ = get_parameter("pbs_resolution").as_double();

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

    // Подписка на цели
    goals_sub_ = create_subscription<std_msgs::msg::String>(
        get_parameter("goals_topic").as_string(), 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          on_goals(msg->data);
        });

    // Паблишеры путей — по одному на каждого робота
    path_pubs_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i) {
      const std::string topic = "/robot_" + std::to_string(i) + "/mapf_path";
      path_pubs_[i] = create_publisher<nav_msgs::msg::Path>(topic, 10);
    }

    RCLCPP_INFO(get_logger(),
        "mapf_planner ready: %d robots, time_step=%.3f s",
        num_robots_, time_step_sec_);
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
  // Обработка запроса на планирование
  // ------------------------------------------------------------------
  void on_goals(const std::string& json) {
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "Map not ready yet, ignoring goals");
      return;
    }

    // Парсим JSON
    std::vector<GoalEntry> entries;
    if (!parse_goals_json(json, entries)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse goals JSON:\n%s", json.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Planning for %zu agents...", entries.size());

    // Конвертируем в Agent (grid-координаты)
    std::vector<Agent> agents;
    agents.reserve(entries.size());

    for (const auto& e : entries) {
      Agent a;
      a.id    = static_cast<size_t>(e.id);
      a.start = world_to_cell(e.sx, e.sy, map_origin_x_, map_origin_y_,
                               map_resolution_, grid_.rows);
      a.goal  = world_to_cell(e.gx, e.gy, map_origin_x_, map_origin_y_,
                               map_resolution_, grid_.rows);

      // Проверяем что клетки не в стене
      if (grid_.blocked[a.start.row * grid_.cols + a.start.col]) {
        RCLCPP_WARN(get_logger(),
            "Agent %zu: start (%.2f, %.2f) -> cell (%zu, %zu) is blocked! Skipping.",
            a.id, e.sx, e.sy, a.start.row, a.start.col);
        continue;
      }
      if (grid_.blocked[a.goal.row * grid_.cols + a.goal.col]) {
        RCLCPP_WARN(get_logger(),
            "Agent %zu: goal (%.2f, %.2f) -> cell (%zu, %zu) is blocked! Skipping.",
            a.id, e.gx, e.gy, a.goal.row, a.goal.col);
        continue;
      }

      agents.push_back(a);
    }

    if (agents.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid agents after filtering");
      return;
    }

    // Запускаем PBS
    solver_.set_map(&grid_);

    std::vector<Path> paths;
    const auto t0 = now();

    const bool ok = solver_.solve(agents, paths);

    const double elapsed_ms =
        (now() - t0).nanoseconds() / 1.0e6;

    if (!ok) {
      RCLCPP_ERROR(get_logger(), "PBS failed to find solution (%.1f ms)", elapsed_ms);
      return;
    }

    RCLCPP_INFO(get_logger(),
        "PBS solved in %.1f ms, publishing %zu paths", elapsed_ms, paths.size());

    // Публикуем пути
    const rclcpp::Time base_time = now();
    for (size_t i = 0; i < agents.size(); ++i) {
      const size_t robot_id = agents[i].id;
      if (robot_id >= static_cast<size_t>(num_robots_)) {
        RCLCPP_WARN(get_logger(), "Robot id %zu >= num_robots %d, skip", robot_id, num_robots_);
        continue;
      }

      auto ros_path = make_ros_path(paths[i], base_time);
      path_pubs_[robot_id]->publish(ros_path);

      RCLCPP_DEBUG(get_logger(),
          "Published path for robot_%zu: %zu waypoints", robot_id, paths[i].size());
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
  // Поля
  // ------------------------------------------------------------------
  int    num_robots_;
  double time_step_sec_;
  double pbs_resolution_  = 0.2;

  bool   map_ready_       = false;
  double map_origin_x_    = 0.0;
  double map_origin_y_    = 0.0;
  double map_resolution_  = 0.2;   // реальный resolution PBS-сетки (после downsampling)

  GridMap    grid_;
  PBSSolver  solver_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        goals_sub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_pubs_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapfPlannerNode>());
  rclcpp::shutdown();
  return 0;
}