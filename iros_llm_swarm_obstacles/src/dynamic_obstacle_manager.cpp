#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "iros_llm_swarm_interfaces/srv/add_circle.hpp"
#include "iros_llm_swarm_interfaces/srv/add_rectangle.hpp"
#include "iros_llm_swarm_interfaces/srv/add_door.hpp"
#include "iros_llm_swarm_interfaces/srv/remove_obstacle.hpp"
#include "iros_llm_swarm_interfaces/srv/open_door.hpp"
#include "iros_llm_swarm_interfaces/srv/close_door.hpp"
#include "iros_llm_swarm_interfaces/srv/list_obstacles.hpp"
#include "iros_llm_swarm_interfaces/msg/circle_obstacle.hpp"
#include "iros_llm_swarm_interfaces/msg/rectangle_obstacle.hpp"
#include "iros_llm_swarm_interfaces/msg/door.hpp"

using OccGrid   = nav_msgs::msg::OccupancyGrid;
using Marker    = visualization_msgs::msg::Marker;
using MarkerArr = visualization_msgs::msg::MarkerArray;
using Circle    = iros_llm_swarm_interfaces::msg::CircleObstacle;
using Rect      = iros_llm_swarm_interfaces::msg::RectangleObstacle;
using Door      = iros_llm_swarm_interfaces::msg::Door;

namespace srv = iros_llm_swarm_interfaces::srv;

class DynamicObstacleManager : public rclcpp::Node
{
public:
  DynamicObstacleManager()
  : Node("dynamic_obstacle_manager")
  {
    declare_parameter("raw_map_topic", std::string("/raw_map"));
    declare_parameter("scenario_file", std::string(""));
    declare_parameter("scenario_name", std::string(""));

    const std::string raw_topic = get_parameter("raw_map_topic").as_string();

    rclcpp::QoS tl(rclcpp::KeepLast(1));
    tl.transient_local();

    map_pub_    = create_publisher<OccGrid>("/map", tl);
    marker_pub_ = create_publisher<MarkerArr>("/obstacles/markers", tl);

    map_sub_ = create_subscription<OccGrid>(
        raw_topic, tl,
        [this](OccGrid::SharedPtr msg) { on_raw_map(msg); });

    srv_add_circle_ = create_service<srv::AddCircle>(
        "/obstacles/add_circle",
        [this](std::shared_ptr<srv::AddCircle::Request> req,
               std::shared_ptr<srv::AddCircle::Response> res) { handle_add_circle(req, res); });
    srv_add_rect_ = create_service<srv::AddRectangle>(
        "/obstacles/add_rectangle",
        [this](std::shared_ptr<srv::AddRectangle::Request> req,
               std::shared_ptr<srv::AddRectangle::Response> res) { handle_add_rect(req, res); });
    srv_add_door_ = create_service<srv::AddDoor>(
        "/obstacles/add_door",
        [this](std::shared_ptr<srv::AddDoor::Request> req,
               std::shared_ptr<srv::AddDoor::Response> res) { handle_add_door(req, res); });
    srv_remove_ = create_service<srv::RemoveObstacle>(
        "/obstacles/remove",
        [this](std::shared_ptr<srv::RemoveObstacle::Request> req,
               std::shared_ptr<srv::RemoveObstacle::Response> res) { handle_remove(req, res); });
    srv_open_ = create_service<srv::OpenDoor>(
        "/doors/open",
        [this](std::shared_ptr<srv::OpenDoor::Request> req,
               std::shared_ptr<srv::OpenDoor::Response> res) { handle_open_door(req, res); });
    srv_close_ = create_service<srv::CloseDoor>(
        "/doors/close",
        [this](std::shared_ptr<srv::CloseDoor::Request> req,
               std::shared_ptr<srv::CloseDoor::Response> res) { handle_close_door(req, res); });
    srv_list_ = create_service<srv::ListObstacles>(
        "/obstacles/list",
        [this](std::shared_ptr<srv::ListObstacles::Request> req,
               std::shared_ptr<srv::ListObstacles::Response> res) { handle_list(req, res); });

    load_scenario_yaml();

    RCLCPP_INFO(get_logger(), "DynamicObstacleManager ready (listening on %s)",
        raw_topic.c_str());
  }

private:
  // ── Map ingestion ──────────────────────────────────────────────────────────

  void on_raw_map(OccGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    base_map_ = *msg;
    has_map_  = true;
    RCLCPP_INFO_ONCE(get_logger(), "Base map received: %dx%d @ %.3fm",
        msg->info.width, msg->info.height, msg->info.resolution);
    publish_merged_locked();
  }

  // ── Cell blocking helpers ──────────────────────────────────────────────────

  void block_circle(OccGrid & g, const Circle & c)
  {
    const double res = g.info.resolution;
    const double ox  = g.info.origin.position.x;
    const double oy  = g.info.origin.position.y;
    const int    gw  = static_cast<int>(g.info.width);
    const int    gh  = static_cast<int>(g.info.height);

    const int r_cells = static_cast<int>(std::ceil(c.radius / res));
    const int cx = static_cast<int>((c.position.x - ox) / res);
    const int cy = static_cast<int>((c.position.y - oy) / res);

    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        const int nx = cx + dx, ny = cy + dy;
        if (nx < 0 || nx >= gw || ny < 0 || ny >= gh) continue;
        const double wx = ox + (nx + 0.5) * res;
        const double wy = oy + (ny + 0.5) * res;
        if (std::hypot(wx - c.position.x, wy - c.position.y) <= c.radius)
          g.data[static_cast<size_t>(ny * gw + nx)] = 100;
      }
    }
  }

  void block_rect(OccGrid & g, double cx, double cy, double w, double h)
  {
    const double res = g.info.resolution;
    const double ox  = g.info.origin.position.x;
    const double oy  = g.info.origin.position.y;
    const int    gw  = static_cast<int>(g.info.width);
    const int    gh  = static_cast<int>(g.info.height);

    const int c0 = std::max(0,  static_cast<int>((cx - w / 2.0 - ox) / res));
    const int c1 = std::min(gw, static_cast<int>((cx + w / 2.0 - ox) / res) + 1);
    const int r0 = std::max(0,  static_cast<int>((cy - h / 2.0 - oy) / res));
    const int r1 = std::min(gh, static_cast<int>((cy + h / 2.0 - oy) / res) + 1);

    for (int r = r0; r < r1; ++r)
      for (int c = c0; c < c1; ++c)
        g.data[static_cast<size_t>(r * gw + c)] = 100;
  }

  // ── Merge + publish ────────────────────────────────────────────────────────

  void publish_merged_locked()  // call with mutex_ held
  {
    if (!has_map_) return;
    auto merged = base_map_;
    for (const auto & [id, c] : circles_)    block_circle(merged, c);
    for (const auto & [id, r] : rectangles_) block_rect(merged, r.position.x, r.position.y, r.width, r.height);
    for (const auto & [id, d] : doors_)
      if (!d.is_open) block_rect(merged, d.position.x, d.position.y, d.width, d.height);
    map_pub_->publish(merged);
    publish_markers_locked();
  }

  // ── Markers ────────────────────────────────────────────────────────────────

  Marker make_marker(int id_int, const std::string & ns,
                     double x, double y, double sx, double sy, double sz,
                     int shape, float r, float g, float b, float a)
  {
    Marker m;
    m.header.frame_id = base_map_.header.frame_id.empty() ? "map" : base_map_.header.frame_id;
    m.header.stamp    = now();
    m.ns              = ns;
    m.id              = id_int;
    m.type            = shape;
    m.action          = Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = sz / 2.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
    m.color.r = r;  m.color.g = g;  m.color.b = b;  m.color.a = a;
    return m;
  }

  void publish_markers_locked()
  {
    MarkerArr arr;

    // Clear all previously published markers before republishing
    Marker del_all;
    del_all.header.frame_id = base_map_.header.frame_id.empty() ? "map" : base_map_.header.frame_id;
    del_all.header.stamp    = now();
    del_all.action          = Marker::DELETEALL;
    arr.markers.push_back(del_all);

    int uid = 0;
    for (const auto & [id, c] : circles_)
      arr.markers.push_back(make_marker(uid++, "circles",
          c.position.x, c.position.y,
          c.radius * 2, c.radius * 2, 0.2,
          Marker::CYLINDER, 1.0f, 0.5f, 0.0f, 0.8f));

    for (const auto & [id, r] : rectangles_)
      arr.markers.push_back(make_marker(uid++, "rectangles",
          r.position.x, r.position.y,
          r.width, r.height, 0.2,
          Marker::CUBE, 1.0f, 0.5f, 0.0f, 0.8f));

    for (const auto & [id, d] : doors_) {
      float dr = d.is_open ? 0.0f : 1.0f;
      float dg = d.is_open ? 1.0f : 0.0f;
      float da = d.is_open ? 0.3f : 0.8f;
      arr.markers.push_back(make_marker(uid++, "doors",
          d.position.x, d.position.y,
          d.width, d.height, 0.2,
          Marker::CUBE, dr, dg, 0.0f, da));
    }

    marker_pub_->publish(arr);
  }

  // ── Service handlers ───────────────────────────────────────────────────────

  void handle_add_circle(
      const std::shared_ptr<srv::AddCircle::Request>  req,
      std::shared_ptr<srv::AddCircle::Response>       res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (req->id.empty()) { res->success = false; res->message = "id required"; return; }
    Circle c; c.id = req->id; c.position = req->position; c.radius = req->radius;
    circles_[req->id] = c;
    publish_merged_locked();
    res->success = true;
    res->message = "circle " + req->id + " added";
    RCLCPP_INFO(get_logger(), "Added circle '%s' at (%.2f, %.2f) r=%.2f",
        req->id.c_str(), req->position.x, req->position.y, req->radius);
  }

  void handle_add_rect(
      const std::shared_ptr<srv::AddRectangle::Request> req,
      std::shared_ptr<srv::AddRectangle::Response>      res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (req->id.empty()) { res->success = false; res->message = "id required"; return; }
    Rect r; r.id = req->id; r.position = req->position; r.width = req->width; r.height = req->height;
    rectangles_[req->id] = r;
    publish_merged_locked();
    res->success = true;
    res->message = "rectangle " + req->id + " added";
    RCLCPP_INFO(get_logger(), "Added rectangle '%s' at (%.2f, %.2f)",
        req->id.c_str(), req->position.x, req->position.y);
  }

  void handle_add_door(
      const std::shared_ptr<srv::AddDoor::Request> req,
      std::shared_ptr<srv::AddDoor::Response>      res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (req->id.empty()) { res->success = false; res->message = "id required"; return; }
    Door d; d.id = req->id; d.position = req->position;
    d.width = req->width; d.height = req->height; d.is_open = req->is_open;
    doors_[req->id] = d;
    publish_merged_locked();
    res->success = true;
    res->message = "door " + req->id + " added";
    RCLCPP_INFO(get_logger(), "Added door '%s' at (%.2f, %.2f) %s",
        req->id.c_str(), req->position.x, req->position.y, d.is_open ? "OPEN" : "CLOSED");
  }

  void handle_remove(
      const std::shared_ptr<srv::RemoveObstacle::Request>  req,
      std::shared_ptr<srv::RemoveObstacle::Response>       res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    const bool removed = circles_.erase(req->id) || rectangles_.erase(req->id) || doors_.erase(req->id);
    if (removed) publish_merged_locked();
    res->success = removed;
    res->message = removed ? ("removed " + req->id) : ("not found: " + req->id);
  }

  void handle_open_door(
      const std::shared_ptr<srv::OpenDoor::Request>  req,
      std::shared_ptr<srv::OpenDoor::Response>       res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = doors_.find(req->door_id);
    if (it == doors_.end()) {
      res->success = false; res->message = "door not found: " + req->door_id; return;
    }
    it->second.is_open = true;
    publish_merged_locked();
    res->success = true; res->message = req->door_id + " opened";
    RCLCPP_INFO(get_logger(), "Door '%s' opened", req->door_id.c_str());
  }

  void handle_close_door(
      const std::shared_ptr<srv::CloseDoor::Request>  req,
      std::shared_ptr<srv::CloseDoor::Response>       res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = doors_.find(req->door_id);
    if (it == doors_.end()) {
      res->success = false; res->message = "door not found: " + req->door_id; return;
    }
    it->second.is_open = false;
    publish_merged_locked();
    res->success = true; res->message = req->door_id + " closed";
    RCLCPP_INFO(get_logger(), "Door '%s' closed", req->door_id.c_str());
  }

  void handle_list(
      const std::shared_ptr<srv::ListObstacles::Request>,
      std::shared_ptr<srv::ListObstacles::Response> res)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto & [id, c] : circles_)    res->circles.push_back(c);
    for (const auto & [id, r] : rectangles_) res->rectangles.push_back(r);
    for (const auto & [id, d] : doors_)      res->doors.push_back(d);
  }

  // ── YAML loading ───────────────────────────────────────────────────────────

  void load_scenario_yaml()
  {
    const std::string scenario_file = get_parameter("scenario_file").as_string();
    const std::string scenario_name = get_parameter("scenario_name").as_string();
    if (scenario_file.empty() || scenario_name.empty()) return;

    YAML::Node root;
    try {
      root = YAML::LoadFile(scenario_file);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not load scenario YAML %s: %s",
          scenario_file.c_str(), e.what());
      return;
    }

    auto scenarios = root["scenarios"];
    if (!scenarios || !scenarios[scenario_name]) {
      RCLCPP_WARN(get_logger(), "Scenario '%s' not found in %s",
          scenario_name.c_str(), scenario_file.c_str());
      return;
    }

    auto obs = scenarios[scenario_name]["obstacles"];
    if (!obs) return;

    std::lock_guard<std::mutex> lk(mutex_);
    size_t n_circles = 0, n_rects = 0, n_doors = 0;

    for (const auto & item : obs["circles"]) {
      Circle c; c.id = item["id"].as<std::string>();
      auto pos = item["position"];
      c.position.x = pos[0].as<double>(); c.position.y = pos[1].as<double>();
      c.radius = item["radius"].as<double>();
      circles_[c.id] = c; ++n_circles;
    }
    for (const auto & item : obs["rectangles"]) {
      Rect r; r.id = item["id"].as<std::string>();
      auto pos = item["position"];
      r.position.x = pos[0].as<double>(); r.position.y = pos[1].as<double>();
      r.width = item["width"].as<double>(); r.height = item["height"].as<double>();
      rectangles_[r.id] = r; ++n_rects;
    }
    for (const auto & item : obs["doors"]) {
      Door d; d.id = item["id"].as<std::string>();
      auto pos = item["position"];
      d.position.x = pos[0].as<double>(); d.position.y = pos[1].as<double>();
      d.width = item["width"].as<double>(); d.height = item["height"].as<double>();
      d.is_open = item["default_open"] ? item["default_open"].as<bool>() : true;
      doors_[d.id] = d; ++n_doors;
    }

    RCLCPP_INFO(get_logger(),
        "Loaded from scenario '%s': %zu circles, %zu rects, %zu doors",
        scenario_name.c_str(), n_circles, n_rects, n_doors);
  }

  // ── State ──────────────────────────────────────────────────────────────────

  std::mutex mutex_;
  OccGrid    base_map_;
  bool       has_map_ = false;

  std::unordered_map<std::string, Circle> circles_;
  std::unordered_map<std::string, Rect>   rectangles_;
  std::unordered_map<std::string, Door>   doors_;

  rclcpp::Publisher<OccGrid>::SharedPtr    map_pub_;
  rclcpp::Publisher<MarkerArr>::SharedPtr  marker_pub_;
  rclcpp::Subscription<OccGrid>::SharedPtr map_sub_;

  rclcpp::Service<srv::AddCircle>::SharedPtr      srv_add_circle_;
  rclcpp::Service<srv::AddRectangle>::SharedPtr   srv_add_rect_;
  rclcpp::Service<srv::AddDoor>::SharedPtr        srv_add_door_;
  rclcpp::Service<srv::RemoveObstacle>::SharedPtr srv_remove_;
  rclcpp::Service<srv::OpenDoor>::SharedPtr       srv_open_;
  rclcpp::Service<srv::CloseDoor>::SharedPtr      srv_close_;
  rclcpp::Service<srv::ListObstacles>::SharedPtr  srv_list_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacleManager>());
  rclcpp::shutdown();
  return 0;
}
