// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.

#include "iros_llm_rviz_tool/send_llm_goal_tool.hpp"

#include <algorithm>
#include <cmath>

#include <QIcon>
#include <QInputDialog>
#include <QStringList>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/point.hpp>

namespace iros_llm_rviz_tool
{

namespace
{

constexpr double kSpreadSpacing = 1.5;     // metres between adjacent goals
constexpr double kBoundsMargin  = 0.5;     // keep goals off the wall

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

}  // namespace


SendLlmGoalTool::SendLlmGoalTool()
{
  shortcut_key_ = 'g';
  setName("Send LLM Goal");
  setIcon(QIcon(rviz_common::loadPixmap(
    "package://iros_llm_rviz_tool/icons/send_llm_goal.svg")));
}

SendLlmGoalTool::~SendLlmGoalTool() = default;


void SendLlmGoalTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();

  auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();

  cmd_client_ = rclcpp_action::create_client<LlmCommand>(node_, "/llm/command");

  loadMap(map_name_);

  RCLCPP_INFO(node_->get_logger(),
    "SendLlmGoalTool ready (map=%s, %zu groups)",
    map_name_.c_str(), groups_.size());
}


void SendLlmGoalTool::loadMap(const std::string & map_name)
{
  groups_.clear();
  bounds_ = MapBounds{};

  std::string yaml_path;
  try {
    yaml_path = ament_index_cpp::get_package_share_directory(
      "iros_llm_orchestrator") + "/prompts/maps/" + map_name + ".yaml";
  } catch (const std::exception & exc) {
    RCLCPP_WARN(node_->get_logger(),
      "Could not locate iros_llm_orchestrator share dir: %s", exc.what());
    return;
  }

  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    if (root["bounds"]) {
      const auto b = root["bounds"];
      if (b["x_min"]) bounds_.x_min = b["x_min"].as<double>();
      if (b["x_max"]) bounds_.x_max = b["x_max"].as<double>();
      if (b["y_min"]) bounds_.y_min = b["y_min"].as<double>();
      if (b["y_max"]) bounds_.y_max = b["y_max"].as<double>();
    }

    if (root["robot_groups"]) {
      for (const auto & kv : root["robot_groups"]) {
        RobotGroup g;
        g.color = kv.first.as<std::string>();
        if (kv.second["color"]) {
          g.color = kv.second["color"].as<std::string>();
        }
        if (kv.second["ids"]) {
          for (const auto & id : kv.second["ids"]) {
            g.ids.push_back(id.as<int>());
          }
        }
        if (kv.second["aliases"]) {
          for (const auto & a : kv.second["aliases"]) {
            g.aliases.push_back(a.as<std::string>());
          }
        }
        if (!g.ids.empty()) {
          groups_.push_back(std::move(g));
        }
      }
    }
  } catch (const std::exception & exc) {
    RCLCPP_WARN(node_->get_logger(),
      "Failed to parse %s: %s", yaml_path.c_str(), exc.what());
  }
}


std::vector<std::pair<double, double>> SendLlmGoalTool::spreadGoals(
  double cx, double cy, std::size_t n) const
{
  std::vector<std::pair<double, double>> out;
  out.reserve(n);

  if (n == 0) {
    return out;
  }
  if (n == 1) {
    out.emplace_back(cx, cy);
    return out;
  }

  // Same algorithm as user_chat_node._spread_goals: tight rosette for
  // small groups, simple grid for larger ones.
  if (n <= 8) {
    const double r = (n > 2)
      ? kSpreadSpacing / (2.0 * std::sin(M_PI / static_cast<double>(n)))
      : kSpreadSpacing / 2.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
      out.emplace_back(cx + r * std::cos(a), cy + r * std::sin(a));
    }
  } else {
    const std::size_t cols = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::round(std::sqrt(static_cast<double>(n)))));
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t col = i % cols;
      const std::size_t row = i / cols;
      out.emplace_back(
        cx + (static_cast<double>(col) - (static_cast<double>(cols) - 1.0) / 2.0)
             * kSpreadSpacing,
        cy + static_cast<double>(row) * kSpreadSpacing);
    }
  }
  return out;
}


void SendLlmGoalTool::onPoseSet(double x, double y, double /*theta*/)
{
  if (!node_) {
    return;
  }

  // Out-of-bounds check first — never send a goal outside the map.
  if (x < bounds_.x_min || x > bounds_.x_max ||
      y < bounds_.y_min || y > bounds_.y_max)
  {
    setStatus(QString("out of bounds (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    return;
  }

  if (groups_.empty()) {
    setStatus("no robot groups loaded — check map YAML");
    return;
  }

  QStringList items;
  for (const auto & g : groups_) {
    items << QString::fromStdString(g.color);
  }
  bool ok = false;
  const QString chosen = QInputDialog::getItem(
    nullptr, "Send LLM Goal", "Pick a robot group:",
    items, 0, false, &ok);
  if (!ok || chosen.isEmpty()) {
    setStatus("cancelled");
    return;
  }

  const auto it = std::find_if(groups_.begin(), groups_.end(),
    [&](const RobotGroup & g) {
      return QString::fromStdString(g.color) == chosen;
    });
  if (it == groups_.end()) {
    setStatus("group not found");
    return;
  }

  if (!cmd_client_ || !cmd_client_->action_server_is_ready()) {
    setStatus("/llm/command not available");
    return;
  }

  LlmCommand::Goal goal;
  goal.mode   = "mapf";
  goal.reason = "operator click via SendLlmGoalTool";
  goal.robot_ids.reserve(it->ids.size());
  for (int id : it->ids) {
    goal.robot_ids.push_back(static_cast<uint32_t>(id));
  }

  const auto spread = spreadGoals(x, y, it->ids.size());
  goal.goals.reserve(spread.size());
  for (const auto & [gx, gy] : spread) {
    geometry_msgs::msg::Point p;
    p.x = clamp(gx, bounds_.x_min + kBoundsMargin, bounds_.x_max - kBoundsMargin);
    p.y = clamp(gy, bounds_.y_min + kBoundsMargin, bounds_.y_max - kBoundsMargin);
    p.z = 0.0;
    goal.goals.push_back(p);
  }

  cmd_client_->async_send_goal(goal);
  setStatus(QString("sent %1 robots → (%2, %3)")
    .arg(it->ids.size())
    .arg(x, 0, 'f', 2)
    .arg(y, 0, 'f', 2));
}

}  // namespace iros_llm_rviz_tool

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_tool::SendLlmGoalTool, rviz_common::Tool)
