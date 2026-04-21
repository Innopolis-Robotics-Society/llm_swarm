#include "iros_llm_swarm_bt/swarm_bt_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace iros_llm_swarm_bt
{

namespace
{
std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

std::vector<double> split_csv_doubles(const std::string & s)
{
  std::vector<double> out;
  for (const auto & t : split_csv(s)) {
    out.push_back(std::stod(t));
  }
  return out;
}

std::vector<int> split_csv_ints(const std::string & s)
{
  std::vector<int> out;
  for (const auto & t : split_csv(s)) {
    out.push_back(std::stoi(t));
  }
  return out;
}
}  // namespace

MapfPlan::MapfPlan(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList MapfPlan::providedPorts()
{
  return {
    BT::InputPort<double>("goal_x"),
    BT::InputPort<double>("goal_y"),
    BT::InputPort<double>("goal_spacing", 1.0, "Spacing between robot goal points"),
    BT::InputPort<std::string>("robot_ids_csv"),
    BT::OutputPort<bool>("mapf_ok"),
    BT::OutputPort<std::string>("mapf_info")
  };
}

BT::NodeStatus MapfPlan::tick()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto goal_x = getInput<double>("goal_x");
  auto goal_y = getInput<double>("goal_y");
  auto goal_spacing = getInput<double>("goal_spacing");
  auto robot_ids_csv = getInput<std::string>("robot_ids_csv");

  if (!goal_x || !goal_y || !robot_ids_csv) {
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "Missing goal_x, goal_y or robot_ids_csv");
    return BT::NodeStatus::FAILURE;
  }

  const double spacing = goal_spacing ? goal_spacing.value() : 1.0;

  const auto robot_ids = split_csv_ints(robot_ids_csv.value());
  if (robot_ids.empty()) {
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "robot_ids_csv is empty");
    return BT::NodeStatus::FAILURE;
  }

  auto client = rclcpp_action::create_client<SetGoals>(node, "/swarm/set_goals");
  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "/swarm/set_goals action not available");
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "action unavailable");
    return BT::NodeStatus::FAILURE;
  }

  SetGoals::Goal goal_msg;

  const size_t n = robot_ids.size();
  const size_t cols_n =
    static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(n))));

  for (size_t i = 0; i < n; ++i) {
    const auto id = robot_ids[i];
    goal_msg.robot_ids.push_back(id);

    const size_t row_i = i / cols_n;
    const size_t col_i = i % cols_n;

    geometry_msgs::msg::Point p;
    p.x = goal_x.value() +
      (static_cast<double>(col_i) - (static_cast<double>(cols_n) - 1.0) / 2.0) * spacing;
    p.y = goal_y.value() +
      (static_cast<double>(row_i) - (static_cast<double>(cols_n) - 1.0) / 2.0) * spacing;
    p.z = 0.0;

    goal_msg.goals.push_back(p);

    RCLCPP_INFO(
      node->get_logger(),
      "MapfPlan: robot_id=%d -> goal=(%.2f, %.2f)",
      id, p.x, p.y);
  }

  auto goal_handle_future = client->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed sending MAPF goal");
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "send goal failed");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "MAPF goal rejected");
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "goal rejected");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future, 120s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed waiting for MAPF result");
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "timeout waiting for result");
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "action did not succeed");
    return BT::NodeStatus::FAILURE;
  }

  const auto & res = wrapped.result;
  std::ostringstream oss;
  oss << "msg=" << res->message
    << ", planned=" << res->num_agents_planned
    << ", time_ms=" << res->planning_time_ms
    << ", pbs_exp=" << res->pbs_expansions
    << ", max_path=" << res->max_path_length;


  setOutput("mapf_ok", res->success);
  setOutput("mapf_info", oss.str());
  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

SetFormation::SetFormation(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetFormation::providedPorts()
{
  return {
    BT::InputPort<std::string>("formation_id"),
    BT::InputPort<std::string>("leader_ns"),
    BT::InputPort<std::string>("follower_ns_csv"),
    BT::InputPort<std::string>("offsets_x_csv"),
    BT::InputPort<std::string>("offsets_y_csv"),
    BT::InputPort<bool>("activate", true, "Activate formation immediately"),
    BT::OutputPort<bool>("formation_enabled"),
    BT::OutputPort<std::string>("active_formation")
  };
}

BT::NodeStatus SetFormation::tick()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto formation_id = getInput<std::string>("formation_id");
  auto leader_ns = getInput<std::string>("leader_ns");
  auto follower_ns_csv = getInput<std::string>("follower_ns_csv");
  auto offsets_x_csv = getInput<std::string>("offsets_x_csv");
  auto offsets_y_csv = getInput<std::string>("offsets_y_csv");
  auto activate = getInput<bool>("activate");

  if (!formation_id || !leader_ns || !follower_ns_csv || !offsets_x_csv || !offsets_y_csv ||
    !activate)
  {
    return BT::NodeStatus::FAILURE;
  }

  auto followers = split_csv(follower_ns_csv.value());
  auto xs = split_csv_doubles(offsets_x_csv.value());
  auto ys = split_csv_doubles(offsets_y_csv.value());
  if (followers.size() != xs.size() || followers.size() != ys.size()) {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: invalid follower/offset sizes");
    return BT::NodeStatus::FAILURE;
  }

  auto client = node->create_client<SetFormationSrv>("/formation/set");
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "/formation/set service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<SetFormationSrv::Request>();
  req->formation_id = formation_id.value();
  req->leader_ns = leader_ns.value();
  req->follower_ns = followers;
  req->offsets_x = xs;
  req->offsets_y = ys;
  req->activate = activate.value();

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    return BT::NodeStatus::FAILURE;
  }

  auto res = future.get();
  setOutput("formation_enabled", res->success && activate.value());
  setOutput("active_formation", formation_id.value());
  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

DisableFormation::DisableFormation(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList DisableFormation::providedPorts()
{
  return {
    BT::InputPort<std::string>("formation_id"),
    BT::OutputPort<bool>("formation_enabled")
  };
}

BT::NodeStatus DisableFormation::tick()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto formation_id = getInput<std::string>("formation_id");
  if (!formation_id) {
    return BT::NodeStatus::FAILURE;
  }

  auto client = node->create_client<DisbandFormationSrv>("/formation/disband");
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "/formation/disband service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<DisbandFormationSrv::Request>();
  req->formation_id = formation_id.value();

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    return BT::NodeStatus::FAILURE;
  }

  auto res = future.get();
  setOutput("formation_enabled", false);
  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace iros_llm_swarm_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<iros_llm_swarm_bt::MapfPlan>("MapfPlan");
  factory.registerNodeType<iros_llm_swarm_bt::SetFormation>("SetFormation");
  factory.registerNodeType<iros_llm_swarm_bt::DisableFormation>("DisableFormation");
}