#include "iros_llm_swarm_bt/swarm_bt_nodes.hpp"

#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace iros_llm_swarm_bt
{

// ===========================================================================
// MapfPlan
// ===========================================================================

MapfPlan::MapfPlan(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = rclcpp_action::create_client<SetGoals>(node, "/swarm/set_goals");
}

BT::PortsList MapfPlan::providedPorts()
{
  return {
    BT::InputPort<std::vector<int>>("robot_ids",
      "Robot IDs to navigate"),
    BT::InputPort<std::vector<geometry_msgs::msg::Point>>("goals",
      "Target positions, one per robot_id"),
    BT::OutputPort<bool>("mapf_ok"),
    BT::OutputPort<std::string>("mapf_info"),
  };
}

BT::NodeStatus MapfPlan::onStart()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto robot_ids = getInput<std::vector<int>>("robot_ids");
  auto goals     = getInput<std::vector<geometry_msgs::msg::Point>>("goals");

  if (!robot_ids || !goals) {
    RCLCPP_ERROR(node->get_logger(), "MapfPlan: missing robot_ids or goals port");
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "missing inputs");
    return BT::NodeStatus::FAILURE;
  }
  if (robot_ids->size() != goals->size()) {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan: robot_ids size %zu != goals size %zu",
      robot_ids->size(), goals->size());
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "size mismatch");
    return BT::NodeStatus::FAILURE;
  }
  if (robot_ids->empty()) {
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "empty robot_ids");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node->get_logger(), "MapfPlan: /swarm/set_goals not available");
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "action server unavailable");
    return BT::NodeStatus::FAILURE;
  }

  SetGoals::Goal goal_msg;
  for (auto id : robot_ids.value()) {
    goal_msg.robot_ids.push_back(static_cast<uint32_t>(id));
  }
  goal_msg.goals = goals.value();

  goal_handle_future_ = client_->async_send_goal(goal_msg);
  goal_handle_.reset();
  result_future_ = {};

  RCLCPP_INFO(node->get_logger(),
    "MapfPlan: sent goal for %zu robots", robot_ids->size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MapfPlan::onRunning()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Waiting for goal handle
  if (!goal_handle_) {
    if (!future_ready(goal_handle_future_)) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node->get_logger(), "MapfPlan: goal rejected");
      setOutput("mapf_ok", false);
      setOutput("mapf_info", "goal rejected");
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = client_->async_get_result(goal_handle_);
  }

  // Waiting for result
  if (!future_ready(result_future_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "action did not succeed");
    return BT::NodeStatus::FAILURE;
  }

  const auto & res = wrapped.result;
  std::ostringstream oss;
  oss << "msg=" << res->message
      << " planned=" << res->num_agents_planned
      << " time_ms=" << res->planning_time_ms
      << " replans=" << res->total_replans;

  setOutput("mapf_ok", res->success);
  setOutput("mapf_info", oss.str());

  RCLCPP_INFO(node->get_logger(), "MapfPlan: done — %s", oss.str().c_str());

  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void MapfPlan::onHalted()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(), "MapfPlan: halted — cancelling action");

  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  } else if (future_ready(goal_handle_future_)) {
    auto gh = goal_handle_future_.get();
    if (gh) client_->async_cancel_goal(gh);
  }

  goal_handle_.reset();
  result_future_ = {};
}

// ===========================================================================
// SetFormation
// ===========================================================================

SetFormation::SetFormation(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node->create_client<SetFormationSrv>("/formation/set");
}

BT::PortsList SetFormation::providedPorts()
{
  return {
    BT::InputPort<std::string>("formation_id"),
    BT::InputPort<std::string>("leader_ns"),
    BT::InputPort<std::vector<std::string>>("follower_ns"),
    BT::InputPort<std::vector<double>>("offsets_x"),
    BT::InputPort<std::vector<double>>("offsets_y"),
    BT::InputPort<bool>("activate", true, "Activate formation immediately"),
    BT::OutputPort<bool>("formation_enabled"),
    BT::OutputPort<std::string>("active_formation"),
  };
}

BT::NodeStatus SetFormation::onStart()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto formation_id = getInput<std::string>("formation_id");
  auto leader_ns    = getInput<std::string>("leader_ns");
  auto follower_ns  = getInput<std::vector<std::string>>("follower_ns");
  auto offsets_x    = getInput<std::vector<double>>("offsets_x");
  auto offsets_y    = getInput<std::vector<double>>("offsets_y");
  auto activate     = getInput<bool>("activate");

  if (!formation_id || !leader_ns || !follower_ns || !offsets_x || !offsets_y || !activate) {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: missing input ports");
    return BT::NodeStatus::FAILURE;
  }
  if (follower_ns->size() != offsets_x->size() ||
      follower_ns->size() != offsets_y->size())
  {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: follower_ns / offsets size mismatch");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: /formation/set not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<SetFormationSrv::Request>();
  req->formation_id = formation_id.value();
  req->leader_ns    = leader_ns.value();
  req->follower_ns  = follower_ns.value();
  req->offsets_x    = offsets_x.value();
  req->offsets_y    = offsets_y.value();
  req->activate     = activate.value();

  future_ = client_->async_send_request(req).future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetFormation::onRunning()
{
  if (!future_ready(future_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto res = future_.get();
  auto formation_id = getInput<std::string>("formation_id");
  auto activate     = getInput<bool>("activate");

  setOutput("formation_enabled", res->success && activate.value_or(true));
  setOutput("active_formation",  formation_id.value_or(""));

  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void SetFormation::onHalted()
{
  // Service calls can't be cancelled in ROS 2 — just drop the future
  future_ = {};
}

// ===========================================================================
// DisableFormation
// ===========================================================================

DisableFormation::DisableFormation(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node->create_client<DeactivateFormationSrv>("/formation/deactivate");
}

BT::PortsList DisableFormation::providedPorts()
{
  return {
    BT::InputPort<std::string>("formation_id"),
    BT::OutputPort<bool>("formation_enabled"),
  };
}

BT::NodeStatus DisableFormation::onStart()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto formation_id = getInput<std::string>("formation_id");
  if (!formation_id) {
    RCLCPP_ERROR(node->get_logger(), "DisableFormation: missing formation_id");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "DisableFormation: /formation/deactivate not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<DeactivateFormationSrv::Request>();
  req->formation_id = formation_id.value();

  future_ = client_->async_send_request(req).future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DisableFormation::onRunning()
{
  if (!future_ready(future_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto res = future_.get();
  setOutput("formation_enabled", false);
  return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void DisableFormation::onHalted()
{
  future_ = {};
}

// ===========================================================================
// CheckMode
// ===========================================================================

CheckMode::CheckMode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList CheckMode::providedPorts()
{
  return {
    BT::InputPort<std::string>("mode",     "Current mode from blackboard"),
    BT::InputPort<std::string>("expected", "Mode to match against"),
  };
}

BT::NodeStatus CheckMode::tick()
{
  auto mode     = getInput<std::string>("mode");
  auto expected = getInput<std::string>("expected");

  if (!mode || !expected) {
    return BT::NodeStatus::FAILURE;
  }
  return (mode.value() == expected.value())
    ? BT::NodeStatus::SUCCESS
    : BT::NodeStatus::FAILURE;
}

}  // namespace iros_llm_swarm_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<iros_llm_swarm_bt::MapfPlan>("MapfPlan");
  factory.registerNodeType<iros_llm_swarm_bt::SetFormation>("SetFormation");
  factory.registerNodeType<iros_llm_swarm_bt::DisableFormation>("DisableFormation");
  factory.registerNodeType<iros_llm_swarm_bt::CheckMode>("CheckMode");
}