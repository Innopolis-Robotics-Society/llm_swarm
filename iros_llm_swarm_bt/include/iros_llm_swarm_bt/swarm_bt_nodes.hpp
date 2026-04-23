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
  llm_client_ = rclcpp_action::create_client<LlmDecision>(node, "/llm/decision");

  // ROS param: how often to send periodic info log to LLM (0 = disabled)
  node->declare_parameter("llm_log_interval_sec", 0.0);
  llm_log_interval_sec_ = node->get_parameter("llm_log_interval_sec").as_double();

  // ROS param: info ring buffer size
  node->declare_parameter("llm_info_buffer_size", 50);
  max_info_buffer_ = static_cast<std::size_t>(
    node->get_parameter("llm_info_buffer_size").as_int());
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
    BT::OutputPort<std::string>("mapf_warn"),
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

  // Reset state
  {
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    info_buffer_.clear();
  }
  {
    std::lock_guard<std::mutex> lk(decision_mutex_);
    pending_decision_ = "";
  }
  llm_pending_ = false;
  goal_handle_.reset();
  result_future_ = {};
  llm_goal_handle_.reset();
  llm_result_future_ = {};
  last_llm_log_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Build goal
  SetGoals::Goal goal_msg;
  for (auto id : robot_ids.value()) {
    goal_msg.robot_ids.push_back(static_cast<uint32_t>(id));
  }
  goal_msg.goals = goals.value();

  // Send goal with feedback callback
  rclcpp_action::Client<SetGoals>::SendGoalOptions opts;
  opts.feedback_callback =
    [this](GoalHandle::SharedPtr gh,
           const std::shared_ptr<const Feedback> fb) {
      on_feedback(gh, fb);
    };

  goal_handle_future_ = client_->async_send_goal(goal_msg, opts);

  RCLCPP_INFO(node->get_logger(),
    "MapfPlan: sent goal for %zu robots", robot_ids->size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MapfPlan::onRunning()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // ---- Check pending LLM decision ------------------------------------
  {
    std::lock_guard<std::mutex> lk(decision_mutex_);
    if (!pending_decision_.empty()) {
      const std::string dec = pending_decision_;
      pending_decision_ = "";
      llm_pending_ = false;

      if (dec == "abort") {
        RCLCPP_WARN(node->get_logger(), "MapfPlan: LLM decision=abort");
        setOutput("mapf_ok", false);
        setOutput("mapf_info", "aborted by LLM");
        cancel_mapf();
        return BT::NodeStatus::FAILURE;
      } else if (dec == "replan") {
        RCLCPP_INFO(node->get_logger(), "MapfPlan: LLM decision=replan");
        // Signal blackboard — runner will set new goal and switch mode
        config().blackboard->set<std::string>("@mapf_decision", "replan");
        cancel_mapf();
        return BT::NodeStatus::FAILURE;
      }
      // "wait" — fall through, continue RUNNING
      RCLCPP_INFO(node->get_logger(), "MapfPlan: LLM decision=wait, continuing");
    }
  }

  // ---- Check LLM result future (async, non-blocking) -----------------
  if (llm_pending_ && !llm_goal_handle_ && future_ready(llm_goal_handle_future_)) {
    llm_goal_handle_ = llm_goal_handle_future_.get();
    if (llm_goal_handle_) {
      llm_result_future_ = llm_client_->async_get_result(llm_goal_handle_);
    } else {
      RCLCPP_WARN(node->get_logger(), "MapfPlan: LLM goal rejected");
      llm_pending_ = false;
    }
  }
  if (llm_pending_ && llm_goal_handle_ && future_ready(llm_result_future_)) {
    auto wrapped = llm_result_future_.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      std::lock_guard<std::mutex> lk(decision_mutex_);
      pending_decision_ = wrapped.result->decision;
    } else {
      RCLCPP_WARN(node->get_logger(), "MapfPlan: LLM action did not succeed");
      llm_pending_ = false;
    }
  }

  // ---- Wait for MAPF goal handle ------------------------------------
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

  // ---- Wait for MAPF result -----------------------------------------
  if (!future_ready(result_future_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    setOutput("mapf_ok", false);
    setOutput("mapf_info", "action transport error");
    return BT::NodeStatus::FAILURE;
  }

  const auto & res = wrapped.result;
  std::ostringstream oss;
  oss << "msg=" << res->message
      << " planned=" << res->num_agents_planned
      << " time_ms=" << res->planning_time_ms
      << " replans=" << res->total_replans;

  setOutput("mapf_info", oss.str());

  if (res->num_agents_planned == 0) {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan: FAILURE — no agents planned. %s", oss.str().c_str());
    setOutput("mapf_ok", false);
    return BT::NodeStatus::FAILURE;
  }

  if (!res->success) {
    RCLCPP_WARN(node->get_logger(),
      "MapfPlan: partial plan (%u agents). %s",
      res->num_agents_planned, oss.str().c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "MapfPlan: done — %s", oss.str().c_str());
  }

  setOutput("mapf_ok", true);
  return BT::NodeStatus::SUCCESS;
}

void MapfPlan::onHalted()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(), "MapfPlan: halted");
  cancel_mapf();
  // Cancel pending LLM call too
  if (llm_goal_handle_) {
    llm_client_->async_cancel_goal(llm_goal_handle_);
    llm_goal_handle_.reset();
  }
  llm_pending_ = false;
  {
    std::lock_guard<std::mutex> lk(decision_mutex_);
    pending_decision_ = "";
  }
}

void MapfPlan::cancel_mapf()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  } else if (future_ready(goal_handle_future_)) {
    auto gh = goal_handle_future_.get();
    if (gh) client_->async_cancel_goal(gh);
  }
  result_future_ = {};
}

// ---------------------------------------------------------------------------
// feedback callback — called from ROS executor thread, must be lock-safe
// ---------------------------------------------------------------------------
void MapfPlan::on_feedback(
  GoalHandle::SharedPtr /*gh*/,
  const std::shared_ptr<const Feedback> fb)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Build a one-line summary of this feedback tick
  std::ostringstream line;
  line << "[t=" << fb->elapsed_ms << "ms"
       << " status=" << fb->status
       << " arrived=" << fb->robots_arrived
       << " active=" << fb->robots_active
       << " stall=" << fb->robot_stall
       << " replans=" << fb->replans_done
       << "]";
  if (!fb->info.empty())    line << " INFO: "    << fb->info;
  if (!fb->warning.empty()) line << " WARN: "    << fb->warning;

  const std::string line_str = line.str();

  // Always add to ring buffer
  {
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    info_buffer_.push_back(line_str);
    while (info_buffer_.size() > max_info_buffer_) {
      info_buffer_.pop_front();
    }
  }

  // WARNING — immediate async LLM call (flush buffer + warning)
  if (!fb->warning.empty()) {
    RCLCPP_WARN(node->get_logger(), "MapfPlan feedback WARN: %s", fb->warning.c_str());
    setOutput("mapf_warn", fb->warning);
    send_to_llm("WARN", fb->warning);
    return;
  }

  // INFO (periodic log) — send to LLM if interval enabled and elapsed
  if (!fb->info.empty() && llm_log_interval_sec_ > 0.0) {
    const auto now = node->now();
    const bool first = (last_llm_log_time_.nanoseconds() == 0);
    const double since = first ? llm_log_interval_sec_ + 1.0
                                : (now - last_llm_log_time_).seconds();
    if (since >= llm_log_interval_sec_) {
      last_llm_log_time_ = now;
      send_to_llm("INFO", fb->info);
    }
    return;
  }

  // OK — just logged into buffer, nothing more to do
}

// ---------------------------------------------------------------------------
// send_to_llm — async, does not block onRunning
// ---------------------------------------------------------------------------
void MapfPlan::send_to_llm(const std::string & level, const std::string & event)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Don't pile up LLM calls — skip if one is already in flight
  if (llm_pending_) {
    RCLCPP_WARN(node->get_logger(),
      "MapfPlan: LLM call already in flight, skipping new %s event", level.c_str());
    return;
  }

  if (!llm_client_->action_server_is_ready()) {
    RCLCPP_WARN(node->get_logger(),
      "MapfPlan: /llm/decision not available, skipping %s event", level.c_str());
    return;
  }

  LlmDecision::Goal goal;
  goal.level = level;
  goal.event = event;

  // Snapshot log buffer
  {
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    for (const auto & s : info_buffer_) {
      goal.log_buffer.push_back(s);
    }
  }

  llm_goal_handle_.reset();
  llm_result_future_ = {};
  llm_goal_handle_future_ = llm_client_->async_send_goal(goal);
  llm_pending_ = true;

  RCLCPP_INFO(node->get_logger(),
    "MapfPlan: sent %s event to LLM (%zu log lines)", level.c_str(), goal.log_buffer.size());
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
  client_ = node->create_client<DisbandFormationSrv>("/formation/disband");
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
    RCLCPP_ERROR(node->get_logger(), "DisableFormation: /formation/disband not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<DisbandFormationSrv::Request>();
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