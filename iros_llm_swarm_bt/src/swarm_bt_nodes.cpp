#include "iros_llm_swarm_bt/swarm_bt_nodes.hpp"

#include <cstdio>
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
    BT::InputPort<std::vector<int>>(
      "robot_ids",
      "Robot IDs to navigate"),
    BT::InputPort<std::vector<geometry_msgs::msg::Point>>(
      "goals",
      "Target positions, one per robot_id"),
  };
}

BT::NodeStatus MapfPlan::onStart()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto robot_ids = getInput<std::vector<int>>("robot_ids");
  auto goals = getInput<std::vector<geometry_msgs::msg::Point>>("goals");

  if (!robot_ids || !goals) {
    RCLCPP_ERROR(node->get_logger(), "MapfPlan: missing robot_ids or goals port");
    return BT::NodeStatus::FAILURE;
  }
  if (robot_ids->size() != goals->size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "MapfPlan: robot_ids size %zu != goals size %zu",
      robot_ids->size(), goals->size());
    return BT::NodeStatus::FAILURE;
  }
  if (robot_ids->empty()) {
    RCLCPP_ERROR(node->get_logger(), "empty robot_ids");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->action_server_is_ready()) {
    RCLCPP_ERROR(node->get_logger(), "MapfPlan: /swarm/set_goals not available");
    return BT::NodeStatus::FAILURE;
  }

  // Reset state from any previous run of this node
  goal_handle_.reset();
  result_future_ = {};
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    pending_snapshot_ = FeedbackSnapshot{};
  }

  // Observer channel — telemetry only
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "OK");
    bb->set<std::string>("@active_action", "MapfPlan");
    bb->set<std::string>("@action_summary", "");
    bb->set<std::string>("@last_error", "");
  }

  // Build goal message
  SetGoals::Goal goal_msg;
  for (auto id : robot_ids.value()) {
    goal_msg.robot_ids.push_back(static_cast<uint32_t>(id));
  }
  goal_msg.goals = goals.value();

  // Send goal with feedback callback (runs on ROS executor thread)
  rclcpp_action::Client<SetGoals>::SendGoalOptions opts;
  opts.feedback_callback =
    [this](GoalHandle::SharedPtr gh,
      const std::shared_ptr<const Feedback> fb) {
      on_feedback(gh, fb);
    };

  goal_handle_future_ = client_->async_send_goal(goal_msg, opts);

  RCLCPP_INFO(
    node->get_logger(),
    "MapfPlan: sent goal for %zu robots", robot_ids->size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MapfPlan::onRunning()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // ---- Apply pending feedback snapshot (written by executor thread) -------
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    if (pending_snapshot_.updated) {
      auto bb = config().blackboard;
      bb->set<std::string>("@action_summary", pending_snapshot_.summary);
      bb->set<std::string>("@action_status", pending_snapshot_.status);
      if (!pending_snapshot_.error.empty()) {
        bb->set<std::string>("@last_error", pending_snapshot_.error);
      }
      pending_snapshot_.updated = false;
    }
  }

  // ---- Wait for MAPF goal handle ------------------------------------------
  if (!goal_handle_) {
    if (!future_ready(goal_handle_future_)) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node->get_logger(), "MapfPlan: goal rejected");
      config().blackboard->set<std::string>("@action_status", "ERROR");
      config().blackboard->set<std::string>("@last_error", "goal rejected");
      return BT::NodeStatus::FAILURE;
    }
    result_future_ = client_->async_get_result(goal_handle_);
  }

  // ---- Wait for MAPF result -----------------------------------------------
  if (!future_ready(result_future_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    config().blackboard->set<std::string>("@action_status", "ERROR");
    config().blackboard->set<std::string>("@last_error", "action transport error");
    return BT::NodeStatus::FAILURE;
  }

  const auto & res = wrapped.result;
  std::ostringstream oss;
  oss << "msg=" << res->message
      << " planned=" << res->num_agents_planned
      << " time_ms=" << res->planning_time_ms
      << " replans=" << res->total_replans;

  // No agents planned at all — hard failure
  if (res->num_agents_planned == 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "MapfPlan: FAILURE — no agents planned. %s", oss.str().c_str());
    config().blackboard->set<std::string>("@action_status", "ERROR");
    config().blackboard->set<std::string>(
      "@last_error",
      std::string("no agents planned: ") + res->message);
    return BT::NodeStatus::FAILURE;
  }

  // Partial plan — some routes succeeded, not all. Return SUCCESS but
  // expose WARN in telemetry. External controller reads /bt/state and
  // decides whether to issue a fresh /llm/command (replan with different
  // goals, drop the failing robots, etc.).
  if (!res->success) {
    const std::string warn_msg = "partial plan: " + res->message;
    RCLCPP_WARN(
      node->get_logger(),
      "MapfPlan: partial plan (%u agents). %s",
      res->num_agents_planned, oss.str().c_str());
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "WARN");
    bb->set<std::string>("@last_error", warn_msg);
    bb->set<std::string>("@action_summary", oss.str());
    bb->set<std::string>("@active_action", "none");
    return BT::NodeStatus::SUCCESS;
  }

  // Full success
  RCLCPP_INFO(node->get_logger(), "MapfPlan: done — %s", oss.str().c_str());
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "OK");
    bb->set<std::string>("@active_action", "none");
    bb->set<std::string>("@last_error", "");
  }
  return BT::NodeStatus::SUCCESS;
}

void MapfPlan::onHalted()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(), "MapfPlan: halted");
  cancel_mapf();

  // Surface the halt in /bt/state so the operator sees a clear terminal
  // marker instead of a frozen "MapfPlan / OK" snapshot.
  auto bb = config().blackboard;
  bb->set<std::string>("@action_status", "HALTED");
  std::string prev_err;
  try {
    prev_err = bb->get<std::string>("@last_error");
  } catch (const std::exception &) {
  }
  if (prev_err.empty()) {
    bb->set<std::string>("@last_error", "MapfPlan halted");
  }

  // Discard any buffered snapshot — node is being halted
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    pending_snapshot_ = FeedbackSnapshot{};
  }
}

void MapfPlan::cancel_mapf()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  } else if (future_ready(goal_handle_future_)) {
    auto gh = goal_handle_future_.get();
    if (gh) {client_->async_cancel_goal(gh);}
  }
  result_future_ = {};
}

// ---------------------------------------------------------------------------
// feedback callback — called from ROS executor thread, must be lock-safe.
// Writes only to pending_snapshot_ under snapshot_mutex_. onRunning() picks
// it up on the BT thread and applies to the blackboard.
// ---------------------------------------------------------------------------
void MapfPlan::on_feedback(
  GoalHandle::SharedPtr /*gh*/,
  const std::shared_ptr<const Feedback> fb)
{
  char prefix[160];
  const int n = std::snprintf(
    prefix, sizeof(prefix),
    "[t=%lldms status=%s arrived=%u active=%u stall=%u replans=%u]",
    static_cast<long long>(fb->elapsed_ms),
    fb->status.c_str(),
    static_cast<unsigned>(fb->robots_arrived),
    static_cast<unsigned>(fb->robots_active),
    static_cast<unsigned>(fb->robot_stall),
    static_cast<unsigned>(fb->replans_done));
  std::string line_str;
  line_str.reserve(
    (n > 0 ? static_cast<size_t>(n) : 0) +
    fb->info.size() + fb->warning.size() + 16);
  line_str.assign(prefix, n > 0 ? static_cast<size_t>(n) : 0);
  if (!fb->info.empty()) {line_str += " INFO: ";    line_str += fb->info;}
  if (!fb->warning.empty()) {line_str += " WARN: ";    line_str += fb->warning;}

  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    pending_snapshot_.summary = line_str;
    pending_snapshot_.updated = true;

    if (!fb->warning.empty()) {
      pending_snapshot_.status = "WARN";
      pending_snapshot_.error = fb->warning;
    } else {
      pending_snapshot_.status = "OK";
      pending_snapshot_.error.clear();
    }
  }
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
    BT::OutputPort<std::string>("formation_warn"),
  };
}

bool SetFormation::start_service_call()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto formation_id = getInput<std::string>("formation_id");
  auto leader_ns = getInput<std::string>("leader_ns");
  auto follower_ns = getInput<std::vector<std::string>>("follower_ns");
  auto offsets_x = getInput<std::vector<double>>("offsets_x");
  auto offsets_y = getInput<std::vector<double>>("offsets_y");
  auto activate = getInput<bool>("activate");

  if (!formation_id || !leader_ns || !follower_ns || !offsets_x || !offsets_y || !activate) {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: missing input ports");
    last_error_ = "missing input ports";
    return false;
  }
  if (follower_ns->size() != offsets_x->size() ||
    follower_ns->size() != offsets_y->size())
  {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: follower_ns / offsets size mismatch");
    last_error_ = "follower_ns / offsets size mismatch";
    return false;
  }

  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: /formation/set not available");
    last_error_ = "/formation/set service unavailable";
    return false;
  }

  auto req = std::make_shared<SetFormationSrv::Request>();
  req->formation_id = formation_id.value();
  req->leader_ns = leader_ns.value();
  req->follower_ns = follower_ns.value();
  req->offsets_x = offsets_x.value();
  req->offsets_y = offsets_y.value();
  req->activate = activate.value();

  future_ = client_->async_send_request(req).future.share();
  return true;
}

BT::NodeStatus SetFormation::onStart()
{
  last_error_.clear();

  // Observer channel — telemetry only
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "OK");
    bb->set<std::string>("@active_action", "SetFormation");
    bb->set<std::string>("@last_error", "");
  }

  if (!start_service_call()) {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "ERROR");
    bb->set<std::string>("@last_error", last_error_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetFormation::onRunning()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Service future not ready yet
  if (!future_ready(future_)) {
    return BT::NodeStatus::RUNNING;
  }

  // Service answered
  auto res = future_.get();
  auto formation_id = getInput<std::string>("formation_id");
  auto activate = getInput<bool>("activate");

  if (res->success) {
    setOutput("formation_enabled", activate.value_or(true));
    setOutput("active_formation", formation_id.value_or(""));
    {
      auto bb = config().blackboard;
      bb->set<std::string>("@action_status", "OK");
      bb->set<std::string>("@active_action", "none");
      bb->set<std::string>("@last_error", "");
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Service returned success=false — return FAILURE.
  // External controller reads /bt/state and decides whether to retry or
  // pick a different formation via /llm/command.
  const std::string error_msg = res->message.empty() ? "unknown" : res->message;
  last_error_ = "formation setup failed: " + error_msg;
  RCLCPP_WARN(node->get_logger(), "SetFormation: %s", last_error_.c_str());
  setOutput("formation_enabled", false);
  setOutput("formation_warn", last_error_);
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "ERROR");
    bb->set<std::string>("@last_error", last_error_);
    bb->set<std::string>("@action_summary", last_error_);
  }
  return BT::NodeStatus::FAILURE;
}

void SetFormation::onHalted()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(), "SetFormation: halted");
  future_ = {};

  // Surface the halt in /bt/state — matches MapfPlan semantics
  auto bb = config().blackboard;
  bb->set<std::string>("@action_status", "HALTED");
  std::string prev_err;
  try {
    prev_err = bb->get<std::string>("@last_error");
  } catch (const std::exception &) {
  }
  if (prev_err.empty()) {
    bb->set<std::string>("@last_error", "SetFormation halted");
  }
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
    BT::OutputPort<std::string>("disband_warn"),
  };
}

bool DisableFormation::start_service_call()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  auto formation_id = getInput<std::string>("formation_id");
  if (!formation_id) {
    RCLCPP_ERROR(node->get_logger(), "DisableFormation: missing formation_id");
    last_error_ = "missing formation_id input";
    return false;
  }

  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(node->get_logger(), "DisableFormation: /formation/deactivate not available");
    last_error_ = "/formation/deactivate service unavailable";
    return false;
  }

  auto req = std::make_shared<DeactivateFormationSrv::Request>();
  req->formation_id = formation_id.value();

  future_ = client_->async_send_request(req).future.share();
  return true;
}

BT::NodeStatus DisableFormation::onStart()
{
  last_error_.clear();

  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "OK");
    bb->set<std::string>("@active_action", "DisableFormation");
    bb->set<std::string>("@last_error", "");
  }

  if (!start_service_call()) {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "ERROR");
    bb->set<std::string>("@last_error", last_error_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DisableFormation::onRunning()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (!future_ready(future_)) {
    return BT::NodeStatus::RUNNING;
  }

  auto res = future_.get();
  if (res->success) {
    setOutput("formation_enabled", false);
    {
      auto bb = config().blackboard;
      bb->set<std::string>("@action_status", "OK");
      bb->set<std::string>("@active_action", "none");
      bb->set<std::string>("@last_error", "");
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Disband failed — formation_manager reported error. From the caller's
  // POV the formation is still active. External controller decides next step.
  const std::string error_msg = res->message.empty() ? "unknown" : res->message;
  last_error_ = "formation disband failed: " + error_msg;
  RCLCPP_WARN(node->get_logger(), "DisableFormation: %s", last_error_.c_str());
  setOutput("formation_enabled", true);
  setOutput("disband_warn", last_error_);
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "ERROR");
    bb->set<std::string>("@last_error", last_error_);
    bb->set<std::string>("@action_summary", last_error_);
  }
  return BT::NodeStatus::FAILURE;
}

void DisableFormation::onHalted()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node->get_logger(), "DisableFormation: halted");
  future_ = {};

  auto bb = config().blackboard;
  bb->set<std::string>("@action_status", "HALTED");
  std::string prev_err;
  try {
    prev_err = bb->get<std::string>("@last_error");
  } catch (const std::exception &) {
  }
  if (prev_err.empty()) {
    bb->set<std::string>("@last_error", "DisableFormation halted");
  }
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
    BT::InputPort<std::string>("mode", "Current mode from blackboard"),
    BT::InputPort<std::string>("expected", "Mode to match against"),
  };
}

BT::NodeStatus CheckMode::tick()
{
  auto mode = getInput<std::string>("mode");
  auto expected = getInput<std::string>("expected");

  if (!mode || !expected) {
    return BT::NodeStatus::FAILURE;
  }
  return (mode.value() == expected.value()) ?
         BT::NodeStatus::SUCCESS :
         BT::NodeStatus::FAILURE;
}

// ===========================================================================
// RunOnce
// ===========================================================================

RunOnce::RunOnce(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{}

BT::PortsList RunOnce::providedPorts()
{
  return {
    BT::InputPort<int>(
      "trigger",
      "Re-run the child when this value changes. "
      "Bind to @command_seq written by LlmCommandReceiver."),
  };
}

BT::NodeStatus RunOnce::tick()
{
  auto trigger = getInput<int>("trigger");
  const int current_trigger = trigger.value_or(0);

  // Trigger changed (new command from controller) — reset state. If the
  // child is still running mid-execution, halt it so onHalted() cleans up
  // before we restart with the new inputs on this very tick.
  if (current_trigger != last_trigger_) {
    last_trigger_ = current_trigger;
    if (child_node_->status() == BT::NodeStatus::RUNNING) {
      child_node_->halt();
    }
    already_done_ = false;
    stored_status_ = BT::NodeStatus::IDLE;
  }

  // Already finished — return RUNNING (masking child's terminal status)
  // so the parent sequence keeps the tree alive. The terminal status is
  // observable via /bt/state telemetry written by the child action node.
  if (already_done_) {
    return BT::NodeStatus::RUNNING;
  }

  setStatus(BT::NodeStatus::RUNNING);
  const auto child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::SUCCESS ||
    child_status == BT::NodeStatus::FAILURE)
  {
    already_done_ = true;
    stored_status_ = child_status;
    return BT::NodeStatus::RUNNING;   // hide terminal from parent
  }

  return child_status;  // RUNNING or IDLE passes through
}

void RunOnce::halt()
{
  // External halt (parent ReactiveFallback switching branches, tree
  // shutdown, etc). Reset state so next tick starts the child fresh.
  // last_trigger_ is intentionally preserved: if the controller hasn't
  // issued a new command, "the same command" is re-run on re-entry.
  already_done_ = false;
  stored_status_ = BT::NodeStatus::IDLE;
  BT::DecoratorNode::halt();
}

// ===========================================================================
// publish_bt_state + BTStatePublisher
// ===========================================================================

namespace
{
std::string bb_get_str(
  const BT::Blackboard::Ptr & bb,
  const std::string & key,
  const std::string & def)
{
  try {
    return bb->get<std::string>(key);
  } catch (const std::exception &) {
    return def;
  }
}
}  // namespace

void publish_bt_state(
  const BT::Blackboard::Ptr & bb,
  rclcpp::Publisher<iros_llm_swarm_interfaces::msg::BTState>::SharedPtr publisher,
  const rclcpp::Clock::SharedPtr & clock)
{
  if (!publisher || !bb) {
    return;
  }
  iros_llm_swarm_interfaces::msg::BTState msg;

  msg.mode = bb_get_str(bb, "@mode", "idle");
  msg.action_status = bb_get_str(bb, "@action_status", "OK");
  msg.active_action = bb_get_str(bb, "@active_action", "none");
  msg.action_summary = bb_get_str(bb, "@action_summary", "");
  msg.last_error = bb_get_str(bb, "@last_error", "");
  msg.formation_id = bb_get_str(bb, "@formation_id", "");
  msg.leader_ns = bb_get_str(bb, "@leader_ns", "");

  try {
    auto ids = bb->get<std::vector<int>>("@robot_ids");
    msg.robot_ids.reserve(ids.size());
    for (auto id : ids) {
      msg.robot_ids.push_back(static_cast<uint32_t>(id));
    }
  } catch (const std::exception &) {
  }
  try {
    msg.goals = bb->get<std::vector<geometry_msgs::msg::Point>>("@goals");
  } catch (const std::exception &) {
  }

  // Formation health — populated by FormationHealthMonitor. Defaults
  // correspond to "no active formation".
  try {
    msg.formation_state = static_cast<uint8_t>(bb->get<int>("@formation_state"));
  } catch (const std::exception &) {
    msg.formation_state = 0;
  }
  try {
    msg.formation_failure_code = static_cast<uint8_t>(bb->get<int>("@formation_failure_code"));
  } catch (const std::exception &) {
    msg.formation_failure_code = 0;
  }
  msg.formation_failure_reason = bb_get_str(bb, "@formation_failure_reason", "");
  try {
    msg.formation_max_error_m = static_cast<float>(bb->get<double>("@formation_max_error_m"));
  } catch (const std::exception &) {
    msg.formation_max_error_m = -1.0f;
  }
  try {
    msg.formation_mean_error_m = static_cast<float>(bb->get<double>("@formation_mean_error_m"));
  } catch (const std::exception &) {
    msg.formation_mean_error_m = -1.0f;
  }

  msg.stamp_ms = clock ?
    static_cast<int64_t>(clock->now().nanoseconds() / 1000000) :
    0;

  try {
    msg.llm_thinking = bb->get<bool>("@llm_thinking");
  } catch (const std::exception &) {
    msg.llm_thinking = false;
  }

  publisher->publish(msg);
}

BTStatePublisher::BTStatePublisher(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = node->create_publisher<BTStateMsg>("/bt/state", bt_state_qos());
  clock_ = node->get_clock();
}

BT::PortsList BTStatePublisher::providedPorts()
{
  return {};
}

BT::NodeStatus BTStatePublisher::tick()
{
  publish_bt_state(config().blackboard, publisher_, clock_);
  return BT::NodeStatus::SUCCESS;
}

// ===========================================================================
// FormationHealthMonitor
// ===========================================================================

FormationHealthMonitor::FormationHealthMonitor(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Reliable QoS matches the publisher in formation_monitor_node.
  sub_ = node->create_subscription<FormationsStatusMsg>(
    "/formations/status",
    rclcpp::QoS(10).reliable(),
    [this](const FormationsStatusMsg::SharedPtr msg) {
      on_formation_status(msg);
    });
}

BT::PortsList FormationHealthMonitor::providedPorts()
{
  return {};
}

void FormationHealthMonitor::on_formation_status(const FormationsStatusMsg::SharedPtr msg)
{
  // Executor thread — only updates the cache under a mutex. tick() (BT thread)
  // reads from this cache and writes to the blackboard.
  std::lock_guard<std::mutex> lk(cache_mutex_);
  for (const auto & fs : msg->formations) {
    cache_[fs.formation_id] = fs;
  }
}

BT::NodeStatus FormationHealthMonitor::tick()
{
  auto bb = config().blackboard;

  // When mode != "formation" there is no active formation to monitor — clear
  // the cache so stale DEGRADED/BROKEN entries from a previous mission don't
  // leak into the next mode and trigger spurious WARN/ERROR escalations.
  std::string mode;
  try {
    mode = bb->get<std::string>("@mode");
  } catch (const std::exception &) {
  }
  if (mode != "formation") {
    {
      std::lock_guard<std::mutex> lk(cache_mutex_);
      cache_.clear();
    }
    // Reset formation fields so /bt/state reflects "no active formation".
    bb->set<int>("@formation_state", 0);         // INACTIVE
    bb->set<int>("@formation_failure_code", 0);  // NONE
    bb->set<std::string>("@formation_failure_reason", "");
    bb->set<double>("@formation_max_error_m", -1.0);
    bb->set<double>("@formation_mean_error_m", -1.0);
    return BT::NodeStatus::SUCCESS;
  }

  // In formation mode — look up the active formation's cached health.
  std::string formation_id;
  try {
    formation_id = bb->get<std::string>("@formation_id");
  } catch (const std::exception &) {
  }

  FormationStatusMsg fs;
  bool have_fs = false;
  if (!formation_id.empty()) {
    std::lock_guard<std::mutex> lk(cache_mutex_);
    auto it = cache_.find(formation_id);
    if (it != cache_.end()) {
      fs = it->second;
      have_fs = true;
    }
  }

  if (!have_fs) {
    // No status yet — leave existing values intact (formation may still be
    // initializing). Do not escalate based on missing data.
    return BT::NodeStatus::SUCCESS;
  }

  // Publish formation health fields to blackboard for BTStatePublisher.
  bb->set<int>("@formation_state", static_cast<int>(fs.state));
  bb->set<int>("@formation_failure_code", static_cast<int>(fs.failure_code));
  bb->set<std::string>("@formation_failure_reason", fs.failure_reason);
  bb->set<double>("@formation_max_error_m", static_cast<double>(fs.max_error_m));
  bb->set<double>("@formation_mean_error_m", static_cast<double>(fs.mean_error_m));

  // Escalate @action_status based on formation health.
  // Mirrors how MapfPlan feedback escalates the same key during MAPF.
  std::string action_status;
  try {
    action_status = bb->get<std::string>("@action_status");
  } catch (const std::exception &) {
  }

  if (fs.state == FormationStatusMsg::STATE_BROKEN &&
    action_status != "ERROR")
  {
    const std::string err = fs.failure_reason.empty() ?
      "formation broken" : fs.failure_reason;
    bb->set<std::string>("@action_status", "ERROR");
    bb->set<std::string>("@last_error", err);
    bb->set<std::string>(
      "@action_summary",
      "formation " + formation_id + " broken: " + err);
  } else if (fs.state == FormationStatusMsg::STATE_DEGRADED &&
    action_status == "OK")
  {
    const std::string warn =
      "formation degraded, max_error=" +
      std::to_string(fs.max_error_m) + "m";
    bb->set<std::string>("@action_status", "WARN");
    bb->set<std::string>("@last_error", warn);
    bb->set<std::string>("@action_summary", warn);
  }
  // Only clear WARN — never overwrite a genuine ERROR with OK.
  else if (fs.state == FormationStatusMsg::STATE_STABLE &&
    action_status == "WARN")
  {
    bb->set<std::string>("@action_status", "OK");
    bb->set<std::string>("@last_error", "");
  }

  return BT::NodeStatus::SUCCESS;
}

// ===========================================================================
// LlmCommandReceiver
// ===========================================================================

LlmCommandReceiver::LlmCommandReceiver(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  using namespace std::placeholders;
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  action_server_ = rclcpp_action::create_server<LlmCommand>(
    node,
    "/llm/command",
    std::bind(&LlmCommandReceiver::handle_goal, this, _1, _2),
    std::bind(&LlmCommandReceiver::handle_cancel, this, _1),
    std::bind(&LlmCommandReceiver::handle_accepted, this, _1));
}

BT::PortsList LlmCommandReceiver::providedPorts()
{
  return {};
}

rclcpp_action::GoalResponse LlmCommandReceiver::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const LlmCommand::Goal> goal)
{
  // Always accept well-formed goals. If a previous goal is still queued in
  // pending_goal_ we will supersede it in handle_accepted — the newer goal
  // is always the more relevant one (e.g. user typing "stop" on top of a
  // mapf in flight). Rejecting here would surface as a confusing
  // "BT rejected goal" in the chat.
  if (goal->mode != "idle" && goal->mode != "mapf" && goal->mode != "formation") {
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto rid : goal->robot_ids) {
    if (rid >= kMaxRobotId) {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      RCLCPP_WARN(
        node->get_logger(),
        "LlmCommandReceiver: rejecting goal — robot_id %u exceeds cap %u",
        rid, kMaxRobotId);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (goal->mode == "mapf" &&
    goal->goals.size() != goal->robot_ids.size())
  {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_WARN(
      node->get_logger(),
      "LlmCommandReceiver: rejecting mapf goal — robot_ids size %zu != "
      "goals size %zu",
      goal->robot_ids.size(), goal->goals.size());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LlmCommandReceiver::handle_cancel(
  const std::shared_ptr<GoalHandle>/*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LlmCommandReceiver::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::shared_ptr<GoalHandle> superseded;
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    // If a previous goal hasn't been applied yet, we will succeed it as
    // "superseded" so its caller doesn't hang inside get_result_async.
    superseded = pending_handle_;
    pending_goal_ = goal_handle->get_goal();
    pending_handle_ = goal_handle;
  }

  if (superseded) {
    auto stale = std::make_shared<LlmCommand::Result>();
    stale->success = true;
    stale->info = "superseded by newer goal";
    superseded->succeed(stale);
  }

  auto fb = std::make_shared<LlmCommand::Feedback>();
  fb->stage = "received";
  goal_handle->publish_feedback(fb);
  // Blackboard writes happen on the next BT tick (see tick()).
}

BT::NodeStatus LlmCommandReceiver::tick()
{
  std::shared_ptr<const LlmCommand::Goal> goal;
  std::shared_ptr<GoalHandle> handle;
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    if (pending_goal_) {
      goal = pending_goal_;
      handle = pending_handle_;
      pending_goal_.reset();
      pending_handle_.reset();
    }
  }

  if (goal) {
    apply_to_blackboard(goal);

    auto fb = std::make_shared<LlmCommand::Feedback>();
    fb->stage = "applied";
    handle->publish_feedback(fb);

    auto result = std::make_shared<LlmCommand::Result>();
    result->success = true;
    result->info = "command applied to blackboard";
    handle->succeed(result);
  }

  return BT::NodeStatus::SUCCESS;
}

void LlmCommandReceiver::apply_to_blackboard(
  std::shared_ptr<const LlmCommand::Goal> goal)
{
  auto bb = config().blackboard;
  auto node = bb->get<rclcpp::Node::SharedPtr>("node");

  RCLCPP_INFO(
    node->get_logger(),
    "LlmCommandReceiver: applying mode=%s reason=%s",
    goal->mode.c_str(), goal->reason.c_str());

  bb->set<std::string>("@mode", goal->mode);
  bb->set<std::string>("@llm_reason", goal->reason);

  // Clear opposite-mode fields so BTStatePublisher/PassiveObserver/event
  // prompts never see leftover state from a previous step (e.g. a mapf
  // step's @robot_ids leaking into a subsequent formation, or vice versa).
  if (goal->mode == "mapf") {
    bb->set<std::string>("@formation_id", "");
    bb->set<std::string>("@leader_ns", "");
    bb->set<std::vector<std::string>>("@follower_ns", {});
    bb->set<std::vector<double>>("@offsets_x", {});
    bb->set<std::vector<double>>("@offsets_y", {});
    if (!goal->robot_ids.empty()) {
      std::vector<int> ids(goal->robot_ids.begin(), goal->robot_ids.end());
      bb->set<std::vector<int>>("@robot_ids", ids);
      bb->set<std::vector<geometry_msgs::msg::Point>>("@goals", goal->goals);
    }
  } else if (goal->mode == "formation") {
    bb->set<std::vector<int>>("@robot_ids", {});
    bb->set<std::vector<geometry_msgs::msg::Point>>("@goals", {});
    if (!goal->formation_id.empty()) {
      bb->set<std::string>("@formation_id", goal->formation_id);
      bb->set<std::string>("@leader_ns", goal->leader_ns);
      bb->set<std::vector<std::string>>("@follower_ns", goal->follower_ns);
      bb->set<std::vector<double>>("@offsets_x", goal->offsets_x);
      bb->set<std::vector<double>>("@offsets_y", goal->offsets_y);
    }
  } else {  // idle — wipe everything mission-related.
    bb->set<std::vector<int>>("@robot_ids", {});
    bb->set<std::vector<geometry_msgs::msg::Point>>("@goals", {});
    bb->set<std::string>("@formation_id", "");
    bb->set<std::string>("@leader_ns", "");
    bb->set<std::vector<std::string>>("@follower_ns", {});
    bb->set<std::vector<double>>("@offsets_x", {});
    bb->set<std::vector<double>>("@offsets_y", {});
  }

  // Bump command sequence so RunOnce decorators observe a fresh trigger
  // and re-tick their wrapped action nodes — even when @mode is unchanged
  // (e.g. controller sends a second mapf command with new goals).
  int seq = 0;
  try {
    seq = bb->get<int>("@command_seq");
  } catch (const std::exception &) {
  }
  bb->set<int>("@command_seq", seq + 1);
}

}  // namespace iros_llm_swarm_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<iros_llm_swarm_bt::MapfPlan>("MapfPlan");
  factory.registerNodeType<iros_llm_swarm_bt::SetFormation>("SetFormation");
  factory.registerNodeType<iros_llm_swarm_bt::DisableFormation>("DisableFormation");
  factory.registerNodeType<iros_llm_swarm_bt::CheckMode>("CheckMode");
  factory.registerNodeType<iros_llm_swarm_bt::RunOnce>("RunOnce");
  factory.registerNodeType<iros_llm_swarm_bt::FormationHealthMonitor>("FormationHealthMonitor");
  factory.registerNodeType<iros_llm_swarm_bt::BTStatePublisher>("BTStatePublisher");
  factory.registerNodeType<iros_llm_swarm_bt::LlmCommandReceiver>("LlmCommandReceiver");
}
