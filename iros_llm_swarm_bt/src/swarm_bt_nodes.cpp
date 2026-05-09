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
  if (!node->has_parameter("llm_log_interval_sec")) {
    node->declare_parameter("llm_log_interval_sec", 0.0);
  }
  llm_log_interval_sec_ = node->get_parameter("llm_log_interval_sec").as_double();

  // ROS param: info ring buffer size
  if (!node->has_parameter("llm_info_buffer_size")) {
    node->declare_parameter("llm_info_buffer_size", 50);
  }
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

  // Observer channel: snapshot state for BTStatePublisher
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "OK");
    bb->set<std::string>("@active_action", "MapfPlan");
    bb->set<std::string>("@action_summary", "");
    bb->set<std::string>("@last_error", "");
  }

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

  // ---- Apply pending feedback snapshot (written by executor thread) -----
  // All blackboard writes MUST happen here, never inside on_feedback.
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    if (pending_snapshot_.updated) {
      auto bb = config().blackboard;
      bb->set<std::string>("@action_summary", pending_snapshot_.summary);
      bb->set<std::string>("@action_status",  pending_snapshot_.status);
      if (!pending_snapshot_.error.empty()) {
        bb->set<std::string>("@last_error", pending_snapshot_.error);
      }

      if (!pending_snapshot_.warn_event.empty()) {
        RCLCPP_WARN(node->get_logger(),
          "MapfPlan feedback WARN: %s", pending_snapshot_.warn_event.c_str());
        setOutput("mapf_warn", pending_snapshot_.warn_event);
        if (!llm_pending_) {
          send_to_llm("WARN", pending_snapshot_.warn_event);
        }
      } else if (!pending_snapshot_.info_event.empty() && !llm_pending_) {
        const auto now = node->now();
        const bool first = (last_llm_log_time_.nanoseconds() == 0);
        const double since = first ? llm_log_interval_sec_ + 1.0
                                    : (now - last_llm_log_time_).seconds();
        if (since >= llm_log_interval_sec_) {
          last_llm_log_time_ = now;
          send_to_llm("INFO", pending_snapshot_.info_event);
        }
      }

      pending_snapshot_.updated = false;
      pending_snapshot_.warn_event.clear();
      pending_snapshot_.info_event.clear();
    }
  }

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
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", "aborted by LLM");
        config().blackboard->set<bool>("@llm_thinking", false);
        cancel_mapf();
        return BT::NodeStatus::FAILURE;
      } else if (dec == "replan") {
        RCLCPP_INFO(node->get_logger(), "MapfPlan: LLM decision=replan");
        config().blackboard->set<std::string>("@mapf_decision", "replan");
        config().blackboard->set<std::string>("@mode", "idle");
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", "replan requested by LLM");
        config().blackboard->set<bool>("@llm_thinking", false);
        cancel_mapf();
        return BT::NodeStatus::FAILURE;
      }
      // "wait" — LLM reviewed the event and decided to continue.
      // Reset action_status to OK so PassiveObserver (channel 2) doesn't
      // re-trigger on the same stale WARN on the next BTState publish.
      RCLCPP_INFO(node->get_logger(), "MapfPlan: LLM decision=wait, continuing");
      config().blackboard->set<std::string>("@action_status", "OK");
      config().blackboard->set<std::string>("@last_error", "");
      config().blackboard->set<bool>("@llm_thinking", false);
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

  setOutput("mapf_info", oss.str());

  if (res->num_agents_planned == 0) {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan: FAILURE — no agents planned. %s", oss.str().c_str());
    setOutput("mapf_ok", false);
    config().blackboard->set<std::string>("@action_status", "ERROR");
    config().blackboard->set<std::string>("@last_error",
      std::string("no agents planned: ") + res->message);
    return BT::NodeStatus::FAILURE;
  }

  if (!res->success) {
    // Partial plan — some agents could not be routed (PBS failed or
    // start/goal blocked). This is recoverable in some cases (different
    // goals, fewer agents) but the current plan is stale.
    // Escalate to WARN so channel 1 (LLM decision) gets to decide.
    const std::string warn_msg =
      std::string("partial plan: ") + res->message +
      " planned=" + std::to_string(res->num_agents_planned) +
      "/" + std::to_string(res->num_agents_planned);  // BT will fill total
    RCLCPP_WARN(node->get_logger(),
      "MapfPlan: partial plan (%u agents). %s",
      res->num_agents_planned, oss.str().c_str());
    setOutput("mapf_ok", false);
    config().blackboard->set<std::string>("@action_status", "WARN");
    config().blackboard->set<std::string>("@last_error", warn_msg);
    config().blackboard->set<std::string>(
      "@action_summary", oss.str());
    // Do NOT return FAILURE yet — let LLM decide via send_to_llm.
    // If no LLM call is pending, trigger one now.
    if (!llm_pending_) {
      send_to_llm("WARN", warn_msg);
    }
    return BT::NodeStatus::RUNNING;
  } else {
    RCLCPP_INFO(node->get_logger(), "MapfPlan: done — %s", oss.str().c_str());
  }

  setOutput("mapf_ok", true);
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
  // Cancel pending LLM call too
  if (llm_goal_handle_) {
    llm_client_->async_cancel_goal(llm_goal_handle_);
    llm_goal_handle_.reset();
  }
  llm_pending_ = false;
  auto bb = config().blackboard;
  bb->set<bool>("@llm_thinking", false);
  // Surface the halt in /bt/state so the operator sees a clear terminal
  // marker instead of a frozen "MapfPlan / OK" snapshot.
  bb->set<std::string>("@action_status", "HALTED");
  std::string prev_err;
  try { prev_err = bb->get<std::string>("@last_error"); } catch (...) {}
  if (prev_err.empty()) {
    bb->set<std::string>("@last_error", "MapfPlan halted");
  }
  {
    std::lock_guard<std::mutex> lk(decision_mutex_);
    pending_decision_ = "";
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

  // Ring buffer — has its own mutex, safe from any thread
  {
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    info_buffer_.push_back(line_str);
    while (info_buffer_.size() > max_info_buffer_) {
      info_buffer_.pop_front();
    }
  }

  // Buffer blackboard writes + LLM-trigger intent for onRunning() (BT thread).
  // NEVER write to blackboard here — it is not thread-safe.
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    pending_snapshot_.summary = line_str;
    pending_snapshot_.updated = true;

    if (!fb->warning.empty()) {
      pending_snapshot_.status     = "WARN";
      pending_snapshot_.error      = fb->warning;
      pending_snapshot_.warn_event = fb->warning;
      pending_snapshot_.info_event.clear();
    } else {
      pending_snapshot_.status = "OK";
      pending_snapshot_.error.clear();
      pending_snapshot_.warn_event.clear();
      // Stash the info string; onRunning will decide whether the
      // periodic-log interval has elapsed before actually sending it.
      pending_snapshot_.info_event =
        (!fb->info.empty() && llm_log_interval_sec_ > 0.0) ? fb->info : "";
    }
  }
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
  config().blackboard->set<bool>("@llm_thinking", true);

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
  client_     = node->create_client<SetFormationSrv>("/formation/set");
  llm_client_ = rclcpp_action::create_client<LlmDecision>(node, "/llm/decision");

  if (!node->has_parameter("formation_llm_max_retries")) {
    node->declare_parameter("formation_llm_max_retries", 2);
  }
  max_retries_ = static_cast<int>(
    node->get_parameter("formation_llm_max_retries").as_int());
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
  auto leader_ns    = getInput<std::string>("leader_ns");
  auto follower_ns  = getInput<std::vector<std::string>>("follower_ns");
  auto offsets_x    = getInput<std::vector<double>>("offsets_x");
  auto offsets_y    = getInput<std::vector<double>>("offsets_y");
  auto activate     = getInput<bool>("activate");

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

  if (!client_->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "SetFormation: /formation/set not available");
    last_error_ = "/formation/set service unavailable";
    return false;
  }

  auto req = std::make_shared<SetFormationSrv::Request>();
  req->formation_id = formation_id.value();
  req->leader_ns    = leader_ns.value();
  req->follower_ns  = follower_ns.value();
  req->offsets_x    = offsets_x.value();
  req->offsets_y    = offsets_y.value();
  req->activate     = activate.value();

  future_ = client_->async_send_request(req).future.share();
  return true;
}

BT::NodeStatus SetFormation::onStart()
{
  retry_count_ = 0;
  last_error_.clear();
  llm_pending_ = false;
  llm_goal_handle_.reset();
  llm_goal_handle_future_ = {};
  llm_result_future_ = {};
  {
    std::lock_guard<std::mutex> lk(llm_decision_mutex_);
    llm_pending_decision_.clear();
  }

  // Observer channel: snapshot state
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

  // 1) A verdict came back from a previous LLM call — apply it.
  {
    std::lock_guard<std::mutex> lk(llm_decision_mutex_);
    if (!llm_pending_decision_.empty()) {
      const std::string dec = llm_pending_decision_;
      llm_pending_decision_.clear();
      llm_pending_ = false;

      if (dec == "abort") {
        RCLCPP_WARN(node->get_logger(), "SetFormation: LLM decision=abort");
        setOutput("formation_enabled", false);
        setOutput("formation_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }
      if (dec == "replan") {
        RCLCPP_INFO(node->get_logger(), "SetFormation: LLM decision=replan");
        config().blackboard->set<std::string>("@formation_decision", "replan");
        setOutput("formation_enabled", false);
        setOutput("formation_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }

      // "wait" (or empty, treated as wait) — retry if budget allows.
      if (retry_count_ >= max_retries_) {
        RCLCPP_WARN(node->get_logger(),
          "SetFormation: retry budget exhausted (%d), aborting", max_retries_);
        setOutput("formation_enabled", false);
        setOutput("formation_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }
      ++retry_count_;
      RCLCPP_INFO(node->get_logger(),
        "SetFormation: LLM decision=wait, retrying (%d/%d)",
        retry_count_, max_retries_);
      if (!start_service_call()) {
        setOutput("formation_enabled", false);
        setOutput("formation_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  // 2) LLM request in flight — poll its futures, then keep waiting.
  if (llm_pending_) {
    poll_llm_result();
    return BT::NodeStatus::RUNNING;
  }

  // 3) Service future not ready yet — stay RUNNING.
  if (!future_ready(future_)) {
    return BT::NodeStatus::RUNNING;
  }

  // 4) Service answered.
  auto res = future_.get();
  auto formation_id = getInput<std::string>("formation_id");
  auto activate     = getInput<bool>("activate");

  if (res->success) {
    setOutput("formation_enabled", activate.value_or(true));
    setOutput("active_formation",  formation_id.value_or(""));
    {
      auto bb = config().blackboard;
      bb->set<std::string>("@action_status", "OK");
      bb->set<std::string>("@active_action", "none");
      bb->set<std::string>("@last_error", "");
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Failed — remember error, ask LLM, stay RUNNING until verdict arrives.
  const std::string error_msg = res->message.empty() ? "unknown" : res->message;
  last_error_ = "formation setup failed: " + error_msg;
  RCLCPP_WARN(node->get_logger(), "SetFormation: %s, asking LLM", last_error_.c_str());
  setOutput("formation_warn", last_error_);
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "WARN");
    bb->set<std::string>("@last_error", last_error_);
    bb->set<std::string>("@action_summary", last_error_);
  }

  send_to_llm("WARN", last_error_);

  // send_to_llm sets llm_pending_decision_ = "abort" if the server is down,
  // so the next tick will see that and fail. Either way, RUNNING is correct
  // here — the verdict application block above runs on the following tick.
  return BT::NodeStatus::RUNNING;
}

void SetFormation::onHalted()
{
  future_ = {};
  cancel_llm();
}

void SetFormation::send_to_llm(const std::string & level, const std::string & event)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (llm_pending_) {
    RCLCPP_WARN(node->get_logger(),
      "SetFormation: LLM call already in flight, skipping new %s", level.c_str());
    return;
  }
  if (!llm_client_->action_server_is_ready()) {
    RCLCPP_WARN(node->get_logger(),
      "SetFormation: /llm/decision not available, aborting on failure");
    // Safe default: abort — no LLM means no retry logic can run.
    std::lock_guard<std::mutex> lk(llm_decision_mutex_);
    llm_pending_decision_ = "abort";
    return;
  }

  LlmDecision::Goal goal;
  goal.level = level;
  goal.event = event;
  // No rolling feedback buffer for services — leave log_buffer empty.

  llm_goal_handle_.reset();
  llm_result_future_ = {};
  llm_goal_handle_future_ = llm_client_->async_send_goal(goal);
  llm_pending_ = true;

  RCLCPP_INFO(node->get_logger(),
    "SetFormation: sent %s event to LLM", level.c_str());
}

void SetFormation::poll_llm_result()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (!llm_goal_handle_ && future_ready(llm_goal_handle_future_)) {
    llm_goal_handle_ = llm_goal_handle_future_.get();
    if (llm_goal_handle_) {
      llm_result_future_ = llm_client_->async_get_result(llm_goal_handle_);
    } else {
      RCLCPP_WARN(node->get_logger(), "SetFormation: LLM goal rejected");
      llm_pending_ = false;
      // Without a verdict we can't decide — fall back to abort.
      std::lock_guard<std::mutex> lk(llm_decision_mutex_);
      llm_pending_decision_ = "abort";
    }
  }
  if (llm_goal_handle_ && future_ready(llm_result_future_)) {
    auto wrapped = llm_result_future_.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      std::lock_guard<std::mutex> lk(llm_decision_mutex_);
      llm_pending_decision_ = wrapped.result->decision;
    } else {
      RCLCPP_WARN(node->get_logger(), "SetFormation: LLM action did not succeed");
      llm_pending_ = false;
      std::lock_guard<std::mutex> lk(llm_decision_mutex_);
      llm_pending_decision_ = "abort";
    }
  }
}

void SetFormation::cancel_llm()
{
  if (llm_goal_handle_) {
    llm_client_->async_cancel_goal(llm_goal_handle_);
    llm_goal_handle_.reset();
  }
  llm_pending_ = false;
  std::lock_guard<std::mutex> lk(llm_decision_mutex_);
  llm_pending_decision_.clear();
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
  client_     = node->create_client<DeactivateFormationSrv>("/formation/deactivate");
  llm_client_ = rclcpp_action::create_client<LlmDecision>(node, "/llm/decision");

  if (!node->has_parameter("disband_llm_max_retries")) {
    node->declare_parameter("disband_llm_max_retries", 2);
  }
  max_retries_ = static_cast<int>(
    node->get_parameter("disband_llm_max_retries").as_int());
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

  if (!client_->wait_for_service(2s)) {
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
  retry_count_ = 0;
  last_error_.clear();
  llm_pending_ = false;
  llm_goal_handle_.reset();
  llm_goal_handle_future_ = {};
  llm_result_future_ = {};
  {
    std::lock_guard<std::mutex> lk(llm_decision_mutex_);
    llm_pending_decision_.clear();
  }

  // Observer channel: snapshot state
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

  // 1) Apply LLM verdict if one arrived.
  {
    std::lock_guard<std::mutex> lk(llm_decision_mutex_);
    if (!llm_pending_decision_.empty()) {
      std::string dec = llm_pending_decision_;
      llm_pending_decision_.clear();
      llm_pending_ = false;

      if (dec == "abort" || dec == "replan") {
        if (dec == "replan") {
          RCLCPP_WARN(node->get_logger(),
            "DisableFormation: collapsing replan -> abort (no replan semantics here)");
        } else {
          RCLCPP_WARN(node->get_logger(), "DisableFormation: LLM decision=abort");
        }
        // Could not disband — formation remains active from the caller's POV.
        setOutput("formation_enabled", true);
        setOutput("disband_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }

      // "wait" (or empty) — retry if budget allows.
      if (retry_count_ >= max_retries_) {
        RCLCPP_WARN(node->get_logger(),
          "DisableFormation: retry budget exhausted (%d), aborting", max_retries_);
        setOutput("formation_enabled", true);
        setOutput("disband_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }
      ++retry_count_;
      RCLCPP_INFO(node->get_logger(),
        "DisableFormation: LLM decision=wait, retrying (%d/%d)",
        retry_count_, max_retries_);
      if (!start_service_call()) {
        setOutput("formation_enabled", true);
        setOutput("disband_warn", last_error_);
        config().blackboard->set<std::string>("@action_status", "ERROR");
        config().blackboard->set<std::string>("@last_error", last_error_);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  // 2) LLM in flight.
  if (llm_pending_) {
    poll_llm_result();
    return BT::NodeStatus::RUNNING;
  }

  // 3) Service not ready yet.
  if (!future_ready(future_)) {
    return BT::NodeStatus::RUNNING;
  }

  // 4) Service answered.
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

  const std::string error_msg = res->message.empty() ? "unknown" : res->message;
  last_error_ = "formation disband failed: " + error_msg;
  RCLCPP_WARN(node->get_logger(),
    "DisableFormation: %s, asking LLM", last_error_.c_str());
  setOutput("disband_warn", last_error_);
  {
    auto bb = config().blackboard;
    bb->set<std::string>("@action_status", "WARN");
    bb->set<std::string>("@last_error", last_error_);
    bb->set<std::string>("@action_summary", last_error_);
  }

  send_to_llm("WARN", last_error_);

  return BT::NodeStatus::RUNNING;
}

void DisableFormation::onHalted()
{
  future_ = {};
  cancel_llm();
}

void DisableFormation::send_to_llm(const std::string & level, const std::string & event)
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (llm_pending_) {
    RCLCPP_WARN(node->get_logger(),
      "DisableFormation: LLM call already in flight, skipping new %s", level.c_str());
    return;
  }
  if (!llm_client_->action_server_is_ready()) {
    RCLCPP_WARN(node->get_logger(),
      "DisableFormation: /llm/decision not available, aborting on failure");
    std::lock_guard<std::mutex> lk(llm_decision_mutex_);
    llm_pending_decision_ = "abort";
    return;
  }

  LlmDecision::Goal goal;
  goal.level = level;
  goal.event = event;

  llm_goal_handle_.reset();
  llm_result_future_ = {};
  llm_goal_handle_future_ = llm_client_->async_send_goal(goal);
  llm_pending_ = true;

  RCLCPP_INFO(node->get_logger(),
    "DisableFormation: sent %s event to LLM", level.c_str());
}

void DisableFormation::poll_llm_result()
{
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (!llm_goal_handle_ && future_ready(llm_goal_handle_future_)) {
    llm_goal_handle_ = llm_goal_handle_future_.get();
    if (llm_goal_handle_) {
      llm_result_future_ = llm_client_->async_get_result(llm_goal_handle_);
    } else {
      RCLCPP_WARN(node->get_logger(), "DisableFormation: LLM goal rejected");
      llm_pending_ = false;
      std::lock_guard<std::mutex> lk(llm_decision_mutex_);
      llm_pending_decision_ = "abort";
    }
  }
  if (llm_goal_handle_ && future_ready(llm_result_future_)) {
    auto wrapped = llm_result_future_.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      std::lock_guard<std::mutex> lk(llm_decision_mutex_);
      llm_pending_decision_ = wrapped.result->decision;
    } else {
      RCLCPP_WARN(node->get_logger(), "DisableFormation: LLM action did not succeed");
      llm_pending_ = false;
      std::lock_guard<std::mutex> lk(llm_decision_mutex_);
      llm_pending_decision_ = "abort";
    }
  }
}

void DisableFormation::cancel_llm()
{
  if (llm_goal_handle_) {
    llm_client_->async_cancel_goal(llm_goal_handle_);
    llm_goal_handle_.reset();
  }
  llm_pending_ = false;
  std::lock_guard<std::mutex> lk(llm_decision_mutex_);
  llm_pending_decision_.clear();
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

// ===========================================================================
// BTStatePublisher
// ===========================================================================

namespace {
std::string bb_get_str(
  const BT::Blackboard::Ptr & bb,
  const std::string & key,
  const std::string & def)
{
  try {
    return bb->get<std::string>(key);
  } catch (...) {
    return def;
  }
}
}  // namespace

void publish_bt_state(
  const BT::Blackboard::Ptr & bb,
  rclcpp::Publisher<iros_llm_swarm_interfaces::msg::BTState>::SharedPtr publisher,
  const rclcpp::Clock::SharedPtr & clock,
  const iros_llm_swarm_interfaces::msg::FormationStatus * fs)
{
  if (!publisher || !bb) {
    return;
  }
  iros_llm_swarm_interfaces::msg::BTState msg;

  msg.mode           = bb_get_str(bb, "@mode", "idle");
  msg.action_status  = bb_get_str(bb, "@action_status", "OK");
  msg.active_action  = bb_get_str(bb, "@active_action", "none");
  msg.action_summary = bb_get_str(bb, "@action_summary", "");
  msg.last_error     = bb_get_str(bb, "@last_error", "");
  msg.formation_id   = bb_get_str(bb, "@formation_id", "");
  msg.leader_ns      = bb_get_str(bb, "@leader_ns", "");

  try {
    auto ids = bb->get<std::vector<int>>("@robot_ids");
    msg.robot_ids.reserve(ids.size());
    for (auto id : ids) {
      msg.robot_ids.push_back(static_cast<uint32_t>(id));
    }
  } catch (...) {}
  try {
    msg.goals = bb->get<std::vector<geometry_msgs::msg::Point>>("@goals");
  } catch (...) {}

  if (fs) {
    msg.formation_state          = fs->state;
    msg.formation_failure_code   = fs->failure_code;
    msg.formation_failure_reason = fs->failure_reason;
    msg.formation_max_error_m    = fs->max_error_m;
    msg.formation_mean_error_m   = fs->mean_error_m;
  } else {
    msg.formation_state          = 0;     // INACTIVE
    msg.formation_failure_code   = 0;     // NONE
    msg.formation_failure_reason = "";
    msg.formation_max_error_m    = -1.0f;
    msg.formation_mean_error_m   = -1.0f;
  }

  msg.stamp_ms = clock
    ? static_cast<int64_t>(clock->now().nanoseconds() / 1000000)
    : 0;

  try {
    msg.llm_thinking = bb->get<bool>("@llm_thinking");
  } catch (...) {
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

  // Subscribe to formation monitor. Reliable QoS matches the publisher in
  // formation_monitor_node.
  formation_status_sub_ = node->create_subscription<FormationsStatusMsg>(
    "/formations/status",
    rclcpp::QoS(10).reliable(),
    [this](const FormationsStatusMsg::SharedPtr msg) {
      on_formation_status(msg);
    });
}

BT::PortsList BTStatePublisher::providedPorts()
{
  return {};
}

void BTStatePublisher::on_formation_status(const FormationsStatusMsg::SharedPtr msg)
{
  // Called from ROS executor thread — only updates the cache under a mutex.
  // tick() (BT thread) reads from this cache; all blackboard writes happen there.
  std::lock_guard<std::mutex> lk(formation_cache_mutex_);
  for (const auto & fs : msg->formations) {
    formation_cache_[fs.formation_id] = fs;
  }
}

BT::NodeStatus BTStatePublisher::tick()
{
  auto bb = config().blackboard;

  // When mode != "formation" there is no active formation to monitor — clear
  // the cache so stale DEGRADED/BROKEN entries from a previous mission don't
  // leak into the next mode and trigger spurious WARN/ERROR escalations.
  std::string mode;
  try { mode = bb->get<std::string>("@mode"); } catch (...) {}
  if (mode != "formation") {
    std::lock_guard<std::mutex> lk(formation_cache_mutex_);
    formation_cache_.clear();
  }

  std::string formation_id;
  try { formation_id = bb->get<std::string>("@formation_id"); } catch (...) {}

  FormationStatusMsg cached_fs;
  bool have_fs = false;
  if (!formation_id.empty()) {
    std::lock_guard<std::mutex> lk(formation_cache_mutex_);
    auto it = formation_cache_.find(formation_id);
    if (it != formation_cache_.end()) {
      cached_fs = it->second;
      have_fs = true;
    }
  }

  // Mirror formation health into @action_status / @last_error so PassiveObserver
  // and user_chat see the same WARN/ERROR transitions they get from MAPF feedback.
  if (have_fs) {
    std::string action_status;
    try { action_status = bb->get<std::string>("@action_status"); } catch (...) {}

    if (cached_fs.state == FormationStatusMsg::STATE_BROKEN &&
        action_status != "ERROR")
    {
      const std::string err = cached_fs.failure_reason.empty()
        ? "formation broken" : cached_fs.failure_reason;
      bb->set<std::string>("@action_status", "ERROR");
      bb->set<std::string>("@last_error",    err);
      bb->set<std::string>("@action_summary",
        "formation " + formation_id + " broken: " + err);
    }
    else if (cached_fs.state == FormationStatusMsg::STATE_DEGRADED &&
             action_status == "OK")
    {
      const std::string warn =
        "formation degraded, max_error=" +
        std::to_string(cached_fs.max_error_m) + "m";
      bb->set<std::string>("@action_status", "WARN");
      bb->set<std::string>("@last_error",    warn);
      bb->set<std::string>("@action_summary", warn);
    }
    // Only clear WARN — never overwrite a genuine ERROR with OK.
    else if (cached_fs.state == FormationStatusMsg::STATE_STABLE &&
             action_status == "WARN")
    {
      bb->set<std::string>("@action_status", "OK");
      bb->set<std::string>("@last_error",    "");
    }
  }

  publish_bt_state(bb, publisher_, clock_, have_fs ? &cached_fs : nullptr);
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
    std::bind(&LlmCommandReceiver::handle_goal,     this, _1, _2),
    std::bind(&LlmCommandReceiver::handle_cancel,   this, _1),
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
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LlmCommandReceiver::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
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

  RCLCPP_INFO(node->get_logger(),
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
}

}  // namespace iros_llm_swarm_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<iros_llm_swarm_bt::MapfPlan>("MapfPlan");
  factory.registerNodeType<iros_llm_swarm_bt::SetFormation>("SetFormation");
  factory.registerNodeType<iros_llm_swarm_bt::DisableFormation>("DisableFormation");
  factory.registerNodeType<iros_llm_swarm_bt::CheckMode>("CheckMode");
  factory.registerNodeType<iros_llm_swarm_bt::BTStatePublisher>("BTStatePublisher");
  factory.registerNodeType<iros_llm_swarm_bt::LlmCommandReceiver>("LlmCommandReceiver");
}