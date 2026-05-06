#pragma once

#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "iros_llm_swarm_interfaces/action/llm_command.hpp"
#include "iros_llm_swarm_interfaces/action/llm_decision.hpp"
#include "iros_llm_swarm_interfaces/action/set_goals.hpp"
#include "iros_llm_swarm_interfaces/msg/bt_state.hpp"
#include "iros_llm_swarm_interfaces/msg/formations_status.hpp"
#include "iros_llm_swarm_interfaces/msg/formation_status.hpp"
#include "iros_llm_swarm_interfaces/srv/deactivate_formation.hpp"
#include "iros_llm_swarm_interfaces/srv/set_formation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace iros_llm_swarm_bt
{

// ---------------------------------------------------------------------------
// Helper: non-blocking future check
// ---------------------------------------------------------------------------
template<typename F>
inline bool future_ready(const F & f)
{
  return f.valid() &&
    f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

// ---------------------------------------------------------------------------
// LLM types shared by every BT node that talks to /llm/decision
// ---------------------------------------------------------------------------
using LlmDecision      = iros_llm_swarm_interfaces::action::LlmDecision;
using LlmGoalHandle    = rclcpp_action::ClientGoalHandle<LlmDecision>;
using LlmWrappedResult = rclcpp_action::ClientGoalHandle<LlmDecision>::WrappedResult;

// ---------------------------------------------------------------------------
// MapfPlan
// ---------------------------------------------------------------------------
class MapfPlan : public BT::StatefulActionNode
{
public:
  using SetGoals      = iros_llm_swarm_interfaces::action::SetGoals;
  using GoalHandle    = rclcpp_action::ClientGoalHandle<SetGoals>;
  using WrappedResult = rclcpp_action::ClientGoalHandle<SetGoals>::WrappedResult;
  using Feedback      = SetGoals::Feedback;

  MapfPlan(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  // MAPF action client
  rclcpp_action::Client<SetGoals>::SharedPtr  client_;
  std::shared_future<GoalHandle::SharedPtr>   goal_handle_future_;
  std::shared_future<WrappedResult>           result_future_;
  std::shared_ptr<GoalHandle>                 goal_handle_;

  // LLM action client
  rclcpp_action::Client<LlmDecision>::SharedPtr llm_client_;
  std::shared_future<LlmGoalHandle::SharedPtr>  llm_goal_handle_future_;
  std::shared_future<LlmWrappedResult>          llm_result_future_;
  std::shared_ptr<LlmGoalHandle>                llm_goal_handle_;
  bool                                          llm_pending_{false};

  // Decision from LLM — written in feedback cb, read in onRunning
  std::mutex   decision_mutex_;
  std::string  pending_decision_;  // "wait" | "abort" | "replan" | ""

  // Info log ring buffer
  std::mutex              buffer_mutex_;
  std::deque<std::string> info_buffer_;
  std::size_t             max_info_buffer_{50};

  // Snapshot written from the ROS executor thread (on_feedback),
  // applied to the blackboard only inside onRunning() (BT thread).
  // Blackboard is not thread-safe — never write it outside the BT thread.
  struct FeedbackSnapshot {
    std::string summary;
    std::string status;       // "OK" | "WARN"
    std::string error;
    std::string warn_event;   // non-empty → trigger send_to_llm("WARN", ...)
    std::string info_event;   // non-empty → candidate for periodic INFO log
    bool updated{false};
  };
  std::mutex         snapshot_mutex_;
  FeedbackSnapshot   pending_snapshot_;

  // Periodic log to LLM (0 = disabled)
  double        llm_log_interval_sec_{0.0};
  rclcpp::Time  last_llm_log_time_{0, 0, RCL_ROS_TIME};

  void on_feedback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Feedback> feedback);

  void send_to_llm(const std::string & level, const std::string & event);
  void cancel_mapf();
};

// ---------------------------------------------------------------------------
// SetFormation — async wrapper around /formation/set service
//
// Flow when the service returns success=false:
//   1. send WARN event to /llm/decision with the error message,
//   2. stay RUNNING until the verdict arrives,
//   3. apply the verdict:
//        "abort"  -> FAILURE, formation_warn set,
//        "replan" -> FAILURE, @formation_decision="replan" on blackboard,
//        "wait"   -> retry the service call (bounded by formation_llm_max_retries).
// ---------------------------------------------------------------------------
class SetFormation : public BT::StatefulActionNode
{
public:
  using SetFormationSrv = iros_llm_swarm_interfaces::srv::SetFormation;
  using ServiceFuture   = rclcpp::Client<SetFormationSrv>::SharedFuture;

  SetFormation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  rclcpp::Client<SetFormationSrv>::SharedPtr client_;
  ServiceFuture                              future_;

  // LLM action client
  rclcpp_action::Client<LlmDecision>::SharedPtr llm_client_;
  std::shared_future<LlmGoalHandle::SharedPtr>  llm_goal_handle_future_;
  std::shared_future<LlmWrappedResult>          llm_result_future_;
  std::shared_ptr<LlmGoalHandle>                llm_goal_handle_;
  bool                                          llm_pending_{false};

  std::mutex  llm_decision_mutex_;
  std::string llm_pending_decision_;   // "wait" | "abort" | "replan" | ""

  // Retry budget for "wait" verdicts
  int retry_count_{0};
  int max_retries_{2};

  // Last error string surfaced to LLM prompt / output port
  std::string last_error_;

  bool start_service_call();
  void send_to_llm(const std::string & level, const std::string & event);
  void poll_llm_result();
  void cancel_llm();
};

// ---------------------------------------------------------------------------
// DisableFormation — async wrapper around /formation/deactivate service
//
// Same pattern as SetFormation on failure, but since there is no plan to
// "replan" for a disband, the replan verdict is collapsed to abort.
// ---------------------------------------------------------------------------
class DisableFormation : public BT::StatefulActionNode
{
public:
  using DeactivateFormationSrv = iros_llm_swarm_interfaces::srv::DeactivateFormation;
  using ServiceFuture          = rclcpp::Client<DeactivateFormationSrv>::SharedFuture;

  DisableFormation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  rclcpp::Client<DeactivateFormationSrv>::SharedPtr client_;
  ServiceFuture                                     future_;

  rclcpp_action::Client<LlmDecision>::SharedPtr llm_client_;
  std::shared_future<LlmGoalHandle::SharedPtr>  llm_goal_handle_future_;
  std::shared_future<LlmWrappedResult>          llm_result_future_;
  std::shared_ptr<LlmGoalHandle>                llm_goal_handle_;
  bool                                          llm_pending_{false};

  std::mutex  llm_decision_mutex_;
  std::string llm_pending_decision_;

  int retry_count_{0};
  int max_retries_{2};

  std::string last_error_;

  bool start_service_call();
  void send_to_llm(const std::string & level, const std::string & event);
  void poll_llm_result();
  void cancel_llm();
};

// ---------------------------------------------------------------------------
// CheckMode — reads "@mode" from blackboard, compares with "expected" port
// ---------------------------------------------------------------------------
class CheckMode : public BT::ConditionNode
{
public:
  CheckMode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// /bt/state QoS — kept reliable so terminal one-shot states (HALTED / ERROR)
// don't get dropped during 100ms mode flips. Other subscribers can still
// declare BEST_EFFORT — that's compatible with a RELIABLE publisher.
// ---------------------------------------------------------------------------
inline rclcpp::QoS bt_state_qos()
{
  return rclcpp::QoS(20).reliable();
}

// ---------------------------------------------------------------------------
// publish_bt_state — snapshot the blackboard into a BTState message and
// publish it on the supplied publisher. Used both by BTStatePublisher (every
// tick) and by test_bt_runner directly (to flush a terminal snapshot before
// haltTree wipes the blackboard).
// ---------------------------------------------------------------------------
void publish_bt_state(
  const BT::Blackboard::Ptr & blackboard,
  rclcpp::Publisher<iros_llm_swarm_interfaces::msg::BTState>::SharedPtr publisher,
  const rclcpp::Clock::SharedPtr & clock);

// ---------------------------------------------------------------------------
// BTStatePublisher — each tick snapshots blackboard and publishes /bt/state.
// Always returns SUCCESS so it does not break surrounding ReactiveSequence.
// Also subscribes to /formations/status and escalates action_status when
// the active formation degrades or breaks — mirrors how MapfPlan feedback
// feeds into @action_status during MAPF execution.
// ---------------------------------------------------------------------------
class BTStatePublisher : public BT::SyncActionNode
{
public:
  using BTStateMsg          = iros_llm_swarm_interfaces::msg::BTState;
  using FormationsStatusMsg = iros_llm_swarm_interfaces::msg::FormationsStatus;
  using FormationStatusMsg  = iros_llm_swarm_interfaces::msg::FormationStatus;

  BTStatePublisher(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<BTStateMsg>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  // Formation monitor cache — written from the ROS executor thread via the
  // /formations/status subscription, read inside tick() (BT thread).
  // Protected by formation_cache_mutex_.
  rclcpp::Subscription<FormationsStatusMsg>::SharedPtr formation_status_sub_;
  mutable std::mutex formation_cache_mutex_;
  std::unordered_map<std::string, FormationStatusMsg> formation_cache_;

  void on_formation_status(const FormationsStatusMsg::SharedPtr msg);
  std::string get_str(const std::string & key, const std::string & def = "");
};

// ---------------------------------------------------------------------------
// LlmCommandReceiver — action server on /llm/command.
// Goals arrive from PassiveObserver on the executor thread; the actual
// blackboard writes happen in tick() (BT thread). A mutex protects the
// pending slot because blackboard is not thread-safe.
// ---------------------------------------------------------------------------
class LlmCommandReceiver : public BT::SyncActionNode
{
public:
  using LlmCommand = iros_llm_swarm_interfaces::action::LlmCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<LlmCommand>;

  LlmCommandReceiver(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp_action::Server<LlmCommand>::SharedPtr action_server_;
  std::mutex pending_mutex_;
  std::shared_ptr<const LlmCommand::Goal> pending_goal_;
  std::shared_ptr<GoalHandle> pending_handle_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const LlmCommand::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  void apply_to_blackboard(std::shared_ptr<const LlmCommand::Goal> goal);
};

}  // namespace iros_llm_swarm_bt