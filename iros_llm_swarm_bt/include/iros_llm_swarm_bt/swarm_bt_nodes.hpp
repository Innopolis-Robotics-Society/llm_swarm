#pragma once

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "iros_llm_swarm_interfaces/action/llm_command.hpp"
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
// MapfPlan
//
// Wraps the /swarm/set_goals action. Reports status via blackboard telemetry
// keys (@action_status, @last_error, @action_summary, @active_action) — no
// LLM advisory channel and no self-writes to control-flow keys like @mode.
//
// Result semantics:
//   - full success      → SUCCESS, @action_status = "OK"
//   - partial plan      → SUCCESS, @action_status = "WARN", mapf_ok = false
//                         (external controller decides whether to issue
//                         replan via /llm/command)
//   - no agents planned → FAILURE, @action_status = "ERROR"
//   - transport error   → FAILURE, @action_status = "ERROR"
//   - halted            → onHalted writes @action_status = "HALTED"
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
  rclcpp_action::Client<SetGoals>::SharedPtr  client_;
  std::shared_future<GoalHandle::SharedPtr>   goal_handle_future_;
  std::shared_future<WrappedResult>           result_future_;
  std::shared_ptr<GoalHandle>                 goal_handle_;

  // Snapshot written from the ROS executor thread (on_feedback),
  // applied to the blackboard only inside onRunning() (BT thread).
  // Blackboard is not thread-safe — never write it outside the BT thread.
  struct FeedbackSnapshot {
    std::string summary;
    std::string status;       // "OK" | "WARN"
    std::string error;
    bool updated{false};
  };
  std::mutex         snapshot_mutex_;
  FeedbackSnapshot   pending_snapshot_;

  void on_feedback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Feedback> feedback);

  void cancel_mapf();
};

// ---------------------------------------------------------------------------
// SetFormation — async wrapper around /formation/set service
//
// Reports status via blackboard telemetry keys. On service failure returns
// FAILURE — retry / replan / abort decisions belong to the external
// controller, which sees the error via /bt/state and can issue a fresh
// /llm/command if appropriate.
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

  std::string last_error_;

  bool start_service_call();
};

// ---------------------------------------------------------------------------
// DisableFormation — async wrapper around /formation/deactivate service
//
// Same telemetry-only pattern as SetFormation.
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

  std::string last_error_;

  bool start_service_call();
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
// RunOnce — decorator that ticks the child once, then returns RUNNING
// indefinitely (masking the child's terminal SUCCESS/FAILURE from the parent).
// Re-ticks the child when:
//   - external halt() (e.g. parent ReactiveFallback switching branches)
//   - the trigger input port changes value
//
// The child's terminal status is observable via blackboard telemetry
// (@action_status, @active_action, @last_error) — external controllers read
// /bt/state, not the tree's root status.
//
// Use in XML:
//   <RunOnce trigger="{@command_seq}">
//     <MapfPlan ... />
//   </RunOnce>
//
// LlmCommandReceiver increments @command_seq every time it applies a new
// command, so a fresh /llm/command goal causes a fresh tick of the wrapped
// node, even if the surrounding mode-branch never short-circuited to idle.
// ---------------------------------------------------------------------------
class RunOnce : public BT::DecoratorNode
{
public:
  RunOnce(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  bool already_done_{false};
  BT::NodeStatus stored_status_{BT::NodeStatus::IDLE};
  int last_trigger_{-1};
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
// haltTree wipes the blackboard). Formation health fields are read from the
// blackboard (populated by FormationHealthMonitor).
// ---------------------------------------------------------------------------
void publish_bt_state(
  const BT::Blackboard::Ptr & blackboard,
  rclcpp::Publisher<iros_llm_swarm_interfaces::msg::BTState>::SharedPtr publisher,
  const rclcpp::Clock::SharedPtr & clock);

// ---------------------------------------------------------------------------
// FormationHealthMonitor — subscribes to /formations/status, caches per-
// formation health, publishes formation fields to the blackboard for
// BTStatePublisher to read, and escalates @action_status when the active
// formation degrades or breaks. Always returns SUCCESS.
//
// Owns the /formations/status subscription. Run this BEFORE BTStatePublisher
// in the tree so that escalations and formation fields are visible in the
// same tick's /bt/state snapshot.
// ---------------------------------------------------------------------------
class FormationHealthMonitor : public BT::SyncActionNode
{
public:
  using FormationsStatusMsg = iros_llm_swarm_interfaces::msg::FormationsStatus;
  using FormationStatusMsg  = iros_llm_swarm_interfaces::msg::FormationStatus;

  FormationHealthMonitor(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  // Cache written from the ROS executor thread, read inside tick() (BT thread).
  rclcpp::Subscription<FormationsStatusMsg>::SharedPtr sub_;
  mutable std::mutex cache_mutex_;
  std::unordered_map<std::string, FormationStatusMsg> cache_;

  void on_formation_status(const FormationsStatusMsg::SharedPtr msg);
};

// ---------------------------------------------------------------------------
// BTStatePublisher — pure publisher. Each tick snapshots the blackboard and
// publishes /bt/state. Always returns SUCCESS so it does not break the
// surrounding ReactiveSequence. No subscriptions, no side effects on the
// blackboard — formation health comes from FormationHealthMonitor.
// ---------------------------------------------------------------------------
class BTStatePublisher : public BT::SyncActionNode
{
public:
  using BTStateMsg = iros_llm_swarm_interfaces::msg::BTState;

  BTStatePublisher(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<BTStateMsg>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
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

  // Hard upper bound on robot ids accepted by /llm/command. Far above the
  // documented 20-robot fleet ceiling — exists only to reject obviously
  // malformed / hostile goals (an attacker publishing robot_id = 1e9 should
  // never be echoed into /bt/state or the JSONL audit log).
  static constexpr uint32_t kMaxRobotId = 1024;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const LlmCommand::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  void apply_to_blackboard(std::shared_ptr<const LlmCommand::Goal> goal);
};

}  // namespace iros_llm_swarm_bt