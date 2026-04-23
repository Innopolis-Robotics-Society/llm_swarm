#pragma once

#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "iros_llm_swarm_interfaces/action/llm_decision.hpp"
#include "iros_llm_swarm_interfaces/action/set_goals.hpp"
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
// ---------------------------------------------------------------------------
class MapfPlan : public BT::StatefulActionNode
{
public:
  using SetGoals      = iros_llm_swarm_interfaces::action::SetGoals;
  using GoalHandle    = rclcpp_action::ClientGoalHandle<SetGoals>;
  using WrappedResult = rclcpp_action::ClientGoalHandle<SetGoals>::WrappedResult;
  using Feedback      = SetGoals::Feedback;

  using LlmDecision      = iros_llm_swarm_interfaces::action::LlmDecision;
  using LlmGoalHandle    = rclcpp_action::ClientGoalHandle<LlmDecision>;
  using LlmWrappedResult = rclcpp_action::ClientGoalHandle<LlmDecision>::WrappedResult;

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
};

// ---------------------------------------------------------------------------
// DisableFormation — async wrapper around /formation/deactivate service
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

}  // namespace iros_llm_swarm_bt