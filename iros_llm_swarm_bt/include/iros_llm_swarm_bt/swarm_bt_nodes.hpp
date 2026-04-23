#pragma once

#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
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
// Sends robot_ids + goals to /swarm/set_goals (long-lived action).
// RUNNING while action executes. onHalted() cancels the action.
// Ports: robot_ids (vector<int>), goals (vector<Point>) — caller provides coordinates
// ---------------------------------------------------------------------------
class MapfPlan : public BT::StatefulActionNode
{
public:
  using SetGoals      = iros_llm_swarm_interfaces::action::SetGoals;
  using GoalHandle    = rclcpp_action::ClientGoalHandle<SetGoals>;
  using WrappedResult = rclcpp_action::ClientGoalHandle<SetGoals>::WrappedResult;

  MapfPlan(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  rclcpp_action::Client<SetGoals>::SharedPtr client_;
  std::shared_future<GoalHandle::SharedPtr>  goal_handle_future_;
  std::shared_future<WrappedResult>          result_future_;
  std::shared_ptr<GoalHandle>                goal_handle_;
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
  using ServiceFuture       = rclcpp::Client<DeactivateFormationSrv>::SharedFuture;

  DisableFormation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  rclcpp::Client<DeactivateFormationSrv>::SharedPtr client_;
  ServiceFuture                                  future_;
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