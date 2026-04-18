#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "iros_llm_swarm_interfaces/action/set_goals.hpp"
#include "iros_llm_swarm_interfaces/srv/disband_formation.hpp"
#include "iros_llm_swarm_interfaces/srv/set_formation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace iros_llm_swarm_bt 
{

class MapfPlan : public BT::SyncActionNode
{
public:
  using SetGoals = iros_llm_swarm_interfaces::action::SetGoals;

  MapfPlan(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetFormation : public BT::SyncActionNode
{
public:
  using SetFormationSrv = iros_llm_swarm_interfaces::srv::SetFormation;

  SetFormation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class DisableFormation : public BT::SyncActionNode
{
public:
  using DisbandFormationSrv = iros_llm_swarm_interfaces::srv::DisbandFormation;

  DisableFormation(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace iros_llm_swarm_bt