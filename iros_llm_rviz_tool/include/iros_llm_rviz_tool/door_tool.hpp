#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "iros_llm_swarm_interfaces/srv/open_door.hpp"
#include "iros_llm_swarm_interfaces/srv/close_door.hpp"

namespace iros_llm_rviz_tool
{

class DoorTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  using OpenDoor  = iros_llm_swarm_interfaces::srv::OpenDoor;
  using CloseDoor = iros_llm_swarm_interfaces::srv::CloseDoor;

  DoorTool();
  ~DoorTool() override = default;

  void onInitialize() override;
  void activate() override {}
  void deactivate() override {}
  int  processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<OpenDoor>::SharedPtr  open_client_;
  rclcpp::Client<CloseDoor>::SharedPtr close_client_;

  rviz_common::properties::StringProperty * door_id_prop_ = nullptr;
};

}  // namespace iros_llm_rviz_tool
