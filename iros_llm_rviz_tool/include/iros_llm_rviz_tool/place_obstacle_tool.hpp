#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include "iros_llm_swarm_interfaces/srv/add_circle.hpp"
#include "iros_llm_swarm_interfaces/srv/add_rectangle.hpp"
#include "iros_llm_swarm_interfaces/srv/add_door.hpp"
#include "iros_llm_swarm_interfaces/srv/list_obstacles.hpp"
#include "iros_llm_swarm_interfaces/srv/remove_obstacle.hpp"

namespace iros_llm_rviz_tool
{

class PlaceObstacleTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  using AddCircle      = iros_llm_swarm_interfaces::srv::AddCircle;
  using AddRectangle   = iros_llm_swarm_interfaces::srv::AddRectangle;
  using AddDoor        = iros_llm_swarm_interfaces::srv::AddDoor;
  using ListObstacles  = iros_llm_swarm_interfaces::srv::ListObstacles;
  using RemoveObstacle = iros_llm_swarm_interfaces::srv::RemoveObstacle;

  PlaceObstacleTool();
  ~PlaceObstacleTool() override = default;

  void onInitialize() override;
  void activate() override {}
  void deactivate() override {}
  int  processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  bool getGroundPosition(rviz_common::ViewportMouseEvent & event, double & wx, double & wy);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<AddCircle>::SharedPtr      circle_client_;
  rclcpp::Client<AddRectangle>::SharedPtr   rect_client_;
  rclcpp::Client<AddDoor>::SharedPtr        door_client_;
  rclcpp::Client<ListObstacles>::SharedPtr  list_client_;
  rclcpp::Client<RemoveObstacle>::SharedPtr remove_client_;

  rviz_common::properties::EnumProperty  * shape_prop_   = nullptr;
  rviz_common::properties::FloatProperty * radius_prop_  = nullptr;
  rviz_common::properties::FloatProperty * width_prop_   = nullptr;
  rviz_common::properties::FloatProperty * height_prop_  = nullptr;
  rviz_common::properties::BoolProperty  * is_open_prop_ = nullptr;

  std::atomic<bool> pending_remove_{false};
  int obs_counter_ = 0;
};

}  // namespace iros_llm_rviz_tool
