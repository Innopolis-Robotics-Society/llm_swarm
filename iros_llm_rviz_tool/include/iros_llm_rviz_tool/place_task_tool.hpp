#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "iros_llm_swarm_interfaces/srv/add_task.hpp"
#include "iros_llm_swarm_interfaces/srv/reset_task.hpp"
#include "iros_llm_swarm_interfaces/srv/remove_task.hpp"

namespace iros_llm_rviz_tool
{

class PlaceTaskTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  using AddTask    = iros_llm_swarm_interfaces::srv::AddTask;
  using ResetTask  = iros_llm_swarm_interfaces::srv::ResetTask;
  using RemoveTask = iros_llm_swarm_interfaces::srv::RemoveTask;

  PlaceTaskTool();
  ~PlaceTaskTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int  processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  bool getGroundPosition(rviz_common::ViewportMouseEvent & event, double & wx, double & wy);
  void placePoint(double wx, double wy);
  void placeCarry(double px, double py, double dx, double dy);
  void resetNearest(double wx, double wy);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<AddTask>::SharedPtr    add_client_;
  rclcpp::Client<ResetTask>::SharedPtr  reset_client_;
  rclcpp::Client<RemoveTask>::SharedPtr remove_client_;

  rviz_common::properties::EnumProperty   * type_prop_   = nullptr;
  rviz_common::properties::StringProperty * label_prop_  = nullptr;
  rviz_common::properties::FloatProperty  * radius_prop_ = nullptr;

  // carry two-click state
  std::optional<std::pair<double, double>> pending_pickup_;

  // tasks placed this session: id -> position (for nearest-reset lookup)
  std::map<std::string, std::pair<double, double>> placed_;
  std::atomic<bool> pending_reset_{false};
  int task_counter_ = 0;
};

}  // namespace iros_llm_rviz_tool
