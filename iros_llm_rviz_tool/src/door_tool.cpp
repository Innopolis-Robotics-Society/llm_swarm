#include "iros_llm_rviz_tool/door_tool.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

namespace iros_llm_rviz_tool
{

DoorTool::DoorTool()
{
  shortcut_key_ = 'd';
  setName("Door Control");
}

void DoorTool::onInitialize()
{
  setIcon(rviz_common::loadPixmap(
    "package://iros_llm_rviz_tool/icons/send_llm_goal.svg"));

  auto abs = context_->getRosNodeAbstraction().lock();
  if (!abs) return;
  node_ = abs->get_raw_node();

  open_client_  = node_->create_client<OpenDoor>("/doors/open");
  close_client_ = node_->create_client<CloseDoor>("/doors/close");

  door_id_prop_ = new rviz_common::properties::StringProperty(
    "Door ID", "", "ID of door to open/close", getPropertyContainer());
}

int DoorTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!event.leftUp() && !event.rightUp()) return Render;

  const std::string door_id = door_id_prop_->getStdString();
  if (door_id.empty()) {
    RCLCPP_WARN(node_->get_logger(), "DoorTool: Door ID is empty");
    return Render;
  }

  if (event.leftUp()) {
    auto req     = std::make_shared<OpenDoor::Request>();
    req->door_id = door_id;
    open_client_->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "DoorTool: open '%s'", door_id.c_str());
  } else {
    auto req     = std::make_shared<CloseDoor::Request>();
    req->door_id = door_id;
    close_client_->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "DoorTool: close '%s'", door_id.c_str());
  }

  return Render;
}

}  // namespace iros_llm_rviz_tool

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_tool::DoorTool, rviz_common::Tool)
