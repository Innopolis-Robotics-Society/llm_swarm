#include "iros_llm_rviz_tool/door_tool.hpp"

#include <cmath>
#include <limits>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>

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

  list_client_  = node_->create_client<ListObstacles>("/obstacles/list");
  open_client_  = node_->create_client<OpenDoor>("/doors/open");
  close_client_ = node_->create_client<CloseDoor>("/doors/close");
}

bool DoorTool::getGroundPosition(
    rviz_common::ViewportMouseEvent & event, double & wx, double & wy)
{
  auto * rw     = event.panel->getRenderWindow();
  auto * camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(rw);
  auto * vp     = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(rw);
  if (!camera || !vp) return false;
  Ogre::Ray ray = camera->getCameraToViewportRay(
    static_cast<float>(event.x) / static_cast<float>(vp->getActualWidth()),
    static_cast<float>(event.y) / static_cast<float>(vp->getActualHeight()));
  Ogre::Plane ground(Ogre::Vector3::UNIT_Z, 0.0f);
  auto result = ray.intersects(ground);
  if (!result.first) return false;
  Ogre::Vector3 pt = ray.getPoint(result.second);
  wx = static_cast<double>(pt.x);
  wy = static_cast<double>(pt.y);
  return true;
}

int DoorTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (!event.leftUp()) return Render;

  double wx, wy;
  if (!getGroundPosition(event, wx, wy)) return Render;

  if (pending_.exchange(true)) return Render;  // drop if already waiting

  auto req = std::make_shared<ListObstacles::Request>();
  list_client_->async_send_request(req,
    [this, wx, wy](rclcpp::Client<ListObstacles>::SharedFuture future) {
      pending_ = false;
      auto resp = future.get();
      if (resp->doors.empty()) {
        RCLCPP_WARN(node_->get_logger(), "DoorTool: no doors registered");
        return;
      }

      size_t nearest  = 0;
      double min_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < resp->doors.size(); ++i) {
        const auto & d = resp->doors[i];
        const double dist = std::hypot(d.position.x - wx, d.position.y - wy);
        if (dist < min_dist) { min_dist = dist; nearest = i; }
      }

      const auto & door = resp->doors[nearest];
      if (door.is_open) {
        auto req2    = std::make_shared<CloseDoor::Request>();
        req2->door_id = door.id;
        close_client_->async_send_request(req2);
        RCLCPP_INFO(node_->get_logger(), "DoorTool: closing '%s'", door.id.c_str());
      } else {
        auto req2    = std::make_shared<OpenDoor::Request>();
        req2->door_id = door.id;
        open_client_->async_send_request(req2);
        RCLCPP_INFO(node_->get_logger(), "DoorTool: opening '%s'", door.id.c_str());
      }
    });

  return Render;
}

}  // namespace iros_llm_rviz_tool

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_tool::DoorTool, rviz_common::Tool)
