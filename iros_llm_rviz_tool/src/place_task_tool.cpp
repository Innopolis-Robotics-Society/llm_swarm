#include "iros_llm_rviz_tool/place_task_tool.hpp"

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

PlaceTaskTool::PlaceTaskTool()
{
  shortcut_key_ = 'k';
  setName("Place Task");
}

void PlaceTaskTool::onInitialize()
{
  setIcon(rviz_common::loadPixmap(
    "package://iros_llm_rviz_tool/icons/send_llm_goal.svg"));

  auto abs = context_->getRosNodeAbstraction().lock();
  if (!abs) return;
  node_ = abs->get_raw_node();

  add_client_    = node_->create_client<AddTask>("/tasks/add");
  reset_client_  = node_->create_client<ResetTask>("/tasks/reset");
  remove_client_ = node_->create_client<RemoveTask>("/tasks/remove");

  type_prop_ = new rviz_common::properties::EnumProperty(
    "Type", "point", "Task type to place", getPropertyContainer());
  type_prop_->addOption("point", 0);
  type_prop_->addOption("carry", 1);

  label_prop_ = new rviz_common::properties::StringProperty(
    "Label", "Task", "Display label", getPropertyContainer());

  radius_prop_ = new rviz_common::properties::FloatProperty(
    "Radius (m)", 1.5f, "Completion radius", getPropertyContainer());
  radius_prop_->setMin(0.1f);
}

void PlaceTaskTool::activate()
{
  pending_pickup_.reset();
}

void PlaceTaskTool::deactivate()
{
  pending_pickup_.reset();
}

bool PlaceTaskTool::getGroundPosition(
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

int PlaceTaskTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  double wx, wy;
  if (!getGroundPosition(event, wx, wy)) return Render;

  const std::string task_type = type_prop_->getStdString();

  if (event.leftUp()) {
    if (task_type == "point") {
      placePoint(wx, wy);
    } else {
      // carry: two-click — first sets pickup, second sets dropoff
      if (!pending_pickup_) {
        pending_pickup_ = {wx, wy};
        RCLCPP_INFO(node_->get_logger(),
          "PlaceTaskTool: carry pickup set at (%.2f, %.2f) — click dropoff", wx, wy);
      } else {
        auto [px, py] = *pending_pickup_;
        pending_pickup_.reset();
        placeCarry(px, py, wx, wy);
      }
    }
  } else if (event.rightUp()) {
    pending_pickup_.reset();
    resetNearest(wx, wy);
  }

  return Render;
}

void PlaceTaskTool::placePoint(double wx, double wy)
{
  const std::string id = "task_" + std::to_string(task_counter_++);
  auto req = std::make_shared<AddTask::Request>();
  req->task.id       = id;
  req->task.type     = "point";
  req->task.label    = label_prop_->getStdString();
  req->task.radius   = static_cast<double>(radius_prop_->getFloat());
  req->task.position = {wx, wy};
  req->task.dropoff  = {0.0, 0.0};

  placed_[id] = {wx, wy};
  add_client_->async_send_request(req);
  RCLCPP_INFO(node_->get_logger(),
    "PlaceTaskTool: point '%s' at (%.2f, %.2f)", id.c_str(), wx, wy);
}

void PlaceTaskTool::placeCarry(double px, double py, double dx, double dy)
{
  const std::string id = "task_" + std::to_string(task_counter_++);
  auto req = std::make_shared<AddTask::Request>();
  req->task.id       = id;
  req->task.type     = "carry";
  req->task.label    = label_prop_->getStdString();
  req->task.radius   = static_cast<double>(radius_prop_->getFloat());
  req->task.position = {px, py};
  req->task.dropoff  = {dx, dy};

  placed_[id] = {px, py};
  add_client_->async_send_request(req);
  RCLCPP_INFO(node_->get_logger(),
    "PlaceTaskTool: carry '%s' pickup (%.2f, %.2f) dropoff (%.2f, %.2f)",
    id.c_str(), px, py, dx, dy);
}

void PlaceTaskTool::resetNearest(double wx, double wy)
{
  if (placed_.empty()) return;
  if (pending_reset_.exchange(true)) return;

  std::string nearest_id;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & [id, pos] : placed_) {
    const double d = std::hypot(pos.first - wx, pos.second - wy);
    if (d < min_dist) { min_dist = d; nearest_id = id; }
  }

  pending_reset_ = false;
  if (nearest_id.empty()) return;

  auto req = std::make_shared<ResetTask::Request>();
  req->id  = nearest_id;
  reset_client_->async_send_request(req,
    [this, nearest_id](rclcpp::Client<ResetTask>::SharedFuture future) {
      const auto resp = future.get();
      if (resp->success) {
        RCLCPP_INFO(node_->get_logger(),
          "PlaceTaskTool: reset '%s'", nearest_id.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "PlaceTaskTool: reset '%s' failed: %s",
          nearest_id.c_str(), resp->message.c_str());
      }
    });
}

}  // namespace iros_llm_rviz_tool

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_tool::PlaceTaskTool, rviz_common::Tool)
