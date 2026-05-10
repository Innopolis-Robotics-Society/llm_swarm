#include "iros_llm_rviz_tool/place_obstacle_tool.hpp"

#include <cmath>
#include <limits>

#include <QIcon>
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

PlaceObstacleTool::PlaceObstacleTool()
{
  shortcut_key_ = 'b';
  setName("Place Obstacle");
}

void PlaceObstacleTool::onInitialize()
{
  setIcon(rviz_common::loadPixmap(
    "package://iros_llm_rviz_tool/icons/send_llm_goal.svg"));

  auto abs = context_->getRosNodeAbstraction().lock();
  if (!abs) return;
  node_ = abs->get_raw_node();

  circle_client_ = node_->create_client<AddCircle>("/obstacles/add_circle");
  rect_client_   = node_->create_client<AddRectangle>("/obstacles/add_rectangle");
  door_client_   = node_->create_client<AddDoor>("/obstacles/add_door");
  remove_client_ = node_->create_client<RemoveObstacle>("/obstacles/remove");

  shape_prop_ = new rviz_common::properties::EnumProperty(
    "Shape", "circle", "Obstacle shape to place", getPropertyContainer());
  shape_prop_->addOption("circle",    0);
  shape_prop_->addOption("rectangle", 1);
  shape_prop_->addOption("door",      2);

  radius_prop_ = new rviz_common::properties::FloatProperty(
    "Radius (m)", 0.5f, "Circle radius", getPropertyContainer());
  radius_prop_->setMin(0.05f);

  width_prop_ = new rviz_common::properties::FloatProperty(
    "Width (m)", 1.5f, "Rectangle / door width", getPropertyContainer());
  width_prop_->setMin(0.05f);

  height_prop_ = new rviz_common::properties::FloatProperty(
    "Height (m)", 0.4f, "Rectangle / door height", getPropertyContainer());
  height_prop_->setMin(0.05f);

  is_open_prop_ = new rviz_common::properties::BoolProperty(
    "Door: start open", true, "Initial state for door shape", getPropertyContainer());
}

bool PlaceObstacleTool::getGroundPosition(
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

int PlaceObstacleTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  double wx, wy;
  if (!getGroundPosition(event, wx, wy)) return Render;

  if (event.leftUp()) {
    const std::string shape = shape_prop_->getStdString();
    const std::string id    = "obs_" + std::to_string(obs_counter_++);
    placed_.emplace_back(id, wx, wy);

    if (shape == "circle") {
      auto req        = std::make_shared<AddCircle::Request>();
      req->id         = id;
      req->position.x = wx; req->position.y = wy;
      req->radius     = static_cast<double>(radius_prop_->getFloat());
      circle_client_->async_send_request(req);
      RCLCPP_INFO(node_->get_logger(), "PlaceObstacleTool: circle '%s' at (%.2f, %.2f)", id.c_str(), wx, wy);

    } else if (shape == "rectangle") {
      auto req        = std::make_shared<AddRectangle::Request>();
      req->id         = id;
      req->position.x = wx; req->position.y = wy;
      req->width      = static_cast<double>(width_prop_->getFloat());
      req->height     = static_cast<double>(height_prop_->getFloat());
      rect_client_->async_send_request(req);
      RCLCPP_INFO(node_->get_logger(), "PlaceObstacleTool: rectangle '%s' at (%.2f, %.2f)", id.c_str(), wx, wy);

    } else if (shape == "door") {
      auto req        = std::make_shared<AddDoor::Request>();
      req->id         = id;
      req->position.x = wx; req->position.y = wy;
      req->width      = static_cast<double>(width_prop_->getFloat());
      req->height     = static_cast<double>(height_prop_->getFloat());
      req->is_open    = is_open_prop_->getBool();
      door_client_->async_send_request(req);
      RCLCPP_INFO(node_->get_logger(), "PlaceObstacleTool: door '%s' at (%.2f, %.2f)", id.c_str(), wx, wy);
    }

  } else if (event.rightUp()) {
    if (placed_.empty()) return Render;
    size_t nearest  = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < placed_.size(); ++i) {
      const double d = std::hypot(std::get<1>(placed_[i]) - wx, std::get<2>(placed_[i]) - wy);
      if (d < min_dist) { min_dist = d; nearest = i; }
    }
    const std::string rm_id = std::get<0>(placed_[nearest]);
    placed_.erase(placed_.begin() + static_cast<ptrdiff_t>(nearest));
    auto req = std::make_shared<RemoveObstacle::Request>();
    req->id  = rm_id;
    remove_client_->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "PlaceObstacleTool: removed '%s'", rm_id.c_str());
  }

  return Render;
}

}  // namespace iros_llm_rviz_tool

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_tool::PlaceObstacleTool, rviz_common::Tool)
