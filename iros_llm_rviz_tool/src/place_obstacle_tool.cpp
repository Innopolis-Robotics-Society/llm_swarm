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
  list_client_   = node_->create_client<ListObstacles>("/obstacles/list");
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
    if (pending_remove_.exchange(true)) return Render;

    auto req = std::make_shared<ListObstacles::Request>();
    list_client_->async_send_request(req,
      [this, wx, wy](rclcpp::Client<ListObstacles>::SharedFuture future) {
        pending_remove_ = false;
        auto resp = future.get();

        std::string nearest_id;
        double min_dist = std::numeric_limits<double>::max();

        auto check = [&](const std::string & id, double x, double y) {
          const double d = std::hypot(x - wx, y - wy);
          if (d < min_dist) { min_dist = d; nearest_id = id; }
        };
        for (const auto & c : resp->circles)    check(c.id, c.position.x, c.position.y);
        for (const auto & r : resp->rectangles) check(r.id, r.position.x, r.position.y);
        for (const auto & d : resp->doors)      check(d.id, d.position.x, d.position.y);

        if (nearest_id.empty()) return;
        auto rm_req = std::make_shared<RemoveObstacle::Request>();
        rm_req->id  = nearest_id;
        remove_client_->async_send_request(rm_req);
        RCLCPP_INFO(node_->get_logger(), "PlaceObstacleTool: removed '%s'", nearest_id.c_str());
      });
  }

  return Render;
}

}  // namespace iros_llm_rviz_tool

PLUGINLIB_EXPORT_CLASS(iros_llm_rviz_tool::PlaceObstacleTool, rviz_common::Tool)
