#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace iros_llm_swarm_costmap_plugins {

class ResettingObstacleLayer : public nav2_costmap_2d::ObstacleLayer {
public:
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    resetMaps();
    ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }
};

}  // namespace iros_llm_swarm_costmap_plugins

PLUGINLIB_EXPORT_CLASS(
  iros_llm_swarm_costmap_plugins::ResettingObstacleLayer,
  nav2_costmap_2d::Layer)
