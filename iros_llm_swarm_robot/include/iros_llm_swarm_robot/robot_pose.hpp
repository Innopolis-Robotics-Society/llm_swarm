#pragma once

namespace iros_llm_swarm_robot
{

/** Plain pose snapshot. Owned by MotionControllerNode, read by executors. */
struct RobotPose
{
  double x     = 0.0;
  double y     = 0.0;
  double yaw   = 0.0;
  bool   valid = false;
};

}  // namespace iros_llm_swarm_robot
