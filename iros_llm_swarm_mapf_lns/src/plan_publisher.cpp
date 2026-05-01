// iros_llm_swarm_mapf/src/plan_publisher.cpp

#include "iros_llm_swarm_mapf/plan_publisher.hpp"

#include "iros_llm_swarm_mapf/node_utils.hpp"

namespace lns2_node {

void PlanPublisher::init(rclcpp::Node* node, int num_robots)
{
  plan_pubs_.resize(num_robots);
  cmd_vel_pubs_.resize(num_robots);
  for (int i = 0; i < num_robots; ++i) {
    plan_pubs_[i] = node->create_publisher<MAPFPlanMsg>(
        "/robot_" + std::to_string(i) + "/mapf_plan", 10);
    // Best-effort latched-override publisher for emergency stop.
    // Published only on cancel to zero out velocity before Nav2 finishes
    // processing its cancel request (Nav2 cancel can take 1-3 s).
    cmd_vel_pubs_[i] = node->create_publisher<geometry_msgs::msg::Twist>(
        "/robot_" + std::to_string(i) + "/cmd_vel", 1);
  }
}

PlanPublisher::PublishStats PlanPublisher::publish_mapf_plans(
    const std::vector<lns2::Path>& paths,
    const std::vector<std::uint32_t>& plan_ids_ext,
    double origin_x, double origin_y, double resolution,
    double time_step_sec,
    const rclcpp::Time& stamp)
{
  PublishStats out;
  ++plan_id_counter_;
  active_plan_id_ = plan_id_counter_;
  out.plan_id = active_plan_id_;

  const int N = num_robots();
  for (std::size_t k = 0; k < paths.size() && k < plan_ids_ext.size(); ++k) {
    const std::uint32_t rid = plan_ids_ext[k];
    if (rid >= static_cast<std::uint32_t>(N)) continue;

    MAPFPlanMsg plan_msg;
    plan_msg.header.frame_id = "map";
    plan_msg.header.stamp    = stamp;
    plan_msg.plan_id         = active_plan_id_;

    // Single-cell paths are stub plans (warm-start "stay put"). Treat as
    // empty so the follower cancels rather than masking the failure with
    // a fake PLAN_COMPLETE. Empty paths likewise cancel.
    const bool is_stub = (paths[k].size() == 1);
    if (!paths[k].empty() && !is_stub) {
      plan_msg.steps = grid_path_to_steps(paths[k], origin_x, origin_y,
                                           resolution, time_step_sec);
      ++out.published;
    } else {
      ++out.cancelled;
    }
    plan_pubs_[rid]->publish(plan_msg);
  }
  return out;
}

void PlanPublisher::publish_stop_all(
    const std::vector<std::uint32_t>& active_robot_ids,
    const rclcpp::Time& stamp)
{
  ++plan_id_counter_;
  geometry_msgs::msg::Twist zero_vel;  // all fields default to 0
  const int N = num_robots();
  for (std::uint32_t rid : active_robot_ids) {
    if (rid >= static_cast<std::uint32_t>(N)) continue;
    MAPFPlanMsg empty;
    empty.header.frame_id = "map";
    empty.header.stamp    = stamp;
    empty.plan_id         = plan_id_counter_;
    plan_pubs_[rid]->publish(empty);
    // Direct zero-velocity override so the robot stops immediately,
    // before the path_follower finishes cancelling its Nav2 goal.
    cmd_vel_pubs_[rid]->publish(zero_vel);
  }
}

void PlanPublisher::publish_cancel_for(std::uint32_t rid,
                                        const rclcpp::Time& stamp)
{
  if (rid >= static_cast<std::uint32_t>(num_robots())) return;
  MAPFPlanMsg cancel;
  cancel.header.frame_id = "map";
  cancel.header.stamp    = stamp;
  cancel.plan_id         = ++plan_id_counter_;
  plan_pubs_[rid]->publish(cancel);
}

}  // namespace lns2_node
