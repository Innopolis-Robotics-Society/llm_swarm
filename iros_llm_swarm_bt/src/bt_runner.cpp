/**
 * bt_runner.cpp
 *
 * Production BT host for the IROS LLM Swarm.
 *
 * Loads the behavior tree from the package's behavior_trees/ directory,
 * ticks it at a configurable rate (param bt_freq, default 10 Hz), and
 * spins rclcpp callbacks between ticks. All mode transitions, all
 * blackboard writes, all retry policy decisions live OUTSIDE the BT
 * — either in the external Python LLM agent or in scripts/demo_20_robot.py.
 *
 * ROS params:
 *   ~bt_freq  (double, default 10.0)  — tree tick rate in Hz
 */

#include <chrono>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "geometry_msgs/msg/point.hpp"
#include "iros_llm_swarm_bt/swarm_bt_nodes.hpp"
#include "iros_llm_swarm_interfaces/msg/bt_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using Point   = geometry_msgs::msg::Point;
using BTState = iros_llm_swarm_interfaces::msg::BTState;

class RclcppDebugLogger : public BT::StatusChangeLogger
{
public:
  RclcppDebugLogger(const BT::Tree & tree, rclcpp::Logger logger)
  : BT::StatusChangeLogger(tree.rootNode()), logger_(logger) {}

  void callback(
    BT::Duration /*timestamp*/, const BT::TreeNode & node,
    BT::NodeStatus prev_status, BT::NodeStatus status) override
  {
    RCLCPP_DEBUG(
      logger_, "%-25s %s -> %s",
      node.name().c_str(),
      BT::toStr(prev_status, false).c_str(),
      BT::toStr(status, false).c_str());
  }

  void flush() override {}

private:
  rclcpp::Logger logger_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_runner");

  node->declare_parameter("bt_freq", 10.0);
  const double bt_freq = node->get_parameter("bt_freq").as_double();
  if (bt_freq <= 0.0 || bt_freq > 1000.0) {
    RCLCPP_FATAL(node->get_logger(),
      "bt_freq must be in (0, 1000] Hz, got %f", bt_freq);
    return 1;
  }

  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin("libiros_llm_swarm_bt_nodes.so");

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::string>("@mode", "idle");
  blackboard->set<int>("@command_seq", 0);
  blackboard->set<std::vector<int>>("@robot_ids", {});
  blackboard->set<std::vector<Point>>("@goals", {});

  const auto xml_file =
    ament_index_cpp::get_package_share_directory("iros_llm_swarm_bt") +
    "/behavior_trees/swarm_navigate_to_pose.xml";

  auto tree = factory.createTreeFromFile(xml_file, blackboard);
  RclcppDebugLogger logger(tree, node->get_logger());

  RCLCPP_INFO(node->get_logger(),
    "bt_runner started: ticking tree at %.1f Hz, "
    "send commands via 'ros2 action send_goal /llm/command ...'",
    bt_freq);

  // Tick loop. Tree is reactive: it never reaches a terminal status in
  // steady state (RunOnce masks terminal child statuses with RUNNING).
  // All control flow happens through /llm/command and /bt/state.
  rclcpp::Rate rate(bt_freq);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    tree.tickRoot();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}