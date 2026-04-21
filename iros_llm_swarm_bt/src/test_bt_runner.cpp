#include <iostream>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_bt_runner");

  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin("libiros_llm_swarm_bt_nodes.so");

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);

  // Укажи ID робота или список ID через запятую, например "3" или "1,4"
  blackboard->set<std::string>("robot_ids_csv", "2, 3");

  // Общая цель для выбранных роботов
  blackboard->set<double>("goal_x", 15.0);
  blackboard->set<double>("goal_y", 15.0);

  const auto share_dir =
    ament_index_cpp::get_package_share_directory("iros_llm_swarm_bt");

  const auto xml_file =
    share_dir + "/behavior_trees/swarm_navigate_to_pose.xml";

  auto tree = factory.createTreeFromFile(xml_file, blackboard);
  BT::StdCoutLogger logger_cout(tree);

  auto status = tree.tickRoot();

  std::cout << "Tree status: " << BT::toStr(status) << std::endl;

  try {
    std::cout << "mapf_ok: "
              << std::boolalpha
              << blackboard->get<bool>("mapf_ok") << std::endl;
    std::cout << "mapf_info: "
              << blackboard->get<std::string>("mapf_info") << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "Blackboard read error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}