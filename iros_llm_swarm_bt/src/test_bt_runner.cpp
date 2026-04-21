/**
 * test_bt_runner.cpp
 *
 * BT runner + embedded integration scenario for 20 robots.
 *
 * ROS param:
 *   ~scenario  (bool, default: false)
 *       false -- idle runner, listens on /fleet/cmd  (String "key=value")
 *       true  -- built-in 20-robot test scenario:
 *
 *   Scenario layout (warehouse.world, 30x30m):
 *     Orange  robot_0..9   -- loading zone,   bottom-left  (~2-4,  2-8)
 *     Blue    robot_10..19 -- unloading zone,  top-right    (~26-28, 22-28)
 *
 *   Step 1: MAPF all 20 -> warehouse center (15,15), 4x5 grid, 1.5m spacing
 *   Step 2: Formation WEDGE_20  -- robot_1  leader, robot_0/2..9  followers (orange squad)
 *   Step 3: Formation LINE_BLUE -- robot_10 leader, robot_11..14  followers (blue squad)
 *   Step 4: MAPF cross-swap -- orange robots -> top-right, blue robots -> bottom-left
 *           Real stress test: two clusters crossing the full warehouse
 *   Step 5: MAPF all 20 back home
 *   Step 6: idle
 *
 * Published topics:
 *   /fleet/mode              std_msgs/String
 *   /fleet/mapf_ok           std_msgs/String  "true"/"false"
 *   /fleet/formation_enabled std_msgs/Bool
 */

#include <atomic>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Point = geometry_msgs::msg::Point;

static std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    while (!tok.empty() && tok.front() == ' ') tok.erase(tok.begin());
    if (!tok.empty()) out.push_back(tok);
  }
  return out;
}

static Point make_point(double x, double y)
{
  Point p; p.x = x; p.y = y; p.z = 0.0; return p;
}

// ---------------------------------------------------------------------------
// Scenario thread
// ---------------------------------------------------------------------------
static void run_scenario(
  BT::Blackboard::Ptr bb,
  rclcpp::Logger log,
  std::atomic<bool> & done)
{
  auto set_mode = [&](const std::string & m) {
    bb->set<std::string>("@mode", m);
    RCLCPP_INFO(log, "[scenario] mode -> %s", m.c_str());
  };

  auto wait_mapf = [&](int timeout_sec) -> bool {
    RCLCPP_INFO(log, "[scenario] waiting for MAPF...");
    bb->set<bool>("@mapf_ok", false);
    bb->set<bool>("@mapf_failed", false);
    for (int i = 0; i < timeout_sec * 10; ++i) {
      std::this_thread::sleep_for(100ms);
      try { if (bb->get<bool>("@mapf_ok"))     return true;  } catch (...) {}
      try { if (bb->get<bool>("@mapf_failed")) return false; } catch (...) {}
    }
    RCLCPP_ERROR(log, "[scenario] TIMEOUT waiting for MAPF");
    return false;
  };

  auto wait_formation = [&](int timeout_sec) -> bool {
    RCLCPP_INFO(log, "[scenario] waiting for formation_enabled...");
    bb->set<bool>("@formation_enabled", false);
    bb->set<bool>("@formation_failed", false);
    for (int i = 0; i < timeout_sec * 10; ++i) {
      std::this_thread::sleep_for(100ms);
      try { if (bb->get<bool>("@formation_enabled"))  return true;  } catch (...) {}
      try { if (bb->get<bool>("@formation_failed"))   return false; } catch (...) {}
    }
    RCLCPP_ERROR(log, "[scenario] TIMEOUT waiting for formation");
    return false;
  };

  auto abort = [&](const char * step) {
    RCLCPP_ERROR(log, "=== SCENARIO FAILED at %s ===", step);
    set_mode("idle");
    done = true;
  };

  // Give the stack time to come up
  std::this_thread::sleep_for(3s);

  // -------------------------------------------------------------------------
  // Step 1: MAPF all 20 robots -> warehouse center (15, 15)
  // 4 columns x 5 rows, 1.5m spacing, centered on (15, 15)
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 1: MAPF all 20 -> center (15,15) ===");
  {
    std::vector<int> ids;
    std::vector<Point> goals;
    const int cols = 4;
    const double spacing = 1.5;
    for (int i = 0; i < 20; ++i) {
      ids.push_back(i);
      goals.push_back(make_point(
        15.0 + ((i % cols) - (cols - 1) / 2.0) * spacing,
        15.0 + ((i / cols) - 2.0) * spacing));
    }
    bb->set<std::vector<int>>("@robot_ids", ids);
    bb->set<std::vector<Point>>("@goals", goals);
  }
  set_mode("mapf");
  if (!wait_mapf(240)) { abort("step 1"); return; }
  RCLCPP_INFO(log, "=== STEP 1 OK ===");
  std::this_thread::sleep_for(2s);

  // -------------------------------------------------------------------------
  // Step 2: Formation WEDGE_20 -- orange squad (robot_0..9)
  // robot_1 is leader, robot_0/2..9 form a wedge behind it
  // Two wings: left wing (robot_2,4,6,8) right wing (robot_0,3,5,7,9)
  //   +x=forward, +y=left in leader body frame
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 2: Formation WEDGE_20 (orange squad) ===");
  set_mode("idle");
  std::this_thread::sleep_for(300ms);
  bb->set<std::string>("@formation_id", "wedge_20");
  bb->set<std::string>("@leader_ns",    "robot_1");
  bb->set<std::vector<std::string>>("@follower_ns", {
    "robot_2", "robot_0",
    "robot_4", "robot_3",
    "robot_6", "robot_5",
    "robot_8", "robot_7",
    "robot_9"
  });
  // Alternating left/right, stepping back 1m per row
  bb->set<std::vector<double>>("@offsets_x", {
    -1.0, -1.0,
    -2.0, -2.0,
    -3.0, -3.0,
    -4.0, -4.0,
    -5.0
  });
  bb->set<std::vector<double>>("@offsets_y", {
     0.8, -0.8,
     1.6, -1.6,
     2.4, -2.4,
     3.2, -3.2,
     0.0
  });
  set_mode("formation");
  if (!wait_formation(15)) { abort("step 2"); return; }
  RCLCPP_INFO(log, "=== STEP 2 OK ===");
  std::this_thread::sleep_for(3s);

  // -------------------------------------------------------------------------
  // Step 3: Formation LINE_BLUE -- blue squad (robot_10..14)
  // robot_10 is leader, robot_11..14 line up behind
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 3: Formation LINE_BLUE (blue squad) ===");
  set_mode("idle");
  std::this_thread::sleep_for(300ms);
  bb->set<std::string>("@formation_id", "line_blue");
  bb->set<std::string>("@leader_ns",    "robot_10");
  bb->set<std::vector<std::string>>("@follower_ns", {
    "robot_11", "robot_12", "robot_13", "robot_14"
  });
  bb->set<std::vector<double>>("@offsets_x", {-1.5, -3.0, -4.5, -6.0});
  bb->set<std::vector<double>>("@offsets_y", { 0.0,  0.0,  0.0,  0.0});
  set_mode("formation");
  if (!wait_formation(15)) { abort("step 3"); return; }
  RCLCPP_INFO(log, "=== STEP 3 OK ===");
  std::this_thread::sleep_for(3s);

  // -------------------------------------------------------------------------
  // Step 4: MAPF cross-swap
  // Orange (0..9) -> top-right unloading zone (mirror of blue home)
  // Blue  (10..19) -> bottom-left loading zone (mirror of orange home)
  // Two clusters cross the full warehouse -- stress test for MAPF replanning
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 4: MAPF cross-swap (full warehouse crossing) ===");
  set_mode("idle");
  std::this_thread::sleep_for(300ms);
  {
    std::vector<int> ids;
    std::vector<Point> goals;

    // Orange robots 0..9 -> top-right zone (where blue robots started)
    const std::vector<std::pair<double,double>> blue_home = {
      {26.0, 22.0}, {27.5, 22.0},
      {26.0, 23.5}, {27.5, 23.5},
      {26.0, 25.0}, {27.5, 25.0},
      {26.0, 26.5}, {27.5, 26.5},
      {26.0, 28.0}, {27.5, 28.0},
    };
    for (int i = 0; i < 10; ++i) {
      ids.push_back(i);
      goals.push_back(make_point(blue_home[i].first, blue_home[i].second));
    }

    // Blue robots 10..19 -> bottom-left zone (where orange robots started)
    const std::vector<std::pair<double,double>> orange_home = {
      {2.0, 2.0}, {3.5, 2.0},
      {2.0, 3.5}, {3.5, 3.5},
      {2.0, 5.0}, {3.5, 5.0},
      {2.0, 6.5}, {3.5, 6.5},
      {2.0, 8.0}, {3.5, 8.0},
    };
    for (int i = 0; i < 10; ++i) {
      ids.push_back(10 + i);
      goals.push_back(make_point(orange_home[i].first, orange_home[i].second));
    }

    bb->set<std::vector<int>>("@robot_ids", ids);
    bb->set<std::vector<Point>>("@goals", goals);
  }
  set_mode("mapf");
  if (!wait_mapf(300)) { abort("step 4"); return; }
  RCLCPP_INFO(log, "=== STEP 4 OK ===");
  std::this_thread::sleep_for(2s);

  // -------------------------------------------------------------------------
  // Step 5: MAPF all 20 back to original home positions
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 5: MAPF all 20 -> home ===");
  set_mode("idle");
  std::this_thread::sleep_for(300ms);
  {
    std::vector<int> ids;
    std::vector<Point> goals;

    // Orange robots 0..9 back to bottom-left
    const std::vector<std::pair<double,double>> orange_home = {
      {2.0, 2.0}, {3.5, 2.0},
      {2.0, 3.5}, {3.5, 3.5},
      {2.0, 5.0}, {3.5, 5.0},
      {2.0, 6.5}, {3.5, 6.5},
      {2.0, 8.0}, {3.5, 8.0},
    };
    for (int i = 0; i < 10; ++i) {
      ids.push_back(i);
      goals.push_back(make_point(orange_home[i].first, orange_home[i].second));
    }

    // Blue robots 10..19 back to top-right
    const std::vector<std::pair<double,double>> blue_home = {
      {26.0, 22.0}, {27.5, 22.0},
      {26.0, 23.5}, {27.5, 23.5},
      {26.0, 25.0}, {27.5, 25.0},
      {26.0, 26.5}, {27.5, 26.5},
      {26.0, 28.0}, {27.5, 28.0},
    };
    for (int i = 0; i < 10; ++i) {
      ids.push_back(10 + i);
      goals.push_back(make_point(blue_home[i].first, blue_home[i].second));
    }

    bb->set<std::vector<int>>("@robot_ids", ids);
    bb->set<std::vector<Point>>("@goals", goals);
  }
  set_mode("mapf");
  if (!wait_mapf(300)) { abort("step 5"); return; }
  RCLCPP_INFO(log, "=== STEP 5 OK ===");

  // -------------------------------------------------------------------------
  // Step 6: Idle
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 6: IDLE -- scenario complete ===");
  set_mode("idle");
  std::this_thread::sleep_for(1s);
  RCLCPP_INFO(log, "ALL 20 ROBOTS -- FULL SCENARIO COMPLETED SUCCESSFULLY");
  done = true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_bt_runner");
  node->declare_parameter("scenario", false);
  const bool run_scenario_mode = node->get_parameter("scenario").as_bool();

  // BT setup
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin("libiros_llm_swarm_bt_nodes.so");

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::string>("@mode", "idle");
  blackboard->set<std::vector<int>>("@robot_ids", {});
  blackboard->set<std::vector<Point>>("@goals", {});

  const auto xml_file =
    ament_index_cpp::get_package_share_directory("iros_llm_swarm_bt") +
    "/behavior_trees/swarm_navigate_to_pose.xml";

  auto tree = factory.createTreeFromFile(xml_file, blackboard);
  BT::StdCoutLogger logger(tree);

  // Publishers
  auto pub_mode = node->create_publisher<std_msgs::msg::String>(
    "/fleet/mode", rclcpp::QoS(1).transient_local());
  auto pub_mapf_ok = node->create_publisher<std_msgs::msg::String>(
    "/fleet/mapf_ok", 10);
  auto pub_form_en = node->create_publisher<std_msgs::msg::Bool>(
    "/fleet/formation_enabled", 10);

  // /fleet/cmd subscriber (idle-runner mode only)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd;
  if (!run_scenario_mode) {
    sub_cmd = node->create_subscription<std_msgs::msg::String>(
      "/fleet/cmd", 10,
      [&](const std_msgs::msg::String::SharedPtr msg) {
        const auto sep = msg->data.find('=');
        if (sep == std::string::npos) return;
        const std::string key   = msg->data.substr(0, sep);
        const std::string value = msg->data.substr(sep + 1);
        RCLCPP_INFO(node->get_logger(), "fleet/cmd: %s=%s", key.c_str(), value.c_str());
        if      (key == "mode")         blackboard->set<std::string>("@mode", value);
        else if (key == "formation_id") blackboard->set<std::string>("@formation_id", value);
        else if (key == "leader_ns")    blackboard->set<std::string>("@leader_ns", value);
        else if (key == "robot_ids") {
          std::vector<int> ids;
          for (const auto & t : split_csv(value)) ids.push_back(std::stoi(t));
          blackboard->set<std::vector<int>>("@robot_ids", ids);
        } else if (key == "goals") {
          auto toks = split_csv(value);
          std::vector<Point> goals;
          for (size_t i = 0; i + 1 < toks.size(); i += 2)
            goals.push_back(make_point(std::stod(toks[i]), std::stod(toks[i+1])));
          blackboard->set<std::vector<Point>>("@goals", goals);
        } else if (key == "follower_ns") {
          blackboard->set<std::vector<std::string>>("@follower_ns", split_csv(value));
        } else if (key == "offsets_x") {
          std::vector<double> v;
          for (const auto & t : split_csv(value)) v.push_back(std::stod(t));
          blackboard->set<std::vector<double>>("@offsets_x", v);
        } else if (key == "offsets_y") {
          std::vector<double> v;
          for (const auto & t : split_csv(value)) v.push_back(std::stod(t));
          blackboard->set<std::vector<double>>("@offsets_y", v);
        }
      });
    RCLCPP_INFO(node->get_logger(), "BT runner: idle mode -- listening on /fleet/cmd");
  }

  // Scenario thread
  std::atomic<bool> scenario_done{false};
  std::thread scenario_thread;
  if (run_scenario_mode) {
    RCLCPP_INFO(node->get_logger(), "BT runner: scenario mode -- starting in 3s");
    scenario_thread = std::thread(
      run_scenario, blackboard, node->get_logger(), std::ref(scenario_done));
  }

  // Tick loop
  rclcpp::Rate rate(10);
  std::string last_mode = "idle";

  while (rclcpp::ok()) {
    if (run_scenario_mode && scenario_done) break;

    rclcpp::spin_some(node);

    std::string current_mode = "idle";
    try { current_mode = blackboard->get<std::string>("@mode"); } catch (...) {}

    // Publish mode
    {
      std_msgs::msg::String m; m.data = current_mode;
      pub_mode->publish(m);
    }

    // Transition -> idle: halt any running action immediately
    if (current_mode == "idle" && last_mode != "idle") {
      tree.haltTree();
      RCLCPP_INFO(node->get_logger(), "BT: mode -> idle, tree halted");
    }
    last_mode = current_mode;

    // Skip tick while idle
    if (current_mode == "idle") {
      rate.sleep();
      continue;
    }

    auto status = tree.tickRoot();

    // Terminal status: publish result, halt, go idle
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      const bool ok = (status == BT::NodeStatus::SUCCESS);

      try {
        std_msgs::msg::String m;
        m.data = blackboard->get<bool>("@mapf_ok") ? "true" : "false";
        pub_mapf_ok->publish(m);
      } catch (...) {}
      try {
        std_msgs::msg::Bool m;
        m.data = blackboard->get<bool>("@formation_enabled");
        pub_form_en->publish(m);
      } catch (...) {}

      RCLCPP_INFO(node->get_logger(), "BT: %s -> %s",
        current_mode.c_str(), BT::toStr(status).c_str());

      if (!ok) {
        // Signal failure to scenario thread, then go idle so tree stops ticking
        blackboard->set<bool>("@mapf_failed", true);
        blackboard->set<bool>("@formation_failed", true);
        blackboard->set<std::string>("@mode", "idle");
        RCLCPP_WARN(node->get_logger(), "BT: FAILURE in mode=%s -> idle", current_mode.c_str());
      }

      tree.haltTree();
    }

    rate.sleep();
  }

  if (scenario_thread.joinable()) scenario_thread.join();
  rclcpp::shutdown();
  return 0;
}