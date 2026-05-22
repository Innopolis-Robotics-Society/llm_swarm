/**
 * test_bt_runner.cpp
 *
 * BT host + embedded 20-robot integration scenario.
 *
 * ROS param:
 *   ~scenario  (bool, default: false)
 *       false -- host only: load the tree and tick at 10 Hz. External
 *                control comes via /llm/command action.
 *       true  -- additionally launch a side thread that runs the built-in
 *                20-robot scenario through the /llm/command action client.
 *
 * After the Phase 4 refactor, this file is intentionally thin: the host
 * loop does nothing except spin rclcpp and call tree.tickRoot(). All mode
 * transitions, all blackboard writes, all retry policy decisions live
 * outside the BT — either in the scenario thread or in the external
 * Python LLM agent.
 *
 *   Scenario layout (warehouse.world, 30x30m):
 *     Orange  robot_0..9   -- loading zone,   bottom-left  (~2-4,  2-8)
 *     Blue    robot_10..19 -- unloading zone, top-right    (~26-28, 22-28)
 *
 *   Step 1: MAPF all 20 -> warehouse center (15,15)
 *   Step 2: Formation WEDGE_20  -- orange squad
 *   Step 3: Formation LINE_BLUE -- blue squad
 *   Step 4: MAPF cross-swap     -- orange to top-right, blue to bottom-left
 *   Step 5: MAPF all 20 back home
 *   Step 6: idle
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "geometry_msgs/msg/point.hpp"
#include "iros_llm_swarm_bt/swarm_bt_nodes.hpp"
#include "iros_llm_swarm_interfaces/action/llm_command.hpp"
#include "iros_llm_swarm_interfaces/msg/bt_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using Point      = geometry_msgs::msg::Point;
using LlmCommand = iros_llm_swarm_interfaces::action::LlmCommand;
using BTState    = iros_llm_swarm_interfaces::msg::BTState;
using LlmGoalHandle = rclcpp_action::ClientGoalHandle<LlmCommand>;

static Point make_point(double x, double y)
{
  Point p; p.x = x; p.y = y; p.z = 0.0; return p;
}

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

// ---------------------------------------------------------------------------
// ScenarioClient — wraps the /llm/command action client + a /bt/state
// subscription. Lives on the scenario thread; all ROS interactions go through
// the shared rclcpp node which is spun on the main thread.
// ---------------------------------------------------------------------------
class ScenarioClient
{
public:
  ScenarioClient(rclcpp::Node::SharedPtr node)
  : node_(node), logger_(node->get_logger())
  {
    cmd_client_ = rclcpp_action::create_client<LlmCommand>(node_, "/llm/command");

    bt_state_sub_ = node_->create_subscription<BTState>(
      "/bt/state", iros_llm_swarm_bt::bt_state_qos(),
      [this](BTState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        latest_state_ = *msg;
        have_state_ = true;
        state_cv_.notify_all();
      });
  }

  bool wait_for_server(std::chrono::seconds timeout)
  {
    return cmd_client_->wait_for_action_server(timeout);
  }

  // Send a command and wait for the action result. Returns true if the
  // command was accepted and applied to the blackboard on the BT thread.
  // This does NOT mean the mission has completed — call wait_for_completion
  // for that.
  bool send(const LlmCommand::Goal & goal, std::chrono::seconds timeout = 5s)
  {
    auto gh_future = cmd_client_->async_send_goal(goal);
    if (gh_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(logger_, "[scenario] send_goal timed out");
      return false;
    }
    auto gh = gh_future.get();
    if (!gh) {
      RCLCPP_ERROR(logger_, "[scenario] goal rejected by LlmCommandReceiver");
      return false;
    }
    auto result_future = cmd_client_->async_get_result(gh);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(logger_, "[scenario] get_result timed out");
      return false;
    }
    auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(logger_, "[scenario] action did not succeed");
      return false;
    }
    return wrapped.result->success;
  }

  // Wait until /bt/state reports the mode we asked for AND the action has
  // completed (@active_action == "none"). Returns @action_status —
  // "OK" / "WARN" / "ERROR" / "HALTED" — or "TIMEOUT" on no answer in time.
  std::string wait_for_completion(
    const std::string & expected_mode, std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lk(state_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      // Tail the state stream tick by tick — wait until a fresh /bt/state
      // arrives, then re-evaluate. wait_until returns false on timeout.
      have_state_ = false;
      if (!state_cv_.wait_until(lk, deadline, [this] { return have_state_; })) {
        break;
      }
      if (latest_state_.mode != expected_mode) {
        continue;  // mode change still propagating after the LlmCommand
      }
      if (latest_state_.active_action == "none") {
        // Action node finished — read terminal status.
        return latest_state_.action_status.empty()
          ? std::string("OK")
          : latest_state_.action_status;
      }
    }
    return "TIMEOUT";
  }

  // Best-effort idle reset. Used between scenario steps and on abort.
  void to_idle()
  {
    LlmCommand::Goal g;
    g.mode = "idle";
    g.reason = "scenario reset";
    send(g);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp_action::Client<LlmCommand>::SharedPtr cmd_client_;
  rclcpp::Subscription<BTState>::SharedPtr bt_state_sub_;

  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  BTState latest_state_;
  bool have_state_{false};
};

// ---------------------------------------------------------------------------
// Scenario thread
// ---------------------------------------------------------------------------
static void run_scenario(
  rclcpp::Node::SharedPtr node,
  std::atomic<bool> & done)
{
  auto log = node->get_logger();
  ScenarioClient client(node);

  RCLCPP_INFO(log, "[scenario] waiting for /llm/command action server...");
  if (!client.wait_for_server(20s)) {
    RCLCPP_ERROR(log, "[scenario] /llm/command never came up");
    done = true;
    return;
  }
  RCLCPP_INFO(log, "[scenario] /llm/command ready");

  auto step_failed = [&](const char * step, const std::string & reason) {
    RCLCPP_ERROR(log, "=== SCENARIO FAILED at %s: %s ===", step, reason.c_str());
    client.to_idle();
    done = true;
  };

  // Give the rest of the stack time to come up
  std::this_thread::sleep_for(3s);

  // -------------------------------------------------------------------------
  // Step 1: MAPF all 20 robots -> warehouse center (15, 15), 4x5 grid
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 1: MAPF all 20 -> center (15,15) ===");
  {
    LlmCommand::Goal g;
    g.mode = "mapf";
    g.reason = "scenario step 1: rally to center";
    const int cols = 4;
    const double spacing = 1.5;
    for (int i = 0; i < 20; ++i) {
      g.robot_ids.push_back(static_cast<uint32_t>(i));
      g.goals.push_back(make_point(
        15.0 + ((i % cols) - (cols - 1) / 2.0) * spacing,
        15.0 + ((i / cols) - 2.0) * spacing));
    }
    if (!client.send(g)) { step_failed("step 1 send", "send_goal failed"); return; }
  }
  if (auto status = client.wait_for_completion("mapf", 240s); status != "OK") {
    step_failed("step 1 wait", status); return;
  }
  RCLCPP_INFO(log, "=== STEP 1 OK ===");

  client.to_idle();
  std::this_thread::sleep_for(2s);

  // -------------------------------------------------------------------------
  // Step 2: Formation WEDGE_20 -- orange squad (robot_0..9)
  // robot_1 leader, robot_0/2..9 alternating left/right wings
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 2: Formation WEDGE_20 (orange squad) ===");
  {
    LlmCommand::Goal g;
    g.mode = "formation";
    g.reason = "scenario step 2: wedge orange";
    g.formation_id = "wedge_20";
    g.leader_ns    = "robot_1";
    g.follower_ns  = {
      "robot_2", "robot_0",
      "robot_4", "robot_3",
      "robot_6", "robot_5",
      "robot_8", "robot_7",
      "robot_9"
    };
    g.offsets_x = { -1.0, -1.0, -2.0, -2.0, -3.0, -3.0, -4.0, -4.0, -5.0 };
    g.offsets_y = {  0.8, -0.8,  1.6, -1.6,  2.4, -2.4,  3.2, -3.2,  0.0 };
    if (!client.send(g)) { step_failed("step 2 send", "send_goal failed"); return; }
  }
  if (auto status = client.wait_for_completion("formation", 15s); status != "OK") {
    step_failed("step 2 wait", status); return;
  }
  RCLCPP_INFO(log, "=== STEP 2 OK ===");
  std::this_thread::sleep_for(3s);

  client.to_idle();
  std::this_thread::sleep_for(300ms);

  // -------------------------------------------------------------------------
  // Step 3: Formation LINE_BLUE -- blue squad (robot_10..14)
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 3: Formation LINE_BLUE (blue squad) ===");
  {
    LlmCommand::Goal g;
    g.mode = "formation";
    g.reason = "scenario step 3: line blue";
    g.formation_id = "line_blue";
    g.leader_ns    = "robot_10";
    g.follower_ns  = { "robot_11", "robot_12", "robot_13", "robot_14" };
    g.offsets_x    = { -1.5, -3.0, -4.5, -6.0 };
    g.offsets_y    = {  0.0,  0.0,  0.0,  0.0 };
    if (!client.send(g)) { step_failed("step 3 send", "send_goal failed"); return; }
  }
  if (auto status = client.wait_for_completion("formation", 15s); status != "OK") {
    step_failed("step 3 wait", status); return;
  }
  RCLCPP_INFO(log, "=== STEP 3 OK ===");
  std::this_thread::sleep_for(3s);

  client.to_idle();
  std::this_thread::sleep_for(300ms);

  // -------------------------------------------------------------------------
  // Step 4: MAPF cross-swap — orange goes to blue home, blue goes to orange
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 4: MAPF cross-swap ===");
  {
    LlmCommand::Goal g;
    g.mode = "mapf";
    g.reason = "scenario step 4: cross-swap";

    // Orange 0..9 -> blue home (top-right)
    const std::vector<std::pair<double,double>> blue_home = {
      {26.0, 22.0}, {27.5, 22.0},
      {26.0, 23.5}, {27.5, 23.5},
      {26.0, 25.0}, {27.5, 25.0},
      {26.0, 26.5}, {27.5, 26.5},
      {26.0, 28.0}, {27.5, 28.0},
    };
    for (int i = 0; i < 10; ++i) {
      g.robot_ids.push_back(static_cast<uint32_t>(i));
      g.goals.push_back(make_point(blue_home[i].first, blue_home[i].second));
    }

    // Blue 10..19 -> orange home (bottom-left)
    const std::vector<std::pair<double,double>> orange_home = {
      {2.0, 2.0}, {3.5, 2.0},
      {2.0, 3.5}, {3.5, 3.5},
      {2.0, 5.0}, {3.5, 5.0},
      {2.0, 6.5}, {3.5, 6.5},
      {2.0, 8.0}, {3.5, 8.0},
    };
    for (int i = 0; i < 10; ++i) {
      g.robot_ids.push_back(static_cast<uint32_t>(10 + i));
      g.goals.push_back(make_point(orange_home[i].first, orange_home[i].second));
    }

    if (!client.send(g)) { step_failed("step 4 send", "send_goal failed"); return; }
  }
  if (auto status = client.wait_for_completion("mapf", 300s); status != "OK") {
    step_failed("step 4 wait", status); return;
  }
  RCLCPP_INFO(log, "=== STEP 4 OK ===");
  std::this_thread::sleep_for(2s);

  client.to_idle();
  std::this_thread::sleep_for(300ms);

  // -------------------------------------------------------------------------
  // Step 5: MAPF all 20 back home
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 5: MAPF all 20 -> home ===");
  {
    LlmCommand::Goal g;
    g.mode = "mapf";
    g.reason = "scenario step 5: home";

    const std::vector<std::pair<double,double>> orange_home = {
      {2.0, 2.0}, {3.5, 2.0},
      {2.0, 3.5}, {3.5, 3.5},
      {2.0, 5.0}, {3.5, 5.0},
      {2.0, 6.5}, {3.5, 6.5},
      {2.0, 8.0}, {3.5, 8.0},
    };
    for (int i = 0; i < 10; ++i) {
      g.robot_ids.push_back(static_cast<uint32_t>(i));
      g.goals.push_back(make_point(orange_home[i].first, orange_home[i].second));
    }

    const std::vector<std::pair<double,double>> blue_home = {
      {26.0, 22.0}, {27.5, 22.0},
      {26.0, 23.5}, {27.5, 23.5},
      {26.0, 25.0}, {27.5, 25.0},
      {26.0, 26.5}, {27.5, 26.5},
      {26.0, 28.0}, {27.5, 28.0},
    };
    for (int i = 0; i < 10; ++i) {
      g.robot_ids.push_back(static_cast<uint32_t>(10 + i));
      g.goals.push_back(make_point(blue_home[i].first, blue_home[i].second));
    }

    if (!client.send(g)) { step_failed("step 5 send", "send_goal failed"); return; }
  }
  if (auto status = client.wait_for_completion("mapf", 300s); status != "OK") {
    step_failed("step 5 wait", status); return;
  }
  RCLCPP_INFO(log, "=== STEP 5 OK ===");

  // -------------------------------------------------------------------------
  // Step 6: Idle
  // -------------------------------------------------------------------------
  RCLCPP_INFO(log, "=== STEP 6: IDLE ===");
  client.to_idle();
  std::this_thread::sleep_for(1s);
  RCLCPP_INFO(log, "ALL 20 ROBOTS -- FULL SCENARIO COMPLETED SUCCESSFULLY");
  done = true;
}

// ---------------------------------------------------------------------------
// main — host the BT tree, tick at 10 Hz, that's it
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
  blackboard->set<int>("@command_seq", 0);
  blackboard->set<std::vector<int>>("@robot_ids", {});
  blackboard->set<std::vector<Point>>("@goals", {});

  const auto xml_file =
    ament_index_cpp::get_package_share_directory("iros_llm_swarm_bt") +
    "/behavior_trees/swarm_navigate_to_pose.xml";

  auto tree = factory.createTreeFromFile(xml_file, blackboard);
  RclcppDebugLogger logger(tree, node->get_logger());

  // Optional embedded scenario
  std::atomic<bool> scenario_done{false};
  std::thread scenario_thread;
  if (run_scenario_mode) {
    RCLCPP_INFO(node->get_logger(),
      "BT runner: scenario mode -- starting scenario in 3s");
    scenario_thread = std::thread(run_scenario, node, std::ref(scenario_done));
  } else {
    RCLCPP_INFO(node->get_logger(),
      "BT runner: host mode -- send commands via "
      "'ros2 action send_goal /llm/command iros_llm_swarm_interfaces/action/LlmCommand ...'");
  }

  // Tick loop. Tree is reactive: it never reaches a terminal status in
  // steady state (RunOnce masks terminal child statuses with RUNNING).
  // All control flow happens through /llm/command and /bt/state.
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    if (run_scenario_mode && scenario_done) {
      break;
    }
    rclcpp::spin_some(node);
    tree.tickRoot();
    rate.sleep();
  }

  if (scenario_thread.joinable()) scenario_thread.join();
  rclcpp::shutdown();
  return 0;
}