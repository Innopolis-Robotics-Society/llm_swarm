#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "iros_llm_swarm_interfaces/action/llm_decision.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trigger_llm_decision_warn_test");

  using LlmDecision = iros_llm_swarm_interfaces::action::LlmDecision;
  using GoalHandle = rclcpp_action::ClientGoalHandle<LlmDecision>;

  auto client = rclcpp_action::create_client<LlmDecision>(node, "/llm/decision");
  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan LLM: /llm/decision not available for WARN test");
    rclcpp::shutdown();
    return 1;
  }

  const std::string level = "WARN";
  const std::string event =
    "debug fake MapfPlan feedback warning: robot_0 stalled";

  RCLCPP_WARN(node->get_logger(),
    "MapfPlan LLM: feedback warning detected level=WARN event='%s'",
    event.c_str());
  RCLCPP_INFO(node->get_logger(),
    "MapfPlan LLM: decision pending=false");

  LlmDecision::Goal goal;
  goal.level = level;
  goal.event = event;
  goal.log_buffer = {
    "[debug status=executing arrived=0 active=1 stall=1 replans=0] WARN: "
    "robot_0 stalled",
  };

  RCLCPP_INFO(node->get_logger(),
    "MapfPlan LLM: sending /llm/decision level=%s event='%s' "
    "log_buffer_size=%zu",
    goal.level.c_str(), goal.event.c_str(), goal.log_buffer.size());

  rclcpp_action::Client<LlmDecision>::SendGoalOptions opts;
  opts.feedback_callback =
    [node](GoalHandle::SharedPtr,
           const std::shared_ptr<const LlmDecision::Feedback> fb) {
      RCLCPP_INFO(node->get_logger(),
        "MapfPlan LLM: decision feedback stage=%s", fb->stage.c_str());
    };

  auto goal_future = client->async_send_goal(goal, opts);
  if (rclcpp::spin_until_future_complete(
      node, goal_future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan LLM: timed out sending WARN test goal");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan LLM: WARN test goal rejected");
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::seconds(60)) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan LLM: timed out waiting for WARN test result");
    rclcpp::shutdown();
    return 1;
  }

  auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(),
      "MapfPlan LLM: WARN test action did not succeed");
    rclcpp::shutdown();
    return 1;
  }

  const auto decision = wrapped.result->decision;
  RCLCPP_INFO(node->get_logger(),
    "MapfPlan LLM: decision result decision=%s", decision.c_str());
  RCLCPP_INFO(node->get_logger(),
    "MapfPlan LLM: applying decision %s", decision.c_str());

  rclcpp::shutdown();
  return 0;
}
