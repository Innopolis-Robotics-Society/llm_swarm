cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Custom interfaces ------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FormationConfig.msg"
  "msg/FormationsConfig.msg"
  "msg/FormationStatus.msg"
  "msg/FormationsStatus.msg"
  "msg/MAPFStep.msg"
  "msg/MAPFPlan.msg"
  "msg/FollowerStatus.msg"
  "msg/BTState.msg"
  "srv/SetFormation.srv"
  "srv/RemoveFormation.srv"
  "srv/ActivateFormation.srv"
  "srv/DeactivateFormation.srv"
  "srv/GetFormation.srv"
  "srv/ListFormations.srv"
  "srv/LoadFormations.srv"
  "srv/SaveFormations.srv"
  "action/SetGoals.action"
  "action/LlmDecision.action"
  "action/LlmCommand.action"
  "action/LlmChat.action"
  "msg/LlmEvent.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()