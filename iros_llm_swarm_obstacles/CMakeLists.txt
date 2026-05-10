cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(iros_llm_swarm_interfaces REQUIRED)

add_executable(dynamic_obstacle_manager src/dynamic_obstacle_manager.cpp)
ament_target_dependencies(dynamic_obstacle_manager
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  yaml_cpp_vendor
  iros_llm_swarm_interfaces
)
target_link_libraries(dynamic_obstacle_manager yaml-cpp)
install(TARGETS dynamic_obstacle_manager DESTINATION lib/${PROJECT_NAME})

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
