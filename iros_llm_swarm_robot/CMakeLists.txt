cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(nav2_msgs     REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iros_llm_swarm_interfaces REQUIRED)


add_executable(pbs_motion_controller src/pbs_motion_controller.cpp)

ament_target_dependencies(pbs_motion_controller
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  iros_llm_swarm_interfaces
)


add_executable(lns_motion_controller src/lns_motion_controller.cpp)

ament_target_dependencies(lns_motion_controller
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  iros_llm_swarm_interfaces
)

install(TARGETS
  pbs_motion_controller
  lns_motion_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()