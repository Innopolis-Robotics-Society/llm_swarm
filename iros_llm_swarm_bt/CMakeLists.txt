cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iros_llm_swarm_interfaces REQUIRED)

add_library(iros_llm_swarm_bt_nodes SHARED
  src/swarm_bt_nodes.cpp
)

target_compile_definitions(iros_llm_swarm_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(iros_llm_swarm_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(iros_llm_swarm_bt_nodes
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  geometry_msgs
  iros_llm_swarm_interfaces
)

add_executable(test_bt_runner
  src/test_bt_runner.cpp
)

target_include_directories(test_bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(test_bt_runner
  iros_llm_swarm_bt_nodes
)

ament_target_dependencies(test_bt_runner
  ament_index_cpp
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  geometry_msgs
  iros_llm_swarm_interfaces
)

install(
  TARGETS
    iros_llm_swarm_bt_nodes
    test_bt_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(iros_llm_swarm_bt_nodes)

ament_package()