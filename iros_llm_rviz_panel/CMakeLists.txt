cmake_minimum_required(VERSION 3.8)
project(iros_llm_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt MOC handling — required for Q_OBJECT / signals / slots.
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(iros_llm_swarm_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ---------------------------------------------------------------------------
# Library — must be SHARED for pluginlib to load it at runtime.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  include/iros_llm_rviz_panel/llm_panel.hpp
  include/iros_llm_rviz_panel/action_summary.hpp
  include/iros_llm_rviz_panel/sparkline.hpp
  src/llm_panel.cpp
  src/action_summary.cpp
  src/sparkline.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rviz_common
  rviz_default_plugins
  pluginlib
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  iros_llm_swarm_interfaces
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rviz_common
  rclcpp
  rclcpp_action
  iros_llm_swarm_interfaces
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_summary
    test/test_action_summary.cpp
    src/action_summary.cpp
  )
  target_include_directories(test_action_summary PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
