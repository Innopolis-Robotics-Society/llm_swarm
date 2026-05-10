cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_mapf_lns)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------------
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(nav2_msgs     REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iros_llm_swarm_interfaces REQUIRED)

# -------------------------------------------------------------------------
# lns2 library (pure algorithm, no ROS deps)
# -------------------------------------------------------------------------
add_library(lns2 STATIC
  src/lns2/types.cpp
  src/lns2/collision_table.cpp
  src/lns2/solution.cpp
  src/lns2/soft_astar.cpp
  src/lns2/destroy_operators.cpp
  src/lns2/lns2_solver.cpp
  src/lns2/warm_start.cpp
  src/lns2/gridmap_utils.cpp
)

target_include_directories(lns2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(lns2 PUBLIC cxx_std_17)
target_compile_options(lns2 PRIVATE
  $<$<CONFIG:Release>:-O3 -DNDEBUG>
  $<$<CONFIG:RelWithDebInfo>:-O2 -g>
  $<$<CONFIG:Debug>:-O0 -g>
)

# -------------------------------------------------------------------------
# mapf_lns2_node (ROS 2 planner node)
# -------------------------------------------------------------------------
add_executable(mapf_lns2_node
  src/mapf_lns2_node.cpp
  src/node_utils.cpp
  src/plan_publisher.cpp
  src/robot_lifecycle.cpp
)

target_link_libraries(mapf_lns2_node lns2)

ament_target_dependencies(mapf_lns2_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  iros_llm_swarm_interfaces
)

target_compile_options(mapf_lns2_node PRIVATE
  $<$<CONFIG:Release>:-O3 -DNDEBUG>
  $<$<CONFIG:RelWithDebInfo>:-O2 -g>
  $<$<CONFIG:Debug>:-O0 -g>
)

# -------------------------------------------------------------------------
# Install
# -------------------------------------------------------------------------
install(TARGETS 
  mapf_lns2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lns2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()