cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_mapf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# -------------------------------------------------------------------------
# Зависимости
# -------------------------------------------------------------------------
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(nav2_msgs     REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)

# -------------------------------------------------------------------------
# mapf_planner_node  (PBS планировщик)
# -------------------------------------------------------------------------
add_executable(mapf_planner_node src/mapf_planner_node.cpp)

target_include_directories(mapf_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mapf_planner_node
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

# -------------------------------------------------------------------------
# path_follower_node  (исполнитель пути для одного робота)
# -------------------------------------------------------------------------
add_executable(path_follower_node src/path_follower_node.cpp)

target_include_directories(path_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_follower_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
)

# -------------------------------------------------------------------------
# Установка
# -------------------------------------------------------------------------
install(TARGETS
  mapf_planner_node
  path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()