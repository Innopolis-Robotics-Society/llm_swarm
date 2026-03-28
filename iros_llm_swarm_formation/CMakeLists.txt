cmake_minimum_required(VERSION 3.8)
project(iros_llm_swarm_formation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(iros_llm_swarm_interfaces REQUIRED)

# ---- C++ controller node ----------------------------------------------------
add_executable(formation_controller_node src/formation_controller_node.cpp)

ament_target_dependencies(formation_controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  iros_llm_swarm_interfaces
)

install(TARGETS
  formation_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Python manager node ----------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  iros_llm_swarm_formation/formation_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME formation_manager_node
)

# ---- Config & launch --------------------------------------------------------
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ---- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()