cmake_minimum_required(VERSION 3.8)
project(admittance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────────────────────────────────────────────
# Dependencies (ONLY what your node uses)
# ─────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# ─────────────────────────────────────────────
# Executable
# ─────────────────────────────────────────────
add_executable(admittance_control_node src/admittance_control.cpp)

ament_target_dependencies(admittance_control_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# ─────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────
install(TARGETS
  admittance_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()