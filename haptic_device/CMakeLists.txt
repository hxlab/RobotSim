cmake_minimum_required(VERSION 3.8)
project(haptic_device)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(franka_msgs REQUIRED)


set(OPENHAPTICS_DIR "/opt/OpenHaptics/Developer/3.4-0")

if(NOT EXISTS ${OPENHAPTICS_DIR})
  message(FATAL_ERROR "OpenHaptics not found at: ${OPENHAPTICS_DIR}")
endif()

include_directories(${OPENHAPTICS_DIR}/include)
link_directories(${OPENHAPTICS_DIR}/lib/x86_64-linux-gnu)

# ========== HAPTIC DEVICE NODE ==========
add_executable(haptic_device_node src/haptic_device_node.cpp)
target_compile_options(haptic_device_node PRIVATE -g -O0 -W -Dlinux -D_cplusplus)
ament_target_dependencies(haptic_device_node
  rclcpp std_msgs geometry_msgs tf2_ros tf2 ros_gz_interfaces)
target_link_libraries(haptic_device_node HDU HD rt stdc++ m)



# ========== INSTALL ==========
install(TARGETS
  haptic_device_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
