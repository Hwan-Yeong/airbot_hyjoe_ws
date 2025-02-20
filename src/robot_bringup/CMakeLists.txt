cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Include directories
include_directories(
  include  # Include your project's header directory
  ${ament_INCLUDE_DIRS}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(robot_bringup src/uart_communication.cpp src/uart_helpers.cpp)
ament_target_dependencies(robot_bringup
  rclcpp
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  serial
  robot_custom_msgs
  nav2_msgs
)

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS
  robot_bringup
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
