cmake_minimum_required(VERSION 3.8)
project(airbot_error_manager)

set(CMAKE_BUILD_TYPE Debug) #set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################
######## ERROR_MANAGER #########
add_executable(error_manager
  src/error_manager/error_manager.cpp
  src/error_manager/node.cpp
)
ament_target_dependencies(error_manager
  rclcpp
  tf2_geometry_msgs
  tf2
  std_msgs
  robot_custom_msgs
)
install(TARGETS error_manager
  DESTINATION lib/${PROJECT_NAME}
)

################################
######## ERROR_MONITOR #########
add_executable(error_monitor_node
  src/error_monitor/error_monitor.cpp
  src/error_monitor/error_monitor_node.cpp
  src/error_monitor/main.cpp
)
ament_target_dependencies(error_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  robot_custom_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)
install(TARGETS error_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

################################
############ GENERAL ###########
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
