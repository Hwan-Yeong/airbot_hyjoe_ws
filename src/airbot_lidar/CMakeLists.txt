cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(airbot_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/sdk
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

####### airbot_lidar #######
add_executable(airbot_lidar
  src/lidar/node_lidar_ros.cpp
  sdk/calibration.cpp
  sdk/lidar_data_processing.cpp
  sdk/mtime.cpp
  sdk/node_lidar.cpp
  sdk/point_cloud_optimize.cpp
  sdk/serial_port.cpp
  sdk/timer.cpp
)
ament_target_dependencies(airbot_lidar
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)
install(TARGETS airbot_lidar
  DESTINATION lib/${PROJECT_NAME}
)

####### airbot_scan_merger #######
add_executable(airbot_lidar_merger
  src/scan_merger/laser_scan_merger.cpp
)
ament_target_dependencies(airbot_lidar_merger 
  rclcpp 
  std_msgs 
  sensor_msgs
  pcl_conversions
  rcl_interfaces
  rcutils
  tf2_geometry_msgs
  geometry_msgs
)
install(TARGETS airbot_lidar_merger
  DESTINATION lib/${PROJECT_NAME}
)

####### airbot_pointcloud_to_laserscan #######
add_executable(pointcloud_to_laserscan_node
  src/pointcloud_to_laserscan/pointcloud_to_laserscan_node.cpp
)
ament_target_dependencies(pointcloud_to_laserscan_node 
  rclcpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  message_filters
)
install(TARGETS pointcloud_to_laserscan_node
  DESTINATION lib/${PROJECT_NAME}
)

####### general #######
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config sdk 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
