cmake_minimum_required(VERSION 3.5)
project(airbot_motion_anomaly_detector)

set(CMAKE_BUILD_TYPE Release) #set(CMAKE_BUILD_TYPE Debug) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Define the executable
add_executable(anomaly_detector src/anomaly_detector.cpp)
target_include_directories(anomaly_detector PUBLIC include)

ament_target_dependencies(anomaly_detector rclcpp sensor_msgs std_msgs geometry_msgs robot_custom_msgs tf2 tf2_ros tf2_geometry_msgs)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
# Install the node
install(TARGETS
  anomaly_detector 
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
