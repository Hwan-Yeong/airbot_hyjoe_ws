cmake_minimum_required(VERSION 3.8)
project(airbot_node_manager)

set(CMAKE_BUILD_TYPE Release) #set(CMAKE_BUILD_TYPE Debug) 

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

# Include directories
include_directories(
  include  # Include your project's header directory
  ${catkin_INCLUDE_DIRS}
)

# Executable (if needed)
add_executable(node_manager
  src/node_manager/node_manager.cpp
  src/node.cpp
)

# Link dependencies
ament_target_dependencies(node_manager
  rclcpp
  tf2_geometry_msgs
  std_msgs
  robot_custom_msgs
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS
node_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
