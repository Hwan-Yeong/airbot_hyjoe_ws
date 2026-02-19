cmake_minimum_required(VERSION 3.8)
project(sensor_gui)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

add_executable(sensor_gui
  src/main.cpp
  src/main_window.cpp
  src/ros_node.cpp
  include/sensor_gui/main_window.hpp
  include/sensor_gui/ros_node.hpp
)

ament_target_dependencies(sensor_gui
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  tf2_ros
  geometry_msgs
  rviz_common
  rviz_default_plugins
  robot_custom_msgs
)

target_link_libraries(sensor_gui Qt5::Widgets)

install(TARGETS sensor_gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
