cmake_minimum_required(VERSION 3.8)
project(sensor_gui)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include ${tinyxml2_INCLUDE_DIRS})

add_executable(sensor_gui
  src/main.cpp
  src/main_window.cpp
  src/ros_node.cpp
  src/point_cloud_visualizer.cpp
  src/teleop_window.cpp
  src/s_curve_profile.cpp
  src/robot_model.cpp
  include/sensor_gui/main_window.hpp
  include/sensor_gui/ros_node.hpp
  include/sensor_gui/point_cloud_visualizer.hpp
  include/sensor_gui/teleop_window.hpp
  include/sensor_gui/s_curve_profile.hpp
)

ament_target_dependencies(sensor_gui
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  robot_custom_msgs
  tinyxml2_vendor
)

target_link_libraries(sensor_gui Qt5::Widgets Qt5::OpenGL GL GLU ${tinyxml2_LIBRARIES})

install(TARGETS sensor_gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
