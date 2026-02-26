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
find_package(Bullet REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS "ui")

include_directories(include ${tinyxml2_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS})

add_executable(sensor_gui
  src/main.cpp
  src/window/main_window.cpp
  src/node/ros_node.cpp
  src/visualizer/point_cloud_visualizer.cpp
  src/window/teleop_window.cpp
  src/window/obstacle_window.cpp
  src/util/s_curve_profile.cpp
  src/visualizer/robot_model.cpp
  src/physics/physics_world.cpp
  src/window/robot_model_window.cpp
  include/sensor_gui/window/main_window.hpp
  include/sensor_gui/node/ros_node.hpp
  include/sensor_gui/visualizer/point_cloud_visualizer.hpp
  include/sensor_gui/window/teleop_window.hpp
  include/sensor_gui/window/obstacle_window.hpp
  include/sensor_gui/window/robot_model_window.hpp
  include/sensor_gui/util/s_curve_profile.hpp
  include/sensor_gui/physics/physics_world.hpp
  resources/resources.qrc
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

target_link_libraries(sensor_gui Qt5::Widgets Qt5::OpenGL GL GLU ${tinyxml2_LIBRARIES} ${BULLET_LIBRARIES})

install(TARGETS sensor_gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
