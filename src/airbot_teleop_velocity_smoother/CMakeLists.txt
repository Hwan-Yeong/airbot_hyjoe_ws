cmake_minimum_required(VERSION 3.8)
project(airbot_teleop_velocity_smoother)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(teleop_velocity_smoother_node
  src/teleop_velocity_smoother_node.cpp
  src/main.cpp
)

target_include_directories(teleop_velocity_smoother_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(teleop_velocity_smoother_node
  "rclcpp"
  "geometry_msgs"
)

install(TARGETS teleop_velocity_smoother_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()