cmake_minimum_required(VERSION 3.8)
project(A1_maneuver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(state_manager REQUIRED)
find_package(builtin_interfaces REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Executable (if needed)
add_executable(A1_maneuver src/main.cpp src/maneuver.cpp)

# Link dependencies
ament_target_dependencies(A1_maneuver
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  nav2_msgs
  robot_custom_msgs
  state_manager
)

#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS A1_maneuver
  DESTINATION lib/${PROJECT_NAME}/
)

# Install headers
install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()