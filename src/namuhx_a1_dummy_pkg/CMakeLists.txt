cmake_minimum_required(VERSION 3.8)
project(namuhx_a1_dummy_pkg)

set(CMAKE_BUILD_TYPE Debug) #set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

################################
##### NAMUHX_A1_DUMMY_NODE #####
add_executable(namuhx_a1_dummy
  src/main.cpp
  src/namuhx_a1_dummy_node.cpp
)
ament_target_dependencies(namuhx_a1_dummy
  rclcpp
  std_msgs
  sensor_msgs
  robot_custom_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  vision_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(namuhx_a1_dummy)
install(TARGETS namuhx_a1_dummy
  DESTINATION lib/${PROJECT_NAME}
)

################################
############ GENERAL ###########
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
