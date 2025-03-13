cmake_minimum_required(VERSION 3.8)
project(airbot_error_manager)

set(CMAKE_BUILD_TYPE Debug) #set(CMAKE_BUILD_TYPE Debug)
set(PROJECT_LIB ${PROJECT_NAME}_lib)

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
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${YAML_CPP_LIBRARY_DIRS})

################################
######## ERROR_MANAGER #########
set(src_files
  src/error_manager/error_manager_node.cpp
  src/error_manager/main.cpp
)

set(dependencies
  rclcpp
  tf2_geometry_msgs
  tf2
  std_msgs
  robot_custom_msgs
  yaml-cpp
)

# Add library
add_library(${PROJECT_LIB} STATIC ${src_files})
target_link_libraries(${PROJECT_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_LIB} ${dependencies})

install(TARGETS ${PROJECT_LIB}
  DESTINATION lib/${PROJECT_NAME}
)

# Add executable
add_executable(error_manager_node src/error_manager/main.cpp)
target_link_libraries(error_manager_node ${PROJECT_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(error_manager_node ${dependencies})

# Installation
install(TARGETS error_manager_node
  DESTINATION
  lib/${PROJECT_NAME}
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
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
