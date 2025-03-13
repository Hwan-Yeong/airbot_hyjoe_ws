cmake_minimum_required(VERSION 3.8)
project(airbot_sensor_interface)

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
find_package(yaml_cpp_vendor REQUIRED)

include_directories(
  include
)

################################
####### SENSOR_INTERFACE #######
add_executable(airbot_sensor_interface_node
  src/sensor_interface/main.cpp
  src/sensor_interface/sensor_interface_node.cpp
  src/sensor_interface/modules/tof/pointcloud_tof.cpp
  src/sensor_interface/modules/tof/tof_row_34_processor.cpp
  src/sensor_interface/modules/camera/pointcloud_camera.cpp
  src/sensor_interface/modules/camera/logging/camera_object_logger.cpp
  src/sensor_interface/modules/cliff/pointcloud_cliff.cpp
  src/sensor_interface/modules/collision/pointcloud_collision.cpp
  src/utils/frame_converter.cpp
  src/utils/pointcloud_generator.cpp
  src/utils/boundingbox_generator.cpp
)
ament_target_dependencies(airbot_sensor_interface_node
  rclcpp
  std_msgs
  sensor_msgs
  robot_custom_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  vision_msgs
  visualization_msgs
)
install(TARGETS airbot_sensor_interface_node
DESTINATION lib/${PROJECT_NAME}
)

################################
######### PARAM_SETTER #########
add_executable(param_manager_node
  src/param_manager/param_manager_node.cpp
  src/param_manager/main.cpp
)
ament_target_dependencies(param_manager_node
  rclcpp
  std_msgs
)
install(TARGETS param_manager_node
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
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
