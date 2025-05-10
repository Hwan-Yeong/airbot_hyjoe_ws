cmake_minimum_required(VERSION 3.8)
project(airbot_sensor_manager)

set(CMAKE_BUILD_TYPE Release) #set(CMAKE_BUILD_TYPE Debug) 

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
##### SENSOR_TO_POINTCLOUD #####
add_executable(sensor_to_pointcloud
  src/airbot_sensor_to_pointcloud/main.cpp
  src/airbot_sensor_to_pointcloud/sensor_to_pointcloud_node.cpp
  src/airbot_sensor_to_pointcloud/modules/tof/pointcloud_tof.cpp
  src/airbot_sensor_to_pointcloud/modules/camera/pointcloud_camera.cpp
  src/airbot_sensor_to_pointcloud/modules/camera/logging/camera_object_logger.cpp
  src/airbot_sensor_to_pointcloud/modules/cliff/pointcloud_cliff.cpp
  src/airbot_sensor_to_pointcloud/modules/collision/pointcloud_collision.cpp
  src/utils/frame_converter.cpp
  src/utils/pointcloud_generator.cpp
  src/utils/boundingbox_generator.cpp
)
ament_target_dependencies(sensor_to_pointcloud 
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
install(TARGETS sensor_to_pointcloud
DESTINATION lib/${PROJECT_NAME}
)

################################
######### PARAM_SETTER #########
add_executable(param_setter
  src/dynamic_param_setter/param_setter/param_setter.cpp
  src/dynamic_param_setter/node.cpp
)
ament_target_dependencies(param_setter
  rclcpp
  std_msgs
)
install(TARGETS param_setter
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
