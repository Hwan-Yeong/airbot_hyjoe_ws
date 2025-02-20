cmake_minimum_required(VERSION 3.8)
project(airbot_sensor_manager)

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
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sensor_to_pointcloud
  src/node.cpp
  src/airbot_sensor_to_pointcloud/sensor_to_pointcloud.cpp
  src/airbot_sensor_to_pointcloud/tof/pointcloud_tof.cpp
  src/airbot_sensor_to_pointcloud/camera/pointcloud_camera.cpp
  src/airbot_sensor_to_pointcloud/cliff/pointcloud_cliff.cpp
  src/utils/frame_converter.cpp
  src/utils/pointcloud_generator.cpp
  src/utils/boundingbox_generator.cpp
  src/logger/camera_object_logger.cpp
)

add_executable(param_setter
  src/dynamic_param_setter/param_setter.cpp
)
ament_target_dependencies(param_setter
  rclcpp
  std_msgs
)
install(TARGETS param_setter
  DESTINATION lib/${PROJECT_NAME})

#####################################
############# TEST CODE #############
# add_executable(param_setter
#   test/test_frame_changer.cpp
# )
# ament_target_dependencies(param_setter
#   rclcpp
# )
# install(TARGETS param_setter
#   DESTINATION lib/${PROJECT_NAME})
#####################################

ament_target_dependencies(sensor_to_pointcloud 
  rclcpp 
  sensor_msgs 
  robot_custom_msgs
  builtin_interfaces
  tf2
  tf2_geometry_msgs
  vision_msgs
  visualization_msgs
)

install(TARGETS sensor_to_pointcloud
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/airbot_sensor_to_pointcloud/
  DESTINATION include/airbot_sensor_to_pointcloud)

install(DIRECTORY include/logger/
  DESTINATION include/logger)

install(DIRECTORY include/utils/
  DESTINATION include/utils)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
