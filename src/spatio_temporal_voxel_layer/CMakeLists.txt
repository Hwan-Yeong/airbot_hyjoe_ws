cmake_minimum_required(VERSION 3.5)
project(spatio_temporal_voxel_layer)

# CMake 정책 설정 (경고 해결)
cmake_policy(SET CMP0074 NEW) # PCL_ROOT 관련
cmake_policy(SET CMP0077 NEW) # BUILD_SHARED_LIBS 관련

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get a linker error when there are undefined symbols
add_link_options(-Wl,--no-undefined)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

set(OPENVDB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/external/openvdb_vendor")

if(NOT EXISTS "${OPENVDB_ROOT}/lib/libopenvdb.so")
  message(FATAL_ERROR "Prebuilt OpenVDB not found in external/openvdb_vendor")
endif()

include_directories(
  include
  ${BOOST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OPENVDB_ROOT}/include
)

link_directories(${OPENVDB_ROOT}/lib)

remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL REQUIRED COMPONENTS filters common)

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

set(dependencies
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  sensor_msgs
  std_msgs
  std_srvs
  laser_geometry
  message_filters
  pcl_conversions
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
  builtin_interfaces
)

set(library_name spatio_temporal_voxel_layer_core)

add_definitions(${EIGEN3_DEFINITIONS})

set(srv_files "srv/SaveGrid.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(${library_name} SHARED
  src/spatio_temporal_voxel_layer.cpp
  src/spatio_temporal_voxel_grid.cpp
  src/measurement_buffer.cpp
  src/frustum_models/depth_camera_frustum.cpp
  src/frustum_models/three_dimensional_lidar_frustum.cpp
  src/vdb2pc.cpp
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${library_name}
  "${cpp_typesupport_target}"
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OPENVDB_ROOT}/lib/libopenvdb.so
)

ament_target_dependencies(${library_name} ${dependencies})
ament_environment_hooks(env_hook/50.openvdb_library_path.sh)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# openvdb 라이브러리 복사
install(
  DIRECTORY ${OPENVDB_ROOT}/lib/
  DESTINATION lib
)

install(
  DIRECTORY ${OPENVDB_ROOT}/include/
  DESTINATION include
)

install(DIRECTORY example
  DESTINATION share/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_package()
