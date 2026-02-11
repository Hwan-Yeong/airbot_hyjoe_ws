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

# 패키지 내부의 pre-built 바이너리 경로 설정
set(PREBUILT_OPENVDB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/openvdb_vendor")

if(EXISTS "${PREBUILT_OPENVDB_DIR}")
  message(STATUS "Using local pre-built openvdb_vendor from: ${PREBUILT_OPENVDB_DIR}")
  # openvdb_vendor 가이드에 따라 CMake 경로 추가
  set(openvdb_vendor_DIR "${PREBUILT_OPENVDB_DIR}/share/openvdb_vendor/cmake")
  # 2. 실제 OpenVDB 라이브러리 및 헤더 경로 수동 설정 (바이너리 이식 시 CMake 절대경로 문제 해결)
  list(INSERT CMAKE_PREFIX_PATH 0 "${PREBUILT_OPENVDB_DIR}")
  find_package(openvdb_vendor REQUIRED NO_DEFAULT_PATH)

  # 3. 설치 규칙 추가: 빌드 결과물(install/)에 바이너리 포함
  install(DIRECTORY ${PREBUILT_OPENVDB_DIR}/ DESTINATION .)

  # 4. 런타임 경로 보장: LD_LIBRARY_PATH 에 라이브러리 경로 추가 (환경 훅)
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/export_openvdb_lib_path.sh"
  )
else()
  find_package(openvdb_vendor REQUIRED)
endif()

remove_definitions(-DDISABLE_LIBUSB-1.0)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PCL REQUIRED COMPONENTS filters common)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
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
  openvdb_vendor
)

set(library_name spatio_temporal_voxel_layer_core)

add_definitions(${EIGEN3_DEFINITIONS})

set(srv_files "srv/SaveGrid.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

include_directories(
    include
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${library_name} SHARED
            src/spatio_temporal_voxel_layer.cpp
            src/spatio_temporal_voxel_grid.cpp
            src/measurement_buffer.cpp
            src/frustum_models/depth_camera_frustum.cpp
            src/frustum_models/three_dimensional_lidar_frustum.cpp
            src/vdb2pc.cpp
)

target_link_libraries(${library_name} "${cpp_typesupport_target}")
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${library_name}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenVDB::openvdb
)

ament_target_dependencies(${library_name} ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
