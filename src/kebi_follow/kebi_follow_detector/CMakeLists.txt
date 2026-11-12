cmake_minimum_required(VERSION 3.8)
project(kebi_follow_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

set(DEPS rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros
  tf2_msgs kebi_follow_msgs)
find_package(ament_cmake REQUIRED)
foreach(dep ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ---------------------------------------------------------------- inference backends
# RKNN (RK3588 NPU): -DRKNN_RT_DIR=<dir with lib/librknnrt.so and include/rknn_api.h>, aarch64 only.
# The runtime is not part of this repository; it is installed next to the libraries of this package.
set(RKNN_RT_DIR "" CACHE PATH "RKNN runtime dir (lib/librknnrt.so, include/rknn_api.h)")
set(KEBI_HAVE_RKNN OFF)
if(RKNN_RT_DIR AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  find_library(RKNNRT_LIBRARY NAMES rknnrt PATHS "${RKNN_RT_DIR}/lib" NO_DEFAULT_PATH)
  find_path(RKNN_INCLUDE_DIR rknn_api.h PATHS "${RKNN_RT_DIR}/include" NO_DEFAULT_PATH)
  if(RKNNRT_LIBRARY AND RKNN_INCLUDE_DIR)
    set(KEBI_HAVE_RKNN ON)
  endif()
endif()
if(KEBI_HAVE_RKNN)
  message(STATUS "kebi_follow_detector: RKNN backend ON (${RKNNRT_LIBRARY})")
else()
  message(STATUS "kebi_follow_detector: RKNN backend OFF (needs aarch64 + RKNN_RT_DIR, got "
    "'${RKNN_RT_DIR}' on ${CMAKE_SYSTEM_PROCESSOR})")
endif()
# OpenCV DNN (CPU, ONNX): PC tests
list(FIND OpenCV_LIB_COMPONENTS opencv_dnn _dnn_index)
if(_dnn_index GREATER -1)
  set(KEBI_HAVE_OPENCV_DNN ON)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)
else()
  set(KEBI_HAVE_OPENCV_DNN OFF)
endif()
message(STATUS "kebi_follow_detector: OpenCV ${OpenCV_VERSION}, DNN backend ${KEBI_HAVE_OPENCV_DNN}")

# ---------------------------------------------------------------- core library (no ROS)
add_library(${PROJECT_NAME}_core SHARED
  src/yolo_decode.cpp
  src/time_offset.cpp
  src/depth_fusion.cpp
  src/backend_factory.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS})
if(KEBI_HAVE_OPENCV_DNN)
  target_sources(${PROJECT_NAME}_core PRIVATE src/opencv_dnn_backend.cpp)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE KEBI_HAVE_OPENCV_DNN)
endif()
if(KEBI_HAVE_RKNN)
  target_sources(${PROJECT_NAME}_core PRIVATE src/rknn_backend.cpp)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE KEBI_HAVE_RKNN)
  target_include_directories(${PROJECT_NAME}_core SYSTEM PRIVATE ${RKNN_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME}_core ${RKNNRT_LIBRARY})
  # self-contained install space: librknnrt.so next to our libraries, found through RUNPATH $ORIGIN
  install(FILES ${RKNNRT_LIBRARY} DESTINATION lib)
endif()
set_target_properties(${PROJECT_NAME}_core PROPERTIES INSTALL_RPATH "$ORIGIN")

# ---------------------------------------------------------------- component (container + executable)
add_library(${PROJECT_NAME}_component SHARED
  src/cam_leg_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPS})
set_target_properties(${PROJECT_NAME}_component PROPERTIES INSTALL_RPATH "$ORIGIN")
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "kebi_follow::CamLegDetector"
  EXECUTABLE cam_leg_detector_node)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # decode: synthetic tensors; with KEBI_DET_TEST_DATA=<dir> also stored NPU dumps vs the Python reference
  ament_add_gtest(test_yolo_decode test/test_yolo_decode.cpp)
  target_link_libraries(test_yolo_decode ${PROJECT_NAME}_core)
  ament_add_gtest(test_time_offset test/test_time_offset.cpp)
  target_link_libraries(test_time_offset ${PROJECT_NAME}_core)
  ament_add_gtest(test_depth_fusion test/test_depth_fusion.cpp)
  target_link_libraries(test_depth_fusion ${PROJECT_NAME}_core)
endif()

ament_package()
