cmake_minimum_required(VERSION 3.8)
project(kebi_follow_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

set(DEPS rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs kebi_follow_msgs)
find_package(ament_cmake REQUIRED)
foreach(dep ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)

# ROS 비의존 추적 코어: 컴포넌트, 단위 시험, 합성 시나리오·bag 재생 도구가 같이 쓴다
add_library(${PROJECT_NAME}_core STATIC
  src/leg_detector.cpp
  src/kalman_cv.cpp
  src/hungarian.cpp
  src/people_tracker.cpp
  src/target_manager.cpp
  src/leg_tracker_core.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)

# 컴포넌트 라이브러리: 컨테이너(kebi_follow_me launch)와 단독 실행 파일 둘 다 이걸 쓴다
add_library(${PROJECT_NAME}_component SHARED
  src/leg_tracker.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPS})
# 컨테이너에 올릴 때가 아니라 빌드 때 미해결 심볼을 잡는다
target_link_options(${PROJECT_NAME}_component PRIVATE -Wl,--no-undefined)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "kebi_follow::LegTracker"
  EXECUTABLE leg_tracker_node)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 합성 시나리오 (2D 라이다 광선 추적 + 보행 모델 + 카메라 모의). 로봇 비의존
  add_library(${PROJECT_NAME}_sim STATIC
    test/sim/scan_sim.cpp
    test/sim/scenario.cpp
  )
  target_include_directories(${PROJECT_NAME}_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(${PROJECT_NAME}_sim PUBLIC ${PROJECT_NAME}_core)

  foreach(t test_leg_detector test_kalman_cv test_hungarian test_people_tracker test_target_manager test_scenarios)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} ${PROJECT_NAME}_sim)
  endforeach()
  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config ${PROJECT_NAME}_core)
  target_compile_definitions(test_config PRIVATE
    KEBI_TRACKER_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/leg_tracker.yaml")

  # 시나리오 평가 도구 (지표 표·CSV 출력, 설치하지 않음)
  add_executable(scenario_eval test/tools/scenario_eval.cpp)
  target_link_libraries(scenario_eval ${PROJECT_NAME}_sim)

  # bag 재생 도구: /scan + /tf (+ /kebi_follow/cam_legs) → 코어 → CSV (설치하지 않음)
  find_package(rosbag2_cpp QUIET)
  if(rosbag2_cpp_FOUND)
    add_executable(bag_replay test/tools/bag_replay.cpp)
    target_link_libraries(bag_replay ${PROJECT_NAME}_core)
    ament_target_dependencies(bag_replay rclcpp rosbag2_cpp sensor_msgs tf2 tf2_msgs kebi_follow_msgs)
  endif()
endif()

ament_package()
