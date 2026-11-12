cmake_minimum_required(VERSION 3.8)
project(kebi_follow_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

set(DEPS rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs visualization_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs follow_msgs robot_custom_msgs kebi_follow_msgs)
find_package(ament_cmake REQUIRED)
foreach(dep ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# ROS 무관 판단 코드 (상태기계·추종 법칙·가감속·안전 게이트·depth 장애물). 단위 시험이 직접 링크한다
add_library(${PROJECT_NAME}_core STATIC
  src/follow_fsm.cpp
  src/follow_law.cpp
  src/velocity_limiter.cpp
  src/safety_gate.cpp
  src/depth_obstacle_checker.cpp
  src/controller_core.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 컴포넌트 라이브러리: 컨테이너(kebi_follow_me launch)와 단독 실행 파일 둘 다 이걸 쓴다
add_library(${PROJECT_NAME}_component SHARED
  src/follow_controller.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPS})
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "kebi_follow::FollowController"
  EXECUTABLE follow_controller_node
  EXECUTOR MultiThreadedExecutor)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t fsm follow_law velocity_limiter safety_gate depth_obstacle_checker controller_core pose_history)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} ${PROJECT_NAME}_core)
  endforeach()
  # 노드 단위 시험: 실제 노드를 같은 프로세스에 띄운다 (도메인·DDS 격리는 시험 main 이 한다)
  ament_add_gtest(test_node test/test_node.cpp TIMEOUT 300)
  target_link_libraries(test_node ${PROJECT_NAME}_component ${PROJECT_NAME}_core)
  ament_target_dependencies(test_node ${DEPS})
endif()

ament_package()
