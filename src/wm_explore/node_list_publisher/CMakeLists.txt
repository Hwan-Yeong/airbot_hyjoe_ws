cmake_minimum_required(VERSION 3.8)
project(node_list_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(node_list src/main.cpp)
ament_target_dependencies(node_list rclcpp std_msgs)

install(TARGETS node_list
  DESTINATION lib/${PROJECT_NAME})

ament_package()
