cmake_minimum_required(VERSION 3.8)
project(warmup_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(explore_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
        include
        include/${PROJECT_NAME}
)

set(dependencies
  rclcpp
  explore_msgs
  geometry_msgs
  rclcpp_action
)

set(source_list
        src/main.cpp
        src/main.cpp
        src/presentation/warmup_server.cpp
        include/presentation/warmup_server.hpp
        include/domain/ros_define.hpp
        src/application/move.cpp
        include/application/move.hpp
)

set(node_name
        warmup_server_node
)

add_executable(${node_name}
        ${source_list}
)

ament_target_dependencies(${node_name}
        ${dependencies}
)

install(TARGETS
        ${node_name}
        DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY
#        param
#        DESTINATION share/${PROJECT_NAME}
#)

ament_package()
