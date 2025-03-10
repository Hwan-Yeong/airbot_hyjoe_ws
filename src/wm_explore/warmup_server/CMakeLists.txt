cmake_minimum_required(VERSION 3.8)
project(warmup_server)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(explore_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
        include
        include/${PROJECT_NAME}
)

set(dependencies
        rclcpp
        explore_msgs
        geometry_msgs
        sensor_msgs
        rclcpp_action
        nav2_msgs
        nav2_util
        visualization_msgs
)

set(source_list
        src/main.cpp
        src/main.cpp
        src/presentation/warmup_server.cpp
        include/presentation/warmup_server.hpp
        include/domain/ros_define.hpp
        src/application/move.cpp
        include/application/move.hpp
        include/domain/entity/vertex.hpp
        src/application/search/safe_area.cpp
        include/application/search/safe_area.hpp
        include/domain/search_define.hpp
        src/application/search/nav_goal_trans.cpp
        include/application/search/nav_goal_trans.hpp
        include/domain/entity/pose.hpp
        src/application/trans/TfTrans.cpp
        include/application/trans/TfTrans.hpp
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
