cmake_minimum_required(VERSION 3.8)
project(explore)

# Add compile options for security and optimization
add_compile_options(
        -fstack-protector-strong  # Enable stack protector
        -flto                     # Enable link-time optimization
)

# Add linker flags for memory protections
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro,-z,now")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(explore_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(EXECUTABLE_NAME explore)
#set(LIBRARY_NAME ${EXECUTABLE_NAME}_core)

set(DEPENDENCIES
        rclcpp
        std_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        nav2_msgs
        nav2_costmap_2d
        nav_msgs
        map_msgs
        geometry_msgs
        visualization_msgs
        explore_msgs
        lifecycle_msgs
)

include_directories(include)

install(DIRECTORY include
        DESTINATION include)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME})

#add_library(${LIBRARY_NAME} SHARED
#        src/explore/frontier_search.cpp
#        src/explore/costmap_client.cpp
#        src/explore/explore.cpp
#)

add_executable(${EXECUTABLE_NAME} src/main.cpp
        src/explore/frontier_search.cpp
        src/explore/costmap_client.cpp
        src/explore/explore.cpp
        src/explore/lifecycle_node_checker.cpp
        include/explore/lifecycle_node_checker.hpp
        src/explore/warm_up_client.cpp
        include/explore/warm_up_client.hpp
)

ament_target_dependencies(${EXECUTABLE_NAME}
        ${DEPENDENCIES}
)

target_include_directories(${EXECUTABLE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${EXECUTABLE_NAME} ${rclcpp_LIBRARIES})

ament_target_dependencies(${EXECUTABLE_NAME}
        ${DEPENDENCIES}
)
#
#install(TARGETS ${EXECUTABLE_NAME}
#        DESTINATION lib)

install(TARGETS ${EXECUTABLE_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY include/
#        DESTINATION include/)
#
#install(DIRECTORY launch/ config/ params/
#        DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
