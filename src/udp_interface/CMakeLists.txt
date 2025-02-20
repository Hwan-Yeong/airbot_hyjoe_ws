cmake_minimum_required(VERSION 3.5)
project(udp_interface)

# Set build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add compile options for debugging
add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_bringup REQUIRED)


# Add compile options for security and optimization
add_compile_options(
  -fstack-protector-strong  # Enable stack protector
  -flto                     # Enable link-time optimization
)

# Add linker flags for memory protections
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro,-z,now")


# Include directories
include_directories(
  include  # Include your project's header directory
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/udp_interface  # Include path for libNetwork.h
)

find_path(LIBNETWORK_INCLUDE_DIR
  NAMES libNetwork.h
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/udp_interface
  NO_DEFAULT_PATH
)

# Find libNetwork.so (specify the path if necessary)
find_library(LIBNETWORK_LIBRARY 
  NAMES Network libNetwork 
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/udp_interface
  NO_DEFAULT_PATH
)

if(NOT LIBNETWORK_INCLUDE_DIR OR NOT LIBNETWORK_LIBRARY)
  message(FATAL_ERROR "libNetwork headers or library not found. Check the paths.")
endif()

add_executable(udp_communication src/main.cpp src/udp_communication.cpp)

# add_library(udp_communication_lib SHARED src/udp_communication.cpp)

ament_target_dependencies(udp_communication
  rclcpp
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  serial
  robot_custom_msgs
  nav2_msgs
  robot_bringup
)

target_link_libraries(udp_communication ${LIBNETWORK_LIBRARY})

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS
  udp_communication
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
