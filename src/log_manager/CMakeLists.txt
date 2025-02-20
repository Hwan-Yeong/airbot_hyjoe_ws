cmake_minimum_required(VERSION 3.8)
project(log_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 
add_compile_options(-std=c++17)

# Default to C++17
#set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include  # Include your project's header directory
  ${catkin_INCLUDE_DIRS}
)

# Create the executable
add_executable(log_manager src/log_manager.cpp)

ament_target_dependencies(log_manager
  rclcpp
  std_msgs
)

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS
log_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
