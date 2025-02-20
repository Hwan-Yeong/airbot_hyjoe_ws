cmake_minimum_required(VERSION 3.8)
project(airbot_ai_interface)

# Add compile options for security and optimization
add_compile_options(
  -fstack-protector-strong  # Enable stack protector
  -flto                     # Enable link-time optimization
)

# Add linker flags for memory protections
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro,-z,now")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Include directories
include_directories(
  include  # Include your project's header directory
  ${catkin_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the executable
add_executable(airbot_ai_interface src/ai_interface.cpp)

ament_target_dependencies(airbot_ai_interface
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
  tf2_geometry_msgs
  nav2_msgs
  robot_custom_msgs
  builtin_interfaces
)

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS
airbot_ai_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
