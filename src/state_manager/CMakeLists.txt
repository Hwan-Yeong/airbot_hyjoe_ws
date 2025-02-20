cmake_minimum_required(VERSION 3.8)
project(state_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_compile_options(
  -fstack-protector-strong  # Enable stack protector
  -flto                     # Enable link-time optimization
)

# Add linker flags for memory protections
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro,-z,now")


# Include directories
include_directories(
  # include  # Include your project's header directory
  ${ament_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src/states
  ${PROJECT_SOURCE_DIR}/src/utils
)

set(LIB_SOURCES
  src/state_manager.cpp
  ${PROJECT_SOURCE_DIR}/src/states/idle.cpp
  ${PROJECT_SOURCE_DIR}/src/states/navigation.cpp
  ${PROJECT_SOURCE_DIR}/src/states/docking.cpp
  ${PROJECT_SOURCE_DIR}/src/states/undocking.cpp
  ${PROJECT_SOURCE_DIR}/src/states/on_station.cpp
  ${PROJECT_SOURCE_DIR}/src/states/auto_mapping.cpp
  ${PROJECT_SOURCE_DIR}/src/states/manual_mapping.cpp
  ${PROJECT_SOURCE_DIR}/src/states/return_charger.cpp
  ${PROJECT_SOURCE_DIR}/src/states/state_base.cpp
  #${PROJECT_SOURCE_DIR}/src/states/factory_navigation.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/state_utils.cpp
)

# Executable (if needed)
# add_executable(state_manager ${LIB_SOURCES} )
add_library(state_manager_lib ${LIB_SOURCES} )

# Executable
add_executable(state_manager src/node.cpp)

# Link dependencies
ament_target_dependencies(state_manager_lib
  rclcpp
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  robot_custom_msgs
  nav2_msgs
)

#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(state_manager state_manager_lib)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets (adjust DESTINATION paths if necessary)
install(TARGETS
state_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  state_manager_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
