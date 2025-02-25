cmake_minimum_required(VERSION 3.8)
project(airbot_log)

# Set build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Add compile options for debugging and security
add_compile_options(
  -Wall -Wextra -Wpedantic  # Enable warnings
  -g                        # Enable debugging
  -fstack-protector-strong  # Enable stack protector
  -flto                     # Enable link-time optimization
)

# Add linker flags for memory protections
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro,-z,now")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# Include directories
include_directories(
  include  # Include your project's header directory
)

# Find libLOGNetwork headers (keep in include/airbot_log)
find_path(LIBNETWORK_INCLUDE_DIR
  NAMES libNetwork.h
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/airbot_log
  NO_DEFAULT_PATH
)

# Find libLOGNetwork.so (keep in include/airbot_log)
find_library(LIBNETWORK_LIBRARY 
  NAMES LOGNetwork libLOGNetwork
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/airbot_log
  NO_DEFAULT_PATH
)

if(NOT LIBNETWORK_INCLUDE_DIR OR NOT LIBNETWORK_LIBRARY)
  message(FATAL_ERROR "libLOGNetwork headers or library not found. Check the paths.")
endif()

# Executable
add_executable(logging_amr src/logging_amr.cpp)

# Link dependencies
ament_target_dependencies(logging_amr rclcpp spdlog)

target_include_directories(logging_amr PRIVATE ${LIBNETWORK_INCLUDE_DIR})
target_link_libraries(logging_amr ${LIBNETWORK_LIBRARY} fmt::fmt spdlog::spdlog)

# Install targets
install(TARGETS
  logging_amr
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/airbot_log/
  DESTINATION include/airbot_log
)

# Ensure libLOGNetwork.so is installed (keep in include/airbot_log)
install(FILES ${LIBNETWORK_LIBRARY}
  DESTINATION lib
)

ament_package()
