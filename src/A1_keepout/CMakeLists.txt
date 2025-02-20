cmake_minimum_required(VERSION 3.5)
project(A1_keepout)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_executable(A1_keepout src/main.cpp src/A1_keepout.cpp)
ament_target_dependencies(A1_keepout rclcpp std_msgs nlohmann_json sensor_msgs robot_custom_msgs)

install(TARGETS A1_keepout 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
