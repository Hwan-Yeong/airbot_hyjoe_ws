cmake_minimum_required(VERSION 3.8)
project(airbot_navigation)

set(CMAKE_BUILD_TYPE Release) #set(CMAKE_BUILD_TYPE Debug) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY launch rviz params behavior_tree
  DESTINATION share/${PROJECT_NAME})


ament_export_dependencies(rclpy)
ament_package()
