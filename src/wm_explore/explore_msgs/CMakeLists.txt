cmake_minimum_required(VERSION 3.8)
project(explore_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies 
	rosidl_default_generators
	builtin_interfaces
	geometry_msgs
	std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
        # "srv/WarmupRequest.srv"
	"action/Warmup.action"
	"srv/OptimizeFrontiers.srv"
        DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
