cmake_minimum_required(VERSION 3.5)
project(A1_perception)

set(PROJECT_LIB ${PROJECT_NAME}_lib)
set(PROJECT_TEST ${PROJECT_NAME}_test)

set(BUILD_TESTING OFF)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_custom_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}
	${PCL_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${YAML_CPP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Source files
set(src_files 
	src/perception_node.cpp
	src/filter/filter.cpp
	src/filter/filter_factory.cpp
)

set(dependencies
	rclcpp
	std_msgs
	nav_msgs
	sensor_msgs
	vision_msgs
	pcl_conversions
	geometry_msgs
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
    PCL
	robot_custom_msgs
)

# Add library
add_library(${PROJECT_LIB} STATIC ${src_files})
target_link_libraries(${PROJECT_LIB} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_LIB} ${dependencies})

install(TARGETS
  ${PROJECT_LIB}
  DESTINATION lib/${PROJECT_NAME})

# Add executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIB} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Installation
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION 
  lib/${PROJECT_NAME})

install(DIRECTORY
	launch params
	DESTINATION
	share/${PROJECT_NAME}
)
#if(BUILD_TESTING )
#	find_package(ament_cmake_gtest REQUIRED)
#	
#	# Add test executable
#	ament_add_gtest(${PROJECT_TEST} 
#		test/test_main.cpp
#		test/test_filter.cpp
#	)
#	
#	target_link_libraries(${PROJECT_TEST}
#		${PROJECT_LIB}
#		${PCL_LIBRARIES}
#		${YAML_CPP_LIBRARIES}
#	)
#	ament_target_dependencies(${PROJECT_TEST} ${dependencies})
#	
#	install(TARGETS
#	${PROJECT_TEST}
#	DESTINATION lib/${PROJECT_NAME})
#endif()


# if(CMAKE_BUILD_TYPE MATCHES Debug)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
#     set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
# endif()

ament_package()
