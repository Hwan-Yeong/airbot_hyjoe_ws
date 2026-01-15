cmake_minimum_required(VERSION 3.8)
project(follow_msgs)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
set( msg_files
	"msg/Detect.msg"  
	"msg/TrackerUserArray.msg"  
	"msg/TrackerUser.msg"  
	"msg/UiClient.msg"
	)
ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
	)
ament_package()
