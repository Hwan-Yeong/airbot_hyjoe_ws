cmake_minimum_required(VERSION 3.8)
project(robot_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbnormalEventData.msg"  
  "msg/BatteryStatus.msg"
  "msg/BlockArea.msg"
  "msg/BlockAreaList.msg"
  "msg/BottomIrData.msg"
  "msg/Position.msg"
  "msg/MotorStatus.msg"
  "msg/TofData.msg"
  "msg/ImuCalibration.msg"
  "msg/CameraData.msg"
  "msg/CameraDataArray.msg"
  "msg/StationData.msg"
  "msg/ErrorList.msg"
  "msg/RpmControl.msg"
  "msg/ErrorListArray.msg"
  "msg/LineLaserData.msg"
  "msg/LineLaserDataArray.msg"
  "msg/RobotState.msg"
  "msg/NaviState.msg"
  "msg/MoveNRotation.msg"
  DEPENDENCIES std_msgs
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

ament_package()
