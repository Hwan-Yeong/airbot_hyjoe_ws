#include "state_manager/utils/state_utils.hpp"
#include "rclcpp/qos.hpp"

std::string json_path = "/home/airbot/app_rw/stationPose/station_pose.json";

namespace airbot_state {

StateUtils::StateUtils(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
  rclcpp::QoS A1_qos_profile(rclcpp::KeepLast(1));
  A1_qos_profile.best_effort();

  rclcpp::QoS qos_state_profile = rclcpp::QoS(5)
                            .reliable()
                            .durability_volatile();
  
  //hjkim : 노드 생성 후 topic 메세지 전송으로 transient_local 로 설정.
  rclcpp::QoS qos_localization_profile = rclcpp::QoS(5)
                            .reliable()
                            .transient_local();
  
  //hjkim : 모드 요청 cmd는 마지막 한개만 가지고있으면 됨
  rclcpp::QoS qos_localization_cmd_profile = rclcpp::QoS(1)
                            .reliable()
                            .transient_local();
  
  rclcpp::QoS qos_best_effort_profile = rclcpp::QoS(5)
                            .best_effort()
                            .durability_volatile();

  initializeData();

  param_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&StateUtils::paramCallback, this, std::placeholders::_1));
  
  rclcpp::QoS wm_qos_profile = rclcpp::QoS(1).reliable().transient_local();
  maneuver_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/maneuver/use", A1_qos_profile);
  req_clear_costmap_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/clear/costmap", A1_qos_profile);
  req_estimatePose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/request/pose", A1_qos_profile);
  init_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/init_pose", A1_qos_profile);
  req_stop_localization_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/request/stop", A1_qos_profile);

  mapping_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_explore", wm_qos_profile);
  
  reset_odom_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);
  lidar_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
  tof_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);
  camera_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_camera", 10);
  move_fail_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/error/s_code/unreachable_goal", 10);
  alternative_dest_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/alternative_dest_error", 10);
  direct_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  sensor_manager_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_sensor_manager", 10);
  robot_state_pub_ = node_->create_publisher<robot_custom_msgs::msg::RobotState>("/state_datas", qos_state_profile);
  node_status_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/node_status", 10);
  navi_state_pub_ = node_->create_publisher<robot_custom_msgs::msg::NaviState>("/navi_datas", 10);
  dock_pub = node_->create_publisher<std_msgs::msg::UInt8>("/docking_cmd", 10);
  life_cycle_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/lifecycle_cmd", rclcpp::SystemDefaultsQoS());
  batterySleep_cmd_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/cmd_battery_sleep", 10);
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  localization_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("error/s_code/localization_fail",10);
  //motor_mode_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/cmd_motor_mode", 1);
  remote_block_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_remote_block", 1);
  emergency_stop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);

  block_area_pub_ = node_->create_publisher<robot_custom_msgs::msg::BlockAreaList>("/block_areas", 10);
  block_wall_pub_ = node_->create_publisher<robot_custom_msgs::msg::BlockAreaList>("/block_walls", 10);

  cmd_localization_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_localization_mode",qos_localization_cmd_profile);

  #if USE_LIDAR_STATE_CHECK > 0
  scanHz_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/scanHz_state", 10, std::bind(&StateUtils::scanHzStateCallback, this, std::placeholders::_1));
  lidar_state_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("/lidar_state", 10, std::bind(&StateUtils::lidarStateCallback, this, std::placeholders::_1));
  camera_data_sub = node_->create_subscription<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10, std::bind(&StateUtils::cameraCallback, this, std::placeholders::_1));
  #else
  scan_monitor_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/scan_monitor_cmd", 10);
  scan_front_state_sub = node_->create_subscription<std_msgs::msg::Bool>("/scan_state_front", 10, std::bind(&StateUtils::scan_front_state_callback, this, std::placeholders::_1));
  scan_back_state_sub = node_->create_subscription<std_msgs::msg::Bool>("/scan_state_back", 10, std::bind(&StateUtils::scan_back_state_callback, this, std::placeholders::_1));
  #endif

  tf_monitor_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/tf_monitor_cmd", 10);
  reboot_ready_complete_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/reboot_ready_complete", 10);
  keepout_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/keepout_enable", 10);  

  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  amcl_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
    std::bind(&StateUtils::amclCallback, this, std::placeholders::_1));
	
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>("/station_data", 10,
    std::bind(&StateUtils::stationData_callback, this, std::placeholders::_1));

  station_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/station_pose", 1,
    std::bind(&StateUtils::stationPoseCallack, this, std::placeholders::_1));
  
  path_plan_destination_sub = node_->create_subscription<std_msgs::msg::Int8>("/path_planning/destination", 1,
    std::bind(&StateUtils::pathPlanDestinationCallback, this, std::placeholders::_1));

  req_target_sub_ = node_->create_subscription<robot_custom_msgs::msg::Position>("/move_target", 10,
    std::bind(&StateUtils::move_target_callback, this, std::placeholders::_1));

  move_charger_sub_ = node_->create_subscription<std_msgs::msg::Empty>("/move_charger", 10,
    std::bind(&StateUtils::move_charger_callback, this, std::placeholders::_1));

  req_rotation_target_sub_ = node_->create_subscription<robot_custom_msgs::msg::MoveNRotation>("/move_n_rotation", 10,
    std::bind(&StateUtils::move_rotation_callback, this, std::placeholders::_1));

  inspection_mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/inspection_mode", 10,
    std::bind(&StateUtils::inspection_mode_callback, this, std::placeholders::_1));
  
  factory_mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/factory_mode", 10,
    std::bind(&StateUtils::factory_mode_callback, this, std::placeholders::_1));

  motor_status_sub = node_->create_subscription<robot_custom_msgs::msg::MotorStatus>("/motor_status", 10,
     std::bind(&StateUtils::motorCallback, this, std::placeholders::_1));
  
  recovery_navi_bringup_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/recovery_navi_bringup",rclcpp::SystemDefaultsQoS(),
    std::bind(&StateUtils::recoveryNaviBringupCallback, this, std::placeholders::_1));

  ready_robot_sub_ = node_->create_subscription<std_msgs::msg::Empty>("/ready_reboot",rclcpp::SystemDefaultsQoS(),
    std::bind(&StateUtils::readyRobotCallback, this, std::placeholders::_1));
  
  //tof sensor 상태 모니터를 위해 상시 구독
  tof_status_sub = node_->create_subscription<robot_custom_msgs::msg::TofData>("/tof_data", 10, 
    std::bind(&StateUtils::tofCallback, this, std::placeholders::_1));
  
  imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>("/imu_data", 10, std::bind(&StateUtils::imuCallback, this, std::placeholders::_1));

  battery_sub = node_->create_subscription<robot_custom_msgs::msg::BatteryStatus>("/battery_status", 10, std::bind(&StateUtils::batteryStatusCallback, this, std::placeholders::_1));

  navi_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();   
  navi_goal_options.result_callback = std::bind(&StateUtils::goalResultCallback, this, std::placeholders::_1);
  navi_goal_options.goal_response_callback = std::bind(&StateUtils::goalResponseCallback, this, std::placeholders::_1);
  enableMonitorSensorRecoveryTimer();

  node_client_ = rclcpp_action::create_client<robot_custom_msgs::action::ManageNode>(node, "manage_node");

  control_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_,"/controller_server");
  global_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_,"/global_costmap/global_costmap");
  local_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_,"/local_costmap/local_costmap");
  map_server_client_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

  map_server_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/lifecycle_controller/map_server_state",
        rclcpp::QoS(1).reliable().durability_volatile(),
        std::bind(&StateUtils::setMapServerNodeActive, this, std::placeholders::_1));

  navigator_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/lifecycle_controller/navigator_state",
        rclcpp::QoS(1).reliable().durability_volatile(),
        std::bind(&StateUtils::setNaviNodeActive, this, std::placeholders::_1));
  
  map_copy_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/map_copy_complete", 10,
      std::bind(&StateUtils::mapCopyCallback, this, std::placeholders::_1));

  maneuver_state_sub_ = node_->create_subscription<std_msgs::msg::Int8MultiArray>("/maneuver/state", 1, std::bind(&StateUtils::maneuverStateCallback, this, std::placeholders::_1));
  perception_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/perception/state", 1, std::bind(&StateUtils::perceptionStateCallback, this, std::placeholders::_1));

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StateUtils::odom_callback, this, std::placeholders::_1));
  odom_status_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("/odom_status", 1, std::bind(&StateUtils::odom_status_callback, this, std::placeholders::_1));

  localize_complete_sub = node_->create_subscription<std_msgs::msg::Bool>("/localization/complete", 1,
    std::bind(&StateUtils::localizationComplete_callback, this, std::placeholders::_1));

  initial_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1,
    std::bind(&StateUtils::initial_pose_callback, this, std::placeholders::_1));

  localization_state_sub = node_->create_subscription<std_msgs::msg::Bool>("/localization/state", 1, 
    std::bind(&StateUtils::localization_state_callback, this, std::placeholders::_1));

  localization_mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/localization_mode",qos_localization_profile, std::bind(&StateUtils::localizationModeCallback, this, std::placeholders::_1));

  mapinfo_chaged_sub_ = node_->create_subscription<std_msgs::msg::Empty>("/mapinfo_changed", qos_best_effort_profile, std::bind(&StateUtils::mapInfoChangedCallback, this, std::placeholders::_1));

  recovery_local_sub_ = node_->create_subscription<std_msgs::msg::Int8>("/recovery_local", rclcpp::SystemDefaultsQoS(), std::bind(&StateUtils::recoveryLocalCallback, this, std::placeholders::_1));

  keepout_state_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("/keepout_state", 1, std::bind(&StateUtils::keepoutStateCallback, this, std::placeholders::_1));

  manual_control_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/soc_vel", 10, std::bind(&StateUtils::manualControlCallback, this, std::placeholders::_1));
}

void StateUtils::monitorRPMStopTimer()
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  double current_time = steady_clock.now().seconds();
  double stop_check_elapsed_time = current_time - stop_check_start_time;

  RCLCPP_INFO(node_->get_logger(), "[stopDriving-monitorRPMStop] right rpm:[%d] left rpm:[%d] || elapsed time: %.2f" ,right_motor_rpm,left_motor_rpm, stop_check_elapsed_time);
  if( right_motor_rpm == 0 && left_motor_rpm == 0 ){
    RCLCPP_INFO(node_->get_logger(), "[stopDriving-monitorRPMStop] ALL MOTOR STOP! || right rpm:[%d]/left rpm:[%d]\n" ,right_motor_rpm,left_motor_rpm);
    stop_driving_done = true;
  } else if( stop_check_elapsed_time >= 2.0 ){
    RCLCPP_INFO(node_->get_logger(), "[stopDriving-monitorRPMStop] Timeout 2sec. Force stop. elapsed: %.2f", stop_check_elapsed_time); 
    stop_driving_done = true;
  }

  if( stop_driving_done ){
    disableMonitorRPMStopTimer();
  }
}
void StateUtils::disableMonitorRPMStopTimer()
{
  //saveLastPosition();
  publishSenSorManagerOff();
  publishClearCostMap();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //hjkim : sensor manager off & costmap clear before maneuver & perception off
  enableManeuverCommand(false);//publishManeuverOff();
  publishVelocityCommand(0.0,0.0);
  if (stopdriving_rpmstop_timer_) {  // 등록되어 있다면 해제
    stopdriving_rpmstop_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "monitorRPMStopTimer disable");
  }else{
    RCLCPP_INFO(node_->get_logger(), "monitorRPMStopTimer is already disabled");
  }
}

void StateUtils::initializeData()
{
  initParameters();
  initializePose();
  
  pre_state_id = ROBOT_STATE::IDLE;
  pre_status_id = ROBOT_STATUS::VOID;
  state_id = ROBOT_STATE::IDLE;
  status_id = ROBOT_STATUS::VOID;
  movingstate_id = NAVI_STATE::IDLE;
  movingfail_id = NAVI_FAIL_REASON::VOID;
  node_status_id = NODE_STATUS::IDLE;
  scan_callback_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  bCheckAmclAfterLocalization = false;
  battery_percentage_ = 100;
}

void StateUtils::initParameters()
{
  node_->declare_parameter("enable_lidar_onoff", true);
  node_->declare_parameter("enable_camera_onoff", true);
  node_->declare_parameter("enable_tof_onoff", true);

  node_->get_parameter("enable_lidar_onoff", enable_lidar_onoff_);
  node_->get_parameter("enable_camera_onoff", enable_camera_onoff_);
  node_->get_parameter("enable_tof_onoff", enable_tof_onoff_);

  node_->declare_parameter("direct_velocity_v", 0.4);
  node_->declare_parameter("direct_velocity_w", 0.6);
  node_->declare_parameter("factory_velocity_w", 0.7);
  node_->declare_parameter("undocking_distance", 0.5);

  node_->get_parameter("direct_velocity_v", direct_velocity_v_);
  node_->get_parameter("direct_velocity_w", direct_velocity_w_);
  node_->get_parameter("factory_velocity_w", factory_velocity_w_);
  node_->get_parameter("undocking_distance", undocking_distance_);

  node_->declare_parameter("odom_reset_timeout", 5.0);
  node_->declare_parameter("lidar_wait_timeout", 10.0);
  node_->declare_parameter("tof_wait_timeout", 11.0);
  node_->declare_parameter("camera_wait_timeout", 3.0);

  node_->declare_parameter("odom_reset_retry_count", 3);
  node_->declare_parameter("lidar_retry_count", 3);
  node_->declare_parameter("tof_retry_count", 3);
  node_->declare_parameter("camera_retry_count", 3);
  node_->declare_parameter("localization_retry_count", 3);
  node_->declare_parameter("localization_check_error_count", 3);

  node_->declare_parameter("move_goal_retry_count", 1);
  node_->declare_parameter("sensor_off_time", 60.0);

  node_->declare_parameter("return_charger_try_docking_distance_th_m", 2.0);

  // 🏷️ 파라미터 값 가져오기
  node_->get_parameter("odom_reset_timeout", odom_reset_timeout_);
  node_->get_parameter("lidar_wait_timeout", lidar_wait_timeout_);
  node_->get_parameter("tof_wait_timeout", tof_wait_timeout_);
  node_->get_parameter("camera_wait_timeout", camera_wait_timeout_);

  node_->get_parameter("odom_reset_retry_count", odom_reset_retry_count_);
  node_->get_parameter("lidar_retry_count", lidar_retry_count_);
  node_->get_parameter("tof_retry_count", tof_retry_count_);
  node_->get_parameter("camera_retry_count", camera_retry_count_);
  node_->get_parameter("localization_retry_count", localization_retry_count_);
  node_->get_parameter("localization_check_error_count", localization_check_error_count_);

  node_->get_parameter("move_goal_retry_count", move_goal_retry_count_);
  node_->get_parameter("sensor_off_time", sensor_off_time_);

  node_->get_parameter("return_charger_try_docking_distance_th_m", return_charger_try_docking_distance_th_m_);

  RCLCPP_INFO(node_->get_logger(), "Updated enable_lidar_onoff: %s", enable_lidar_onoff_ ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "Updated enable_camera_onoff: %s", enable_camera_onoff_ ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "Updated enable_tof_onoff: %s", enable_tof_onoff_ ? "true" : "false");

  RCLCPP_INFO(node_->get_logger(), "direct_velocity_v: %.2f", direct_velocity_v_);
  RCLCPP_INFO(node_->get_logger(), "direct_velocity_w: %.2f", direct_velocity_w_);
  RCLCPP_INFO(node_->get_logger(), "factory_velocity_w: %.2f", factory_velocity_w_);
  RCLCPP_INFO(node_->get_logger(), "undocking_distance: %.2f", undocking_distance_);

  RCLCPP_INFO(node_->get_logger(), "odom_reset_timeout: %.2f", odom_reset_timeout_);
  RCLCPP_INFO(node_->get_logger(), "lidar_wait_timeout: %.2f", lidar_wait_timeout_);
  RCLCPP_INFO(node_->get_logger(), "tof_wait_timeout: %.2f", tof_wait_timeout_);
  RCLCPP_INFO(node_->get_logger(), "camera_wait_timeout: %.2f", camera_wait_timeout_);

  RCLCPP_INFO(node_->get_logger(), "odom_reset_retry_count: %u", odom_reset_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "lidar_retry_count: %u", lidar_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "tof_retry_count: %u", tof_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "camera_retry_count: %u", camera_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "localization_retry_count: %u", localization_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "localization_check_error_count: %u", localization_check_error_count_);
  RCLCPP_INFO(node_->get_logger(), "move_goal_retry_count: %u", move_goal_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "sensor_off_time: %.2f", sensor_off_time_);

  RCLCPP_INFO(node_->get_logger(), "return_charger_try_docking_distance_th_m: %.1f deg", return_charger_try_docking_distance_th_m_);
}

void StateUtils::clearStationPoseData()
{
  station_pose.x = -0.3;
  station_pose.y = 0.0;
  station_pose.theta = 0.0;
  station_position_msg.header.stamp = rclcpp::Clock().now();  // 타임스탬프 초기화 (예: 0초)
  station_position_msg.header.frame_id = "map";  // 좌표계 설정
  station_position_msg.pose.pose.position.x = station_pose.x;
  station_position_msg.pose.pose.position.y = station_pose.y;
  station_position_msg.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, station_pose.theta);
  station_position_msg.pose.pose.orientation = tf2::toMsg(q);
  station_position_msg.pose.covariance.fill(0.0);
  setInitPoseByStationPose();
}

void StateUtils::clearRobotPoseData()
{
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.theta = 0.0;
  robot_position_msg.header.stamp = rclcpp::Clock().now();  // 타임스탬프 초기화 (예: 0초)
  robot_position_msg.header.frame_id = "map";  // 좌표계 설정
  robot_position_msg.pose.pose.position.x = robot_pose.x;
  robot_position_msg.pose.pose.position.y = robot_pose.y;
  robot_position_msg.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_pose.theta);
  robot_position_msg.pose.pose.orientation = tf2::toMsg(q);
  robot_position_msg.pose.covariance.fill(0.0);

  last_pose = robot_pose;
  last_position_msg = robot_position_msg;
}

void StateUtils::setInitPoseByStationPose() {

  init_pose.x = station_pose.x + 0.3 * std::cos(station_pose.theta);
  init_pose.y = station_pose.y + 0.3 * std::sin(station_pose.theta);
  init_pose.theta = station_pose.theta;
  init_pose_msg = station_position_msg; 
  init_pose_msg.pose.pose.position.x = init_pose.x;
  init_pose_msg.pose.pose.position.y = init_pose.y;

  RCLCPP_INFO(node_->get_logger(), "[setInitPoseByStationPose] init_pose(%.2f,%.2f,%.2f(DEG)), station_pose(%.2f,%.2f,%.2f(DEG))",
  init_pose.x, init_pose.y, RAD2DEG(init_pose.theta), station_pose.x, station_pose.y, RAD2DEG(station_pose.theta));
}

void StateUtils::initializePose()
{
  clearRobotPoseData();
  clearStationPoseData();
  
  if(checkRecoveryReboot() && loadRecoveryJasonFile()){
      setRecoveryRebootflag(true);
  }

  bool bExistStationJason = isFileExists(json_path);
  bool bLoadStationJason = false;
  if(bExistStationJason){
    bLoadStationJason = loadStationPose(json_path);
  }else{
    createDefaultStationPoseFile(json_path);
  }
  
  if(bExistStationJason && bLoadStationJason){
    setInitPoseByStationPose(); //update init_pose
  }else{
    saveStationPoseToJson(json_path);
    RCLCPP_INFO(node_->get_logger(), "[initializePose] FAIL initializePose reason [%s], [%s]",bExistStationJason ? "STATION JSON EXIST " : "STATION JSON NOT EXIST", bLoadStationJason ? "STATION JSON LOAD SUCCESS" : "STATION JSON LOAD FAIL");
  }
}

void StateUtils::resetInitPose() {
  clearStationPoseData();
  saveStationPoseToJson(json_path);
  RCLCPP_INFO(node_->get_logger(), "[resetInitPose] init_pose(%.2f,%.2f,%.2f(DEG)), station_pose(%.2f,%.2f,%.2f(DEG))",
  init_pose.x, init_pose.y,RAD2DEG(init_pose.theta), station_pose.x, station_pose.y,RAD2DEG(station_pose.theta));
}

rcl_interfaces::msg::SetParametersResult StateUtils::paramCallback(const std::vector<rclcpp::Parameter>& params)
{  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
      if(param.get_name() == "enable_lidar_onoff") {
          enable_lidar_onoff_ = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Updated enable_lidar_onoff: %s", enable_lidar_onoff_ ? "true" : "false");
      }
      else if (param.get_name() == "enable_camera_onoff") {
          enable_camera_onoff_ = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Updated enable_camera_onoff: %s", enable_camera_onoff_ ? "true" : "false");
      }
      else if (param.get_name() == "enable_tof_onoff") {
          enable_tof_onoff_ = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Updated enable_tof_onoff: %s", enable_tof_onoff_ ? "true" : "false");
      }
      else if (param.get_name() == "direct_velocity_v") {
        direct_velocity_v_ = param.as_double();
        RCLCPP_INFO(node_->get_logger(), "Updated direct_velocity_v: %.2f", direct_velocity_v_);
      }
      else if (param.get_name() == "direct_velocity_w") {
        direct_velocity_w_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated direct_velocity_w: %.2f", direct_velocity_w_);
      }
      else if (param.get_name() == "factory_velocity_w") {
        factory_velocity_w_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated factory_velocity_w: %.2f", factory_velocity_w_);
      }
      else if (param.get_name() == "undocking_distance") {
        undocking_distance_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated undocking_distance: %.2f", undocking_distance_);
      }
      else if (param.get_name() == "odom_reset_timeout") {
        odom_reset_timeout_ = param.as_double();
        RCLCPP_INFO(node_->get_logger(), "Updated odom_reset_timeout: %.2f", odom_reset_timeout_);
      }
      else if (param.get_name() == "lidar_wait_timeout") {
          lidar_wait_timeout_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated lidar_wait_timeout: %.2f", lidar_wait_timeout_);
      }
      else if (param.get_name() == "tof_wait_timeout") {
          tof_wait_timeout_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated tof_wait_timeout: %.2f", tof_wait_timeout_);
      }
      else if (param.get_name() == "camera_wait_timeout") {
          camera_wait_timeout_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated camera_wait_timeout: %.2f", camera_wait_timeout_);
      }
      else if (param.get_name() == "odom_reset_retry_count") {
          odom_reset_retry_count_ = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Updated odom_reset_retry_count: %u", odom_reset_retry_count_);
      }
      else if (param.get_name() == "lidar_retry_count") {
          lidar_retry_count_ = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Updated lidar_retry_count: %u", lidar_retry_count_);
      }
      else if (param.get_name() == "tof_retry_count") {
          tof_retry_count_ = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Updated tof_retry_count: %u", tof_retry_count_);
      }
      else if (param.get_name() == "camera_retry_count") {
          camera_retry_count_ = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Updated camera_retry_count: %u", camera_retry_count_);
      }
      else if (param.get_name() == "localization_retry_count") {
          localization_retry_count_ = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Updated localization_retry_count: %u", localization_retry_count_);
      }
      else if(param.get_name() == "localization_check_error_count") {
          localization_check_error_count_ = param.as_int();
          RCLCPP_INFO(node_->get_logger(), "Updated localization_check_error_count: %u", localization_check_error_count_);
      }
      else if (param.get_name() == "move_goal_retry_count") {
        move_goal_retry_count_ = param.as_int();
        RCLCPP_INFO(node_->get_logger(), "Updated move_goal_retry_count: %u", move_goal_retry_count_);
      }
      else if (param.get_name() == "sensor_off_time") {
        sensor_off_time_ = param.as_int();
        RCLCPP_INFO(node_->get_logger(), "Updated sensor_off_time: %.2f", sensor_off_time_);
      }
  }
  return result;
}

void StateUtils::enableArrivedGoalSensorsOffTimer()
{
  if(bInspectionMode){
    RCLCPP_INFO(node_->get_logger(), "enableArrivedGoalSensorsOffTimer is not available in InspectionMode");
    return;
  }
  
  arrived_goal_start_time_ = std::chrono::steady_clock::now();
  if(!arrivedgoal_sensoroff_timer_){
    arrivedgoal_sensoroff_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateUtils::monitor_ArrivedGoal_SensorsOff, this));
    if(state_id == ROBOT_STATE::ONSTATION){
      RCLCPP_INFO(node_->get_logger(), "enable ONSTATION SensorsOffTimer");
    }else{
      RCLCPP_INFO(node_->get_logger(), "enable ArrivedGoal SensorsOffTimer");
    }
    
  }else{
    RCLCPP_INFO(node_->get_logger(), "enableArrivedGoalSensorsOffTimer is already ");
  }
}

void StateUtils::disableArrivedGoalSensorsOffTimer()
{
  if (arrivedgoal_sensoroff_timer_) {  // 등록되어 있다면 해제
    arrivedgoal_sensoroff_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "OffSensorTimer disable");
  }else{
    RCLCPP_INFO(node_->get_logger(), "OffSensorTimer is already disabled");
  }
}

void StateUtils::monitor_ArrivedGoal_SensorsOff()
{
  bool bSensorOff = false;
  
  double waitTime = getSteadyClockRunningSeconds(arrived_goal_start_time_);
  double threshold = sensor_off_time_;
  
  if(bInspectionMode){
    RCLCPP_INFO(node_->get_logger(), "[monitor_ArrivedGoal_SensorsOff]timer disable for InspectionMode");
    disableArrivedGoalSensorsOffTimer();
    return;
  }

  if(state_id == ROBOT_STATE::IDLE || state_id == ROBOT_STATE::ERROR || state_id == ROBOT_STATE::ONSTATION || state_id == ROBOT_STATE::FACTORY_NAVIGATION){
    threshold = 5.0;
    if(waitTime >= threshold){
      bSensorOff = true;
      RCLCPP_INFO(node_->get_logger(), "waitTime is [%.2f] sec over STATE : %s",threshold,enumToString(state_id).c_str());
    }
  }else{
    if(waitTime >= threshold){
      bSensorOff = true;
      RCLCPP_INFO(node_->get_logger(), "waitTime is [%.2f] sec over from Arrived Goal STATE : %s",threshold,enumToString(state_id).c_str());
    }
  }
  
  if(bSensorOff){
    publishAllSensorOff();
    disableArrivedGoalSensorsOffTimer();
  }
}

void StateUtils::enableLocalizationStop()
{
  if(stop_localization_timer_){
    RCLCPP_INFO(node_->get_logger(), "already localization stop timer created");
  }else{
    stop_localization_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&StateUtils::stopLocalizationTimer, this));
    RCLCPP_INFO(node_->get_logger(), "localization stop timer created");
  }
}

void StateUtils::disableLocalizationStop()
{
  if(stop_localization_timer_){
    stop_localization_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "localization stop timer reset");
  }
}

#if USE_LIDAR_STATE_CHECK == 0
void StateUtils::enableSensorcallback()
{
  if(enable_lidar_onoff_){
    //250917 KKS : state callback을  상시로 변경하면서 필요없어짐.
    //lidar_front_state_on = false;
    //lidar_back_state_on = false;
    if(!scan_sub){
      scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&StateUtils::scan_callback, this, std::placeholders::_1));
      RCLCPP_INFO(node_->get_logger(), "[enableSensorcallback] create scan_sub");
    }else{
      RCLCPP_INFO(node_->get_logger(), "[enableSensorcallback] scan_sub is already created");
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[enableSensorcallback] lidar sensor is disabled");
  }

  if(enable_camera_onoff_){
    if(!camera_data_sub){
      camera_data_sub = node_->create_subscription<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10, std::bind(&StateUtils::cameraCallback, this, std::placeholders::_1));
      RCLCPP_INFO(node_->get_logger(), "[enableSensorcallback] create camera_data_sub");
    }else{
      RCLCPP_INFO(node_->get_logger(), "[enableSensorcallback] camera_data_sub is already created");
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[enableSensorcallback] camera sensor is disabled");
  }

  RCLCPP_INFO(node_->get_logger(), "enableSensorcallback");
}

void StateUtils::disableSensorcallback()
{
  if (scan_sub) {
    scan_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "scan callback disabled");
  }

  if (camera_data_sub) {
    camera_data_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "camera callback disabled");
  }
}
#endif

void StateUtils::enableManeuverCommand(bool cmd){
  RCLCPP_INFO(node_->get_logger(), "[enableManeuverCommand] prev cmd[%s], new cmd[%s]",maneuverCmd ? "ON" : "OFF", cmd ? "ON" : "OFF");
  if(maneuverCmd == cmd){
    RCLCPP_INFO(node_->get_logger(), "[enableManeuverCommand]enableManeuverCommand already done this cmd[%s]", cmd ? "ON" : "OFF");
  }
  bReady_maneuver_perception = false;
  maneuverCommunicateError = false;
  perceptionCommunicateError = false;
  reset_maneuver_sub_count = 0;
  reset_perception_sub_count = 0;
  bReadyManeuver = false;
  bReadyPerception = false;
  receivePerceptionState = false;
  maneuver_state_array.data.clear();

  if(!maneuver_state_monitor_timer_){
    maneuver_state_monitor_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&StateUtils::maneuverStateMonitor, this));
    RCLCPP_INFO(node_->get_logger(), "[enableManeuverCommand]maneuver state monitor timer created");
  }else{
    RCLCPP_INFO(node_->get_logger(), "[enableManeuverCommand]maneuver state monitor timer already exist");
  }

  if(cmd){
    publishManeuverOn();
  }else{
    publishManeuverOff();
  }
  maneuver_start_time = std::chrono::steady_clock::now();
  maneuverCmd = cmd;
}
void StateUtils::disableManeuverCommand()
{
  if(maneuver_state_monitor_timer_){
    maneuver_state_monitor_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "[disableManeuverCommand]maneuver state monitor timer disabled");
  }else{
    RCLCPP_INFO(node_->get_logger(), "[disableManeuverCommand]maneuver state monitor timer already disabled");
  }
}

void StateUtils::maneuverStateMonitor(){
  if(bReadyManeuver && bReadyPerception){
    bReady_maneuver_perception = true;
    disableManeuverCommand();
    RCLCPP_INFO(node_->get_logger(), "[maneuverStateMonitor]maneuver & perception state OK!");
  }else if(getSteadyClockRunningSeconds(maneuver_start_time) > 4){

    if(!bReadyManeuver){
      maneuverCommunicateError = true;
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateMonitor] maneuver communicate error");
    }
    if(!bReadyPerception){
      perceptionCommunicateError = true;
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateMonitor] perception communicate error");
    }
    disableManeuverCommand();
  }else{
    if(maneuverCmd){
      publishManeuverOn();
    }else{
      publishManeuverOff();
    }

    if(maneuver_state_array.data.empty()){
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateMonitor] maneuver state callback received yet");
    }else{
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateMonitor] received data size: %zu", maneuver_state_array.data.size());
      for (size_t i = 0; i < maneuver_state_array.data.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "  data[%zu] = %d", i, static_cast<int>(maneuver_state_array.data[i]));
      }
    }
    if(!receivePerceptionState){
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateMonitor] perception state callback received yet");
    }
  }
}

double StateUtils::quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw; // Return yaw as theta
}

void StateUtils::startMonitorOdomReset() {
  if(!odom_reset_timer_){
    ready_odom = false;
    odom_reset_cnt_ = 0;
    bStartOdomReset = true;
    bSendResetOdomCmd = true;
    bOdomResetDone = false;
    setOdomResetError(false);
    publishClearOdomReset();
    odom_reset_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),std::bind(&StateUtils::monitor_resetOdom, this));
    odom_reset_monitor_start_time_ = std::chrono::steady_clock::now();
  }else{
    RCLCPP_INFO(node_->get_logger(), "odom_reset_timer already exist");
  }
  
  RCLCPP_INFO(node_->get_logger(), "startMonitorOdomReset");
}


void StateUtils::monitor_resetOdom() {
  double runtime = getSteadyClockRunningSeconds(reset_odom_start_time_);
  bool exitMonitor = false;
  if(bSendResetOdomCmd){
    if(odom_reset_cnt_ > 0){
      RCLCPP_INFO(node_->get_logger(),"Retry - ODOM RESET Count : %d",odom_reset_cnt_);
    }else{
      RCLCPP_INFO(node_->get_logger(),"Start-ODOM RESET odom(%.2f,%.2f,%.2f(DEG), imu:(%.2f(DEG),%.2f(DEG),%.2f(DEG)))", odom_.x, odom_.y,RAD2DEG(odom_.theta),
      RAD2DEG(imu_.roll),RAD2DEG(imu_.pitch),RAD2DEG(imu_.yaw));
    }
    publishStartOdomReset();
    bSendResetOdomCmd = false;
    return;
  }

  if (isValidateResetOdom(odom_)) {
    double total_runTime = getSteadyClockRunningSeconds(odom_reset_monitor_start_time_);
    bOdomResetDone = true;
    exitMonitor = true;
    RCLCPP_INFO(node_->get_logger(), "odom-reset Complete runtime : %.2f, retry coount : %d",total_runTime,odom_reset_cnt_);
    odom_reset_cnt_ = 0;
  } else if (runtime >= odom_reset_timeout_) {
    bSendResetOdomCmd = true;
    if (++odom_reset_cnt_ >= odom_reset_retry_count_) {
      RCLCPP_INFO(node_->get_logger(), "ODOM RESET ERROR!! odom(%.2f,%.2f,%.2f(DEG), imu:(%.2f(DEG),%.2f(DEG),%.2f(DEG)))", odom_.x, odom_.y,RAD2DEG(odom_.theta),
      RAD2DEG(imu_.roll),RAD2DEG(imu_.pitch),RAD2DEG(imu_.yaw));
      odom_reset_cnt_ = 0;
      setOdomResetError(true);
      exitMonitor = true;
    }
    publishClearOdomReset();
  }

  if (exitMonitor) {
    stopMonitorOdom();
  }
}

void StateUtils::stopMonitorOdom()
{
  publishClearOdomReset();
  reset_timerResetOdom();
}

void StateUtils::startSensorMonitor()
{
  //hjkim_250812 :  센서가 이미 켜저있는 경우 센서 플래그를 초기화 하지 않고 timer 생성을 하지 않고, 센서(tof/scan/camera) callback을 생성하여, 센서 체크가 종료 된 후 callback이 켜져있는 문제 수정.
  //센서가 켜저있더라도 check Flag를 초기화 하고, 센서 모니터 timer 생성 하도록 수정 --> 완료 시 종료
  #if USE_LIDAR_STATE_CHECK == 0
  lidar_hz_check_cnt = 0;
  #endif
  /*
  bLidarSensorOK :  scanhzState(200ms alltime pub)와 lidarState(200ms alltime pub)를 기준으로 OK 판단하기 때문에 초기화 유지
  bCameraSensorOK : camera가 꺼지면 callback 안들어오기때문에 초기화 유지
  bToFSensorOK : tof센서의 경우 상시 callback이 들어오기 때문에 callback에서 알아서 관리해도 무방하여 초기화 초기화 불필요
  */
 if(!enable_lidar_onoff_){
    RCLCPP_INFO(node_->get_logger(), "[lidarSensorOnchecker] Not Use Lidar ON/OFF");
  }
  if(!enable_tof_onoff_){
    RCLCPP_INFO(node_->get_logger(), "[tofSensorOnchecker] Not Use Tof ON/OFF");
  }
  if(!enable_camera_onoff_){
    RCLCPP_INFO(node_->get_logger(), "[cameraSensorOnchecker] Not Use Camera ON/OFF");
  }

  bLidarSensorOK = false;
  bCameraSensorOK = false;
  tof_retry_cnt = 0;
  lidar_retry_cnt = 0;
  camera_retry_cnt = 0;
  disableArrivedGoalSensorsOffTimer();

  setLidarError(false);
  setToFError(false);
  setSensorReady(false);
  publishAllSensorOn();
  
  if(!sensor_monitor_timer_){
    sensor_monitor_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),std::bind(&StateUtils::monitor_sensor, this));
    RCLCPP_INFO(node_->get_logger(), "[startSensorMonitor] sensor_monitor_timer_ created");
  }else{
    RCLCPP_INFO(node_->get_logger(), "[startSensorMonitor] sensor_monitor_timer_ already created");
  }
  sensor_monitor_start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "startSensorMonitor");
}

void StateUtils::stopSensorMonitor()
{
  #if USE_LIDAR_STATE_CHECK == 0
  disableSensorcallback();
  #endif
  reset_timerSensorMonitor();
  RCLCPP_INFO(node_->get_logger(), "stopSensorMonitor");
}

bool StateUtils::lidarSensorOnchecker(){
  if(!enable_lidar_onoff_){
    //RCLCPP_INFO(node_->get_logger(), "[lidarSensorOnchecker] Not Use Lidar ON/OFF");
    return true;
  }
  if(lidarState == 1 && isScanHzOk){
    if(!bLidarSensorOK){
      RCLCPP_INFO(node_->get_logger(), "[lidarSensorOnchecker] lidarSensor is ON");
    }
    bLidarSensorOK = true;
    return true;
  }else if(getSteadyClockRunningSeconds(lidarOn_time) >= lidar_wait_timeout_){
    if(++lidar_retry_cnt >= lidar_retry_count_){
      lidar_retry_cnt = 0;
      RCLCPP_INFO(node_->get_logger(), "[lidarSensorOnchecker] LIdar Error");
      setLidarError(true);
      stopSensorMonitor();
    }else{
      publishLidarOn();
      RCLCPP_INFO(node_->get_logger(), "[lidarSensorOnchecker] retry lidar On count : %u",lidar_retry_cnt);
    }
  }
  return false;
}

bool StateUtils::tofSensorOnchecker(){
  if(!enable_tof_onoff_){
    //RCLCPP_INFO(node_->get_logger(), "[tofSensorOnchecker] Not Use Tof ON/OFF");
    return true;
  }
  if(bMultiToFSensorOK){
    //RCLCPP_INFO(node_->get_logger(), "[tofSensorOnchecker] tofSensor is ON");
    return true;
  }else{
    if(getSteadyClockRunningSeconds(tofOn_time) >= tof_wait_timeout_){
        if(++tof_retry_cnt >= tof_retry_count_){
          tof_retry_cnt = 0;
          RCLCPP_INFO(node_->get_logger(), "[tofSensorOnchecker] Multi Tof Error");
          setToFError(true);
          stopSensorMonitor();
        }else{
          publishMultiTofOn();
          RCLCPP_INFO(node_->get_logger(), "[tofSensorOnchecker] retry Tof On count : %u",tof_retry_cnt);
        }
      }
  }
  return false;
}

bool StateUtils::cameraSensorOnchecker(){
  if(!enable_camera_onoff_){
    //RCLCPP_INFO(node_->get_logger(), "[cameraSensorOnchecker] Not Use Camera ON/OFF");
    return true;
  }

  if(bCameraSensorOK){
    //RCLCPP_INFO(node_->get_logger(), "[cameraSensorOnchecker] cameraSensor Is ON");
    return true;
  }else{
    if(getSteadyClockRunningSeconds(cameraOn_time) >= camera_wait_timeout_){
      if(++camera_retry_cnt >= camera_retry_count_){
        camera_retry_cnt = 0;
        RCLCPP_INFO(node_->get_logger(), "[cameraSensorOnchecker] Camera Error");
        setCameraError(true);
        stopSensorMonitor();
      }else{
        publishCameraOn();
        RCLCPP_INFO(node_->get_logger(), "[cameraSensorOnchecker] retry Camera On count : %u",camera_retry_cnt);
      }
    }
  }
  return false;
}

void StateUtils::monitor_sensor()
{
  if(lidarSensorOnchecker() && tofSensorOnchecker() && cameraSensorOnchecker()){
    RCLCPP_INFO(node_->get_logger(), "[monitor_sensor] All SenSor is ON");
    setSensorReady(true);
    stopSensorMonitor();
  }
}

void StateUtils::startLocalizationMonitor(LOCALIZATION_TYPE type)
{
  localization_retry_cnt = 0;
  Localtype = type;
  bStartLocalizationStart = true;
  bLocalizationComplete = false;
  disableLocalizationStop(); //hjkim : 네비게이션에서 로컬 시작 후 returncharger 실행 시 로컬 시작타이머와 정지 타이머가 동시 실행되는것을 방지
  setLocalizationError(false);
  if(Localtype == LOCALIZATION_TYPE::INIT_POSE){
    publishLocalizeUndockPose();
  }else if(Localtype == LOCALIZATION_TYPE::ROBOT_POSE){
    publishLocalizePose();
  }else if(Localtype == LOCALIZATION_TYPE::SAVED_POSE){
    publishLocalizeSavedPose();
  }else if(Localtype == LOCALIZATION_TYPE::RECOVERY_POSE){
    publishLocalizeRecoveryPose();
  }else{
    RCLCPP_ERROR(node_->get_logger(), "localization type error");
  }
  
  if(localization_monitor_timer_){
    RCLCPP_INFO(node_->get_logger(), "startLocalizationMonitor localization_monitor_timer_ is allready created");
  }else{
    localization_monitor_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),std::bind(&StateUtils::monitor_localization, this));
    RCLCPP_INFO(node_->get_logger(), "startLocalizationMonitor create localization_monitor_timer_");
  }
  
}

void StateUtils::stopLocalizationMonitor()
{
  reset_timerLocalization();
  enableLocalizationStop();
}

void StateUtils::monitor_localization()
{
  double runTime = getSteadyClockRunningSeconds(localize_start_time);
  double localizationTimeout = 5.0;
  if(bLocalizationComplete){    
    //hjkim : localization complete 되면 global localization off
    setCmdGlobalLocalizationMode(false);
    publishLocalizationMode(false);
    RCLCPP_INFO(node_->get_logger(), "Localization Complete %f",runTime);
    stopLocalizationMonitor();
  }else if(bLocalizationFail || runTime >= localizationTimeout){
    if(++localization_retry_cnt >= localization_retry_count_){
      setLocalizationError(true);
      RCLCPP_INFO(node_->get_logger(), "Localization Error");
      stopLocalizationMonitor();
    }else{
      RCLCPP_INFO(node_->get_logger(), "localization retry count : %u",localization_retry_cnt);
      if(Localtype == LOCALIZATION_TYPE::INIT_POSE){
        publishLocalizeUndockPose();
      }else if(Localtype == LOCALIZATION_TYPE::ROBOT_POSE){
        publishLocalizePose();
      }else if(Localtype == LOCALIZATION_TYPE::SAVED_POSE){
        //saveLastPosition();
        publishLocalizeSavedPose();
      }else if(Localtype == LOCALIZATION_TYPE::RECOVERY_POSE){
        publishLocalizeRecoveryPose();
      }else{
        RCLCPP_ERROR(node_->get_logger(), "localization type error");
      }
    }
  }
}

void StateUtils::stopLocalizationTimer() {
  if(!bLocalizationState){
    disableLocalizationStop();
    RCLCPP_INFO(node_->get_logger(), "stopLocalizationTimer - end ");
  }else{
    publishLocalizeStop();
  }
}

void StateUtils::reset_timerResetOdom() {
    if (odom_reset_timer_) {
      odom_reset_timer_.reset();
      RCLCPP_INFO(node_->get_logger(), "reset_timerResetOdom - end ");
    } else {
      RCLCPP_INFO(node_->get_logger(), "odom_reset_timer is allready reset ");
    }
}

void StateUtils::reset_timerSensorMonitor() {
  if (sensor_monitor_timer_) {
    sensor_monitor_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "reset_timerSensorMonitor ");
  } else {
    RCLCPP_INFO(node_->get_logger(), "reset_timerSensorMonitor is allready reset ");
  }
}

void StateUtils::reset_timerLocalization() {
  if (localization_monitor_timer_) {
    localization_monitor_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "reset_timerLocalization");
  } else {
    RCLCPP_INFO(node_->get_logger(), "reset_timerLocalization is allready reset ");
  }
}

bool StateUtils::isValidateResetOdom(const pose &odom) {
  bool ret = false;
  /*hjkim : odom reset 후 roll+-7, pitch+-7, yaw +-3도 이내 기준이면 초기화 완료*/
  if (odom_status == 0x22 && (fabs(odom.x) <= 0.1) && (fabs(odom.y) <= 0.1) &&
      fabs(odom.theta) <= 0.05  && fabs(imu_.roll) <= 0.12 && fabs(imu_.pitch) <= 0.12) {
    RCLCPP_INFO(node_->get_logger(), "[isValidateResetOdom] ODOM RESET Success status(%02x), odom:(%.2f,%.2f,%.2f(DEG), imu:(%.2f(DEG),%.2f(DEG),%.2f(DEG)))",odom_status,odom.x, odom.y,RAD2DEG(odom.theta),
    RAD2DEG(imu_.roll),RAD2DEG(imu_.pitch),RAD2DEG(imu_.yaw));
    ret = true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "[isValidateResetOdom] status(%02x) odom:(%.2f,%.2f,%.2f(DEG)), imu:(%.2f(DEG),%.2f(DEG),%.2f(DEG))",odom_status,odom.x, odom.y,RAD2DEG(odom.theta),
    RAD2DEG(imu_.roll),RAD2DEG(imu_.pitch),RAD2DEG(imu_.yaw));
  }
  return ret;
}

void StateUtils::batteryStatusCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
  battery_percentage_ = msg->battery_percent;
}

uint8_t StateUtils::getBatteryPercentage()
{
  return battery_percentage_;
}

void StateUtils::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tf2::Quaternion q;
    q.setX(msg->orientation.x);
    q.setY(msg->orientation.y);
    q.setZ(msg->orientation.z);
    q.setW(msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    imu_.roll  = roll;
    imu_.pitch = pitch;
    imu_.yaw   = yaw;
    imu_.ax = msg->linear_acceleration.x;
    imu_.ay = msg->linear_acceleration.y;
    imu_.az = msg->linear_acceleration.z;
}

void StateUtils::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_.x = msg->pose.pose.position.x;
    odom_.y = msg->pose.pose.position.y;
    odom_.theta = quaternion_to_euler(msg->pose.pose.orientation);
    ready_odom = true;
}

void StateUtils::odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    if(odom_status != msg->data){
      RCLCPP_INFO(node_->get_logger(), "odom_status hex[%02x]", msg->data);
    }
    odom_status = msg->data;
}

double StateUtils::getReturnChargerTryDockingDistanceThreshold() {
  return return_charger_try_docking_distance_th_m_;
}

bool StateUtils::odomResetWorkingCheck() {
  bool odom_working_status = false;
  if(odom_reset_timer_){
    odom_working_status = true;
  }
  return odom_working_status;
}

void StateUtils::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    double x,y,theta;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta = quaternion_to_euler(msg->pose.pose.orientation);
    RCLCPP_INFO(node_->get_logger(), "initial_pose_callback(%.2f,%.2f,%.2f(DEG))", x, y, RAD2DEG(theta));
}

void StateUtils::localization_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if(bLocalizationState != msg->data){
    RCLCPP_INFO(node_->get_logger(), "localization_state_callback[%d]",static_cast<int>(msg->data));
  }
  bLocalizationState = msg->data;
}

void StateUtils::localizationComplete_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if(msg->data){
    RCLCPP_INFO(node_->get_logger(), "localization-Complete");
    bLocalizationComplete = true;
    setSkipLocalization(true);
    //publishLifeCycleOff();
  }else{
    RCLCPP_INFO(node_->get_logger(), "localization-Fail");
    bLocalizationFail = true;
  } 
}

void StateUtils::localizationModeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  //hjkim : default is global allowed : not receive(-1), local only(0), global allowed(1)
  LOCALIZATION_MODE prev_mode = localizationMode;
   if(msg->data){
    localizationMode = LOCALIZATION_MODE::GLOBAL_ALLOWED;
  }else{
    localizationMode = LOCALIZATION_MODE::LOCAL_ONLY;
  }
  RCLCPP_INFO(node_->get_logger(), "[localizationModeCallback] msg(%s), mode(%s --> %s)", msg->data?"TRUE":"FALSE",enumToString(prev_mode).c_str(),enumToString(localizationMode).c_str()); 
}

void StateUtils::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  double theta = quaternion_to_euler(msg->pose.pose.orientation);
    if(bCheckAmclAfterLocalization){
      RCLCPP_INFO(node_->get_logger(), "Check amcl Pose after Localization amclCallback Prev(%.2f,%.2f,%.2f(DEG)) New(%.2f,%.2f,%.2f(DEG))",
      robot_pose.x,robot_pose.y,RAD2DEG(robot_pose.theta), msg->pose.pose.position.x, msg->pose.pose.position.y, RAD2DEG(theta));
      bCheckAmclAfterLocalization = false;
    }
    robot_position_msg.header = msg->header;
    robot_position_msg.pose = msg->pose;
    robot_pose.x = robot_position_msg.pose.pose.position.x;
    robot_pose.y = robot_position_msg.pose.pose.position.y;
    robot_pose.theta = theta;
}

void StateUtils::slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    robot_position_msg.header = msg->header;
    robot_position_msg.pose = msg->pose;
    robot_pose.x = robot_position_msg.pose.pose.position.x;
    robot_pose.y = robot_position_msg.pose.pose.position.y;
    robot_pose.theta = quaternion_to_euler(robot_position_msg.pose.pose.orientation);
}

void StateUtils::stationPoseCallack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  station_position_msg = *msg;
  init_pose_msg = *msg;
  pose temp_pose;
  temp_pose.x = station_position_msg.pose.pose.position.x;
  temp_pose.y = station_position_msg.pose.pose.position.y;
  temp_pose.theta = quaternion_to_euler(station_position_msg.pose.pose.orientation);

  if(isStationPoseDefaultValue(temp_pose.x, temp_pose.y, temp_pose.theta)){
    station_pose.x = -0.3;
    station_pose.y = 0.0;
    station_pose.theta = 0.0;
    RCLCPP_INFO(node_->get_logger(), "[stationPoseCallack]Convert Default Value SET STATION POSE(%.2f,%.2f,%.2f(DEG))", station_pose.x, station_pose.y,RAD2DEG(station_pose.theta));

  }else{
    station_pose = temp_pose;
    RCLCPP_INFO(node_->get_logger(), "[stationPoseCallack] SET STATION POSE(%.2f,%.2f,%.2f(DEG))", station_pose.x, station_pose.y,RAD2DEG(station_pose.theta));
  }
  
  setInitPoseByStationPose();
  saveStationPoseToJson(json_path);
}

void StateUtils::motorCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg)
{
  left_motor_rpm = msg->left_motor_rpm;
  right_motor_rpm = msg->right_motor_rpm;
}

void StateUtils::factory_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  setFactoryMode( msg->data );
  resetInitPose(); //hjkim : 팩토리 모드 실행(시작/정지)하면 충전기 좌표 초기화 (맵이 바뀌기 때문에 가상벽과 충전기 위치를 초기화, 가상벽은 udp에서 초기화하기때문에 충전기좌표만 초기화함.)
  if(getFactoryMode()){
    RCLCPP_INFO(node_->get_logger(), "[factory_mode_callback] factory_mode start");
    send_node_goal(NODE_STATUS::NAVI);
  }else{
    RCLCPP_INFO(node_->get_logger(), "[factory_mode_callback] factory_mode stop");
  }
}

#if USE_LIDAR_STATE_CHECK == 0
void StateUtils::scan_front_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if(lidar_front_state_on != msg->data){
    RCLCPP_INFO(node_->get_logger(),"[scan_front_state_callback] Current Front Lidar on/off Status ==> [%s]-->[%s]",lidar_front_state_on ? "ON" : "OFF", msg->data ? "ON" : "OFF");
  }
  lidar_front_state_on = msg->data;
}

void StateUtils::scan_back_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if(lidar_back_state_on != msg->data ){
    RCLCPP_INFO(node_->get_logger(),"[scan_back_state_callback] Current Back Lidar on/off Status ==> [%s]-->[%s]",lidar_back_state_on ? "ON" : "OFF", msg->data ? "ON" : "OFF");
  }
  lidar_back_state_on = msg->data;
}
#endif

void StateUtils::recoveryNaviBringupCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Received recovery_navi_bringup: %s", msg->data ? "IN PROGRESS" : "COMPLETED");
  setNavibringUpRecoveryPause(msg->data);
}

void StateUtils::mapCopyCallback(const std_msgs::msg::Empty::SharedPtr/* msg*/)
{
  ROBOT_STATE current_state = getStateID();
  NODE_STATUS current_node_status = getNodeStatusID();
  RCLCPP_INFO(node_->get_logger(), "[mapCopyCallback] Received map_copy robotState: %s, nodeStatus: %s", enumToString(current_state).c_str(), enumToString(current_node_status).c_str());
  if(current_state == ROBOT_STATE::AUTO_MAPPING || current_state == ROBOT_STATE::MANUAL_MAPPING || 
            current_state == ROBOT_STATE::NAVIGATION || current_state == ROBOT_STATE::RETURN_CHARGER)
  {
    RCLCPP_INFO(node_->get_logger(), "[mapCopyCallback] mapCopyCallback is not possible in state : [%s]", enumToString(current_state).c_str());
  }

  setMapCopyReceived();
  RCLCPP_INFO(node_->get_logger(), "[mapCopyCallback] apply new map next Navigation");
}

void StateUtils::mapInfoChangedCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  bMapInfoChanged = true;
  RCLCPP_INFO(node_->get_logger(), "[mapInfoChangedCallback] map info changed");
}

void StateUtils::manualControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_manual_control_time_ = std::chrono::steady_clock::now();

  if (std::abs(msg->linear.x) > 1e-4 || std::abs(msg->angular.z) > 1e-4) {
    if( !manual_control_flag ){
      manual_control_flag = true;  // 제어값이 들어오면 flag 올림.
    }
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] set manual_control_flag >>>>>>>> %d", manual_control_flag);
  } else {
    manual_control_flag = false; // 0값이 들어오면 제어 flag 내림.
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] Control STOP set manual_control_flag >>>>>>>> %d", manual_control_flag);
  }
}

void StateUtils::monitorManualControlTimeout()
{
  // 마지막 제어 메시지 수신 후 1초 이상 경과하면 제어 중단으로 판단
  if (manual_control_flag && getSteadyClockRunningSeconds(last_manual_control_time_) >= 1.0) {
    manual_control_flag = false;
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] Velocity signal lost. Timeout detected.");
  }
}

void StateUtils::startManualControlMonitor() // manual control 시작시 monitor timer 생성.
{
  if (!manual_control_watchdog_timer_) {
    last_manual_control_time_ = std::chrono::steady_clock::now();
    manual_control_watchdog_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&StateUtils::monitorManualControlTimeout, this));
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] Watchdog monitor started.");
  }
}

void StateUtils::stopManualControlMonitor() //manual control 종료시 monitor timer 삭제.
{
  manual_control_watchdog_timer_.reset();
  manual_control_flag = false;
  RCLCPP_INFO(node_->get_logger(), "[ManualControl] Watchdog monitor stopped and flag reset.");
}

bool StateUtils::getManualControlFlag() {
  return manual_control_flag;
}

bool StateUtils::isStationPoseDefaultValue(double x, double y, double theta) {
  int convertX = static_cast<int>(x * 100);
  int convertY = static_cast<int>(y * 100);
  int convertTheta = static_cast<int>(theta * 100);
  if((convertX == 0 && convertY == 0 && convertTheta == 0) || (convertX == -45 && convertY == 0 && convertTheta == 0)){
    RCLCPP_INFO(node_->get_logger(), "Station Pose Default Value origin (%.2f,%.2f,%.2f)(rad) / convert_int(%d,%d,%d)",x, y,theta, convertX,convertY,convertTheta);
    return true;
  } 

  return false;
}

bool StateUtils::isFileExists(const std::string& path) {
  try{
    return std::filesystem::exists(path);
  }catch(const std::exception& e){
    RCLCPP_INFO(node_->get_logger(), "[isFileExists] std::exception occurred : %s", e.what());
    return false;
  }
}

bool StateUtils::createDefaultStationPoseFile(const std::string& path) {
  try {
    auto dir = std::filesystem::path(path).parent_path();
    std::filesystem::create_directories(dir);
    RCLCPP_INFO(node_->get_logger(), "[createDefaultStationPoseFile] Directory checked/created: %s", dir.c_str());

    // Skip if file already exists
    if (std::filesystem::exists(path)) {
      RCLCPP_INFO(node_->get_logger(), "[createDefaultStationPoseFile] File already exists, skipping creation: %s", path.c_str());
      return true;
    }

    nlohmann::json default_json;
    default_json["station_pose"]["x"] = station_pose.x;
    default_json["station_pose"]["y"] = station_pose.y;
    default_json["station_pose"]["theta"] = station_pose.theta;

    std::ofstream file(path);
    if (!file.is_open()) {
      RCLCPP_INFO(node_->get_logger(), "[createDefaultStationPoseFile] Failed to create file: %s", path.c_str());
      return false;
    }

    file << default_json.dump(4);
    file.close();
    RCLCPP_INFO(node_->get_logger(), "[createDefaultStationPoseFile] Default station_pose file created: %s", path.c_str());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_INFO(node_->get_logger(), "[createDefaultStationPoseFile] std::exception while creating file: %s", e.what());
    return false;
  }
}

void StateUtils::saveStationPoseToJson(const std::string& filename) {
  try{
    nlohmann::json saved_json;
    saved_json["station_pose"] = {
    {"x", station_pose.x},
    {"y", station_pose.y},
    {"theta", station_pose.theta}
    };

    std::ofstream file(filename);
    if (file.is_open()) {
        file << saved_json.dump(4);  // 4칸 들여쓰기
        file.close();
        RCLCPP_INFO(node_->get_logger(), "[saveStationPoseToJson] station_pose saved json: %s, pose(%.2f,%.2f,%.2f(DEG)) ", filename.c_str(),station_pose.x,station_pose.y,RAD2DEG(station_pose.theta));
    } else {
        RCLCPP_INFO(node_->get_logger(), "[saveStationPoseToJson] station_pose save json failed: %s, pose(%.2f,%.2f,%.2f(DEG))", filename.c_str(),station_pose.x,station_pose.y,RAD2DEG(station_pose.theta));
    }
  }catch(const std::exception& e){
    RCLCPP_INFO(node_->get_logger(), "[saveStationPoseToJson] std::exception occurred: %s", e.what());
  }
}

bool StateUtils::loadStationPose(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_INFO(node_->get_logger(), "[loadStationPose] can`t open file: %s", file_path.c_str());
    return false;
  }

  try {
      nlohmann::json load_json;
      file >> load_json;

      if(load_json.contains("station_pose")) {
        station_pose.x = load_json["station_pose"]["x"].get<double>();
        station_pose.y = load_json["station_pose"]["y"].get<double>();
        station_pose.theta = load_json["station_pose"]["theta"].get<double>();
        station_position_msg.pose.pose.position.x = station_pose.x;
        station_position_msg.pose.pose.position.y = station_pose.y;
        station_position_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, station_pose.theta);
        station_position_msg.pose.pose.orientation = tf2::toMsg(q);
        station_position_msg.pose.covariance.fill(0.0);
        RCLCPP_INFO(node_->get_logger(), "[loadStationPose]station_pose parsing complete: %s ,pose(%.2f,%.2f,%.2f(DEG))", file_path.c_str(),station_pose.x,station_pose.y,RAD2DEG(station_pose.theta));
        file.close();
        return true;
      } else {
        RCLCPP_INFO(node_->get_logger(), "[loadStationPose]station_pose parsing failed not found key : %s", file_path.c_str());
        file.close();
        return false;
      }
  } catch (const std::exception& e) {
    RCLCPP_INFO(node_->get_logger(), "[loadStationPose] std::exception station_pose parsing failed : %s", e.what());
    file.close();
    return false;
  }
}

void StateUtils::pathPlanDestinationCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "[pathPlanDestinationCallback] pathPlanDestination prev : %d --> new: %d", pathPlanDestination, msg->data);
  pathPlanDestination = msg->data;
}

#if USE_LIDAR_STATE_CHECK > 0
void StateUtils::scanHzStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if(isScanHzOk != msg->data){
    RCLCPP_INFO(node_->get_logger(), "[scanHzStateCallback] isScanHzOk prev[%s] --> new[%s]", isScanHzOk?"OK":"NOT OK", msg->data?"OK":"NOT OK"); 
  }
  isScanHzOk = msg->data;
}
void StateUtils::lidarStateCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if(lidarState != msg->data){
    RCLCPP_INFO(node_->get_logger(), "[lidarStateCallback] lidarState (0:OFF, 1:ON, 2:STARTING, 3:STOPING) prev : %d --> new: %d", lidarState, msg->data); 
  }
  lidarState = msg->data;
}

#else
void StateUtils::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 현재 시간 가져오기
  if(!lidar_front_state_on || !lidar_back_state_on){
    return;
  }
  
  double diff = getSteadyClockRunningSeconds(lidarOn_time);
  if (msg) {
    rclcpp::Time current_time = msg->header.stamp;
	#if 0 // lidar on/off time stamp log for test
    if (current_time < scan_callback_time_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Received an outdated scan message! Current: %.3f, Previous: %.3f",
                    current_time.seconds(), scan_callback_time_.seconds());
    }
	#endif
    if (scan_callback_time_.nanoseconds() != 0) {
        double period = (current_time - scan_callback_time_).seconds();
        
        //비현실적으로 작은 period (분모 폭발 방지)
        constexpr double kMinDt = 1e-3; // 1 ms
        if (period < kMinDt) {
          // 너무 작은 주기는 버리고 다음 루프에서 다시 측정
          scan_callback_time_ = current_time;
          RCLCPP_INFO(node_->get_logger(), "scan period too small: %.3f", period);
          return;
        }

        double frequency = 1.0 / period;
        // RCLCPP_INFO(node_->get_logger(), "Scan Frequency: %.2f Hz", frequency);
        if (!msg->ranges.empty() && frequency >= 9 && frequency <= 11) {
          if(lidar_hz_check_cnt++ >= 10){
            if(!bLidarSensorOK){
              RCLCPP_INFO(node_->get_logger(), "Lidar Sensor is OK time(%.3f),hz(%.2f)",diff,frequency);
            }
            lidar_hz_check_cnt = 0;
            lidar_retry_cnt = 0;
            bLidarSensorOK = true;
          }
        }else{
          if(msg->ranges.empty()){
            RCLCPP_INFO(node_->get_logger(), "scan data is empty");
          }
          if(frequency < 9 || frequency > 11){
            if(lidar_hz_check_cnt > 0){
              lidar_hz_check_cnt = 0;
            }
            RCLCPP_INFO(node_->get_logger(), "scan frequency-low hz(%.2f)",frequency);
          }
        }
    }
    scan_callback_time_ = current_time;
  } else {
      RCLCPP_WARN(node_->get_logger(), "Received nullptr message!");
  }
}
#endif

void StateUtils::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    //RCLCPP_INFO(node_->get_logger(), "tofCallback");
    double diff = getSteadyClockRunningSeconds(tofOn_time);
    uint8_t new_left = getLowerBits(msg->bot_status);
    uint8_t new_right = getUpperBits(msg->bot_status);

    if( bottom_left != new_left || bottom_right != new_right){
      RCLCPP_INFO(node_->get_logger(), "tof bottom_left status:[%02x]/bottom_right status:[%02x]", new_left, new_right);
      if(new_left == 0x01 && new_right == 0x01){
        RCLCPP_INFO(node_->get_logger(), "tof Sensor Off all");
      }
    }

    bottom_left = new_left;
    bottom_right = new_right;
    
    if( bottom_left == 0x00 && bottom_right == 0x00){
      if(!bMultiToFSensorOK){
        RCLCPP_INFO(node_->get_logger(), "tof Sensor is ON Time(%.3f)",diff);
      }
      tof_retry_cnt = 0;
      bMultiToFSensorOK = true;
    }else{
      if(bMultiToFSensorOK){
        RCLCPP_INFO(node_->get_logger(), "tof Sensor is OFF");
      }
      bMultiToFSensorOK = false;
    }
}

void StateUtils::cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr /*msg*/)
{
    if(!bCameraSensorOK){
      RCLCPP_INFO(node_->get_logger(), "[cameraCallback]camera Sensor is ON");
    }
    camera_retry_cnt = 0;
    bCameraSensorOK = true;
}

bool StateUtils::isStartLocalization()
{
    return bStartLocalizationStart; 
}

bool StateUtils::getLocalizationComplete()
{
    return bLocalizationComplete;
}

bool StateUtils::isStartOdomReset()
{
  return bStartOdomReset;
}

bool StateUtils::getOdomResetDone(){
    bool ret = false;
    if(!bStartOdomReset){
      RCLCPP_INFO(node_->get_logger(), "[getOdomResetDone]not start OdomReset");
    }else if(bOdomResetDone){
      RCLCPP_INFO(node_->get_logger(), "[getOdomResetDone]odom reset completes");
      bStartOdomReset = false;
      bOdomResetDone = false;
      ret = true;
    }
    return ret;
}

void StateUtils::setOdomResetError(bool set)
{
  if(bOdomResetError != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "enable - OdomResetError");
    }else{
      RCLCPP_INFO(node_->get_logger(), "disable - OdomResetError");
    }
  }
  bOdomResetError = set;
}

bool StateUtils::isOdomResetError()
{
  bool ret = false;
  if(!bStartOdomReset){
    RCLCPP_INFO(node_->get_logger(), "[isOdomResetError]not start OdomReset");
  }else if(bOdomResetError){
    RCLCPP_INFO(node_->get_logger(), "[getOdomResetDone]odom reset error");
    bStartOdomReset = false;
    bOdomResetError = false;
    ret = true;
  }
 
  return ret;
}

void StateUtils::setLidarError(bool set)
{
  if(bLidarError != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "enable - LidarError");
    }else{
      RCLCPP_INFO(node_->get_logger(), "disable - LidarError");
    }
  }
  bLidarError = set;
}

bool StateUtils::isLidarError()
{
  return bLidarError;
}

void StateUtils::setToFError(bool set)
{
  if(bTofError != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "enable - ToFError");
    }else{
      RCLCPP_INFO(node_->get_logger(), "disable - ToFError");
    }
  }
  bTofError = set;
}

bool StateUtils::isToFError()
{
  return bTofError;
}

void StateUtils::setCameraError(bool set)
{
  if(bCameraError != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "enable - CameraError");
    }else{
      RCLCPP_INFO(node_->get_logger(), "disable - CameraError");
    }
  }
  bCameraError = set;
}

bool StateUtils::isCamreaError()
{
  return bCameraError;
}

void StateUtils::setLocalizationError(bool set)
{
  if(bLoclizationError != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "enable - LocalizationError");
    }else{
      RCLCPP_INFO(node_->get_logger(), "disable - LocalizationError");
    }
  }
  bLoclizationError = set;
}

bool StateUtils::isLocalizationError()
{
  return bLoclizationError;
}

void StateUtils::publishStartOdomReset()
{
  std_msgs::msg::UInt8 odom_reset_cmd_;
  odom_reset_cmd_.data = 0x01; // Odom IMU Reset
  reset_odom_pub_->publish(odom_reset_cmd_);
  RCLCPP_INFO(node_->get_logger(), "publish-StartOdomReset");
  reset_odom_start_time_ = std::chrono::steady_clock::now();
}

void StateUtils::publishClearOdomReset()
{
  std_msgs::msg::UInt8 odom_reset_cmd_;
  odom_reset_cmd_.data = 0x00; // Odom IMU Reset
  reset_odom_pub_->publish(odom_reset_cmd_);
  RCLCPP_INFO(node_->get_logger(), "publish-ClearOdomReset");
}


void StateUtils::publishLidarOn()
{
    std_msgs::msg::Bool lidar_cmd;
    lidar_cmd.data = true;
    lidar_cmd_pub_->publish(lidar_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishLidarOn");
    lidarOn_time = std::chrono::steady_clock::now();
}

void StateUtils::publishLidarOff()
{
    std_msgs::msg::Bool lidar_cmd;
    lidar_cmd.data = false;
    lidar_cmd_pub_->publish(lidar_cmd);
    saveLastPosition();
    if(getNodeStatusID() == NODE_STATUS::NAVI || getNodeStatusID() == NODE_STATUS::FT_NAVI){
      setSkipLocalization(false);
      RCLCPP_INFO(node_->get_logger(), "lidar off - skip localization flag false node(%s), LastPose(%.2f, %.2f, %.2f)",
      enumToString(getNodeStatusID()).c_str(), last_pose.x, last_pose.y, RAD2DEG(last_pose.theta));
    }
    RCLCPP_INFO(node_->get_logger(), "publishLidarOff");
}

void StateUtils::publishMultiTofOn()
{
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = true;
    tof_cmd_pub_->publish(tof_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishMultiTofOn");
    tofOn_time = std::chrono::steady_clock::now();
}

void StateUtils::publishMultiTofOff()
{
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = false;
    tof_cmd_pub_->publish(tof_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishMultiTofOff");
}

void StateUtils::publishCameraOff()
{
  std_msgs::msg::Bool camera_cmd;
  camera_cmd.data = false;
  camera_cmd_pub_->publish(camera_cmd);
  RCLCPP_INFO(node_->get_logger(), "publishCameraOff");
}
void StateUtils::publishCameraOn()
{
  std_msgs::msg::Bool camera_cmd;
  camera_cmd.data = true;
  camera_cmd_pub_->publish(camera_cmd);
  cameraOn_time = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "publishCameraOn");
}

void StateUtils::publishAllSensorOn()
{
  if(enable_lidar_onoff_){
    publishLidarOn();
  }

  if(enable_tof_onoff_){
    publishMultiTofOn();
  }

  if(enable_camera_onoff_){
    publishCameraOn();
  }
  RCLCPP_INFO(node_->get_logger(), "publishAllSensorOn");
}

void StateUtils::publishAllSensorOff()
{
  if(bInspectionMode){
    RCLCPP_INFO(node_->get_logger(), "[StateUtils][publishAllSensorOff] block Sensor Off for InspectionMode");
    return;
  }
  
  if(enable_lidar_onoff_){
    publishLidarOff();
  }
  if(enable_tof_onoff_){
    publishMultiTofOff();
  }
  if(enable_camera_onoff_){
    publishCameraOff();
  } 
  RCLCPP_INFO(node_->get_logger(), "publishAllSensorOff");
}

void StateUtils::publishClearCostMap()
{
    std_msgs::msg::Empty empty_msg;
    req_clear_costmap_pub_->publish(empty_msg);
    RCLCPP_INFO(node_->get_logger(), "publishClearCostMap");
}

void StateUtils::publishLocalizeUndockPose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    localize_start_time = std::chrono::steady_clock::now();
    req_estimatePose_pub_->publish(init_pose_msg); //station pose set enable
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeUndockPose(%.2f,%.2f,%.2f(DEG))", init_pose.x,init_pose.y,RAD2DEG(init_pose.theta));
}

void StateUtils::publishLocalizePose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    localize_start_time = std::chrono::steady_clock::now();
    req_estimatePose_pub_->publish(robot_position_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizePose(%.2f,%.2f,%.2f(DEG))", robot_pose.x,robot_pose.y,RAD2DEG(robot_pose.theta));
}

void StateUtils::publishLocalizeSavedPose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    geometry_msgs::msg::PoseWithCovarianceStamped msg = getDeadReckoningLastPoseMsg();
    localize_start_time = std::chrono::steady_clock::now();
    double theta = quaternion_to_euler(msg.pose.pose.orientation);
    req_estimatePose_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeSavedPose(%.2f,%.2f,%.2f(DEG))", msg.pose.pose.position.x,msg.pose.pose.position.y,RAD2DEG(theta));
}

void StateUtils::publishLocalizeRecoveryPose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    localize_start_time = std::chrono::steady_clock::now();
    req_estimatePose_pub_->publish(recovery_init_pose_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeRecoveryPose(%.2f,%.2f,%.2f(DEG))", recovery_init_pose.x,recovery_init_pose.y,RAD2DEG(recovery_init_pose.theta));
}


void StateUtils::publishLocalizeStop()
{
    std_msgs::msg::Empty msg;
    bStartLocalizationStart = false;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    req_stop_localization_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalize Stop");
}

void StateUtils::publishManeuverOn()
{
  std_msgs::msg::Bool cmd_maneuver;
  cmd_maneuver.data = true;
  maneuver_cmd_pub_->publish(cmd_maneuver);
  RCLCPP_INFO(node_->get_logger(), "publishManeuverOn");
}
void StateUtils::publishManeuverOff()
{
  std_msgs::msg::Bool cmd_maneuver;
  cmd_maneuver.data = false;
  maneuver_cmd_pub_->publish(cmd_maneuver);
  RCLCPP_INFO(node_->get_logger(), "publishManeuverOff");
}
void StateUtils::publishMappingStart()
{
  std_msgs::msg::Bool cmd_mapping;
  cmd_mapping.data = true;
  mapping_cmd_pub_->publish(cmd_mapping);
  RCLCPP_INFO(node_->get_logger(), "publishMappingStart");
}
void StateUtils::publishMappingStop()
{
  std_msgs::msg::Bool cmd_mapping;
  cmd_mapping.data = false;
  mapping_cmd_pub_->publish(cmd_mapping);
  RCLCPP_INFO(node_->get_logger(), "publishMappingStop");
}

void StateUtils::publishSenSorManagerOn()
{
  std_msgs::msg::Bool cmd_sensor_manager;
  cmd_sensor_manager.data = true;
  sensor_manager_cmd_pub_->publish(cmd_sensor_manager);
  init_pose_pub_->publish(init_pose_msg);
  RCLCPP_INFO(node_->get_logger(), "publishSenSorManagerOn");
}

void StateUtils::publishSenSorManagerOff()
{
  std_msgs::msg::Bool cmd_sensor_manager;
  cmd_sensor_manager.data = false;
  sensor_manager_cmd_pub_->publish(cmd_sensor_manager);
  RCLCPP_INFO(node_->get_logger(), "publishSenSorManagerOff");
}

void StateUtils::publishLifeCycleOff()
{
  std_msgs::msg::UInt8 msg;
  msg.data = 0;
  life_cycle_cmd_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishLifeCycleOff [%d]", msg.data);
}

void StateUtils::publishRobotCommand(REQUEST_ROBOT_CMD robot_cmd_msg)
{
  auto req_state_msg = std_msgs::msg::UInt8();
  req_state_msg.data = static_cast<uint8_t>(robot_cmd_msg);
  req_robot_cmd_pub_->publish(req_state_msg);
  RCLCPP_INFO(node_->get_logger(), "publishRobotCommand [%s]", enumToString(robot_cmd_msg).c_str());
}

// void ManualMapping::publishMotorMode(uint8_t mode)
// {
//   std_msgs::msg::UInt8 msg;
//   msg.data = mode;
//   motor_mode_pub_->publish(msg);
//   RCLCPP_INFO(node_->get_logger(), "[ManualMapping] publish motor mode : 0x%02x", mode);
// }

void StateUtils::publishRemoteBlock(bool set){
  std_msgs::msg::Bool msg;
  msg.data = set;
  remote_block_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishRemoteBlock [%d]", static_cast<int>(set));
}

void StateUtils::publishEmergencyStop(bool set)
{
  std_msgs::msg::Bool msg;
  msg.data = set;
  emergency_stop_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[publishEmergencyStop] emergency stop status[%s]",set ? "EMERGENCY STOP" : "RELEASED");
}

#if USE_LIDAR_STATE_CHECK == 0
void StateUtils::publishScanMonitor(bool set){
  std_msgs::msg::Bool msg;
  msg.data = set;
  scan_monitor_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishScanMonitor [%s]", set ? "START" : "STOP");
}
#endif

void StateUtils::publishTFMonitor(bool set){
   std_msgs::msg::Bool msg;
    msg.data = set;
    tf_monitor_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "publishTFMonitor [%s]", set ? "START" : "STOP");
}


void StateUtils::publishRebootReadyComplete(){
  std_msgs::msg::UInt8 msg;
  msg.data = 0;
  reboot_ready_complete_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishRebootReadyComplete");
}

void StateUtils::publishKeepOutEnable(bool set){
  std_msgs::msg::Bool msg;
  msg.data = set;
  keepout_enable_pub_->publish(msg);
  setKeepoutState(KEEPOUT_STATE::KEEPOUT_VOID);
  keep_out_enable_start_time = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "publishKeepOutEnable [%s]", set ? "ENABLE" : "DISABLE");
}

void StateUtils::publishClearVirtualWall()
{
  robot_custom_msgs::msg::BlockAreaList empty_msgs;
  block_wall_pub_->publish(empty_msgs);
  block_area_pub_->publish(empty_msgs);
  RCLCPP_INFO(node_->get_logger(), "publishClearVirtualWall");
}

void StateUtils::publishLocalizationMode(bool set)
{
#if USE_JSLLOC > 0
  localizationMode = LOCALIZATION_MODE::VOID; 
  std_msgs::msg::Bool msg;
  msg.data = set;
  cmd_localization_mode_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishLocalizationMode [%s]", set ? "GLOBAL_ALLOWED" : "LOCAL_ONLY");
#endif
}

//about ROBOT STATE 
void StateUtils::setAllRobotStateIDs(ROBOT_STATE data_state, ROBOT_STATUS data_status, state_cmd data_cmd){
  pre_state_id = getStateID();
  pre_status_id = getStatusID();
  pre_cmd_ids = getRobotCMDID();

  setStateID(data_state, data_status);
  if( data_state == ROBOT_STATE::IDLE || data_state == ROBOT_STATE::ONSTATION )
  {
    data_cmd.soc_cmd = REQUEST_SOC_CMD::VOID;
    data_cmd.robot_cmd = REQUEST_ROBOT_CMD::VOID;
  }
  setRobotCMDID(data_cmd);
}

void StateUtils::setStateID( const ROBOT_STATE &state, const ROBOT_STATUS &status){
  if(state_id != state){
    RCLCPP_INFO(node_->get_logger(), "SET ROBOT_STATE STATE : %s", enumToString(state).c_str());
    publishRobotState(state, status);
  }
  state_id = state;
  
  if( ( status_id == ROBOT_STATUS::READY || status_id == ROBOT_STATUS::COMPLETE) && status == ROBOT_STATUS::PAUSE){ //현재 status가 ready|complete 상태이고, pause로 변환하라고 내려왔을때.
    setReservePause(true);
    RCLCPP_INFO(node_->get_logger(), "SKIP SET STATUS --> SET RESERVE PAUSE : current status[%s] next status[%s]", enumToString(status_id).c_str(), enumToString(status).c_str());
  }else{
    RCLCPP_INFO(node_->get_logger(), "SET ROBOT_STATUS : %s", enumToString(status).c_str());
    status_id = status;
  }
   
}

ROBOT_STATE StateUtils::getStateID(){
  return state_id;
}

void StateUtils::setStatusID( const ROBOT_STATUS &id){
  if(status_id != id){
    RCLCPP_INFO(node_->get_logger(), "SET ROBOT_STATUS STATUS : %s", enumToString(id).c_str());
    if( id != ROBOT_STATUS::RECOVERY_PAUSE ){
      publishRobotStatus(id);
    }
  }
  pre_status_id = status_id;
  status_id = id;
}
ROBOT_STATUS StateUtils::getStatusID(){
  return status_id;
}

void StateUtils::setRobotCMDID( const state_cmd &datas){
  cmd_ids = datas; 
}

state_cmd StateUtils::getRobotCMDID(){
  return cmd_ids;
}

void StateUtils::setMovingStateID(const NAVI_STATE &id, const NAVI_FAIL_REASON &reason){
  if(movingstate_id != id){
    if(id == NAVI_STATE::READY){
      startSensorMonitor();
    }
    RCLCPP_INFO(node_->get_logger(), "SET NAVI_STATE STATE : %s", enumToString(id).c_str());
    publishMovingState(id,reason); 
  }
  if(movingfail_id != reason){
    RCLCPP_INFO(node_->get_logger(), "SET NAVI_FAIL_REASON : %s", enumToString(reason).c_str());
  }

  movingstate_id = id;
  movingfail_id = reason;
}

NAVI_STATE StateUtils::getMovingStateID(){ 
  return movingstate_id; 
}

void StateUtils::setReadyMoving(const READY_MOVING &set){
  if(readyMoving != set){
    RCLCPP_INFO(node_->get_logger(), "setReadyMoving prev[%s] new[%s]",enumToString(readyMoving).c_str(), enumToString(set).c_str());
  }
  readyMoving = set;
}

READY_MOVING StateUtils::getReadyMoving()
{
  return readyMoving;
}

NAVI_FAIL_REASON StateUtils::getMovingFailID(){ 
  return movingfail_id; 
}

void StateUtils::setNodeStatusID(const NODE_STATUS &id){
    /*hjkim260113 : FOLLOWING 모드를 노드상태로 셋팅하면, 위치인식 판단을 위한 노드상태 확인 시 오류가 생기게 되어,
    로컬을 하지 않아도 되는 상황에서 로컬을 하거나 아래처럼 글로벌 로컬 셋팅을 하게되는 문제가 생김.
    팔로미 모드 실행 시 팔로미 종료 후 복귀를 위해 네비게이션 노드를 실행하게 되며, 노드 상태는 NAVI 노드로 유지하고, 상태관리만 하도록 해야함*/
    if(id == NODE_STATUS::FOLLOWING){
      RCLCPP_INFO(node_->get_logger(), "[StateUtils] skip set Following Node");
      return;
    }
    if(node_status_id != id){
      //hjkim 260113 : 팔로우 모드에서 로컬리제이션 기능 사용으로 인해 팔로우 노드 실행 시 global local mode로 변경되는 문제 수정.
      if(id != NODE_STATUS::NAVI){
        setCmdGlobalLocalizationMode(true);
      }
      RCLCPP_INFO(node_->get_logger(), "[StateUtils]Node Status Change %s -> %s", enumToString(node_status_id).c_str(), enumToString(id).c_str());
      if(id == NODE_STATUS::NAVI){
        publishKeepOutEnable(true);
      }else{
        publishKeepOutEnable(false);
      }
      publishNodeState(id);
    }
    node_status_id = id;
}

NODE_STATUS StateUtils::getNodeStatusID(){
    return node_status_id;
}

ROBOT_STATE StateUtils::getPreStateID(){
  return pre_state_id;
}

void StateUtils::setStartOnStation(bool set)
{
  bStartOnStation = set;
}

pose StateUtils::getRobotPose()
{
  return robot_pose;
}

pose StateUtils::getInitPose()
{
  return init_pose;
}

pose StateUtils::getStationPose()
{
  return station_pose;
}

void StateUtils::setMoveGoalPose(double x, double y, double theta){
  move_goal.x = x;
  move_goal.y = y;
  move_goal.theta = theta;
  RCLCPP_INFO(node_->get_logger(), "[setMoveGoalPose] (%.2f,%.2f,%.2f(DEG))", x, y,RAD2DEG(theta));
}
pose StateUtils::getMoveGoalPose()
{
  return move_goal;
}

bool StateUtils::isStartOnStation()
{
  return bStartOnStation;
}

int StateUtils::getPathPlanDestination()
{
  return pathPlanDestination;
}


void StateUtils::setSensorReady(bool set)
{
  bSensorReady = set;
}

bool StateUtils::isSensorReady()
{
  return bSensorReady;
}

bool StateUtils::isManeuverDone()
{
  NODE_STATUS current_nodestatus = getNodeStatusID();
  if(current_nodestatus == NODE_STATUS::NAVI || current_nodestatus == NODE_STATUS::FT_NAVI || current_nodestatus == NODE_STATUS::AUTO_MAPPING){
    return bReady_maneuver_perception;
  }else{
    RCLCPP_INFO(node_->get_logger(), "[isManeuverDone] [%s]NODE",enumToString(current_nodestatus).c_str());
    return true;
  }
}

bool StateUtils::isManeuverCommunicateError()
{
  return maneuverCommunicateError;
}

bool StateUtils::isPerceptionCommunicateError()
{
  return perceptionCommunicateError;
}

void StateUtils::setOnstationStatus(const bool &data, uint8_t status){
  if(on_station_status != data){
    if(data){
      RCLCPP_INFO(node_->get_logger(), "[setOnstationStatus] OnStation docking-Status hex[%02x]",status);
    }else{
      RCLCPP_INFO(node_->get_logger(), "[setOnstationStatus] Off-Station docking-Status hex[%02x]",status);
    }
  }

  on_station_status = data;

  if(isMapCopyReceived() && getNodeStatusID() == NODE_STATUS::NAVI && on_station_status){
    /*hjkim 250813
      맵 카피로 인해 맵 변경이 된 경우, on station 상태에서만  네비노드를 종료 시키고 다음 이동 명령 시 네비게이션 노드를 다시 실행하여, 맵 로드 시키도록 수정.
      최종 목표는 노드를 끄지 않고, 맵 로드만 다시 실행하도록 반영 해야함.
    */
    RCLCPP_INFO(node_->get_logger(), "[setOnstationStatus] map is changed --> Navi node off for map update");
    send_node_goal(NODE_STATUS::IDLE);
    resetMapCopyReceived();
  }
}
bool StateUtils::getOnstationStatus(){
  return on_station_status;
}

void StateUtils::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
  station_short_signal = msg->sig_short;
  station_long_signal = msg->sig_long;
  station_signal_receiver = msg->receiver_status; 

  uint8_t new_status = (msg->docking_status & 0xF0); //new charging status (upperbits)
  uint8_t current_status = (docking_status & 0xF0); //current charging status (upperbits)

  bool isDocking = (msg->docking_status & 0x01);
  if(isDocking){
    if(!bDockingRunning){
      RCLCPP_INFO(node_->get_logger(), "[stationData_callback] dectect MCU Docking start");
    }
    bDockingRunning = true;
  }else{
    if(bDockingRunning){
      RCLCPP_INFO(node_->get_logger(), "[stationData_callback] dectect MCU Docking stop");
    }
    bDockingRunning = false;
  }


  if(current_status != new_status){
    if(new_status & 0x80){
      RCLCPP_INFO(node_->get_logger(), "[stationData_callback] Docking-Fail from MCU");
    }

    if(new_status & 0x70) {
      if (!ondock_start_time_.has_value()) {
        ondock_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[stationData_callback] On-Dock Start Time");
      }
      RCLCPP_INFO(node_->get_logger(), "[stationData_callback] stationData_callback On Station detect start Docking Status hex[%02x]", msg->docking_status);
    }else {
      ondock_start_time_.reset();
      RCLCPP_INFO(node_->get_logger(), "[stationData_callback] stationData_callback off-station detect Docking Status hex[%02x]", msg->docking_status);
    }
  }

  if (msg->docking_status & 0x70) { // charging
    if (ondock_start_time_.has_value()) {
      double runtime = getOptionalSteadyClockRunningSeconds(ondock_start_time_.value());
      if (new_status & 0x60 || runtime >= 0.5) {
        //RCLCPP_INFO(node_->get_logger(), "[stationData_callback] only detect On Docked Status hex[%02x] runtime[%.2f]", msg->docking_status, runtime);
        setOnstationStatus(true, msg->docking_status);
      }
    }
  } else {
    //RCLCPP_INFO(node_->get_logger(), "[stationData_callback] detect Charging Status hex[%02x]", msg->docking_status);
    setOnstationStatus(false, msg->docking_status);
  }
  docking_status = msg->docking_status;
}

void StateUtils::move_target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg) {
  if(getMovingStateID() == NAVI_STATE::READY || getMovingStateID() == NAVI_STATE::MOVE_GOAL){
    cancelPreviousGoal();
  }
  resetTryMoveTargetCount();
  setReservePause(false); //다음 목적지 수신 reserve_pause clear.
  if(getReserveDocking()){
    setReserveDocking(false); // 다음 목적지 수신시 reserve clear.
  }
  setStartOnStation(false);
  movingData.bStartMoving = true;
  movingData.target_position.x = msg->x;
  movingData.target_position.y = msg->y;
  movingData.target_position.theta = msg->theta;
  movingData.type = std::numeric_limits<uint8_t>::max();
  RCLCPP_INFO(node_->get_logger(), "move_target_callback(%.2f,%.2f,%.2f(DEG))", movingData.target_position.x,movingData.target_position.y,RAD2DEG(movingData.target_position.theta));
}

void StateUtils::move_charger_callback(const std_msgs::msg::Empty::SharedPtr) {
  if(getMovingStateID() == NAVI_STATE::READY || getMovingStateID() == NAVI_STATE::MOVE_GOAL){
    cancelPreviousGoal();
  }
  if(getReservePause()){
    setReservePause(false); // reutrn charger 재수신시 reserve pause clear.
  }
  if(getReserveDocking()){
    setReserveDocking(false); // reutrn charger 재수신시 reserve dock clear.
  }
  resetTryMoveTargetCount();
  setStartOnStation(false);
  bTryMoveCharger = true;
  RCLCPP_INFO(node_->get_logger(), "move_charger_callback");
}

void StateUtils::move_rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg) {
  setStartOnStation(false);
  movingData.bStartMoving = true;
  movingData.target_position.x = msg->x;
  movingData.target_position.y = msg->y;
  movingData.target_position.theta = msg->theta;
  movingData.type = msg->type;
  RCLCPP_INFO(node_->get_logger(), "move_rotation_callback(%.2f,%.2f,%.2f(DEG))", movingData.target_position.x,movingData.target_position.y,RAD2DEG(movingData.target_position.theta));
}

void StateUtils::inspection_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  bInspectionMode = msg->data;
  if(bInspectionMode){
    RCLCPP_INFO(node_->get_logger(), "inspection_mode_callback start");
  }else{
    RCLCPP_INFO(node_->get_logger(), "inspection_mode_callback stop");
  }
}

void StateUtils::maneuverStateCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
  if(msg->data.empty()){
    RCLCPP_INFO(node_->get_logger(), "[maneuverStateCallback] empty");
    return;
  }

  maneuver_state_array = *msg;

  if(maneuverCmd && maneuver_state_array.data[0] == 1){
    if(!bReadyManeuver){
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateCallback] On");
    }
    bReadyManeuver = true;
  }else if(!maneuverCmd && maneuver_state_array.data[0] == 0){
    if(!bReadyManeuver){
      RCLCPP_INFO(node_->get_logger(), "[maneuverStateCallback] Off");
    }
    bReadyManeuver = true;
  }
}

void StateUtils::perceptionStateCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if(maneuverCmd == msg->data){
    if(!bReadyPerception){
      if(msg->data){
        RCLCPP_INFO(node_->get_logger(), "[perceptionStateCallback] On");
      }else{
        RCLCPP_INFO(node_->get_logger(), "[perceptionStateCallback] Off");
      }
    }
    bReadyPerception = true;
  }else{
    RCLCPP_INFO(node_->get_logger(), "[perceptionStateCallback] difference command[%s] wiith state[%s] ",maneuverCmd?"ON":"OFF", msg->data?"ON":"OFF");
  }
  receivePerceptionState = true;
}

bool StateUtils::isReservedMoving(){
  return movingData.bStartMoving;
}

void StateUtils::setReservedMoving(const bool &data){
  movingData.bStartMoving = data;
}

MOVING_DATA StateUtils::getTargetPosition()
{
  MOVING_DATA return_movingData = movingData;
  setReservedMoving(false); //bStartMoving에 따라서 navi에서 동작함.
  return return_movingData;
}

pose StateUtils::getCurrentOdom() // for undocking 
{
  return odom_;
}

void StateUtils::startOdomCheck()
{
  ready_odom = false;
}

bool StateUtils::getPrepareOdomFlag()
{
  return ready_odom;
}

void StateUtils::saveLastPosition()
{
  last_odom = odom_;
  last_pose = robot_pose;
  last_position_msg = robot_position_msg;
  RCLCPP_INFO(node_->get_logger(), "saveLastPosition(%.2f, %.2f, %.2f(DEG), odom(%.2f, %.2f, %.2f(DEG)))",
  last_pose.x,last_pose.y,RAD2DEG(last_pose.theta),last_odom.x,last_odom.y,RAD2DEG(last_odom.theta));
}

geometry_msgs::msg::PoseWithCovarianceStamped StateUtils::getDeadReckoningLastPoseMsg()
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg = last_position_msg;
  double delta_x = odom_.x-last_odom.x;
  double delta_y = odom_.y-last_odom.y;
  //hjkim260105 - odom theta dead-reckoning diff from last odom
  double delta_theta = normalize_angle(odom_.theta-last_odom.theta);

  RCLCPP_INFO(node_->get_logger(), "getDeadReckoningLastPoseMsg last_pose(%.2f, %.2f, %.2f(DEG)), delta odom(%.2f, %.2f, %.2f(DEG), last_odom(%.2f, %.2f, %.2f(DEG)), odom(%.2f, %.2f, %.2f(DEG)))",
  last_pose.x,last_pose.y,RAD2DEG(last_pose.theta),delta_x,delta_y,RAD2DEG(delta_theta),last_odom.x,last_odom.y,RAD2DEG(last_odom.theta),odom_.x,odom_.y,RAD2DEG(odom_.theta));
  
  tf2::Quaternion quat;
  quat.setRPY(0, 0, normalize_angle(last_pose.theta+delta_theta));

  msg.pose.pose.position.x = last_pose.x+delta_x;
  msg.pose.pose.position.y = last_pose.y+delta_y;
  msg.pose.pose.orientation = tf2::toMsg(quat);

  return msg;
}

double StateUtils::normalize_angle(double angle) {
  // Normalize angle to the range [-p, p]
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double StateUtils::getAngle(double base, double current)
{
  double angle = current-base;
  return normalize_angle(angle);
}
double StateUtils::getDistance(pose base, pose current) {
  return std::sqrt((current.x - base.x) * (current.x - base.x) + (current.y - base.y) * (current.y - base.y));
}

void StateUtils::publishMoveFailError()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  move_fail_error_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishMoveFailError");
}

void StateUtils::publishAlternativeDestinationError()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  alternative_dest_error_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "publishAlternativeDestinationError");
}

void StateUtils::publishVelocityCommand(double v, double w)
{
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = w;
    direct_vel_pub_->publish(cmd_msg);
    //RCLCPP_INFO(node_->get_logger(), "publishVelocityCommand V : %f, W : %f ", v,w);
}

void StateUtils::publishRobotState(ROBOT_STATE state, ROBOT_STATUS status)
{
  auto req_state_msg = robot_custom_msgs::msg::RobotState();
  req_state_msg.state = int(state);
  req_state_msg.status = int(status);
  robot_state_pub_->publish(req_state_msg);
  RCLCPP_INFO(node_->get_logger(), "[StateUtils]publishRobotState state : %d, status : %d", req_state_msg.state, req_state_msg.status);
}

void StateUtils::publishRobotStatus(ROBOT_STATUS status)
{
  auto req_state_msg = robot_custom_msgs::msg::RobotState();
  req_state_msg.state = int(getStateID());
  req_state_msg.status = int(status);
  robot_state_pub_->publish(req_state_msg);
  RCLCPP_INFO(node_->get_logger(), "[StateUtils]  publishRobotStatus state : %d, status : %d", req_state_msg.state, req_state_msg.status);
}
void StateUtils::publishNodeState(NODE_STATUS state)
{
  auto node_status_msg = std_msgs::msg::UInt8();
  node_status_msg.data = u_int8_t(state);
  node_status_pub_->publish(node_status_msg);
}

void StateUtils::publishMovingState(NAVI_STATE state, NAVI_FAIL_REASON reason)
{
  auto req_navi_msg = robot_custom_msgs::msg::NaviState();
  req_navi_msg.state = int(state);
  req_navi_msg.fail_reason = int(reason);
  navi_state_pub_->publish(req_navi_msg);
}


double StateUtils::getDirectVelocityV()
{
  return direct_velocity_v_;
}

double StateUtils::getDirectVelocityW()
{
  return direct_velocity_w_;
}

double StateUtils::getFactoryVelocityW()
{
  return factory_velocity_w_;
}

double StateUtils::getUndockingDistance()
{
  return undocking_distance_;
}

uint8_t StateUtils::getLocalErrorCount(){
  return localization_check_error_count_;
}

uint8_t StateUtils::getMoveGoalRetryLimit(){
  return move_goal_retry_count_;
}

double StateUtils::getSensorOffTime(){
  return sensor_off_time_;
}
bool StateUtils::getMoveChargerFlag(){
  bool ret = false;
  if(bTryMoveCharger){
    bTryMoveCharger = false;
    ret = true;
  }
  return ret;
}

bool StateUtils::isSkipLocalization()
{
  return bSkipLocalization;
}

//hjkim :  좌표가 튀는 현상으로 인해, 이동 불가에러가 발생한 경우 로컬을 실행하기 위해 로컬 skip 플래그를 초기화 시키기 위해 추가
void StateUtils::setSkipLocalization(bool set){
  RCLCPP_INFO(node_->get_logger(), "[setSkipLocalization] set : %s", set? "SKIP LOCAL" : "DON`T SKIP LOCAL");
  bSkipLocalization = set;
}

bool StateUtils::isLidarSensorOK(){
  return bLidarSensorOK;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   node manage ACTION CLIENT functions..   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void StateUtils::setNodeClientStatus(const int &status){
  node_client_status = status;
}

int StateUtils::getNodeClientStatus(){
  return node_client_status;
}

void StateUtils::send_node_goal(const NODE_STATUS &require_node) {

  if(require_node == NODE_STATUS::NAVI || require_node == NODE_STATUS::FT_NAVI){
    RCLCPP_INFO(node_->get_logger(), "[%s]node launch - skip localization flag false",enumToString(require_node).c_str());
    setSkipLocalization(false);
  }
  if (!node_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "[StateUtils]Node Action server not available after waiting");
    return;
  }

  if(getNodeClientStatus()!= 0){
    setNodeClientStatus(0);
  }

  auto goal_msg = robot_custom_msgs::action::ManageNode::Goal();
  goal_msg.require_node = static_cast<int>(require_node);

  RCLCPP_INFO(node_->get_logger(), "[StateUtils]Node sending request to manage_node for %d...", static_cast<int>(require_node));

  auto node_goal_options = rclcpp_action::Client<robot_custom_msgs::action::ManageNode>::SendGoalOptions();

  node_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<robot_custom_msgs::action::ManageNode>::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(node_->get_logger(), "[StateUtils]Node launch was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "[StateUtils]Node launchaccepted by server, waiting for result");
        }
      };

  node_goal_options.result_callback =
      [this,require_node](const rclcpp_action::ClientGoalHandle<robot_custom_msgs::action::ManageNode>::WrappedResult & result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          setNodeClientStatus(1);
          setNodeStatusID(require_node);
          RCLCPP_INFO(node_->get_logger(), "[StateUtils]Node launched succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          setNodeClientStatus(-1);
          RCLCPP_ERROR(node_->get_logger(), "[StateUtils]Node launch aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(node_->get_logger(), "[StateUtils]Node launch canceled");
          return;
        default:
          setNodeClientStatus(-1);
          RCLCPP_ERROR(node_->get_logger(), "[StateUtils]Node launch failed with Unknown result code");
          return;
        }
        // 결과
        if (result.result->result == 1) {
          RCLCPP_INFO(node_->get_logger(), "[StateUtils] Node change complete");
        } else {
          RCLCPP_INFO(node_->get_logger(), "[StateUtils] Node change fail");
        }
      };

  node_client_->async_send_goal(goal_msg, node_goal_options);
}

void StateUtils::startDocking() {
  dock_cmd_.data = DOCK_START;
  RCLCPP_INFO(node_->get_logger(), "[Docking] publish Start-Docking");
  dock_pub->publish(dock_cmd_);
}

void StateUtils::stopDocking() {
  dock_cmd_.data = DOCK_STOP;
  RCLCPP_INFO(node_->get_logger(), "[Docking] publish Stop-Docking");
  dock_pub->publish(dock_cmd_);
}

void StateUtils::stopDriving(){
  RCLCPP_INFO(node_->get_logger(), "[StateUtils] stopDriving");
  if(bInspectionMode){
    RCLCPP_INFO(node_->get_logger(), "[StateUtils] inspectionMode skip stopDriving");
  }else{
    NODE_STATUS current_nodestatus = getNodeStatusID();
    ROBOT_STATE current_state = getStateID();
    if(current_nodestatus == NODE_STATUS::NAVI || current_nodestatus == NODE_STATUS::FT_NAVI || current_nodestatus == NODE_STATUS::AUTO_MAPPING || 
      (current_nodestatus == NODE_STATUS::MANUAL_MAPPING && current_state == ROBOT_STATE::RETURN_CHARGER)){
      cancelPreviousGoal();
      if(!stopdriving_rpmstop_timer_){
        stop_driving_done = false;
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        stop_check_start_time = steady_clock.now().seconds();
        stopdriving_rpmstop_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateUtils::monitorRPMStopTimer, this));
      }else{
        RCLCPP_INFO(node_->get_logger(), "stopdriving_rpmstop_timer_ is already ");
      }
    } else{
      //saveLastPosition();
      publishVelocityCommand(0.0,0.0);
    }
  }
}

bool StateUtils::getStopDrivingFlag(){
  return stop_driving_done;
}


void StateUtils::setFactoryMode(const bool &data)
{
  if(bFactoryMode != data){
    bFactoryMode = data;
  }
}

bool StateUtils::getFactoryMode()
{
  return bFactoryMode;
}

void StateUtils::setReservePause(const bool &data){
  if(data){
    RCLCPP_INFO(node_->get_logger(), "[StateUtils] set Reserve Pause True : prev[%d],new[%d]", reserve_working_pause,data);
  } else {
    RCLCPP_INFO(node_->get_logger(), "[StateUtils] set Reserve Pause False : prev[%d],new[%d]", reserve_working_pause,data);
  }
  reserve_working_pause = data;
}

bool StateUtils::getReservePause(){
  return reserve_working_pause;
}

void StateUtils::setReserveDocking(const bool &data){
  if(data){
    RCLCPP_INFO(node_->get_logger(), "[StateUtils] setReserveDocking [True] : prev[%d],new[%d]", reserve_docking,data);
  } else {
    RCLCPP_INFO(node_->get_logger(), "[StateUtils] setReserveDocking [False] : prev[%d],new[%d]", reserve_docking,data);
  }
  reserve_docking = data;
}

bool StateUtils::getReserveDocking(){
  return reserve_docking;
}

void StateUtils::publishBatterySleep()
{
    std_msgs::msg::Empty battery_sleep_cmd;
    batterySleep_cmd_pub_->publish(battery_sleep_cmd);
    RCLCPP_INFO(node_->get_logger(), "[StateUtils] publishBatterySleep");
}

void StateUtils::publishLocalizationFailErrorStatus( const bool &data ) {
  std_msgs::msg::Bool msg;
  msg.data = data;
  localization_error_pub_->publish(msg);
  RCLCPP_WARN(node_->get_logger(), "[StateUtils] [%s] publish Localization Error [%d]", data ? "OCCURED ERROR" : "RELEASED ERROR", data);
}

uint8_t StateUtils::getStationReceiverStatus()
{
  return station_signal_receiver;
}
uint8_t StateUtils::getStationShortSignal()
{
  return station_short_signal;
}
uint8_t StateUtils::getStationLongSignal()
{
  return station_long_signal;
}

void StateUtils::setCheckTryDocking(bool set){
  RCLCPP_WARN(node_->get_logger(), "[setCheckTryDocking] before[%d] -->  set[%d]",bCheckTryDocking,set);
  bCheckTryDocking = set;
}
bool StateUtils::getCheckTryDocking()
{
  return bCheckTryDocking;
}
void StateUtils::setRotatePauseFlag(const bool &data){
  // if( movingstate_id == NAVI_STATE::START_ROTAION){
    RCLCPP_WARN(node_->get_logger(), "[setRotatePauseFlag] before[%d] -->  set[%d]",rotate_pause_flag,data);
  // }
  rotate_pause_flag = data;
}
bool StateUtils::getRotatePauseFlag(){
  return rotate_pause_flag;
}

void StateUtils::setMovingPauseFlag(bool set)
{
  RCLCPP_INFO(node_->get_logger(), "[setMovingPauseFlag] before[%d] -->  set[%d]",moving_pause_flag,set);
  moving_pause_flag = set;
}
bool StateUtils::getMovingPauseFlag()
{
  return moving_pause_flag;
}

void StateUtils::pauseNavigation() {
  moving_pause_flag = true;
  pose currrent_pose = getRobotPose();
  pose move_goal = getMoveGoalPose();
  RCLCPP_INFO(node_->get_logger(), "[StateUtils] Pause move to target robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG))",
   currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y, RAD2DEG(move_goal.theta));
  // Check if the goal handle exists and is in an active state
  if (future_goal_handle_) {
    auto status = future_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
      client_->async_cancel_goal(future_goal_handle_);      
      RCLCPP_INFO(node_->get_logger(), "[StateUtils] Navigation paused");
    } else {
      RCLCPP_WARN(node_->get_logger(),"[StateUtils] Cannot pause, goal is not active. Status: %d", status);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),"[StateUtils] Cannot pause, no valid goal handle");
  }
  setMovingStateID(NAVI_STATE::PAUSE);
}

bool StateUtils::isRetryMoveGoal(NAVI_FAIL_REASON fail_reason)
{
  bool ret = false;
  if(try_move_target_count++ > getMoveGoalRetryLimit()){
    RCLCPP_INFO(node_->get_logger(), "[checkRetryMoveGoal] retry over Fail reason[%s], count[%u]",enumToString(fail_reason).c_str(), try_move_target_count);
  }else{
    ret = true;
    RCLCPP_INFO(node_->get_logger(), "[checkRetryMoveGoal] retry move goal reason[%s], count[%u]",enumToString(fail_reason).c_str(), try_move_target_count);
  }
  return ret;
}

void StateUtils::goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
  pose currrent_pose = getRobotPose();
  pose move_goal = getMoveGoalPose();
  double movingTime = getSteadyClockRunningSeconds(send_goal_start_time);
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "[goalResultCallback] Goal reached successfully. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) moving time[%.2f]",
      currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y,RAD2DEG(move_goal.theta), movingTime);
      if(getPathPlanDestination()){
        setMovingStateID(NAVI_STATE::ALTERNATE_GOAL);
        if(state_id != ROBOT_STATE::RETURN_CHARGER){
          //hjkim 250801 : 복귀 중 대체 목적지 도착 시 returnChargerMonitor에서 원점 도착판단 후 복귀 재시도를 해야해서  센서 OFF 및 STOPDRIVING 처리 하지 않음.
          resetTryMoveTargetCount();
          enableArrivedGoalSensorsOffTimer();
          stopDriving();
        }
      }else{
        setMovingStateID(NAVI_STATE::ARRIVED_GOAL);
        resetTryMoveTargetCount();
        if(state_id != ROBOT_STATE::RETURN_CHARGER){
           //hjkim 250822 : 복귀 도착 시에는 센서를 끄지 안도록 수정
          enableArrivedGoalSensorsOffTimer();
        }
        stopDriving();
      }
      
      break;

    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(node_->get_logger(), "[goalResultCallback] Goal aborted. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) moving time[%.2f]",
        currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y, RAD2DEG(move_goal.theta), movingTime);
        if(isRetryMoveGoal(NAVI_FAIL_REASON::GOAL_ABORT)){
          publishClearCostMap();
          setMovingStateID(NAVI_STATE::READY);
          setReadyMoving(READY_MOVING::SEND_GOAL);
        }else{
          setMovingStateID(NAVI_STATE::FAIL, NAVI_FAIL_REASON::GOAL_ABORT);
          resetTryMoveTargetCount();
          stopDriving();
        }
      break;

    case rclcpp_action::ResultCode::CANCELED:
      if (getStatusID() == ROBOT_STATUS::PAUSE) {
        RCLCPP_INFO(node_->get_logger(), "[goalResultCallback] Goal was canceled due to PAUSE. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) moving time[%.2f]",
        currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y, RAD2DEG(move_goal.theta), movingTime);
        setMovingStateID(NAVI_STATE::PAUSE);
      } else {
        RCLCPP_INFO(node_->get_logger(), "[goalResultCallback] Goal cancel complete. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) moving time[%.2f]",
        currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y, RAD2DEG(move_goal.theta), movingTime);
        bCompleteCanceledMoveGoal = true;
        bStartCancelMoveGoal = false;
      }
      stopDriving();
      break;

    default:
        RCLCPP_WARN(node_->get_logger(), "[goalResultCallback] Unknown error on goal result. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) moving time[%.2f]",
        currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y, RAD2DEG(move_goal.theta), movingTime);
        if(isRetryMoveGoal(NAVI_FAIL_REASON::UNKNOWN)){
          publishClearCostMap();
          setMovingStateID(NAVI_STATE::READY);
          setReadyMoving(READY_MOVING::SEND_GOAL);
        }else{
          setMovingStateID(NAVI_STATE::FAIL, NAVI_FAIL_REASON::UNKNOWN);
          resetTryMoveTargetCount();
          stopDriving();
        }
      break;
  }
  future_goal_handle_.reset();
}

void StateUtils::goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
  pose currrent_pose = getRobotPose();
  pose move_goal = getMoveGoalPose();
  double elapsedResponseTime = getSteadyClockRunningSeconds(send_goal_start_time);
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[goalResponseCallback] Goal rejected. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) response time[%.2f] ",
    currrent_pose.x, currrent_pose.y,RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y,RAD2DEG(move_goal.theta), elapsedResponseTime);
    if(isRetryMoveGoal(NAVI_FAIL_REASON::GOAL_REJECT)){
      publishClearCostMap();
      setMovingStateID(NAVI_STATE::READY);
      setReadyMoving(READY_MOVING::SEND_GOAL);
    }else{
      setMovingStateID(NAVI_STATE::FAIL, NAVI_FAIL_REASON::GOAL_REJECT);
      resetTryMoveTargetCount();
      stopDriving();
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "[goalResponseCallback] Goal accepted. robot(%.2f, %.2f, %.2f(DEG)), goal(%.2f, %.2f, %.2f(DEG)) response time[%.2f]",
    currrent_pose.x, currrent_pose.y, RAD2DEG(currrent_pose.theta), move_goal.x, move_goal.y, RAD2DEG(move_goal.theta), elapsedResponseTime);
    future_goal_handle_ = goal_handle;
    setMovingStateID(NAVI_STATE::MOVE_GOAL);
    publishLifeCycleOff();
  }
}


void StateUtils::moveToTarget(double x, double y, double theta) {
  
  pose current_pose = getRobotPose();

  if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),"[moveToTarget] Action server not available after waiting current(%.2f, %.2f, %.2f(DEG)) goal(%.2f, %.2f, %.2f(DEG))",
    current_pose.x, current_pose.y, RAD2DEG(current_pose.theta),x,y,RAD2DEG(theta));
    if(isRetryMoveGoal(NAVI_FAIL_REASON::SERVER_NO_ACTION)){
      publishClearCostMap();
      setMovingStateID(NAVI_STATE::READY);
      setReadyMoving(READY_MOVING::SEND_GOAL);
    }else{
      setMovingStateID(NAVI_STATE::FAIL, NAVI_FAIL_REASON::SERVER_NO_ACTION);
      resetTryMoveTargetCount();
      stopDriving();
    }
    return;
  }
  
  goal_msg = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();
  goal_msg->pose.pose.position.x = x;
  goal_msg->pose.pose.position.y = y;
  goal_msg->pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  goal_msg->pose.pose.orientation = tf2::toMsg(q);
  goal_msg->pose.header.frame_id = "map";
  goal_msg->pose.header.stamp = node_->now();
  client_->async_send_goal(*goal_msg, navi_goal_options);
  setMoveGoalPose(x,y,theta);
  send_goal_start_time = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "[moveToTarget] robot (%.2f, %.2f, %.2f(DEG)) send goal(%.2f, %.2f, %.2f(DEG))",
  current_pose.x, current_pose.y,RAD2DEG(current_pose.theta),x,y,RAD2DEG(theta));
}

void StateUtils::cancelPreviousGoal(){
  if (future_goal_handle_) {
    pose goal_pose = getMoveGoalPose();
    bStartCancelMoveGoal = true;
    bCompleteCanceledMoveGoal = false;

    auto cancel_callback = [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
      if (response) {
        using CancelResp = action_msgs::srv::CancelGoal_Response;

        switch (response->return_code) {
          case CancelResp::ERROR_NONE:
            RCLCPP_INFO(node_->get_logger(),
              "[cancelPreviousGoal] ✅ Previous goal successfully canceled due to new target.");
            break;

          case CancelResp::ERROR_REJECTED:
            RCLCPP_WARN(node_->get_logger(),
              "[cancelPreviousGoal] Cancel request was rejected by the action server.");
              bCompleteCanceledMoveGoal = true;
            break;

          case CancelResp::ERROR_UNKNOWN_GOAL_ID:
            RCLCPP_WARN(node_->get_logger(),
              "[cancelPreviousGoal] Cancel failed: Unknown goal ID. The goal might not exist or already expired.");
              bCompleteCanceledMoveGoal = true;
            break;

          case CancelResp::ERROR_GOAL_TERMINATED:
            RCLCPP_WARN(node_->get_logger(),
              "[cancelPreviousGoal] Cancel failed: Goal already terminated (completed, canceled, or aborted).");
              bCompleteCanceledMoveGoal = true;
            break;

          default:
            RCLCPP_WARN(node_->get_logger(),
              "[cancelPreviousGoal] Cancel failed: Unknown return code [%d].", response->return_code);
              bCompleteCanceledMoveGoal = true; // it would be better to retry after delay 
            break;
        }
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "[cancelPreviousGoal] Cancel goal response is null (no response received).");
      }

      future_goal_handle_.reset(); // Release the handle to prevent reuse
    };

    client_->async_cancel_goal(future_goal_handle_, std::bind(cancel_callback, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),"[cancelPreviousGoal] cancel Previous goal x: %.2f, y: %.2f, theta: %.2f(DEG)", goal_pose.x, goal_pose.y, RAD2DEG(goal_pose.theta));
  } else {
    RCLCPP_INFO(node_->get_logger(),"[cancelPreviousGoal] There is no goal to cancel.");
  }
}
bool StateUtils::isStartCancelMoveGoal()
{
  return bStartCancelMoveGoal;
}
bool StateUtils::isCancelCompleteMoveGoal()
{
  return bCompleteCanceledMoveGoal;
}

double StateUtils::getOptionalSteadyClockRunningSeconds(const std::optional<std::chrono::steady_clock::time_point>& start_time_opt) {
    if (!start_time_opt.has_value()){
        return 0.0;
    }
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_opt.value());
    return duration.count();
}

double StateUtils::getSteadyClockRunningSeconds(const std::chrono::time_point<std::chrono::steady_clock> &start_time)
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
}

void StateUtils::enableMonitorSensorRecoveryTimer()
{
  if(!monitor_sensor_recovery_timer_){
    monitor_sensor_recovery_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StateUtils::monitorSensorRecovery, this));
    RCLCPP_INFO(node_->get_logger(), "enable Monitor SensorRecovery");
  }else{
    RCLCPP_INFO(node_->get_logger(), "enableMonitorSensorRecovery is already exist");
  }
}

void StateUtils::disableMonitorSensorRecoveryTimer()
{
  if (monitor_sensor_recovery_timer_) {  // 등록되어 있다면 해제
    monitor_sensor_recovery_timer_->cancel();
    monitor_sensor_recovery_timer_.reset();
    RCLCPP_INFO(node_->get_logger(), "SensorRecoveryTimer disable");
  }else{
    RCLCPP_INFO(node_->get_logger(), "SensorRecoveryTimer is already disabled");
  }
}

void StateUtils::monitorSensorRecovery() 
{
  // recovery 데이터 확인하여 flag를 비트로 on/off(추가사항 대비)
  // recovery중인 경우 pause를 위한 flag 세팅
  enum RecoveryFlag{
    NONE = 0,
    ToF_BOTTOM_LEFT = 1 << 0, // multi tof left flag
    ToF_BOTTOM_RIGHT  = 1 << 1 // multi tof right flag
  };

  if(bottom_left & 0x08){
    recovery_flag |= RecoveryFlag::ToF_BOTTOM_LEFT;
  }else{
    recovery_flag &= ~RecoveryFlag::ToF_BOTTOM_LEFT;
  }

  if(bottom_right & 0x08){
    recovery_flag |= RecoveryFlag::ToF_BOTTOM_RIGHT;
  }else{
    recovery_flag &= ~RecoveryFlag::ToF_BOTTOM_RIGHT;
  }


  if(recovery_flag !=0){
    setSensorRecoveryPause(true);
  } else{
    setSensorRecoveryPause(false);
  }
}

void StateUtils::setSensorRecoveryPause(const bool &set)
{
  if(sensor_warning_pause != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "SensorRecovery Detected[%d] / recovery flag[%d]--> Sensor Recovery Pause", set, recovery_flag);
    } else{
      RCLCPP_INFO(node_->get_logger(), "SensorRecovery Finished[%d] --> Can Resume", sensor_warning_pause);
    }
  }
  sensor_warning_pause = set;
}

bool StateUtils::getSensorRecoveryPause()
{
  return sensor_warning_pause;
}

void StateUtils::setNavibringUpRecoveryPause(const bool &set)
{
  if(navibringup_recovery_pause != set){
    if(set){
      RCLCPP_INFO(node_->get_logger(), "NaviBrinup recovery Detected[%d] / recovery flag[%d]--> Pause", static_cast<int>(set),static_cast<int>(recovery_flag));
    } else{
      RCLCPP_INFO(node_->get_logger(), "NaviBrinup recovery Finished[%d] --> Resume", static_cast<int>(navibringup_recovery_pause));
    }
  }
  navibringup_recovery_pause = set;
}
bool StateUtils::getNavibringUpRecoveryPause()
{
  return navibringup_recovery_pause;
}

uint8_t StateUtils::getLowerBits(uint8_t byte) {
  return (byte & 0x0F);
}

uint8_t StateUtils::getUpperBits(uint8_t byte) {
  return ((byte >> 4) & 0x0F);
}


bool StateUtils::isInpectionMode()
{
  return bInspectionMode;
}

void StateUtils::setMappingResumeToErrorFlag(const bool &set){
  mapping_resume_to_error_flag = set;
}
bool StateUtils::getMappingResumeToErrorFlag(){
  return mapping_resume_to_error_flag;
}

void StateUtils::setAutoMappingCompleteState(const bool &set){
  bAutoMappingComplete = set;
}
bool StateUtils::isAutoMappingComplete(){
  return bAutoMappingComplete;
}

void StateUtils::resetTryMoveTargetCount()
{
  try_move_target_count = 0;
   RCLCPP_INFO(node_->get_logger(), "resetTryMoveTargetCount");
}

void StateUtils::setCheckAmclAfterLocalization()
{
  bCheckAmclAfterLocalization = true;
  RCLCPP_INFO(node_->get_logger(), "[setCheckAmclAfterLocalization] Start CheckAmclPose");
}

void StateUtils::setMapServerNodeActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(),"[StateUtils::setMapServerNodeActive] current state: %d -> new state: %d",map_server_activated, msg->data);
  map_server_activated = msg->data;
}

void StateUtils::setNaviNodeActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(),"[StateUtils::setNaviNodeActive] current state: %d -> new state: %d",navigator_activated, msg->data);
  navigator_activated = msg->data;
}

bool StateUtils::getMapServerNodeActive(){
  return map_server_activated;
}

bool StateUtils::getNaviNodeActive(){
  return navigator_activated;
}

bool StateUtils::setMapChange(const int &set){
  std::string map_path;
  std::string map_folder_path;
  switch (set)
  {
  case 0:
    map_folder_path = "/home/airbot/app_rw/map/";
    map_path = map_folder_path + "airbot_map_00.yaml";
    set_map_change_done = false;
    break;
  case 1:
    map_folder_path = "/home/airbot/app_rw/factorymap/";
    map_path = map_folder_path + "airbot_factorymap_00.yaml";
    set_map_change_done = false;
    break;
  }

  if(!checkMapFileExist(map_folder_path)){ //map파일 유무 체크.
    RCLCPP_ERROR(node_->get_logger(), "[StateUtils::setMapChange] MapFile does not Exist.. Aborting.");
    return false;
  }

  if(!map_server_client_->wait_for_service(std::chrono::seconds(10))){
    RCLCPP_ERROR(node_->get_logger(), "[StateUtils::setMapChange] services are not available. Aborting.");
    return false;
  }

  auto map_loader_cb = [this](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
      auto result = future.get();
      if (result->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
          RCLCPP_INFO(this->node_->get_logger(), "[StateUtils:setMapChange] Successfully loaded map.");
          set_map_change_done = true;
      } else {
          RCLCPP_ERROR(this->node_->get_logger(), "[StateUtils:setMapChange] Failed to load map. Result code: %d", result->result);
      }
  };

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_path;
  RCLCPP_INFO(this->node_->get_logger(), "[StateUtils] Request to load map from map_path'%s'", map_path.c_str());
  this->map_server_client_->async_send_request(request, map_loader_cb);
  
  return true;
}

bool StateUtils::setMapParameters(const int &set){
  double set_tolerance = 0.0;
  double set_radius = 0.0;
  double set_inflation_radius = 0.0;
  
  switch (set)
  {
  case 0:
    set_tolerance = 0.25;
    set_radius = 0.19;
    set_map_parameters_done = false;
    set_inflation_radius = 0.45;
    break;
  case 1:
    set_tolerance = 0.15;
    set_radius = 0.1;
    set_map_parameters_done = false;
    set_inflation_radius = 0.3;
    break;
  }

  RCLCPP_INFO(node_->get_logger(), "[setMapParameters] Attempting to set parameters for set_id=%s (tolerance=%.2f, radius=%.2f, inflation = %.2f) ",
    set > 0 ? "Factory": "Normal", set_tolerance, set_radius, set_inflation_radius);

  // 2. 모든 서비스가 사용 가능한지 확인
  if (!control_param_client_->wait_for_service(std::chrono::seconds(10)) ||
      !global_costmap_param_client_->wait_for_service(std::chrono::seconds(10)) ||
      !local_costmap_param_client_->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node_->get_logger(), "[setMapParameters] One or more required services are not available. Aborting.");
    return false;
  }

   auto global_costmap_cb = [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        if (future.get().at(0).successful) {
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task4: Successfully set global_costmap parameter.");
            // 세팅작업 완료
            set_map_parameters_done = true;
        } else {
            RCLCPP_ERROR(this->node_->get_logger(), "[setMapParameters] Task4: Failed to set global_costmap parameter: %s", future.get().at(0).reason.c_str());
        }
    };

    // 3. Local Costmap inflation 파라미터 설정 콜백
    auto local_costmap_inflation_cb = [this, set_radius, global_costmap_cb](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        if (future.get().at(0).successful) {
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task3: Successfully set local_costmap parameter.");
            // 다음 작업: Global Costmap 설정
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task4: Setting global_costmap radius to %f", set_radius);
            this->global_costmap_param_client_->set_parameters({rclcpp::Parameter("robot_radius", set_radius)}, global_costmap_cb);
        } else {
            RCLCPP_ERROR(this->node_->get_logger(), "[setMapParameters] Task3: Failed to set local_costmap parameter: %s", future.get().at(0).reason.c_str());
        }
    };

    // 2. Local Costmap 파라미터 설정 콜백
    auto local_costmap_cb = [this, set_inflation_radius, local_costmap_inflation_cb](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        if (future.get().at(0).successful) {
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task2: Successfully set local_costmap parameter.");
            // 다음 작업: Local Costmap inflation 파라미터 설정
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task3: Setting local_costmap inflation_radius to %f", set_inflation_radius);
            this->local_costmap_param_client_->set_parameters({rclcpp::Parameter("inflation_layer.inflation_radius", set_inflation_radius)}, local_costmap_inflation_cb);
        } else {
            RCLCPP_ERROR(this->node_->get_logger(), "[setMapParameters] Task2: Failed to set local_costmap parameter: %s", future.get().at(0).reason.c_str());
        }
    };

    // 1. 첫 번째 작업: Controller Server 파라미터 설정 콜백
    auto controller_cb = [this, set_radius, local_costmap_cb](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        if (future.get().at(0).successful) {
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task1: Successfully set controller_server parameter.");
            //  Controller Server시 -> 다음 작업: Local Costmap 설정
            RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task2: Setting local_costmap radius to %f", set_radius);
            this->local_costmap_param_client_->set_parameters({rclcpp::Parameter("robot_radius", set_radius)}, local_costmap_cb);
        } else {
            RCLCPP_ERROR(this->node_->get_logger(), "[setMapParameters] Chain-1: Failed to set controller_server parameter: %s", future.get().at(0).reason.c_str());
        }
    };

    // Controller Server부터 set 시작.
    RCLCPP_INFO(this->node_->get_logger(), "[setMapParameters] Task1: Setting controller_server tolerance to %f", set_tolerance);
    control_param_client_->set_parameters({rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", set_tolerance)}, controller_cb);

  return true;
}

bool StateUtils::getSetMapParametersDone(){
  return set_map_parameters_done;
}

bool StateUtils::getSetMapChangeDone(){
  return set_map_change_done;
}

bool StateUtils::checkMapFileExist(const std::string &path){
  std::vector<std::string> check_filelist;
  if (!std::filesystem::is_directory(path)) {
        RCLCPP_INFO(node_->get_logger(), "Error: Directory %s does not exist", path.c_str());
        return false;
  }

  if(path.find("factorymap") != std::string::npos){
    check_filelist = {"airbot_factorymap_00.pgm", "airbot_factorymap_00.yaml"};
  } else{
    check_filelist = {"airbot_map_00.pgm", "airbot_map_00.yaml"};
  }

  for (const auto& filename : check_filelist) {
    std::filesystem::path filePath = std::filesystem::path(path) / filename;

    if (!std::filesystem::is_regular_file(filePath)) {
        RCLCPP_WARN(node_->get_logger(),"Required map file not exist...: %s", filePath.c_str());
        return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Successfully check all map files in '%s'.", path.c_str());

  return true;
}

void StateUtils::reserveMapLoadatferNavRecovery(){
  retry_mapload = 0;
  bStartMapLoad = false;
  request_loadmap_start_time_ = {};
  bNeedToMapLoadAfterRecovery = true;
  RCLCPP_INFO(node_->get_logger(), "[reserveMapLoadatferNavRecovery] set need to map load after nav recovery");
}
bool StateUtils::isNeedToRunMapLoadAfterRecovery(){
  return bNeedToMapLoadAfterRecovery;
}

//map type 0 : airbot_map(default), 1 : factorymap
void StateUtils::mapLoadAfterNavRecoveryProcess(int map_type){

  if(request_loadmap_start_time_.time_since_epoch().count() == 0){
    request_loadmap_start_time_ = std::chrono::steady_clock::now();
  }

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_sec = now-request_loadmap_start_time_;
  double runtime = elapsed_sec.count();
  if(runtime > 20)
  {
    if(bStartMapLoad){
      if(retry_mapload == 0){
        bStartMapLoad = false;
        retry_mapload++;
        request_loadmap_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[mapLoadAfterNavRecoveryProcess] load map 20sec timeout retry");
      }else{
        RCLCPP_INFO(node_->get_logger(), "[mapLoadAfterNavRecoveryProcess] load map 20sec timeout retry 2 times fail");
        bNeedToMapLoadAfterRecovery = false;
        return;
      }
    }else{
      RCLCPP_INFO(node_->get_logger(), "[mapLoadAfterNavRecoveryProcess] map_server not active -> load map 20sec timeout fail");
      bNeedToMapLoadAfterRecovery = false;
      return;
    }
  }

  if(!bStartMapLoad && getMapServerNodeActive()){
    setMapChange(map_type); //hjkim250808 : request map load to map server < 0 : default map / 1 : factory map >
    bStartMapLoad = true;
    RCLCPP_INFO(node_->get_logger(), "[mapLoadAfterNavRecoveryProcess] MapServer active --> request map load to map server");
  }else if(bStartMapLoad && getSetMapChangeDone()){
    bNeedToMapLoadAfterRecovery = false;
    RCLCPP_INFO(node_->get_logger(), "[mapLoadAfterNavRecoveryProcess] load map Complete.");
  }

  return;
}

void StateUtils::setMapCopyReceived()
{
  bReceivedMapCopy = true;
  RCLCPP_INFO(node_->get_logger(), "[setMapCopyReceived] map is changed");
}

bool StateUtils::isMapCopyReceived(){
  return bReceivedMapCopy;
}

void StateUtils::resetMapCopyReceived(){
  bReceivedMapCopy = false;
  RCLCPP_INFO(node_->get_logger(), "[resetMapCopyReceived] changed map clear --> changed map load complete");
}

void StateUtils::map_saver() {
  ROBOT_STATE robot_state = getStateID();
  NODE_STATUS node_status = getNodeStatusID();
  RCLCPP_INFO(node_->get_logger(), "[map_saver] state[%s] node[%s] start map_saver",enumToString(robot_state).c_str(),enumToString(node_status).c_str());

  int map_save_result = std::system("ros2 run nav2_map_server map_saver_cli "
                          "-f /home/airbot/app_rw/map/airbot_map_00 "
                          "--free 0.196 "
                          "--ros-args -p save_map_timeout:=10.0");
  
  RCLCPP_INFO(node_->get_logger(), "[map_saver] map_saver result : %d",map_save_result);

  int map_save_cnt = 0;

  while (map_save_result != 0) {
    if (map_save_cnt++ < 10) {
      RCLCPP_ERROR(node_->get_logger(),"[map_saver] Map save command failed with error code, trying again: %d", map_save_cnt);

      map_save_result = std::system(
          "ros2 run nav2_map_server map_saver_cli "
                          "-f /home/airbot/app_rw/map/airbot_map_00 "
                          "--free 0.196 "
                          "--ros-args -p save_map_timeout:=10.0");

      RCLCPP_ERROR(node_->get_logger(), "[map_saver] retry Map save result : %d", map_save_result);

    } else {
      RCLCPP_ERROR(node_->get_logger(), "[map_saver] Map save command failed over 10times");
      break;
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "[map_saver] end map_saver");
  
  
  if(map_save_result == 0){
    resetInitPose(); //hjkim : 맵핑 시작 시 충전기위치와 초기위치(복귀좌표) 초기화 -> 매핑 완료 후 맵 저장 시 초기화 하도록 수정
    publishClearVirtualWall(); //hjkim : 맵핑 시작 시 가상벽 초기화 --> 매핑 완료 후 맵 저장 시 초기화 하도록 수정
    
    if(isMapCopyReceived()){
      resetMapCopyReceived();
      RCLCPP_INFO(node_->get_logger(), "[map_saver] copy map reset cause new map saved");
    }
  }
}

bool StateUtils::isRunningDocking(){
  return bDockingRunning;
}

void StateUtils::setRecoveryRebootflag(bool set){
  RCLCPP_INFO(node_->get_logger(), "[setRecoveryRebootflag] before[%d] -->  set[%d]",bRecoveryReboot,set);
  bRecoveryReboot = set;
}

bool StateUtils::getRecoveryRebootflag(){
  return bRecoveryReboot;
}

bool StateUtils::removeRecoveryDirectory() {
    const std::filesystem::path dir{"/home/airbot/app_rw/recovery"};

    try {
        if (std::filesystem::exists(dir)) {
            std::uintmax_t count = std::filesystem::remove_all(dir);
            RCLCPP_INFO(node_->get_logger(),
                        "[removeRecoveryDirectory] Deleted %zu" " item(s) at: %s",
                        static_cast<size_t>(count), dir.c_str());
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "[removeRecoveryDirectory]Path does not exist: %s", dir.c_str());
            return false;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[removeRecoveryDirectory] Failed to delete %s: %s", dir.c_str(), e.what());
        return false;
    }
}

bool StateUtils::checkRecoveryReboot(){
    bool isRecoveryDirExist = isFileExists("/home/airbot/app_rw/recovery");
    bool isRecoveryJsonExist = isFileExists("/home/airbot/app_rw/recovery/recovery.json");

    RCLCPP_INFO(node_->get_logger(), "[checkRecoveryReboot] is directory[%s] json[%s]",isRecoveryDirExist ? "exit" : "none",isRecoveryJsonExist ? "exit" : "none");
    if(isRecoveryDirExist && isRecoveryJsonExist){
        RCLCPP_INFO(node_->get_logger(), "[checkRecoveryReboot] recovery reboot complete --> send to Soc recovery Complete");
    }else if(!isRecoveryDirExist){
        RCLCPP_INFO(node_->get_logger(), "[checkRecoveryReboot] there is no recovery directory");
    }else{
        RCLCPP_INFO(node_->get_logger(), "[checkRecoveryReboot] recovery reboot complete but can`t returnCharger");
    }
    return isRecoveryDirExist && isRecoveryJsonExist;
}

bool StateUtils::isRecoveryInitPoseExist(){
  return bSetRecoveryInitPose;
}


void StateUtils::readyRobotCallback(const std_msgs::msg::Empty::SharedPtr/* msg*/)
{
  ROBOT_STATE current_state = getStateID();
  NODE_STATUS current_node_status = getNodeStatusID();
  RCLCPP_INFO(node_->get_logger(), "[readyRobotCallback] Received readyReboot robotState: %s, nodeStatus: %s", enumToString(current_state).c_str(), enumToString(current_node_status).c_str());

  if(bRecoveryReboot){
    //hjkim : 재부팅을 하고, 현재 좌표를 사용하지 않았는데 다시 reboot 명령이 내려오면 수행하지 않음 ( 재부팅하고나서 현재 위치를 모르고 파일로만 가지고 있기때문에 다시 덮어 쓰면 엉뚱한 좌표가 저장됨)
    RCLCPP_INFO(node_->get_logger(), "already recovery jason exists don't save current pose again");
    publishRebootReadyComplete();
    return;
  }

  if(bSavingCurrentPose){
    //hjkim : 이미 재부팅 준비를 하고 있는데 요청이 또오면 실행하지 않음.
    double running_time = getSteadyClockRunningSeconds(save_current_pose_start_time);
    RCLCPP_INFO(node_->get_logger(), "already running reboot ready to save current pose %.2fsec ago",running_time);
    publishRebootReadyComplete();
    return;
  }

  bSavingCurrentPose = true;
  if((bLocalizationComplete && current_state == ROBOT_STATE::NAVIGATION)){
      startSaveCurrentPoseTimer();
  }else{
    publishRebootReadyComplete(); //hjkim : 네비게이션 상태가 아니면, 저장할 필요가 없으니까 바로 reboot 하라고 보냄.
  }
}

void StateUtils::startSaveCurrentPoseTimer()
{
  const std::filesystem::path path = "/home/airbot/app_rw/recovery/";
  std::filesystem::create_directories(path);
  publishRobotCommand(REQUEST_ROBOT_CMD::READY_REBOOT);
  if(save_current_pose_timer_){
    RCLCPP_INFO(node_->get_logger(), "save_current_pose_timer_ is already running");
  }else{
    save_current_pose_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateUtils::runSaveCurrentPoseTimer, this));
  }
  
  save_current_pose_start_time = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "startSaveCurrentPoseTimer");
}

void StateUtils::stopSaveCurrentPoseTimer()
{
  if(save_current_pose_timer_){
    save_current_pose_timer_.reset();
  }
  //bSavingCurrentPose = false; //hjkim : 재부팅 까지 3초 대기 시간동안 다시오면 파일 저장을 다시 하다가 꺼져버리면 파일이 깨질수 있으니, 한번셋팅하면, 꺼질때까지 초기화하지 않음. (재부팅되면 false 되니까)
  RCLCPP_INFO(node_->get_logger(), "stopSaveCurrentPoseTimer");
}

void StateUtils::runSaveCurrentPoseTimer()
{
  ROBOT_STATE current_state = getStateID();
  if(current_state == ROBOT_STATE::IDLE && stop_driving_done){
    bool ret = saveCurrentPoseJsonForRecvocery();
    RCLCPP_INFO(node_->get_logger(), "saveCurrentPoseJsonForRecvocery ret: %s", ret ? "success" : "fail");
    stopSaveCurrentPoseTimer();
    publishRebootReadyComplete();
  }else{
    RCLCPP_INFO(node_->get_logger(), "waiting stop driving");
  }
}

bool StateUtils::saveCurrentPoseJsonForRecvocery() {
  try {
    std::filesystem::path path = "/home/airbot/app_rw/recovery/recovery.json";
    auto dir = std::filesystem::path(path).parent_path();
    std::filesystem::create_directories(dir);
    RCLCPP_INFO(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] Directory checked/created: %s", dir.c_str());

    // // Skip if file already exists
    // if (std::filesystem::exists(path)) {
    //   RCLCPP_INFO(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] File already exists, skipping creation: %s", path.c_str());
    //   return true;
    // }

    nlohmann::json recovery_json;
    recovery_json["robot_pose"]["x"] = robot_pose.x;
    recovery_json["robot_pose"]["y"] = robot_pose.y;
    recovery_json["robot_pose"]["theta"] = robot_pose.theta;

    std::ofstream file(path);
    if (!file.is_open()) {
      RCLCPP_INFO(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] Failed to open file: %s", path.c_str());
      return false;
    }

    file << recovery_json.dump(4);
    file.close();
    RCLCPP_INFO(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] recovery json file created: %s robot_pose(%.2f,%.2f,%.2f(DEG))",
    path.c_str(),robot_pose.x,robot_pose.y,RAD2DEG(robot_pose.theta));

    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
      RCLCPP_ERROR(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] open(%s) failed: %s",
                   path.c_str(), std::strerror(errno));
      return false;
    }
    if (::fsync(fd) != 0) {
      RCLCPP_ERROR(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] fsync(%s) failed: %s",
                   path.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }
    ::close(fd);

    int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
    if (dfd < 0) {
      RCLCPP_ERROR(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] open(dir:%s) failed: %s",
                   dir.c_str(), std::strerror(errno));
      return false;
    }
    if (::fsync(dfd) != 0) {
      RCLCPP_ERROR(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] fsync(dir:%s) failed: %s",
                   dir.c_str(), std::strerror(errno));
      ::close(dfd);
      return false;
    }
    ::close(dfd);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_INFO(node_->get_logger(), "[saveCurrentPoseJsonForRecvocery] std::exception while creating file: %s", e.what());
    return false;
  }
}

bool StateUtils::loadRecoveryJasonFile() {
  std::filesystem::path path = "/home/airbot/app_rw/recovery/recovery.json";
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_INFO(node_->get_logger(), "[loadRecoveryJasonFiles] can`t open file: %s", path.c_str());
    return false;
  }

  try {
      nlohmann::json load_json;
      file >> load_json;

      if(load_json.contains("robot_pose")) {
        recovery_init_pose.x = load_json["robot_pose"]["x"].get<double>();
        recovery_init_pose.y = load_json["robot_pose"]["y"].get<double>();
        recovery_init_pose.theta = load_json["robot_pose"]["theta"].get<double>();
        // Time(0)로 설정 → tf2가 최신 TF를 사용
        recovery_init_pose_msg.header.stamp.sec = 0;
        recovery_init_pose_msg.header.stamp.nanosec = 0;
        recovery_init_pose_msg.header.frame_id = "map";  // 좌표계 설정
        recovery_init_pose_msg.pose.pose.position.x = recovery_init_pose.x;
        recovery_init_pose_msg.pose.pose.position.y = recovery_init_pose.y;
        recovery_init_pose_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, recovery_init_pose.theta);
        recovery_init_pose_msg.pose.pose.orientation = tf2::toMsg(q);
        recovery_init_pose_msg.pose.covariance.fill(0.0);
        bSetRecoveryInitPose = true;
        file.close();
        RCLCPP_INFO(node_->get_logger(), "[loadRecoveryJasonFile] recovery_init_pose parsing complete: %s ,pose(%.2f,%.2f,%.2f(DEG))",
        path.c_str(),recovery_init_pose.x,recovery_init_pose.y,RAD2DEG(recovery_init_pose.theta));
        return true;
      } else {
        RCLCPP_INFO(node_->get_logger(), "[loadRecoveryJasonFile]recovery_init_pose parsing failed not found key : %s", path.c_str());
        file.close();
        return false;
      }
  } catch (const std::exception& e) {
    RCLCPP_INFO(node_->get_logger(), "[loadRecoveryJasonFile] std::exception recovery_init_pose parsing failed : %s", e.what());
    file.close();
    return false;
  }
}

LOCALIZATION_MODE StateUtils::getLocalizationMode() {
  //hjkim : default is global allowed : not receive(-1), local only(0), global allowed(1)
  return localizationMode;
}

bool StateUtils::getCmdGlobalLocalizationMode() {
  return bCmdGlobalLocalizationMode;
}

void StateUtils::setCmdGlobalLocalizationMode(bool localization_mode) {
#if USE_JSLLOC > 0
  bCmdGlobalLocalizationMode = localization_mode;
  RCLCPP_INFO(node_->get_logger(), "[setCmdGlobalLocalizationMode] globallocalization mode set[%d], current-mode[%s] ",
      bCmdGlobalLocalizationMode, enumToString(localizationMode).c_str());
#endif
}

bool StateUtils::getMapInfoChanged() {
  return bMapInfoChanged;
}

void StateUtils::reserveSavingMapFile()
{
  bMapInfoChanged = false;
  bReservedSaveMapFile = true;
  RCLCPP_INFO(node_->get_logger(), "reserveSavingMapFile");
}

bool StateUtils::isReservedSaveMapFile()
{
  return bReservedSaveMapFile;
}

void StateUtils::startSaveMapFile()
{
  copyFile("/home/airbot/app_rw/map/airbot_map_00.pgm", "/home/airbot/app_rw/log/map/airbot_map_00.pgm");
  copyFile("/home/airbot/app_rw/map/airbot_map_00.yaml", "/home/airbot/app_rw/log/map/airbot_map_00.yaml");
  copyFile("/home/airbot/app_rw/A1_keepout/area.json", "/home/airbot/app_rw/log/map/area.json");
  copyFile("/home/airbot/app_rw/A1_keepout/wall.json", "/home/airbot/app_rw/log/map/wall.json");
  copyFile("/home/airbot/app_rw/stationPose/station_pose.json", "/home/airbot/app_rw/log/map/station_pose.json");
  bReservedSaveMapFile = false;
}

bool StateUtils::copyFile(const std::string& source, const std::string& destination) {
    try {
        std::filesystem::path destPath(destination);
        std::filesystem::create_directories(destPath.parent_path());
        std::filesystem::copy_file(source, destination, std::filesystem::copy_options::overwrite_existing);

        // copy 후 fsync 추가
        int fd = ::open(destination.c_str(), O_RDWR);
        if (fd != -1) {
            if (::fsync(fd) == 0) {
                RCLCPP_INFO(node_->get_logger(), "copyFile sync success: %s", destination.c_str());
            } else {
                RCLCPP_WARN(node_->get_logger(), "copyFile sync failed (fsync error): %s", destination.c_str());
            }
            ::close(fd);
        } else {
            RCLCPP_WARN(node_->get_logger(), "copyFile sync failed (open error): %s", destination.c_str());
        }

        RCLCPP_INFO(node_->get_logger(), "source[%s]--> destination[%s] copyFile success", source.c_str(), destination.c_str());
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_INFO(node_->get_logger(), "copyFile error");
        return false;
    }
}

void StateUtils::recoveryLocalCallback(const std_msgs::msg::Int8::SharedPtr msg) {
  RCLCPP_INFO(node_->get_logger(), "recoveryLocalCallback [%d]", msg->data);
  if(msg->data == 1){
    recovery_init_pose = robot_pose;
    recovery_init_pose_msg.header.stamp.sec = 0;
    recovery_init_pose_msg.header.stamp.nanosec = 0;
    recovery_init_pose_msg.header.frame_id = "map";  // 좌표계 설정
    recovery_init_pose_msg.pose.pose.position.x = recovery_init_pose.x;
    recovery_init_pose_msg.pose.pose.position.y = recovery_init_pose.y;
    recovery_init_pose_msg.pose.pose.position.z = 0.0;
    RCLCPP_INFO(node_->get_logger(), "save recovery_init_pose(%.2f,%.2f,%.2f(DEG))",recovery_init_pose.x,recovery_init_pose.y,RAD2DEG(recovery_init_pose.theta));
  }else if(msg->data == 2){
    setCmdGlobalLocalizationMode(true);
    publishLocalizationMode(true);
    startLocalizationMonitor(LOCALIZATION_TYPE::RECOVERY_POSE);
    RCLCPP_INFO(node_->get_logger(), "recovery localization state msg is success");
  }else{
    RCLCPP_INFO(node_->get_logger(), "recovery localization state msg is wrong");
  }
}

void StateUtils::keepoutStateCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
  KEEPOUT_STATE callback_state = static_cast<KEEPOUT_STATE>(msg->data);
  RCLCPP_INFO(node_->get_logger(), "keepoutStateCallback [%d]", msg->data);
  setKeepoutState(callback_state);
}

void StateUtils::setKeepoutState(KEEPOUT_STATE state) {
  if(state != keepout_state){
    RCLCPP_INFO(node_->get_logger(), "setKeepoutState prev[%s] --> curr[%s]",enumToString(keepout_state).c_str(), enumToString(state).c_str());
  }
  keepout_state = state; 
}

KEEPOUT_STATE StateUtils::getKeepoutState() {
  return keepout_state;
}

std::chrono::time_point<std::chrono::steady_clock> StateUtils::getKeepOutEnableStartTime() {
  return keep_out_enable_start_time;
}

}