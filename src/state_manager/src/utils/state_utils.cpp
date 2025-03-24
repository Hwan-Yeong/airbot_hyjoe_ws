#include "state_manager/utils/state_utils.hpp"
#include "rclcpp/qos.hpp"

namespace airbot_state {

StateUtils::StateUtils(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
  initializeData();

  param_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&StateUtils::paramCallback, this, std::placeholders::_1));
  
  rclcpp::QoS qos_profile = rclcpp::QoS(1).transient_local();
  req_clear_costmap_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/clear/costmap", 1);
  req_estimatePose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/request/pose", 1);
  reset_odom_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);
  lidar_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
  tof_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);
  camera_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_camera", 10);
  move_fail_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/error/s_code/unreachable_goal", 10);
  alternative_dest_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/alternative_dest_error", 10);
  direct_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  maneuver_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/maneuver/use", qos_profile);
  mapping_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_explore", qos_profile);
  sensor_manager_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_sensor_manager", qos_profile);
  robot_state_pub_ = node_->create_publisher<robot_custom_msgs::msg::RobotState>("/state_datas", 10);
  node_status_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/node_status", qos_profile);
  navi_state_pub_ = node_->create_publisher<robot_custom_msgs::msg::NaviState>("/navi_datas", 10);

  amcl_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
    std::bind(&StateUtils::amclCallback, this, std::placeholders::_1));

  slam_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_pose", 10,
    std::bind(&StateUtils::slamPoseCallback, this, std::placeholders::_1));
	
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

  node_client_ = rclcpp_action::create_client<robot_custom_msgs::action::ManageNode>(node, "manage_node");
}

void StateUtils::initializeData()
{
  initParameters();
  initInitPose();
  initStationPose();
  pre_state_id = ROBOT_STATE::IDLE;
  pre_status_id = ROBOT_STATUS::VOID;
  state_id = ROBOT_STATE::IDLE;
  status_id = ROBOT_STATUS::VOID;
  movingstate_id = NAVI_STATE::IDLE;
  movingfail_id = NAVI_FAIL_REASON::VOID;
  node_status_id = NODE_STATUS::IDLE;
  scan_callback_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void StateUtils::initParameters()
{
  node_->declare_parameter("use_camera", false);
  node_->declare_parameter("use_multiTof", false);

  node_->get_parameter("use_camera", use_camera_);
  node_->get_parameter("use_multiTof", use_multiTof_);

  node_->declare_parameter("direct_velocity_v", 0.4);
  node_->declare_parameter("direct_velocity_w", 0.0);
  node_->declare_parameter("undocking_distance", 0.5);

  node_->get_parameter("direct_velocity_v", direct_velocity_v_);
  node_->get_parameter("direct_velocity_w", direct_velocity_w_);
  node_->get_parameter("undocking_distance", undocking_distance_);

  node_->declare_parameter("odom_reset_timeout", 5.0);
  node_->declare_parameter("lidar_wait_timeout", 5.0);
  node_->declare_parameter("tof_wait_timeout", 3.0);
  node_->declare_parameter("camera_wait_timeout", 3.0);

  node_->declare_parameter("odom_reset_retry_count", 3);
  node_->declare_parameter("lidar_retry_count", 3);
  node_->declare_parameter("tof_retry_count", 3);
  node_->declare_parameter("camera_retry_count", 3);
  node_->declare_parameter("localization_retry_count", 3);

  node_->declare_parameter("move_goal_retry_count", 1);
  node_->declare_parameter("sensor_off_time", 60.0);

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

  node_->get_parameter("move_goal_retry_count", move_goal_retry_count_);
  node_->get_parameter("sensor_off_time", sensor_off_time_);

  RCLCPP_INFO(node_->get_logger(), "Updated use_camera: %s", use_camera_ ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "Updated use_multiTof: %s", use_multiTof_ ? "true" : "false");

  RCLCPP_INFO(node_->get_logger(), "direct_velocity_v: %.2f", direct_velocity_v_);
  RCLCPP_INFO(node_->get_logger(), "direct_velocity_w: %.2f", direct_velocity_w_);
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
  RCLCPP_INFO(node_->get_logger(), "move_goal_retry_count: %u", move_goal_retry_count_);
  RCLCPP_INFO(node_->get_logger(), "sensor_off_time: %.2f", sensor_off_time_);
}

void StateUtils::initInitPose()
{ 
  init_pose.x = 0.0;
  init_pose.y = 0.0;
  init_pose.theta = 0.0; 

  init_pose_msg.header.stamp = rclcpp::Time(0);  // 타임스탬프 초기화 (예: 0초)
  init_pose_msg.header.frame_id = "map";  // 좌표계 설정

  init_pose_msg.pose.pose.position.x = init_pose.x;
  init_pose_msg.pose.pose.position.y = init_pose.y;
  init_pose_msg.pose.pose.position.z = 0.0;

  init_pose_msg.pose.pose.orientation.x = 0.0;
  init_pose_msg.pose.pose.orientation.y = 0.0;
  init_pose_msg.pose.pose.orientation.z = init_pose.theta;
  init_pose_msg.pose.pose.orientation.w = 1.0;
  init_pose_msg.pose.covariance.fill(0.0);

}

void StateUtils::initStationPose()
{
  station_pose.x = init_pose.x - 0.3 * std::cos(init_pose.theta);
  station_pose.y = init_pose.y - 0.3 * std::sin(init_pose.theta);
  station_pose.theta = 0.0;

  station_position_msg.header.stamp = rclcpp::Time(0);  // 타임스탬프 초기화 (예: 0초)
  station_position_msg.header.frame_id = "map";  // 좌표계 설정

  station_position_msg.pose.pose.position.x = station_pose.x;
  station_position_msg.pose.pose.position.y = station_pose.y;
  station_position_msg.pose.pose.position.z = 0.0;

  station_position_msg.pose.pose.orientation.x = 0.0;
  station_position_msg.pose.pose.orientation.y = 0.0;
  station_position_msg.pose.pose.orientation.z = station_pose.theta;
  station_position_msg.pose.pose.orientation.w = 1.0;
  station_position_msg.pose.covariance.fill(0.0);

}

rcl_interfaces::msg::SetParametersResult StateUtils::paramCallback(const std::vector<rclcpp::Parameter>& params)
{  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
      if (param.get_name() == "use_camera") {
          use_camera_ = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Updated use_camera: %s", use_camera_ ? "true" : "false");
      }
      else if (param.get_name() == "use_multiTof") {
          use_multiTof_ = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Updated use_multiTof: %s", use_multiTof_ ? "true" : "false");
      }
      else if (param.get_name() == "direct_velocity_v") {
        direct_velocity_v_ = param.as_double();
        RCLCPP_INFO(node_->get_logger(), "Updated direct_velocity_v: %.2f", direct_velocity_v_);
      }
      else if (param.get_name() == "direct_velocity_w") {
        direct_velocity_w_ = param.as_double();
          RCLCPP_INFO(node_->get_logger(), "Updated direct_velocity_w: %.2f", direct_velocity_w_);
      }
      else if (param.get_name() == "undocking_distance") {
        direct_velocity_w_ = param.as_double();
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
  arrived_goal_start_time_ = node_->now().seconds();
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
  double waitTime = node_->now().seconds()-arrived_goal_start_time_;
  double threshold = sensor_off_time_;
  if(state_id == ROBOT_STATE::IDLE || state_id == ROBOT_STATE::ERROR || state_id == ROBOT_STATE::ONSTATION){
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
    publishManeuverOff();
    publishSenSorManagerOff();
    publishAllSensorOff();
    disableArrivedGoalSensorsOffTimer();
  }
}

void StateUtils::enableOdomcallback()
{
  RCLCPP_INFO(node_->get_logger(), "enableOdomcallback");
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
    std::bind(&StateUtils::odom_callback, this, std::placeholders::_1));
  odom_status_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("/odom_status", 1,
      std::bind(&StateUtils::odom_status_callback, this, std::placeholders::_1));
}

void StateUtils::disableOdomcallback()
{
  if (odom_sub_) {  // 등록되어 있다면 해제
    odom_sub_.reset();
    RCLCPP_INFO(node_->get_logger(), "odom callback disabled");
    ready_odom = false;
  }

  if (odom_status_sub_) {  // 등록되어 있다면 해제
    odom_status_sub_.reset();
    RCLCPP_INFO(node_->get_logger(), "odom status callback disabled");
  }
}

void StateUtils::enableLocalizationcallback()
{
  RCLCPP_INFO(node_->get_logger(), "enableLocalizationcallback");
  localize_complete_sub = node_->create_subscription<std_msgs::msg::Bool>("/localization/complete", 1,
    std::bind(&StateUtils::localizationComplete_callback, this, std::placeholders::_1));

  initial_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1,
    std::bind(&StateUtils::initial_pose_callback, this, std::placeholders::_1));
}

void StateUtils::disableLocalizationcallback()
{
  if (localize_complete_sub) {  // 등록되어 있다면 해제
    localize_complete_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "localization complete callback disabled");
  }

  if (initial_pose_sub) {  // 등록되어 있다면 해제
    initial_pose_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "initial pose callback disabled");
  }
}

void StateUtils::enableSensorcallback()
{
  scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
    std::bind(&StateUtils::scan_callback, this, std::placeholders::_1));

  if(use_multiTof_){
    tof_status_sub = node_->create_subscription<robot_custom_msgs::msg::TofData>("/tof_data", 10, std::bind(&StateUtils::tofCallback, this, std::placeholders::_1));
  }  
  if(use_camera_){
    camera_data_sub = node_->create_subscription<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10, std::bind(&StateUtils::cameraCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(node_->get_logger(), "enableSensorcallback");
}

void StateUtils::disableSensorcallback()
{
  if (scan_sub) {
    scan_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "scan callback disabled");
  }

  if(use_multiTof_){
    if (tof_status_sub) {
      tof_status_sub.reset();
      RCLCPP_INFO(node_->get_logger(), "tof callback disabled");
    }
  }

  if(use_camera_){
    if (camera_data_sub) {
      camera_data_sub.reset();
      RCLCPP_INFO(node_->get_logger(), "camera callback disabled");
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
  odom_reset_cnt_ = 0;
  enableOdomcallback();
  bStartOdomReset = true;
  bSendResetOdomCmd = true;
  bOdomResetDone = false;
  setOdomResetError(false);
  publishClearOdomReset();
  RCLCPP_ERROR(node_->get_logger(), "startMonitorOdomReset");
  odom_reset_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),std::bind(&StateUtils::monitor_resetOdom, this));
  odom_reset_monitor_start_time_ = node_->now().seconds();
}


void StateUtils::monitor_resetOdom() {
  double runtime = node_->now().seconds() - reset_odom_start_time_;
  bool exitMonitor = false;
  if(bSendResetOdomCmd){
    if(odom_reset_cnt_ > 0){
      RCLCPP_INFO(node_->get_logger(),"Retry - ODOM RESET Count : %d",odom_reset_cnt_);
    }else{
      RCLCPP_INFO(node_->get_logger(),"Start-ODOM RESET");
    }
    publishStartOdomReset();
    bSendResetOdomCmd = false;
    return;
  }

  if (isValidateResetOdom(odom_)) {
    double total_runTime = node_->now().seconds()-odom_reset_monitor_start_time_;
    bOdomResetDone = true;
    exitMonitor = true;
    RCLCPP_INFO(node_->get_logger(), "odom-reset Complete runtime : %f, retry coount : %d",total_runTime,odom_reset_cnt_);
    odom_reset_cnt_ = 0;
  } else if (runtime >= odom_reset_timeout_) {
    bSendResetOdomCmd = true;
    if (++odom_reset_cnt_ >= odom_reset_retry_count_) {
      RCLCPP_INFO(node_->get_logger(), "ODOM RESET ERROR!!");
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
  disableOdomcallback();
  reset_timerResetOdom();
}

void StateUtils::startSensorMonitor()
{
  tof_retry_cnt = 0;
  lidar_retry_cnt = 0;
  camera_retry_cnt = 0;
  if(!bStartSensorOn )
  {
    bStartSensorOn = true;
    bLidarSensorOK = false;
    if(use_multiTof_){
      bMultiToFSensorOK = false;
    }else{
      bMultiToFSensorOK = true;
    }
    if(use_camera_){
      bCameraSensorOK = false;
    }else{
      bCameraSensorOK = true;
    }

    setLidarError(false);
    setToFError(false);
    setSensorReady(false);
    enableSensorcallback();
    publishAllSensorOn();
    sensor_monitor_start_time_ = node_->now().seconds();
    RCLCPP_ERROR(node_->get_logger(), "startSensorMonitor");
    sensor_monitor_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),std::bind(&StateUtils::monitor_sensor, this));
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Sensor Already ON...");
    disableArrivedGoalSensorsOffTimer();
  }
}

void StateUtils::stopSensorMonitor()
{
  disableSensorcallback();
  reset_timerSensorMonitor();
}

void StateUtils::monitor_sensor()
{
  if(bSendLidarCmd){
    bSendLidarCmd = false;
    publishLidarOn();
    RCLCPP_INFO(node_->get_logger(), "retry lidar On count : %u",lidar_retry_cnt);
  }

  if(use_multiTof_){
    if(bSendTofCmd){
      bSendTofCmd = false;
      publishMultiTofOn();
      RCLCPP_INFO(node_->get_logger(), "retry Tof On count : %u",tof_retry_cnt);
    }
  }

  if(use_camera_){
    if(bSendCameraCmd){
      bSendCameraCmd = false;
      publishCameraOn();
      RCLCPP_INFO(node_->get_logger(), "retry Tof On count : %u",camera_retry_cnt);
    }
  }

  double waitLidar = node_->now().seconds()-lidarOn_time;
  
  if(bStartSensorOn)
  {
    if(bLidarSensorOK && bMultiToFSensorOK && bCameraSensorOK){
      //RCLCPP_INFO(node_->get_logger(), "SenSor is OK");
      setSensorReady(true);
      stopSensorMonitor();
    }else{
      if(waitLidar >= lidar_wait_timeout_ && !bLidarSensorOK){
        if(++lidar_retry_cnt >= lidar_retry_count_){
          lidar_retry_cnt = 0;
          RCLCPP_INFO(node_->get_logger(), "LIdar Error");
          setLidarError(true);
          stopSensorMonitor();
        }else{
          //publishLidarOff();
          bSendLidarCmd = true;
        }
      }

      if(use_multiTof_)
      {
        double waitTof = node_->now().seconds()-tofOn_time;
        if(waitTof >= tof_wait_timeout_ && !bMultiToFSensorOK)
        {
            if(++tof_retry_cnt >= tof_retry_count_){
              tof_retry_cnt = 0;
              RCLCPP_INFO(node_->get_logger(), "Multi Tof Error");
              setToFError(true);
              stopSensorMonitor();
            }else{
              //publishMultiTofOff();
              bSendTofCmd = true;
            }
        }
      }

      if(use_camera_)
      {
        double waitCamera = node_->now().seconds()-cameraOn_time;
        if(waitCamera >= camera_wait_timeout_ && !bCameraSensorOK){
          if(++camera_retry_cnt >= camera_retry_count_){
            camera_retry_cnt = 0;
            RCLCPP_INFO(node_->get_logger(), "Camera Error");
            setCameraError(true);
            stopSensorMonitor();
          }else{
            //publishCameraOff();
            bSendCameraCmd = true;
          }
        }
      }
    }
  }
}

void StateUtils::startLocalizationMonitor(LOCALIZATION_TYPE type)
{
  localization_retry_cnt = 0;
  Localtype = type;
  bStartLocalizationStart = true;
  bLocalizationComplete = false;
  setLocalizationError(false);
  enableLocalizationcallback();
  if(Localtype == LOCALIZATION_TYPE::INIT_POSE){
    publishLocalizeUndockPose();
  }else if(Localtype == LOCALIZATION_TYPE::ROBOT_POSE){
    publishLocalizePose();
  }else if(Localtype == LOCALIZATION_TYPE::SAVED_POSE){
    publishLocalizeSavedPose();
  }else{
    RCLCPP_ERROR(node_->get_logger(), "localization type error");
  }
  
  RCLCPP_ERROR(node_->get_logger(), "startLocalizationMonitor");
  localization_monitor_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),std::bind(&StateUtils::monitor_localization, this));
}
void StateUtils::stopLocalizationMonitor()
{
  disableLocalizationcallback();
  reset_timerLocalization();
}
void StateUtils::monitor_localization()
{
  double runTime = node_->now().seconds()-localize_start_time;
  if(bLocalizationComplete){
    RCLCPP_INFO(node_->get_logger(), "Localization Complete %f",runTime);
    stopLocalizationMonitor();
  }else if(bLocalizationFail){
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
        publishLocalizeSavedPose();
      }else{
        RCLCPP_ERROR(node_->get_logger(), "localization type error");
      }
    }
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
  if ((odom.x >= -0.1 && odom.x <= 0.1) && (odom.y >= -0.1 && odom.y <= 0.1) &&
      (odom.theta >= -0.2 && odom.theta <= 0.1)) {
    RCLCPP_INFO(node_->get_logger(), "ODOM RESET Success");
    ret = true;
  }
  return ret;
}

void StateUtils::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_.x = msg->pose.pose.position.x;
    odom_.y = msg->pose.pose.position.y;
    odom_.theta = quaternion_to_euler(msg->pose.pose.orientation);
    ready_odom = true;
}

void StateUtils::odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    odom_status = msg->data;
}

void StateUtils::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
    RCLCPP_INFO(node_->get_logger(), "initial_pose_callback");
}

void StateUtils::localizationComplete_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if(msg->data){
    RCLCPP_INFO(node_->get_logger(), "localization-Complete");
    bLocalizationComplete = true;
  }else{
    RCLCPP_INFO(node_->get_logger(), "localization-Fail");
    bLocalizationFail = true;
  } 
}

void StateUtils::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    robot_position_msg.header = msg->header;
    robot_position_msg.pose = msg->pose;
    robot_pose.x = static_cast<int>(robot_position_msg.pose.pose.position.x * 1000000 + 0.5) / 1000000.0;
    robot_pose.y = static_cast<int>(robot_position_msg.pose.pose.position.y * 1000000 + 0.5) / 1000000.0;
    robot_pose.theta = quaternion_to_euler(robot_position_msg.pose.pose.orientation);
    //RCLCPP_INFO(node_->get_logger(), "amclCallback X : %f , Y : %f, THETA : %f,", robot_pose.x,robot_pose.y,robot_pose.theta);
}

void StateUtils::slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    robot_position_msg.header = msg->header;
    robot_position_msg.pose = msg->pose;
    robot_pose.x = static_cast<int>(robot_position_msg.pose.pose.position.x * 1000000 + 0.5) / 1000000.0;
    robot_pose.y = static_cast<int>(robot_position_msg.pose.pose.position.y * 1000000 + 0.5) / 1000000.0;
    robot_pose.theta = quaternion_to_euler(robot_position_msg.pose.pose.orientation);
}

void StateUtils::stationPoseCallack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  station_position_msg.header = msg->header;
  init_pose_msg.header = msg->header;
  station_position_msg.pose = msg->pose;

  station_pose.x = static_cast<int>(station_position_msg.pose.pose.position.x * 1000000 + 0.5) / 1000000.0;
  station_pose.y = static_cast<int>(station_position_msg.pose.pose.position.y * 1000000 + 0.5) / 1000000.0;
  station_pose.theta = quaternion_to_euler(station_position_msg.pose.pose.orientation);

  init_pose.x = station_pose.x + 0.3 * std::cos(station_pose.theta);
  init_pose.y = station_pose.y + 0.3 * std::sin(station_pose.theta);
  init_pose.theta = station_pose.theta;

  init_pose_msg.pose.pose.position.x = init_pose.x;
  init_pose_msg.pose.pose.position.y = init_pose.y;
  init_pose_msg.pose.pose.position.z = station_position_msg.pose.pose.position.z;
  init_pose_msg.pose.pose.orientation = station_position_msg.pose.pose.orientation;

  RCLCPP_INFO(node_->get_logger(), "SET STATION POSE  %f | %f |%f ", station_pose.x, station_pose.y, station_pose.theta);
  RCLCPP_INFO(node_->get_logger(), "SET INIT POSE  %f | %f |%f ", init_pose.x, init_pose.y, init_pose.theta);
}

void StateUtils::pathPlanDestinationCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  pathPlanDestination = msg->data;
    // robot_position_msg.header = msg->header;
    // robot_position_msg.pose = msg->pose;
    // robot_pose.x = static_cast<int>(robot_position_msg.pose.pose.position.x * 1000000 + 0.5) / 1000000.0;
    // robot_pose.y = static_cast<int>(robot_position_msg.pose.pose.position.y * 1000000 + 0.5) / 1000000.0;
    // robot_pose.theta = quaternion_to_euler(robot_position_msg.pose.pose.orientation);
}

void StateUtils::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 현재 시간 가져오기
  double diff = node_->now().seconds()-lidarOn_time;
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
        double frequency = 1.0 / period;
        // RCLCPP_INFO(node_->get_logger(), "Scan Frequency: %.2f Hz", frequency);
        if (!msg->ranges.empty() && frequency >= 9) {
          if(!bLidarSensorOK){
  
            RCLCPP_INFO(node_->get_logger(), "Lidar Sensor is OK diff : %f ",diff);
          }
          lidar_retry_cnt = 0;
          bLidarSensorOK = true;
        }else{
          if(msg->ranges.empty()){
            RCLCPP_INFO(node_->get_logger(), "scan data is empty");
          }
          if(frequency < 9){
            RCLCPP_INFO(node_->get_logger(), "scan frequency low : %.2f ",frequency);
          }
        }
    }
    scan_callback_time_ = current_time;
  } else {
      RCLCPP_WARN(node_->get_logger(), "Received nullptr message!");
  }
}

void StateUtils::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    //RCLCPP_INFO(node_->get_logger(), "tofCallback");
    double diff = node_->now().seconds()-tofOn_time;
    if(msg->bot_status == 0x00){
      if(!bMultiToFSensorOK){
        RCLCPP_INFO(node_->get_logger(), "tof Sensor is OK Diff : %f",diff);
      }
      tof_retry_cnt = 0;
      bMultiToFSensorOK = true;
    }
}

void StateUtils::cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr /*msg*/)
{
  double diff = node_->now().seconds()-cameraOn_time;

    if(!bCameraSensorOK){
      RCLCPP_INFO(node_->get_logger(), "camera Sensor is OK Diff : %f",diff);
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
    if(bStartOdomReset && bOdomResetDone){
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
  return bOdomResetError;
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
  reset_odom_start_time_ = node_->now().seconds();
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
    lidarOn_time = node_->now().seconds();
}

void StateUtils::publishLidarOff()
{
    bLidarSensorOK = false;
    std_msgs::msg::Bool lidar_cmd;
    lidar_cmd.data = false;
    lidar_cmd_pub_->publish(lidar_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishLidarOff");
}

void StateUtils::publishMultiTofOn()
{
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = true;
    tof_cmd_pub_->publish(tof_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishMultiTofOn");
    tofOn_time = node_->now().seconds();
}

void StateUtils::publishMultiTofOff()
{
    bMultiToFSensorOK = false;
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = false;
    tof_cmd_pub_->publish(tof_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishMultiTofOff");
}

void StateUtils::publishCameraOff()
{
  bCameraSensorOK = false;
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
  cameraOn_time = node_->now().seconds();
  RCLCPP_INFO(node_->get_logger(), "publishCameraOn");
}

void StateUtils::publishAllSensorOn()
{
  bStartSensorOn = true;
    if(use_multiTof_){
      publishMultiTofOn();
    }

    if(use_camera_){
      publishCameraOn();
    }
    publishLidarOn();
    RCLCPP_INFO(node_->get_logger(), "publishAllSensorOn");
}

void StateUtils::publishAllSensorOff()
{
   bStartSensorOn = false;
   if(use_multiTof_){
    publishMultiTofOff();
   }
   if(use_camera_){
    publishCameraOff();
   } 

  publishLidarOff();
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
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(init_pose_msg); //station pose set enable
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeUndockPose X : %f , Y : %f, THETA : %f,", init_pose.x,init_pose.y,init_pose.theta);
}

void StateUtils::publishLocalizePose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(robot_position_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizePose X : %f , Y : %f, THETA : %f,", robot_pose.x,robot_pose.y,robot_pose.theta);
}

void StateUtils::publishLocalizeSavedPose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    bLocalizationFail = false;
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(last_position_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeSavedPose X : %f , Y : %f, THETA : %f,", last_pose.x,last_pose.y,last_pose.theta);
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
void StateUtils::publishMappingOn()
{
  std_msgs::msg::Bool cmd_mapping;
  cmd_mapping.data = true;
  mapping_cmd_pub_->publish(cmd_mapping);
  RCLCPP_INFO(node_->get_logger(), "publishMappingOn");
}
void StateUtils::publishMappingOff()
{
  std_msgs::msg::Bool cmd_mapping;
  cmd_mapping.data = false;
  mapping_cmd_pub_->publish(cmd_mapping);
  RCLCPP_INFO(node_->get_logger(), "publishMappingOff");
}

void StateUtils::publishSenSorManagerOn()
{
  std_msgs::msg::Bool cmd_sensor_manager;
  cmd_sensor_manager.data = true;
  sensor_manager_cmd_pub_->publish(cmd_sensor_manager);
  RCLCPP_INFO(node_->get_logger(), "publishSenSorManagerOn");
}

void StateUtils::publishSenSorManagerOff()
{
  std_msgs::msg::Bool cmd_sensor_manager;
  cmd_sensor_manager.data = false;
  sensor_manager_cmd_pub_->publish(cmd_sensor_manager);
  RCLCPP_INFO(node_->get_logger(), "publishSenSorManagerOff");
}

//about ROBOT STATE 
void StateUtils::setAllRobotStateIDs(ROBOT_STATE data_state, ROBOT_STATUS data_status, state_cmd data_cmd){
  pre_state_id = getStateID();
  pre_status_id = getStatusID();
  pre_cmd_ids = getRobotCMDID();

  setStateID(data_state);
  setStatusID(data_status);
  if( data_state == ROBOT_STATE::IDLE || data_state == ROBOT_STATE::ONSTATION )
  {
    data_cmd.soc_cmd = REQUEST_SOC_CMD::VOID;
    data_cmd.robot_cmd = REQUEST_ROBOT_CMD::VOID;
  }
  setRobotCMDID(data_cmd);
}

void StateUtils::setStateID( const ROBOT_STATE &id){
  state_id = id; 
  publishRobotState(state_id);
}
ROBOT_STATE StateUtils::getStateID(){
  return state_id;
}

void StateUtils::setStatusID( const ROBOT_STATUS &id){
  pre_status_id = getStatusID();
  status_id = id;
  publishRobotStatus(status_id);
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
  }
  if(movingfail_id != reason){
    RCLCPP_INFO(node_->get_logger(), "SET NAVI_FAIL_REASON : %s", enumToString(reason).c_str());
  }

  movingstate_id = id;
  movingfail_id = reason;
  publishMovingState(movingstate_id,movingfail_id); 
}

NAVI_STATE StateUtils::getMovingStateID(){ 
  return movingstate_id; 
}

void StateUtils::setNodeStatusID(const NODE_STATUS &id){
    node_status_id = id;
    publishNodeState(node_status_id);
}

NODE_STATUS StateUtils::getNodeStatusID(){
    return node_status_id;
}

ROBOT_STATE StateUtils::getPreStateID(){
  return pre_state_id;
}

double StateUtils::getLocalizationStartTime(){
  return localize_start_time;
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


void StateUtils::setOnstationStatus(const bool &data){
  on_station_status = data;
}
bool StateUtils::getOnstationStatus(){
  return on_station_status;
}

void StateUtils::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
  bool bOnStation = static_cast<bool>(msg->docking_status & 0x70);
  setOnstationStatus(bOnStation);
}

void StateUtils::move_target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg) {
  disableArrivedGoalSensorsOffTimer();
  setStartOnStation(false);
  if(!movingData.bStartMoving){
    movingData.bStartMoving = true;
    movingData.target_position.x = msg->x;
    movingData.target_position.y = msg->y;
    movingData.target_position.theta = msg->theta;
    movingData.type = std::numeric_limits<uint8_t>::max();
    RCLCPP_INFO(node_->get_logger(), "move_target_callback x : %f , y : %f, theta : %f ", movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
  }
}

void StateUtils::move_charger_callback(const std_msgs::msg::Empty::SharedPtr msg) {
  disableArrivedGoalSensorsOffTimer();
  setStartOnStation(false);
  bTryMoveCharger = true;
  RCLCPP_INFO(node_->get_logger(), "move_charger_callback");
}

void StateUtils::move_rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg) {
  disableArrivedGoalSensorsOffTimer();
  setStartOnStation(false);
  if(!movingData.bStartMoving){
    movingData.bStartMoving = true;
    movingData.target_position.x = msg->x;
    movingData.target_position.y = msg->y;
    movingData.target_position.theta = msg->theta;
    movingData.type = msg->type;
    RCLCPP_INFO(node_->get_logger(), "move_rotation_callback x : %f , y : %f, theta : %f ", movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
  }
}

MOVING_DATA StateUtils::getTargetPosition()
{
  MOVING_DATA return_movingData = movingData;
  movingData.bStartMoving = false; //bStartMoving에 따라서 navi에서 동작함.
  return return_movingData;
}

pose StateUtils::getCurrentOdom() // for undocking 
{
  return odom_;
}
bool StateUtils::getPrepareOdomFlag()
{
  return ready_odom;
}

void StateUtils::saveLastPosition()
{
  last_pose = robot_pose;
  last_position_msg = robot_position_msg;
  RCLCPP_INFO(node_->get_logger(), "saveLastPosition X : %f , Y : %f, THETA : %f,", last_pose.x,last_pose.y,last_pose.theta);
}

double StateUtils::getDistance(pose base, pose current) {
  return std::sqrt((current.x - base.x) * (current.x - base.x) + (current.y - base.y) * (current.y - base.y));
}

void StateUtils::publishMoveFailError()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  move_fail_error_pub_->publish(msg);
}

void StateUtils::publishAlternativeDestinationError()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  alternative_dest_error_pub_->publish(msg);
}

void StateUtils::publishVelocityCommand(double v, double w)
{
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = w;
    direct_vel_pub_->publish(cmd_msg);
    //RCLCPP_INFO(node_->get_logger(), "publishVelocityCommand V : %f, W : %f ", v,w);
}

void StateUtils::publishRobotState(ROBOT_STATE state)
{
  auto req_state_msg = robot_custom_msgs::msg::RobotState();
  req_state_msg.state = int(state);
  req_state_msg.status = int(getStatusID());
  robot_state_pub_->publish(req_state_msg);
}

void StateUtils::publishRobotStatus(ROBOT_STATUS status)
{
  auto req_state_msg = robot_custom_msgs::msg::RobotState();
  req_state_msg.state = int(getStateID());
  req_state_msg.status = int(status);
  robot_state_pub_->publish(req_state_msg);
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

double StateUtils::getUndockingDistance()
{
  return undocking_distance_;
}

uint8_t StateUtils::getMoveGoalRetryCount(){
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
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   node manage ACTION CLIENT functions..   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void StateUtils::setNodeClientStatus(const int &status){
  node_client_status = status;
}

int StateUtils::getNodeClientStatus(){
  return node_client_status;
}

void StateUtils::send_node_goal(const NODE_STATUS &require_node) {
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

  auto send_goal_options = rclcpp_action::Client<robot_custom_msgs::action::ManageNode>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<robot_custom_msgs::action::ManageNode>::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(node_->get_logger(), "[StateUtils]Node launch was rejected by server");
        } else {
          RCLCPP_INFO(node_->get_logger(), "[StateUtils]Node launchaccepted by server, waiting for result");
        }
      };

  send_goal_options.result_callback =
      [this,require_node](const rclcpp_action::ClientGoalHandle<robot_custom_msgs::action::ManageNode>::WrappedResult & result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          setNodeClientStatus(1);
          if( require_node == NODE_STATUS::FT_NAVI){
            setNodeStatusID(NODE_STATUS::NAVI);
          } else {
            setNodeStatusID(require_node);
          }
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

  node_client_->async_send_goal(goal_msg, send_goal_options);
}

}