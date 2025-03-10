#include "state_manager/utils/state_utils.hpp"
#include "rclcpp/qos.hpp"

#define USE_TOF_ONOFF 0
#define USE_CAMERA_ONOFF 1
#define ODOM_RESET_TIMEOUT 5.0
#define ODOE_RESET_RETRY_COUNT 6
#define LOCALIZATION_TIMEOUT 10.0
#define LIDAR_WAIT_TIMEOUT 5.0
#define TOF_WAIT_TIMEOUT 5.0
#define CAMERA_WAIT_TIMEOUT 5.0

namespace airbot_state {

StateUtils::StateUtils(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
  initializeData();
  req_clear_costmap_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/clear/costmap", 1);
  req_nomotion_local_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/request", 1);
  req_estimatePose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/request/pose", 1);
  reset_odom_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);
  lidar_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
  tof_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);
  camera_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_camera", 10);
  move_fail_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/move_fail_error", 10);
  alternative_dest_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/alternative_dest_error", 10);

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

  req_rotation_target_sub_ = node_->create_subscription<robot_custom_msgs::msg::MoveNRotation>("/move_n_rotation", 10,
    std::bind(&StateUtils::move_rotation_callback, this, std::placeholders::_1));
}

void StateUtils::initializeData()
{
  initInitPose();
  initStationPose();
  pre_state_id = ROBOT_STATE::IDLE;
  pre_status_id = ROBOT_STATUS::VOID;
  state_id = ROBOT_STATE::IDLE;
  status_id = ROBOT_STATUS::VOID;
  movingstate_id = NAVI_STATE::IDLE;
  movingfail_id = NAVI_FAIL_REASON::VOID;
  node_status_id = NODE_STATUS::IDLE;
  scan_callback_time_ = rclcpp::Time(0);
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

void StateUtils::enableArrivedGoalSensorsOffTimer()
{
  arrived_goal_start_time_ = node_->now().seconds();
  if(!arrivedgoal_sensoroff_timer_){
    arrivedgoal_sensoroff_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateUtils::monitor_ArrivedGoal_SensorsOff, this));
    RCLCPP_INFO(node_->get_logger(), "enableArrivedGoalSensorsOffTimer");
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
  double waitTime = node_->now().seconds()-arrived_goal_start_time_;
  if(waitTime >= 10){
    RCLCPP_INFO(node_->get_logger(), "10 sec over from Arrived Goal");
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
  localize_complete_sub = node_->create_subscription<std_msgs::msg::Empty>("/localization/complete", 1,
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
  #if USE_TOF_ONOFF > 0  
  tof_status_sub = node_->create_subscription<robot_custom_msgs::msg::TofData>("/tof_data", 10, std::bind(&StateUtils::tofCallback, this, std::placeholders::_1));
  #endif
  #if USE_CAMERA_ONOFF > 0 
  camera_data_sub = node_->create_subscription<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10, std::bind(&StateUtils::cameraCallback, this, std::placeholders::_1));
  #endif
  RCLCPP_INFO(node_->get_logger(), "enableSensorcallback");
}

void StateUtils::disableSensorcallback()
{
  if (scan_sub) {  // 등록되어 있다면 해제
    scan_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "scan callback disabled");
  }
  #if USE_TOF_ONOFF > 0
  if (tof_status_sub) {  // 등록되어 있다면 해제
    tof_status_sub.reset();
    RCLCPP_INFO(node_->get_logger(), "tof callback disabled");
  }
  #endif
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
  } else if (runtime >= ODOM_RESET_TIMEOUT) {
    bSendResetOdomCmd = true;
    if (++odom_reset_cnt_ >= ODOE_RESET_RETRY_COUNT) {
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
  if( !bLidarSensorOK )// tof 끄지 않아 주석 && !bMultiToFSensorOK )
  {
    tof_retry_cnt = 0;
    lidar_retry_cnt = 0;
    camera_retry_cnt = 0;
    bStartSensorOn = true;
    bLidarSensorOK = false;
    #if USE_TOF_ONOFF > 0
    bMultiToFSensorOK = false;
    #else
    bMultiToFSensorOK = true;
    #endif

    #if USE_CAMERA_ONOFF > 0
    bCameraSensorOK = false;
    #else
    bCameraSensorOK = true;
    #endif

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
  #if USE_TOF_ONOFF > 0
  if(bSendTofCmd){
    bSendTofCmd = false;
    publishMultiTofOn();
    RCLCPP_INFO(node_->get_logger(), "retry Tof On count : %u",tof_retry_cnt);
  }
  #endif

  #if USE_CAMERA_ONOFF > 0
  if(bSendCameraCmd){
    bSendCameraCmd = false;
    publishCameraOn();
    RCLCPP_INFO(node_->get_logger(), "retry Tof On count : %u",camera_retry_cnt);
  }
  #endif

  double waitLidar = node_->now().seconds()-lidarOn_time;
#if USE_TOF_ONOFF > 0
  double waitTof = node_->now().seconds()-tofOn_time;
#endif
  double waitCamera = node_->now().seconds()-cameraOn_time;
  
  if(bLidarSensorOK && bMultiToFSensorOK && bCameraSensorOK){
    RCLCPP_INFO(node_->get_logger(), "SenSor is OK");
    setSensorReady(true);
    stopSensorMonitor();
  }else if(waitLidar >= LIDAR_WAIT_TIMEOUT && !bLidarSensorOK){
    if(++lidar_retry_cnt >= 6){
      RCLCPP_INFO(node_->get_logger(), "LIdar Error");
      setLidarError(true);
      stopSensorMonitor();
    }else{
      publishLidarOff();
      bSendLidarCmd = true;
    }
  }
  #if USE_TOF_ONOFF > 0
  else if(waitTof >= TOF_WAIT_TIMEOUT && !bMultiToFSensorOK){
      if(++tof_retry_cnt >= 6){
        RCLCPP_INFO(node_->get_logger(), "Multi Tof Error");
        setToFError(true);
        stopSensorMonitor();
      }else{
        publishMultiTofOff();
        bSendTofCmd = true;
      }
  }
  #endif
  #if USE_CAMERA_ONOFF > 0
  else if(waitCamera >= CAMERA_WAIT_TIMEOUT && !bCameraSensorOK){
      if(++camera_retry_cnt >= 6){
        RCLCPP_INFO(node_->get_logger(), "Camera Error");
        setToFError(true);
        stopSensorMonitor();
      }else{
        publishCameraOff();
        bSendCameraCmd = true;
      }
  }
  #endif
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
    RCLCPP_INFO(node_->get_logger(), "Localization Complete");
    stopLocalizationMonitor();
  }else if(runTime >= LOCALIZATION_TIMEOUT){
    if(++localization_retry_cnt >= 6){
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

void StateUtils::localizationComplete_callback(const std_msgs::msg::Empty::SharedPtr) {
    RCLCPP_INFO(node_->get_logger(), "localizationComplete_callback");
    bLocalizationComplete = true;
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
  rclcpp::Time current_time = msg->header.stamp;
  // RCLCPP_INFO(node_->get_logger(), "scan_callback");
  // 이전 시간과 비교하여 주기 계산
  if (scan_callback_time_.nanoseconds() != 0) {
      double period = (current_time - scan_callback_time_).seconds();
      double frequency = 1.0 / period;
      // RCLCPP_INFO(node_->get_logger(), "Scan Frequency: %.2f Hz", frequency);
      if (!msg->ranges.empty() && frequency >= 9) {
        if(!bLidarSensorOK){

          RCLCPP_INFO(node_->get_logger(), "Lidar Sensor is OK diff : %f ",diff);
        }
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
  // 현재 시간을 저장
  scan_callback_time_ = current_time;
}

void StateUtils::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    //RCLCPP_INFO(node_->get_logger(), "tofCallback");
    double diff = node_->now().seconds()-tofOn_time;
    if(msg->bot_status == 0x00){
      if(!bMultiToFSensorOK){
        RCLCPP_INFO(node_->get_logger(), "tof Sensor is OK Diff : %f",diff);
      }
      bMultiToFSensorOK = true;
    }
}

void StateUtils::cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr /*msg*/)
{
  double diff = node_->now().seconds()-cameraOn_time;

    if(!bCameraSensorOK){
      RCLCPP_INFO(node_->get_logger(), "camera Sensor is OK Diff : %f",diff);
    }
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
    #if USE_TOF_ONOFF > 0
    publishMultiTofOn();
    #endif
    #if USE_TOF_ONOFF > 0
    publishCameraOn();
    #endif
    publishLidarOn();
    RCLCPP_INFO(node_->get_logger(), "publishAllSensorOn");
}

void StateUtils::publishAllSensorOff()
{
    #if USE_TOF_ONOFF > 0
    publishMultiTofOff();
    #endif
    #if USE_TOF_ONOFF > 0
    publishCameraOff();
    #endif
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
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(init_pose_msg); //station pose set enable
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeUndockPose X : %f , Y : %f, THETA : %f,", init_pose.x,init_pose.y,init_pose.theta);
}

void StateUtils::publishLocalizePose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(robot_position_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizePose X : %f , Y : %f, THETA : %f,", robot_pose.x,robot_pose.y,robot_pose.theta);
}

void StateUtils::publishLocalizeSavedPose()
{
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(last_position_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeSavedPose X : %f , Y : %f, THETA : %f,", last_pose.x,last_pose.y,last_pose.theta);
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
}
ROBOT_STATE StateUtils::getStateID(){
  return state_id;
}

void StateUtils::setStatusID( const ROBOT_STATUS &id){
  pre_status_id = getStatusID();
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

void StateUtils::setMovingStateID(const NAVI_STATE &id){
  if(movingstate_id != id){
    if(id == NAVI_STATE::READY){
      startSensorMonitor();
    }else if( (id == NAVI_STATE::ARRIVED_GOAL || id == NAVI_STATE::PAUSE) && state_id != ROBOT_STATE::RETURN_CHARGER ){
      enableArrivedGoalSensorsOffTimer();
    }
    RCLCPP_INFO(node_->get_logger(), "[Navigation] SET Navigation STATE : NAVI_STATE::%s", enumToString(id).c_str());
  }
  movingstate_id = id; 
}

NAVI_STATE StateUtils::getMovingStateID(){ 
  return movingstate_id; 
}

void StateUtils::setMovingFailID(const NAVI_FAIL_REASON &id){
    movingfail_id = id;
}

NAVI_FAIL_REASON StateUtils::getMovingFailID(){
    return movingfail_id;
}

void StateUtils::setNodeStatusID(const NODE_STATUS &id){
    node_status_id = id;
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

bool StateUtils::isValidateNode(const NODE_STATUS &node, double start_time) {
  std::vector<std::string> require_nodes;
  if( node ==  NODE_STATUS::AUTO_MAPPING){
    require_nodes = {
      "/behavior_server",
      "/bt_navigator",
      "/bt_navigator_navigate_to_pose_rclcpp_node",
      "/controller_server",
      "/explorer_node",
      "/planner_server",
      "/robot_pose_publisher_node",
      "/slam_toolbox",
      "/smoother_server",
      "/velocity_smoother",
      "/warmup_server_node",
      "/waypoint_follower"
    };
  } else if( node ==  NODE_STATUS::MANUAL_MAPPING){
    require_nodes = {
      "/robot_pose_publisher_node",
      "/slam_toolbox"
    };
  } else if( node ==  NODE_STATUS::NAVI){
    require_nodes = {
      "/amcl",
      "/behavior_server",
      "/bt_navigator",
      "/bt_navigator_navigate_to_pose_rclcpp_node",
      "/global_costmap/global_costmap",
      "/local_costmap/local_costmap",
      "/controller_server",
      "/planner_server",
      "/map_server",
      "/lifecycle_manager",
      "/nav2_container",
      "/smoother_server",
      "/velocity_smoother",
      "/waypoint_follower"
    };
  }

  static std::vector<std::string> temp_require_nodes;

  if( temp_require_nodes.empty() )
  {
    temp_require_nodes = require_nodes;
  }

  std::vector<std::string> running_nodes = node_->get_node_names();
  int nodes_size = temp_require_nodes.size();
  RCLCPP_ERROR(node_->get_logger(), "[isValidProcess] running node size : %d | Number of Remaining Checking Node size %d ",running_nodes.size(), nodes_size);
  for (int i = 0; i < nodes_size; i++) {
    bool found = false;
    for (const auto& running_node : running_nodes) {
      if (running_node.find(temp_require_nodes[i]) != std::string::npos) {
        found = true;
        double wait_navi_launch = node_->now().seconds()-start_time;
        RCLCPP_INFO(node_->get_logger(), "[isValidProcess] Required [%s] node launched [%s]: time %f", enumToString(node).c_str(),running_node.c_str(), wait_navi_launch);
        break;
      }
    }
    if (!found) {
      // RCLCPP_ERROR(node_->get_logger(), "[isValidProcess] Required [%s] node not found: %s", enumToString(node).c_str(),temp_require_nodes[i].c_str());
      return false;
    } else{
      temp_require_nodes.erase(temp_require_nodes.begin() + i);
    }
  }
      
  if( temp_require_nodes.empty() )
  {
    return true;
  }
  return false;
}

bool StateUtils::startProcess(const std::string& command, const std::string& pidFilePath) {
  int result = std::system(("setsid bash -c '" + command + "' > /dev/null 2>&1 & echo $! > " + pidFilePath).c_str());
  if (result == 0) {
      std::ifstream pidFile(pidFilePath);
      if (pidFile.is_open()) {
          std::string pid;
          std::getline(pidFile, pid);
          pidFile.close();
          if (!pid.empty()) {
              RCLCPP_INFO(node_->get_logger(), "Process started successfully with PID: %s", pid.c_str());
              return true;
          }
      }
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to start the process.");
  return false;
}

bool StateUtils::stopProcess(const std::string &pidFilePath) {
  std::ifstream pidFile(pidFilePath);
  std::string pid;
  if (pidFile.is_open()) {
    std::getline(pidFile, pid);
    pidFile.close();
    if (!pid.empty()) {
      pid_t processGroupID = std::stoi(pid);
      if (kill(-processGroupID, 0) == -1) { // Process no longer exists
        RCLCPP_INFO(node_->get_logger(), "[stopProcess] [%s]Process Already terminated ", pidFilePath.c_str());
        return true; // SIGINT 처리 후 종료 성공
      }

      if (kill(-processGroupID, SIGINT) == 0) { //SIGINT 전송 성공
        sleep(1); // Wait for process to handle SIGINT
        if (kill(-processGroupID, 0) == -1) { // Process no longer exists
          RCLCPP_INFO(node_->get_logger(), "[stopProcess] Process terminated successfully.");
          return true; // SIGINT 처리 후 종료 성공
        } else {
          RCLCPP_WARN(node_->get_logger(),"[stopProcess] Process did not terminate, sending SIGKILL.");
          kill(-processGroupID, SIGKILL);
          sleep(1);
          if (kill(-processGroupID, 0) == -1) {
            RCLCPP_INFO(node_->get_logger(), "[stopProcess] Process terminated by SIGKILL successfully.");
            return true;
          }else{
            RCLCPP_ERROR(node_->get_logger(),"[stopProcess] Process did not terminate by SIGKILL");
            return false;
          }
        }
      } else {
        RCLCPP_ERROR(node_->get_logger(), "[stopProcess] Failed to send SIGINT.");
      }
    }
  }
  return false;
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

}