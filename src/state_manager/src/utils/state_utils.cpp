#include "state_manager/utils/state_utils.hpp"
#include "rclcpp/qos.hpp"

#define USE_TOF_ONOFF 0
#define ODOM_RESET_TIMEOUT 5.0
#define ODOE_RESET_RETRY_COUNT 6
namespace airbot_state {

StateUtils::StateUtils(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
  pre_state_id = ROBOT_STATE::IDLE;
  pre_status_id = ROBOT_STATUS::VOID;
  state_id = ROBOT_STATE::IDLE;
  status_id = ROBOT_STATUS::VOID;
  movingstate_id = NAVI_STATE::IDLE;
  movingfail_id = NAVI_FAIL_REASON::VOID;
  node_status_id = NODE_STATUS::IDLE;
  scan_callback_time_ = rclcpp::Time(0);

  req_clear_costmap_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/clear/costmap", 1);
  req_nomotion_local_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/request", 1);
  req_estimatePose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/request/pose", 1);
  reset_odom_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);
  lidar_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
  tof_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);

  amcl_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
    std::bind(&StateUtils::amclCallback, this, std::placeholders::_1));

  slam_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_pose", 10,
    std::bind(&StateUtils::slamPoseCallback, this, std::placeholders::_1));
	
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>("/station_data", 10,
    std::bind(&StateUtils::stationData_callback, this, std::placeholders::_1));

  station_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/station_pose", 1,
    std::bind(&StateUtils::stationPoseCallack, this, std::placeholders::_1));

  req_target_sub_ = node_->create_subscription<robot_custom_msgs::msg::Position>("/move_target", 10,
    std::bind(&StateUtils::target_callback, this, std::placeholders::_1));
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
  enableOdomcallback();
  bStartOdomReset = true;
  bOdomResetDone = false;
  odom_reset_cmd_.data = 0x01; // Odom IMU Reset
  reset_odom_pub_->publish(odom_reset_cmd_);
  reset_odom_start_time_ = node_->now().seconds();
  RCLCPP_ERROR(node_->get_logger(), "startMonitorOdomReset");
  odom_reset_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&StateUtils::monitor_resetOdom, this));
}


void StateUtils::monitor_resetOdom() {
  double runtime = node_->now().seconds() - reset_odom_start_time_;
  bool exitMonitor = false;
  if(bSendResetOdomCmd){
    RCLCPP_INFO(node_->get_logger(),"ToDo retry : ODOM RESET");
    reset_odom_start_time_ = node_->now().seconds();
    odom_reset_cmd_.data = 0x01; // Odom IMU Reset
    reset_odom_pub_->publish(odom_reset_cmd_);
    bSendResetOdomCmd = false;
    return;
  }

  if (isValidateResetOdom(odom_)) {
    bOdomResetDone = true;
    exitMonitor = true;
    RCLCPP_INFO(node_->get_logger(), "odom-reset Complete");
  } else if (runtime >= ODOM_RESET_TIMEOUT) {
    bSendResetOdomCmd = true;
    ++odom_reset_cnt_;
    RCLCPP_INFO(node_->get_logger(), "ToDo retry Count : %u",odom_reset_cnt_);
    if (odom_reset_cnt_ >= ODOE_RESET_RETRY_COUNT) {
      RCLCPP_INFO(node_->get_logger(), "ODOM RESET ERROR!!");
      odom_reset_cnt_ = 0;
      exitMonitor = true;
    }
    RCLCPP_INFO(node_->get_logger(),
                "ToDo retry : ODOM RESET Fail RunTime : %f, reset-count : %u ",
                runtime, odom_reset_cnt_);
    reset_odom_start_time_ = node_->now().seconds();
    odom_reset_cmd_.data = 0x00; // Odom IMU Reset
    reset_odom_pub_->publish(odom_reset_cmd_);
  }else{
     RCLCPP_INFO(node_->get_logger(),"ODOM Check RunTime : %f , X : %f, Y : %f, THETA : %f, STATUS : %u",
     runtime,odom_.x, odom_.y, odom_.theta,odom_status);
  }
  if (exitMonitor) {
    stopMonitorOdom();
  }
}

void StateUtils::stopMonitorOdom()
{
  disableOdomcallback();
  reset_timerResetOdom();
}

void StateUtils::reset_timerResetOdom() {
    if (odom_reset_timer_) {
      odom_reset_timer_.reset();
      RCLCPP_INFO(node_->get_logger(), "reset_timerResetOdom - end ");
    } else {
      RCLCPP_INFO(node_->get_logger(), "odom_reset_timer is allready reset ");
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
    disableLocalizationcallback();
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
  station_position_msg.pose = msg->pose;
  station_pose.x = static_cast<int>(msg->pose.pose.position.x * 1000000 + 0.5) / 1000000.0;
  station_pose.y = static_cast<int>(msg->pose.pose.position.y * 1000000 + 0.5) / 1000000.0 - 0.3;
  station_pose.theta = quaternion_to_euler(msg->pose.pose.orientation);
  RCLCPP_INFO(node_->get_logger(), "SET STATION POSE  %f | %f |%f ", station_pose.x, station_pose.y, station_pose.theta);
}

void StateUtils::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 현재 시간 가져오기
  rclcpp::Time current_time = msg->header.stamp;
  // RCLCPP_INFO(node_->get_logger(), "scan_callback");
  // 이전 시간과 비교하여 주기 계산
  if (scan_callback_time_.nanoseconds() != 0) {
      double period = (current_time - scan_callback_time_).seconds();
      double frequency = 1.0 / period;
      // RCLCPP_INFO(node_->get_logger(), "Scan Frequency: %.2f Hz", frequency);
      if (!msg->ranges.empty() && frequency >= 9) {
        if(!bLidarSensorOK){
          RCLCPP_INFO(node_->get_logger(), "Lidar Sensor is OK");
        }
        bLidarSensorOK = true;
      }else{
        if(msg->ranges.empty()){
          // RCLCPP_INFO(node_->get_logger(), "scan data is empty");
        }
        if(frequency < 9){
          // RCLCPP_INFO(node_->get_logger(), "scan frequency low : %.2f ",frequency);
        }
      }
  }
  // 현재 시간을 저장
  scan_callback_time_ = current_time;
}

void StateUtils::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    //RCLCPP_INFO(node_->get_logger(), "tofCallback");
    if(msg->bot_status == 0x00){
      if(!bMultiToFSensorOK){
        RCLCPP_INFO(node_->get_logger(), "tof Sensor is OK");
      }
      bMultiToFSensorOK = true;
    }
}

bool StateUtils::isStartLocalization()
{
    return bStartLocalizationStart; 
}

bool StateUtils::getLocalizationComplete()
{
    bool ret = false; // 임시로 true로 만듬 준연. 02/13
    if(bStartLocalizationStart && bLocalizationComplete){
      bLocalizationComplete = false;
      bStartLocalizationStart = false;
      ret = true;
    } 
    return ret;
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
  bOdomResetError = set;
}

bool StateUtils::isOdomREsetError()
{
  return bOdomResetError;
}

void StateUtils::publishLidarOn()
{
    std_msgs::msg::Bool lidar_cmd;
    lidar_cmd.data = true;
    lidar_cmd_pub_->publish(lidar_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishLidarOn");
}

void StateUtils::publishLidarOff()
{
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
}

void StateUtils::publishMultiTofOff()
{
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = false;
    tof_cmd_pub_->publish(tof_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishMultiTofOff");
}

void StateUtils::publishAllSensorOn()
{
    
    #if USE_TOF_ONOFF > 0
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = true;
    tof_cmd_pub_->publish(tof_cmd);
    #endif
    std_msgs::msg::Bool lidar_cmd;
    lidar_cmd.data = true;
    lidar_cmd_pub_->publish(lidar_cmd);
    enableSensorcallback();
    RCLCPP_INFO(node_->get_logger(), "publishAllSensorOn");
}

void StateUtils::publishAllSensorOff()
{
  #if USE_TOF_ONOFF > 0
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = false;
    tof_cmd_pub_->publish(tof_cmd);
    bMultiToFSensorOK = false;
    #endif

    std_msgs::msg::Bool lidar_cmd;
    lidar_cmd.data = false;
    lidar_cmd_pub_->publish(lidar_cmd);
    bLidarSensorOK = false;
    
    RCLCPP_INFO(node_->get_logger(), "publishAllSensorOff");
}

void StateUtils::publishClearCostMap()
{
    std_msgs::msg::Empty empty_msg;
    req_clear_costmap_pub_->publish(empty_msg);
    RCLCPP_INFO(node_->get_logger(), "publishClearCostMap");
}

void StateUtils::publishLocalizeInitPose()
{
    enableLocalizationcallback();
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    localize_start_time = node_->now().seconds();
    req_estimatePose_pub_->publish(station_position_msg); //station pose set enable
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeInitPose");
}

void StateUtils::publishLocalizePose()
{
    enableLocalizationcallback();
    bStartLocalizationStart = true;
    bLocalizationComplete = false;
    localize_start_time = node_->now().seconds();
    //req_estimatePose_pub_->publish(last_position_msg);
    req_estimatePose_pub_->publish(robot_position_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizePose X : %f , Y : %f, THETA : %f,", robot_pose.x,robot_pose.y,robot_pose.theta);
    //RCLCPP_INFO(node_->get_logger(), "publishLocalizePose X : %f , Y : %f, THETA : %f,", last_pose.x,last_pose.y,last_pose.theta);
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
      publishAllSensorOn();
    }else if( (id == NAVI_STATE::ARRIVED_GOAL || id == NAVI_STATE::PAUSE) && state_id != ROBOT_STATE::RETURN_CHARGER ){
      publishAllSensorOff();
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

bool StateUtils::isStartOnStation()
{
  return bStartOnStation;
}

bool StateUtils::isLidarSensorOK()
{
  return bLidarSensorOK;
}
bool StateUtils::isMultiToFSensorOK()
{
  return bMultiToFSensorOK;
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

bool StateUtils::isValidNavigation(const std::string &pidFilePath, double start_time) {
  std::vector<std::string> required_nodes = {
    "/amcl",
    "/behavior_server",
    "/bt_navigator",
    "/bt_navigator_navigate_to_pose_rclcpp_node",
    "/global_costmap/global_costmap",
    "/local_costmap/local_costmap",
    "/controller_server",
    "/planner_server",
    "/map_server",
    "/velocity_smoother",
    "/lifecycle_manager",
    "/nav2_container",
    "/smoother_server",
    "/velocity_smoother",
    "/waypoint_follower"
  }; // navi 필수 node.

  std::ifstream pidFile(pidFilePath);
  std::string pid;
  if (pidFile.is_open()) {
    std::getline(pidFile, pid);
    pidFile.close();
    if (!pid.empty()) {
      pid_t processGroupID = std::stoi(pid);
      if (kill(-processGroupID, 0) == 0) {
        // RCLCPP_INFO(node_->get_logger(), "[isValidProcess] Process running / %s", pidFilePath.c_str());

        std::vector<std::string> running_nodes = node_->get_node_names();
        for (const auto& required_node : required_nodes) {
          bool found = false;
          for (const auto& running_node : running_nodes) {
            if (running_node.find(required_node) != std::string::npos) {
              found = true;
              double wait_navi_launch = node_->now().seconds()-start_time;
              RCLCPP_INFO(node_->get_logger(), "Required Node launched [%s]: time %f", required_node.c_str(), wait_navi_launch);
              break;
            }
          }
          if (!found) {
            // RCLCPP_ERROR(node_->get_logger(), "Required Nav2 node not found: %s", required_node.c_str());
            return false;
          }
        }
      } else {
        // RCLCPP_ERROR(node_->get_logger(), "[isValidProcess] Process not running");
        return false;
      }
    }
  }
  return true;
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
      if (kill(-processGroupID, SIGINT) == 0) {
        sleep(1); // Wait for process to handle SIGINT
        if (kill(-processGroupID, 0) == -1) { // Process no longer exists
          RCLCPP_INFO(node_->get_logger(), "Process terminated successfully.");
          return true;
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "Process did not terminate, sending SIGKILL.");
          kill(-processGroupID, SIGKILL);
        }
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send SIGINT.");
      }
    }
  }
  return false;
}

void StateUtils::target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg) {
  setStartOnStation(false);
  if(!movingData.bStartMoving){
    movingData.bStartMoving = true;
    movingData.target_position.x = msg->x;
    movingData.target_position.y = msg->y;
    movingData.target_position.theta = msg->theta;
    
    RCLCPP_INFO(node_->get_logger(), "target_callback x : %f , y : %f, theta : %f ", movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
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

}