#include "state_manager/utils/state_utils.hpp"

#define ODOM_RESET_TIMEOUT 5.0

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

  req_clear_costmap_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/clear/costmap", 1);
  req_nomotion_local_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/localization/request", 1);
  req_estimatePose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/request/pose", 1);
  reset_odom_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/odom_imu_reset_cmd", 1);
  lidar_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_lidar", 10);
  tof_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/cmd_tof", 10);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                std::bind(&StateUtils::odom_callback, this, std::placeholders::_1));
  odom_status_sub_ = node_->create_subscription<std_msgs::msg::UInt8>("/odom_status", 1,
                std::bind(&StateUtils::odom_status_callback, this, std::placeholders::_1));

  localize_complete_sub = node_->create_subscription<std_msgs::msg::Empty>("/localization/complete", 1,
      std::bind(&StateUtils::localizationComplete_callback, this, std::placeholders::_1));
  
  initial_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1,
    std::bind(&StateUtils::initial_pose_callback, this, std::placeholders::_1));

  amcl_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
      std::bind(&StateUtils::amclCallback, this, std::placeholders::_1));
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
  bStartOdomReset = true;
  bOdomResetDone = false;
  try {
    odom_reset_cmd_.data = 0x01; // Odom IMU Reset
    reset_odom_pub_->publish(odom_reset_cmd_);
    reset_odom_start_time_ = node_->now().seconds();
    RCLCPP_ERROR(node_->get_logger(), "startMonitorOdomReset");
    odom_reset_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&StateUtils::monitor_resetOdom, this));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
  // Set up a timer to periodically check the status
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
    if (++odom_reset_cnt_ >= 254) {
      RCLCPP_INFO(node_->get_logger(), "ToDo retry Over 255");
      odom_reset_cnt_ = 0;
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

void StateUtils::stopMonitorOdom() { reset_timerResetOdom(); }

void StateUtils::reset_timerResetOdom() {
  try {
    // std::lock_guard<std::mutex> lock(odom_reset_timer_mutex_);
    if (odom_reset_timer_) {
      RCLCPP_INFO(node_->get_logger(), "reset_timerResetOdom");
      odom_reset_timer_.reset();
      RCLCPP_INFO(node_->get_logger(), "reset_timerResetOdom - end ");
    } else {
      RCLCPP_INFO(node_->get_logger(), "odom_reset_timer is allready reset ");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
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
  try {
    odom_.x = msg->pose.pose.position.x;
    odom_.y = msg->pose.pose.position.y;
    odom_.theta = quaternion_to_euler(msg->pose.pose.orientation);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}

void StateUtils::odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    odom_status = msg->data;
}

void StateUtils::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "initial_pose_callback");
}

void StateUtils::localizationComplete_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    if(!bLocalizationComplete){
      RCLCPP_INFO(node_->get_logger(), "localizationComplete_callback");
    }
    bLocalizationComplete = true;
}

void StateUtils::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  amcl.header = msg->header;
  amcl.pose = msg->pose;
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
    std_msgs::msg::Bool lidar_cmd;
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = true;
    tof_cmd_pub_->publish(tof_cmd);
    lidar_cmd.data = true;
    lidar_cmd_pub_->publish(lidar_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishAllSensorOn");
}

void StateUtils::publishAllSensorOff()
{
    std_msgs::msg::Bool lidar_cmd;
    std_msgs::msg::Bool tof_cmd;
    tof_cmd.data = false;
    tof_cmd_pub_->publish(tof_cmd);
    lidar_cmd.data = false;
    lidar_cmd_pub_->publish(lidar_cmd);
    RCLCPP_INFO(node_->get_logger(), "publishAllSensorOff");
}

void StateUtils::publishClearCostMap()
{
    std_msgs::msg::Empty empty_msg;
    req_clear_costmap_pub_->publish(empty_msg);
    RCLCPP_INFO(node_->get_logger(), "publishClearCostMap");
}

void StateUtils::publishLocalizeEmpty()
{
    bStartLocalizationStart = true;
    localize_start_time = node_->now().seconds();
    std_msgs::msg::Empty empty_msg;
    req_nomotion_local_pub_->publish(empty_msg);
    RCLCPP_INFO(node_->get_logger(), "publishLocalizeEmpty");
}

void StateUtils::publishLocalizePose()
{
    bStartLocalizationStart = true;
    localize_start_time = node_->now().seconds();
    double x = static_cast<int>(amcl.pose.pose.position.x * 1000000 + 0.5) / 1000000.0;
    double y = static_cast<int>(amcl.pose.pose.position.y * 1000000 + 0.5) / 1000000.0;
    double theta = quaternion_to_euler(amcl.pose.pose.orientation);
    req_estimatePose_pub_->publish(amcl);
    //RCLCPP_INFO(node_->get_logger(), "publishLocalizePose X : %f , Y : %f, THETA : %f," x,y,theta);
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
    if(id == NAVI_STATE::MOVE_GOAL){
      publishMultiTofOn();
    }else{
      publishMultiTofOff();
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
}