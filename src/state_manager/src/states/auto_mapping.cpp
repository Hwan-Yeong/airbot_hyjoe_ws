#include "state_manager/states/auto_mapping.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

AutoMapping::AutoMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void AutoMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Preparing auto_mapping state");

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);  
  mapping_start_time = std::numeric_limits<double>::max();
  state_utils->startMonitorOdomReset();
  state_utils->startSensorMonitor();
  state_utils->setStatusID(ROBOT_STATUS::READY);
  setReadyMapping(READY_MAPPING::CHECK_SENSOR);
}

void AutoMapping::run(const std::shared_ptr<StateUtils> &state_utils) {
  ROBOT_STATUS ready_check;
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] run() -> running AutoMapping state");
  }
  auto req_state_msg = std_msgs::msg::UInt8();

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    ready_check = processMappingReady();
    state_utils->setStatusID(ready_check);
    if(ready_check == ROBOT_STATUS::START){
      state_utils->publishSenSorManagerOn();
      state_utils->publishManeuverOn();
      state_utils->publishMappingOn();
    }
    break;
  case ROBOT_STATUS::START :
    break;
  case ROBOT_STATUS::PAUSE :
    break;
  case ROBOT_STATUS::COMPLETE :
    req_state_msg.data = int(REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING);
    req_robot_cmd_pub_->publish(req_state_msg);
    break;
  case ROBOT_STATUS::FAIL :
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] mapping fail");
    break;      
  default:
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Running UnKown Status");
    break;
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping]Robot on Docking Station!!!");
    state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
  }
}

void AutoMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] post_run() -> exiting auto_mapping state");
  state_utils->saveLastPosition();
  if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_WORKING ){
    map_saver();
    state_utils->send_node_goal(NODE_STATUS::IDLE);
  }
  state_utils->publishSenSorManagerOff();
  state_utils->publishManeuverOff();
  state_utils->publishMappingOff();
  req_robot_cmd_pub_.reset();
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
}

///////////////function in AutoMapping
void AutoMapping::runAutoMapping() {
  reset_subExploreFinish();
  // ******Auto mapping Launch files******
  explore_finish_sub = node_->create_subscription<std_msgs::msg::Empty>(
      "/explore_finish", 10,
      std::bind(&AutoMapping::explore_finish_callback, this,
                std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "AutoMapping Run");
    
}

ROBOT_STATUS AutoMapping::processMappingReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  switch (ready_mapping)
  {
  case READY_MAPPING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyMapping(READY_MAPPING::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] lidar Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] tof Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] CameraError");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }
    break;     
  case READY_MAPPING::CHECK_ODOM_RESET :
    if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] odom reset Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }
    else if(resetOdomChecker()){
      setReadyMapping(READY_MAPPING::LAUNCH_NODE);
    }
    break;
  case READY_MAPPING::LAUNCH_NODE :
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Mapping Node Start Goal Send");
    state_utils->send_node_goal(NODE_STATUS::AUTO_MAPPING);
    setReadyMapping(READY_MAPPING::CHECK_NODE_LAUNCH);
    break;
  case READY_MAPPING::CHECK_NODE_LAUNCH :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] AutoMapping Node Launched");
      setReadyMapping(READY_MAPPING::COMPLETE);
      runAutoMapping();
      ret = ROBOT_STATUS::START;
    }else if(node_result < 0){
      setReadyMapping(READY_MAPPING::FAIL);
      ret = ROBOT_STATUS::FAIL;
    }
    break;   
  default:
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] processMappingReady readyState Error!! : %d", static_cast<int>(ready_mapping));
    break;
  }

  return ret;
}

void AutoMapping::setReadyMapping(READY_MAPPING set)
{
  if(ready_mapping != set){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] setReadyMapping %s", enumToString(set).c_str());
  }
  ready_mapping = set;
}

void AutoMapping::explore_finish_callback(const std_msgs::msg::Empty::SharedPtr) {
    // finish generate change state msg
    auto robot_cmd_msg = std_msgs::msg::UInt8();
    robot_cmd_msg.data = int( REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING );
    req_robot_cmd_pub_->publish(robot_cmd_msg);
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Explore mapping completed ");
    reset_subExploreFinish();
}

void AutoMapping::reset_subExploreFinish() {
    if (explore_finish_sub) {
      explore_finish_sub.reset();
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] reset_subExploreFinish");
    } else {
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] explore_finish_sub is allready reset ");
    }
}

bool AutoMapping::resetOdomChecker()
{
  bool ret = false;
  if(!state_utils->isStartOdomReset()){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] skip odom reset Checker ");
    ret = true;
  }
  else if(state_utils->getOdomResetDone()){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] odom reset Done ");
    ret = true;
  } 

  return ret;
}

void AutoMapping::map_saver() {
  int map_save_result = std::system(
      "ros2 run nav2_map_server map_saver_cli -f /home/airbot/app_rw/map/airbot_map_00 "
      "--ros-args -p save_map_timeout:=10.0");
  int map_save_cnt = 0;

  while (map_save_result != 0) {
    if (map_save_cnt++ < 10) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[AutoMapping] Map save command failed with error code, trying again: %d",
                   map_save_result);
      map_save_result = std::system(
          "ros2 run nav2_map_server map_saver_cli -f /home/airbot/app_rw/map/airbot_map_00 "
          "--ros-args -p save_map_timeout:=10.0");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[AutoMapping] Map save command failed over 10times");
      break;
    }
  }
}

} // namespace airbot_state
