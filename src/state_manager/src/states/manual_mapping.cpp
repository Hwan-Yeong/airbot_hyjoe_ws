#include "state_manager/states/manual_mapping.hpp"

namespace airbot_state {

ManualMapping::ManualMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void ManualMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] pre_run() -> Preparing ManualMapping state");
  
  mapping_start_time = std::numeric_limits<double>::max();
  
  state_utils->startMonitorOdomReset();
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  state_utils->startSensorMonitor();
  state_utils->setStatusID(ROBOT_STATUS::READY);
  setReadyMapping(READY_MAPPING::CHECK_SENSOR);
}

void ManualMapping::setReadyMapping(READY_MAPPING set)
{
  if(ready_mapping != set){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] setReadyMapping %s", enumToString(set).c_str());
  }
  ready_mapping = set;
}

void ManualMapping::run(const std::shared_ptr<StateUtils> &state_utils) {
  ROBOT_STATUS ready_check;
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] run() -> running ManualMapping state");
  }
  auto req_state_msg = std_msgs::msg::UInt8();

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    ready_check = processMappingReady();
    state_utils->setStatusID(ready_check);
    break;
  case ROBOT_STATUS::START :
    break;
  case ROBOT_STATUS::PAUSE :
    break;
  case ROBOT_STATUS::COMPLETE :
    req_state_msg.data = int(REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING);
    req_robot_cmd_pub_->publish(req_state_msg);
    break;
  case ROBOT_STATUS::FAIL :
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] mapping fail");
    break;      
  default:
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Running UnKown Status");
    break;
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping]Robot on Docking Station!!!");
    state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
  }
}

void ManualMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->saveLastPosition();
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] post_run() -> Exiting ManualMapping state");
  req_robot_cmd_pub_.reset();
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
  map_saver();
  state_utils->send_node_goal(NODE_STATUS::IDLE);
}

///////////////function in ManualMapping

ROBOT_STATUS ManualMapping::processMappingReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  switch (ready_mapping)
  {
  case READY_MAPPING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyMapping(READY_MAPPING::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] lidar Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] tof Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] CameraError");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }
    break;     
  case READY_MAPPING::CHECK_ODOM_RESET :
    if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] odom reset Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyMapping(READY_MAPPING::FAIL);
    }
    else if(resetOdomChecker()){
      setReadyMapping(READY_MAPPING::LAUNCH_NODE);
    }
    break;
  case READY_MAPPING::LAUNCH_NODE :
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Mapping Node Start Goal Send");
    state_utils->send_node_goal(NODE_STATUS::MANUAL_MAPPING);
    setReadyMapping(READY_MAPPING::CHECK_NODE_LAUNCH);
    break;
  case READY_MAPPING::CHECK_NODE_LAUNCH :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] ManualMapping Node Launched");
      setReadyMapping(READY_MAPPING::COMPLETE);
      ret = ROBOT_STATUS::START;
    }else if(node_result < 0){
      setReadyMapping(READY_MAPPING::FAIL);
      ret = ROBOT_STATUS::FAIL;
    }
    break;   
  default:
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] processMappingReady readyState Error!! : %d", static_cast<int>(ready_mapping));
    break;
  }

  return ret;
}

bool ManualMapping::resetOdomChecker()
{
  bool ret = false;
  if(!state_utils->isStartOdomReset()){
    RCLCPP_INFO(node_->get_logger(), "[Navigation] skip odom reset Checker ");
    ret = true;
  }
  else if(state_utils->getOdomResetDone()){
    RCLCPP_INFO(node_->get_logger(), "[Navigation] odom reset Done ");
    ret = true;
  } 

  return ret;
}

void ManualMapping::map_saver() {
  int map_save_result = std::system(
      "ros2 run nav2_map_server map_saver_cli -f /home/airbot/app_rw/map/airbot_map_00 "
      "--ros-args -p save_map_timeout:=10.0");
  int map_save_cnt = 0;

  while (map_save_result != 0) {
    if (map_save_cnt++ < 10) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[ManualMapping] Map save command failed with error code, trying again: %d",
                   map_save_result);
      map_save_result = std::system(
          "ros2 run nav2_map_server map_saver_cli -f /home/airbot/app_rw/map/airbot_map_00 "
          "--ros-args -p save_map_timeout:=10.0");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[ManualMapping] Map save command failed over 10times");
      break;
    }
  }
}

} // namespace airbot_state
