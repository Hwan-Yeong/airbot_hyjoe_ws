#include "state_manager/states/manual_mapping.hpp"

namespace airbot_state {

ManualMapping::ManualMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void ManualMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] pre_run() -> Preparing ManualMapping state");
  
  mapping_start_time = std::numeric_limits<double>::max();

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
    exitMappingNode();
  }else if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI){
    exitNavigationNode();
  }

  state_utils->startMonitorOdomReset();
  state_utils->startSensorMonitor();
}

void ManualMapping::run(const std::shared_ptr<StateUtils> &state_utils) {

  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] run() -> running ManualMapping state");
  }

  if(state_utils->isSensorReady())
  {
    if (state_utils->getOdomResetDone()){
      runManualMapping();
    }else if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Preparing OdomResetError");
    }
  }else if(state_utils->isLidarError()){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Preparing LidarError");
  }else if(state_utils->isToFError()){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Preparing ToFError");
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping]Robot on Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING);
    req_robot_cmd_pub_->publish(req_state_msg);
  }
}

void ManualMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->saveLastPosition();
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] post_run() -> Exiting ManualMapping state");
  req_robot_cmd_pub_.reset();
  state_utils->stopMonitorOdom();
  map_saver();
  exitMappingNode();
}

///////////////function in ManualMapping
void ManualMapping::runManualMapping() {
  RCLCPP_INFO(node_->get_logger(), "Manual Mapping Start");
  if (state_utils->startProcess("ros2 launch airbot_slam slam.launch.py",
                   "/home/airbot/mapping_pid.txt")) {
    mapping_start_time = node_->now().seconds();
    state_utils->setNodeStatusID( NODE_STATUS::MANUAL_MAPPING );
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] ManualMapping Node Launch");
    // stopMonitorOdom(); // Stop monitoring Odom status

  } else {
    // reqStatus = REQUEST_STATUS::FAIL;
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] ManualMapping Node launch Fail");
  }
}

void ManualMapping::exitMappingNode() {
  if (state_utils->stopProcess("/home/airbot/mapping_pid.txt")) {
    state_utils->setNodeStatusID( NODE_STATUS::IDLE );
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] exit Mapping Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Fail - kill Mapping Node");
    state_utils->setNodeStatusID( NODE_STATUS::IDLE );
  }
  // rclcpp::Rate waitStop(1000);
  // waitStop.sleep();
}
void ManualMapping::exitNavigationNode()
{
    if(state_utils->stopProcess("/home/airbot/navigation_pid.txt")){
        RCLCPP_INFO(node_->get_logger(), "[ManualMapping] exit Navigation Node");
    }else{
        RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Fail - kill Navigation Node");
    }
    // rclcpp::Rate waitStop(1000);
    // waitStop.sleep();
    
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
