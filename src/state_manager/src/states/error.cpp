#include "state_manager/states/error.hpp"

namespace airbot_state {

Error::Error(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils){
}

void Error::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Error] pre_run() -> Preparing error state");

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  
  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  state_utils->stopSensorMonitor();
  state_utils->publishAllSensorOff();
  if(state_utils->getOnstationStatus()){
    error_onstation = true;
  }
}

void Error::run(const std::shared_ptr<StateUtils> &state_utils) {
  if(state_utils->getStatusID() == ROBOT_STATUS::READY){
    if( exitAllNode() ){
      RCLCPP_INFO(node_->get_logger(), "[Error] run() -> Running error state");
      state_utils->setStatusID(ROBOT_STATUS::START);
    }
    sendStopCMD();
  }
  
  if(state_utils->getOnstationStatus()){
    if( !error_onstation ){ //도킹 위에서 에러가 난 경우, 도킹에서 한번 뺐다가 다시 올리면 해제.

      RCLCPP_INFO(node_->get_logger(), "[Error] Robot get on Docking Station!!!");
      auto req_state_msg = std_msgs::msg::UInt8();
      req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
      req_robot_cmd_pub_->publish(req_state_msg);
    }
  } else {
    error_onstation = false;
  }
}

void Error::post_run(const std::shared_ptr<StateUtils> &) {
  RCLCPP_INFO(node_->get_logger(), "[Error] post_run() -> Exiting error state");
  req_robot_cmd_pub_.reset();
  cmd_vel_pub_.reset();
}

bool Error::exitAllNode() {
  if (state_utils->stopProcess("/home/airbot/mapping_pid.txt") && state_utils->stopProcess("/home/airbot/navigation_pid.txt")) {
    RCLCPP_INFO(node_->get_logger(), "[Error] exit Mapping Node");
    return true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "[Error] Fail - kill Mapping Node");
  }  
  state_utils->setNodeStatusID(NODE_STATUS::IDLE);
  return false;
}

void Error::sendStopCMD() {
  auto cmd_msg = geometry_msgs::msg::Twist(); //state들에서 동작중에 취소하면 동작 유지되어 정지 cmd전달.
  cmd_msg.linear.x = 0.0;
  cmd_msg.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_msg);
}

} // namespace airbot_state
