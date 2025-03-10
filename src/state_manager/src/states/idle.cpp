#include "state_manager/states/idle.hpp"

namespace airbot_state {

Idle::Idle(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils),first_booting(true) {
}

void Idle::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Idle] pre_run() -> Preparing idle state");

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  state_utils->publishAllSensorOff();
}

void Idle::run(const std::shared_ptr<StateUtils> &state_utils) {
  if (first_booting) {
    RCLCPP_INFO(node_->get_logger(), "[Idle] run() -> running Idle state");
    req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
    first_booting = false;
  }

  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Idle] run() -> running Idle state");
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[Idle] Robot get on Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
    req_robot_cmd_pub_->publish(req_state_msg);
  }
}

void Idle::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[Idle] post_run() -> exiting %s state ", enumToString(state_utils->getStateID()).c_str());
  req_robot_cmd_pub_.reset();
}

} // namespace airbot_state
