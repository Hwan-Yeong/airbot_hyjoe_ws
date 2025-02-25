#include "state_manager/states/idle.hpp"

namespace airbot_state {

Idle::Idle(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils),first_booting(true) {
}

void Idle::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Idle] Preparing IDLE STATE");

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  state_utils->publishAllSensorOff();

}

void Idle::run(const std::shared_ptr<StateUtils> &state_utils) {
  
  if (first_booting) {
    first_running = false;
    req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  }

  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Idle] RUN > Idle STATE");
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "Robot get on Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
    req_robot_cmd_pub_->publish(req_state_msg);
  }
}

void Idle::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[Idle] > POST_RUN Exiting %s state ", enumToString(state_utils->getStateID()).c_str());
  req_robot_cmd_pub_.reset();
}

} // namespace airbot_state
