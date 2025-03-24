#include "state_manager/states/on_station.hpp"

namespace airbot_state {

OnStation::OnStation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  
}

void OnStation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[OnStation] pre_run() -> Preparing onstation state");
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  state_utils->enableArrivedGoalSensorsOffTimer();
  state_utils->publishSenSorManagerOff();
  state_utils->publishManeuverOff();
  state_utils->setStatusID(ROBOT_STATUS::READY);
}

void OnStation::run(const std::shared_ptr<StateUtils> &state_utils) {
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[OnStation] run() -> Running onstation state");
  }
  
  if(!state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[OnStation] Robot get out of Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::STOP_ONSTATION);
    req_robot_cmd_pub_->publish(req_state_msg);
  }

  if(state_utils->getStatusID() == ROBOT_STATUS::READY){
    if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
      int node_result = state_utils->getNodeClientStatus();
      if(node_result > 0){
        state_utils->setStatusID(ROBOT_STATUS::START);
      }else if(node_result < 0){
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
      }
    }
  }
}

void OnStation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[OnStation] post_run() -> Exiting %s state ", enumToString(state_utils->getStateID()).c_str());
  req_robot_cmd_pub_.reset();
}

} // namespace airbot_state
