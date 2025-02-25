#include "state_manager/states/on_station.hpp"

namespace airbot_state {

OnStation::OnStation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  
}

void OnStation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[OnStation] > Preparing OnStation state");
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  //state_utils->publishLocalizeInitPose();
  #if USE_TOF_ONOFF > 0
  state_utils->publishMultiTofOff();
  #endif
  bLocalizationComplete = false;
  bSensorOffComplete = false;
}

void OnStation::run(const std::shared_ptr<StateUtils> &state_utils) {
  if(!bSensorOffComplete){
    state_utils->publishLidarOff();
    bSensorOffComplete = true;
  }
  
  if(!state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "Robot get out of Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::STOP_ONSTATION);
    req_robot_cmd_pub_->publish(req_state_msg);
  }
}

void OnStation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[OnStation] > Exiting %s state ", enumToString(state_utils->getStateID()).c_str());
  req_robot_cmd_pub_.reset();
}

} // namespace airbot_state
