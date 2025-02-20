#include "state_manager/states/on_station.hpp"

namespace airbot_state {

OnStation::OnStation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  
}

void OnStation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[OnStation] > Preparing OnStation state");
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>("/station_data", 10, std::bind(&OnStation::stationData_callback, this, std::placeholders::_1));
  //state_utils->publishLocalizeEmpty();
  //state_utils->publishAllSensorOff();
  //state_utils->publishLocalizePose();
  state_utils->publishMultiTofOff();
  bLocalizationComplete = false;
  bSensorOffComplete = false;
}

void OnStation::run(const std::shared_ptr<StateUtils> &state_utils) {
  // if(bLocalizationComplete){
  //   if((state_utils->getLocalizationComplete()) || (!state_utils->isStartLocalization())){
  //     bLocalizationComplete = true;
  //   }
  // }

  // if(bLocalizationComplete && !bSensorOffComplete){
  //   state_utils->publishAllSensorOff();
  //   bSensorOffComplete = true;
  // }
    if(!bSensorOffComplete){
      state_utils->publishAllSensorOff();
      bSensorOffComplete = true;
    }
      
  }

void OnStation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[OnStation] > Exiting OnStation state ");
  station_data_sub.reset();
  req_robot_cmd_pub_.reset();
}

void OnStation::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
    
    bool bOnStation = static_cast<bool>(msg->docking_status & 0x70);
    
    if(!bOnStation){
      RCLCPP_INFO(node_->get_logger(), "Robot get out of Docking Station!!!");
      auto req_state_msg = std_msgs::msg::UInt8();
      req_state_msg.data = int(REQUEST_ROBOT_CMD::STOP_ONSTATION);
      req_robot_cmd_pub_->publish(req_state_msg);
    }
}

} // namespace airbot_state
