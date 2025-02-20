#include "state_manager/states/idle.hpp"

namespace airbot_state {

Idle::Idle(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils),first_booting(true) {
}

void Idle::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Idle] Preparing IDLE STATE");

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>(
      "/station_data", 10,
      std::bind(&Idle::stationData_callback, this, std::placeholders::_1));
  
      state_utils->publishMultiTofOff();

}

void Idle::run(const std::shared_ptr<StateUtils> &state_utils) {
  
  if (first_booting) {
    first_running = false;
    req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
    station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>(
        "/station_data", 10,
        std::bind(&Idle::stationData_callback, this, std::placeholders::_1));
  }

  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Idle] RUN > Idle STATE");
  }
}

void Idle::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[Idle] POST_RUN > Idle state");
  station_data_sub.reset();
  req_robot_cmd_pub_.reset();
}

void Idle::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
  try {
    docking_status = msg->docking_status;
    if (docking_status & 0x70) { // charging
      if (docking_status & 0x60) {
      RCLCPP_INFO(node_->get_logger(), "Robot Status Charging, Docking Status %u", docking_status);}
      auto req_state_msg = std_msgs::msg::UInt8();
      req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
      req_robot_cmd_pub_->publish(req_state_msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}

} // namespace airbot_state
