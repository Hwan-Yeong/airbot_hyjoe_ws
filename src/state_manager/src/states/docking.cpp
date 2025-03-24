#include "state_manager/states/docking.hpp"
#define DOCKING_TIMEOUT 120
namespace airbot_state {

Docking::Docking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils) : stateBase(actionID, node, utils) {
}

void Docking::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Docking] pre_run() -> Preparing docking state");

  dock_pub = node_->create_publisher<std_msgs::msg::UInt8>("/docking_cmd", 10);
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  docking_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/error/s_code/unable_to_dock",10);
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>("/station_data", 10, std::bind(&Docking::stationData_callback, this, std::placeholders::_1));
  start_time = node_->now().seconds();
  startDocking(); 
  if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {
    state_utils->send_node_goal(NODE_STATUS::IDLE);
  }
}

void Docking::run(const std::shared_ptr<StateUtils> &) {
  double runTime = node_->now().seconds()-start_time;
  
  if(runTime >= DOCKING_TIMEOUT){
    publishDockingError();
  }
  
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Docking] run() -> Running docking state");
  }
  
}

void Docking::post_run(const std::shared_ptr<StateUtils> &) {
  RCLCPP_INFO(node_->get_logger(), "[Docking] post_run() -> Exiting docking state");
  stopDocking();
  dock_pub.reset();
  station_data_sub.reset();
  req_robot_cmd_pub_.reset();
}

void Docking::startDocking() {
  dock_cmd_.data = DOCK_START;
  RCLCPP_INFO(node_->get_logger(), "[Docking] publish Start-Docking");
  dock_pub->publish(dock_cmd_);
}

void Docking::stopDocking() {
  dock_cmd_.data = DOCK_STOP;
  RCLCPP_INFO(node_->get_logger(), "[Docking] publish Stop-Docking");
  dock_pub->publish(dock_cmd_);
}

void Docking::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
  try {

    docking_status = msg->docking_status;
    if (docking_status & 0x80) { // docking error
      if (!bDockingError) {
        RCLCPP_INFO(node_->get_logger(), "[Docking] Docking-Fail from MCU");
        bDockingError = true;
      }
    } else {
      if (docking_status & 0x70) { // charging
          if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI && state_utils->getMovingStateID() != NAVI_STATE::ARRIVED_GOAL){
            state_utils->setMovingStateID(NAVI_STATE::ARRIVED_GOAL);
          }
        
        RCLCPP_INFO(node_->get_logger(),"[Docking] Robot Status Charging, Docking Status %u", docking_status);
        stopDocking();
        auto docking_done_msg = std_msgs::msg::UInt8();
        docking_done_msg.data = int(REQUEST_ROBOT_CMD::DONE_DOCKING);
        req_robot_cmd_pub_->publish(docking_done_msg);
      }
      bDockingError = false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}

void Docking::publishDockingError()
{
  RCLCPP_ERROR(node_->get_logger(), "[Docking] publishDockingError() -> publish docking error to error_manager ");
  std_msgs::msg::Bool msg;
  msg.data = true;
  docking_error_pub_->publish(msg);
}

} // namespace airbot_state
