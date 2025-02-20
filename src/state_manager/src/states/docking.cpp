#include "state_manager/states/docking.hpp"

namespace airbot_state {

Docking::Docking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils) : stateBase(actionID, node, utils) {
}

void Docking::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Docking] Preparing Docking STATE");

  dock_pub = node_->create_publisher<std_msgs::msg::UInt8>("/docking_cmd", 10);
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>("/station_data", 10, std::bind(&Docking::stationData_callback, this, std::placeholders::_1));
  startDocking();
  state_utils->publishMultiTofOff();
}

void Docking::run(const std::shared_ptr<StateUtils> &state_utils) {
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Docking] Running Docking with shared data: ");
  }
  
}

void Docking::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[Docking] Exiting Docking");
  dock_cmd_.data = DOCK_STOP;
  dock_pub->publish(dock_cmd_);
  dock_pub.reset();
  station_data_sub.reset();
  req_robot_cmd_pub_.reset();
}

void Docking::startDocking() {
  dock_cmd_.data = DOCK_START;
  RCLCPP_INFO(node_->get_logger(), "Start-Docking");
  dock_pub->publish(dock_cmd_);
}

void Docking::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
  try {

    docking_status = msg->docking_status;
    if (docking_status & 0x80) { // docking error
      if (!bDockingError) {
        // if(robotStatus == ROBOT_STATUS::DOCKING){
        //     RCLCPP_INFO(node_->get_logger(), "Docking Error -  ToDo retry
        //     Code");
        // }else{
        //     RCLCPP_INFO(node_->get_logger(), "Robot is Not Docking But
        //     DockingError");
        // }
        bDockingError = true;
      }
    } else {
      if (docking_status & 0x70) { // charging

        if (docking_status &
            0x60) { // update 2d pose in charging or Charge Complete state
          // Publish only once when docking starts
          if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI && state_utils->getMovingStateID() != NAVI_STATE::ARRIVED_GOAL){
                state_utils->setMovingStateID(NAVI_STATE::ARRIVED_GOAL);
          }
        } else {
          // if(++charging_cnt >= 100){
          //     charging_cnt = 0;
          //     //chargingStartStop(true);
          //     RCLCPP_INFO(node_->get_logger(), "retry charging start docking
          //     status : %d ", docking_status);
          // }
        }
        dock_cmd_.data = DOCK_STOP;
        dock_pub->publish(dock_cmd_);
        RCLCPP_INFO(node_->get_logger(),
                    "Robot Status Charging, Docking Status %u", docking_status);

        auto docking_done_msg = std_msgs::msg::UInt8();
        docking_done_msg.data = int(REQUEST_ROBOT_CMD::DONE_DOCKING);
        req_robot_cmd_pub_->publish(docking_done_msg);
        // }
      } else { // not - charging

        // Reset the flag when not in docking state
        // if(robotStatus == ROBOT_STATUS::CHARGING){
        //     robotStatus = ROBOT_STATUS::IDLE;
        //     // charge_cmd.data= CHARGE_STOP;
        //     // charge_pub->publish(charge_cmd);
        //     RCLCPP_INFO(node_->get_logger(), "UNDOCK BY ?? - Robot Status
        //     IDLE, Docking Status %d",docking_status);
        // }else if(robotStatus == ROBOT_STATUS::DOCKING && docking_status ==
        // 0x00){ //docking_status
        //     dock_cmd_.data= DOCK_START;
        //     dock_pub->publish(dock_cmd_);
        //     RCLCPP_INFO(node_->get_logger(), "Docking Command not Working
        //     Retrying");
        // }
      }
      bDockingError = false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}

} // namespace airbot_state
