#include "state_manager/states/on_station.hpp"

namespace airbot_state {

OnStation::OnStation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  
}

void OnStation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[OnStation] pre_run() -> Preparing onstation state");
  state_utils->enableArrivedGoalSensorsOffTimer();
  state_utils->stopDriving();
  state_utils->stopDocking();

  RCLCPP_INFO(node_->get_logger(), "[OnStation] odom reset start on station");
  state_utils->startMonitorOdomReset();

  if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
    state_utils->send_node_goal(NODE_STATUS::IDLE);
    bChkNode = true;
  }else{
    bChkNode = false;
  }

  #if USE_LIDAR_STATE_CHECK == 0
  state_utils->publishScanMonitor(false);
  #endif
  state_utils->publishTFMonitor(false);

  //hjkim : 사람이 충전기에 올려놓았을 때 복구 폴더 삭제해야함
  if(state_utils->getRecoveryRebootflag()){
    state_utils->removeRecoveryDirectory();
    state_utils->setRecoveryRebootflag(false);
    RCLCPP_INFO(node_->get_logger(),"[OnStation] Recovery Reboot Flag Clear");
  }

  state_utils->setCmdGlobalLocalizationMode(true);
}

void OnStation::run(const std::shared_ptr<StateUtils> &state_utils) {

  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[OnStation] run() -> Running onstation state");
  }
  
  if(state_utils->isReservedSaveMapFile()){
    state_utils->startSaveMapFile();
  }

  if(!state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[OnStation] Robot get out of Docking Station!!!");
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::STOP_ONSTATION);
  }

  if(bChkNode){
      int node_result = state_utils->getNodeClientStatus();
      if(node_result > 0){
        state_utils->setStatusID(ROBOT_STATUS::START);
        bChkNode = false;
      }else if(node_result < 0){
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
        bChkNode = false;
      }
  }else{
    if(state_utils->getStatusID() == ROBOT_STATUS::READY){
      state_utils->setStatusID(ROBOT_STATUS::START);
    }
  }
}

void OnStation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  state_utils->stopMonitorOdom();
  RCLCPP_INFO(node_->get_logger(), "[OnStation] post_run() -> Exiting %s state ", enumToString(state_utils->getStateID()).c_str());
}

} // namespace airbot_state