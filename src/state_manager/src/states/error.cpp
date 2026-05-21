#include "state_manager/states/error.hpp"

namespace airbot_state {

Error::Error(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils){
}

void Error::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Error] pre_run() -> Preparing error state");

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  state_utils->stopDriving();
  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  state_utils->enableArrivedGoalSensorsOffTimer();
  state_utils->enableManeuverCommand(false);
  if(state_utils->getOnstationStatus()){
    error_onstation = true;
  }
}

void Error::run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::run(state_utils);

  if(state_utils->getOnstationStatus()){
    if( !error_onstation ){ //도킹 위에서 에러가 난 경우, 도킹에서 한번 뺐다가 다시 올리면 해제.

      RCLCPP_INFO(node_->get_logger(), "[Error] Robot get on Docking Station!!!");
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
    }
  } else {
    error_onstation = false;
  }

  switch (state_utils->getStatusID()) {
    case ROBOT_STATUS::READY:
      sendStopCMD();
      RCLCPP_INFO(node_->get_logger(), "[Error] run() -> Running error state");
      state_utils->setStatusID(ROBOT_STATUS::START);
      break;
    case ROBOT_STATUS::START:
      if( state_utils->getRotatePauseFlag() ){
        state_utils->setMovingStateID(NAVI_STATE::START_ROTAION);
        state_utils->setRotatePauseFlag(false);
      }
      break;
    case ROBOT_STATUS::PAUSE:
      state_utils->setMovingStateID(NAVI_STATE::PAUSE);
      if( !state_utils->getRotatePauseFlag() && 
          state_utils->getMovingStateID() == NAVI_STATE::START_ROTAION){
        state_utils->setRotatePauseFlag(true);
      }
      break;
    case ROBOT_STATUS::COMPLETE:
      break;
    case ROBOT_STATUS::FAIL:
      break;
    default:
      RCLCPP_INFO(node_->get_logger(), "[Error] ERROR UNKNOWN Status");
      break;
  }

}

void Error::post_run(const std::shared_ptr<StateUtils> &) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Error] post_run() -> Exiting error state");
  cmd_vel_pub_.reset();

  if(state_utils->getPreStateID() == ROBOT_STATE::AUTO_MAPPING && state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_AUTO_MAPPING){
    RCLCPP_INFO(node_->get_logger(), "[Error] set MappingResumeToErrorFlag for resume auto-mapping");
    state_utils->setMappingResumeToErrorFlag(true);
  }
}

void Error::sendStopCMD() {
  auto cmd_msg = geometry_msgs::msg::Twist(); //state들에서 동작중에 취소하면 동작 유지되어 정지 cmd전달.
  cmd_msg.linear.x = 0.0;
  cmd_msg.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_msg);
}

} // namespace airbot_state