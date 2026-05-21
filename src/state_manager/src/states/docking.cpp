#include "state_manager/states/docking.hpp"
#define DOCKING_TIMEOUT 1200

namespace airbot_state {

Docking::Docking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils) : stateBase(actionID, node, utils) {
}

void Docking::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Docking] pre_run() -> Preparing docking state");
  docking_error_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/error/s_code/unable_to_dock",10);
  pause_docking = false;
  if(state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::OFFSTATION_TRY_DOCKING){
    bRedockingReady = false;
    RCLCPP_INFO(node_->get_logger(), "[Docking] offstation try docking check odom-reset");
  }else{
    bRedockingReady = true;
  }
  bSetBaseOdom = false;
  state_utils->startOdomCheck();
}

void Docking::run(const std::shared_ptr<StateUtils> &) {
  stateBase::run(state_utils);
  ROBOT_STATUS current_id = state_utils->getStatusID();
  
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Docking] run() -> Running docking state");
  }

  if(current_id != ROBOT_STATUS::COMPLETE && state_utils->getOnstationStatus() ){
    RCLCPP_INFO(node_->get_logger(), "[Docking] detected on Docking Station!!!");
    state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
    return;
  }

  if(!bSetBaseOdom && state_utils->getPrepareOdomFlag()){
    baseOdom = state_utils->getCurrentOdom();
    tempOdom = baseOdom;
    bSetBaseOdom = true;
  }

  if(bSetBaseOdom){
    pose currentOdom = state_utils->getCurrentOdom();
    double diff_pose_x = (currentOdom.x - tempOdom.x);
    double diff_pose_y = (currentOdom.y - tempOdom.y);
    double diff_pose_theta = state_utils->getAngle(tempOdom.theta,currentOdom.theta);
    double distance = state_utils->getDistance(baseOdom, currentOdom);
    double angle = state_utils->getAngle(baseOdom.theta, currentOdom.theta);

    if(fabs(diff_pose_x) > 0.1 || fabs(diff_pose_y) > 0.1 || fabs(diff_pose_theta) > 0.05236/*3deg*/ ){
      pose robot = state_utils->getRobotPose();
      RCLCPP_INFO(node_->get_logger(), "[checkDocking] detect moving dist[%.2f](M) angle[%.2f](DEG) / odom(%.2f, %.2f, %.2f(deg))/ robot(%.2f, %.2f, %.2f(deg))",
      distance,RAD2DEG(angle),currentOdom.x, currentOdom.y, RAD2DEG(currentOdom.theta),robot.x, robot.y, RAD2DEG(robot.theta));
      tempOdom = currentOdom;
    }
  }

  switch (current_id)
  {
  case ROBOT_STATUS::READY :
    if(state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::OFFSTATION_TRY_DOCKING){
      if(!state_utils->isStartOdomReset()){
        RCLCPP_INFO(node_->get_logger(), "[Docking] odom-reset not be started");
        state_utils->startMonitorOdomReset(); //hjkim : in case odom-reset not be started(unkwon issue) -> odom-reset start
      }else if(state_utils->isOdomResetError()){
        RCLCPP_INFO(node_->get_logger(), "[Docking] offstation try docking odom reset error");
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
      }else if(state_utils->getOdomResetDone()){
        bRedockingReady = true;
        RCLCPP_INFO(node_->get_logger(), "[Docking] offstation try docking odom reset ok");
      }
    }
    if( state_utils->getStopDrivingFlag() && bRedockingReady){
      if(!state_utils->getReservePause()){ //reserve pause 아닐때. docking start
        state_utils->startDocking();
        start_time = std::chrono::steady_clock::now();
        state_utils->setStatusID(ROBOT_STATUS::START);
      }else{ //docking pause
        state_utils->setStatusID(ROBOT_STATUS::PAUSE);
      }
    }
    break;
  case ROBOT_STATUS::START :
    if( pause_docking ){
      pause_docking = false;
      state_utils->setStatusID(ROBOT_STATUS::READY);
    } else{
      runTime =  state_utils->getSteadyClockRunningSeconds(start_time);
      if(runTime >= DOCKING_TIMEOUT){
        RCLCPP_INFO(node_->get_logger(), "[Docking] docking Timeout Fail runtime : %.2f, timeout[%d]sec", runTime, DOCKING_TIMEOUT);
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
      }
    }
    break;
  case ROBOT_STATUS::PAUSE :
    if(!pause_docking){
      state_utils->stopDocking();
      pause_docking = true;
    }
    break;
  case ROBOT_STATUS::COMPLETE :
    state_utils->stopDocking();
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::DONE_DOCKING);
    break;
  case ROBOT_STATUS::FAIL :
    state_utils->stopDocking();
    publishDockingError();
    break;      
  default:
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Running UNKNOWN Status");
    break;
  }
  
}

void Docking::post_run(const std::shared_ptr<StateUtils> &) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Docking] post_run() -> Exiting docking state");
  state_utils->stopDocking();
}

void Docking::publishDockingError()
{
  RCLCPP_ERROR(node_->get_logger(), "[Docking] publishDockingError() -> publish docking error to error_manager ");
  std_msgs::msg::Bool msg;
  msg.data = true;
  docking_error_pub_->publish(msg);
}
} // namespace airbot_state