#include "state_manager/states/undocking.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

UnDocking::UnDocking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
: stateBase(actionID, node, utils) {
}

void UnDocking::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  state_utils->setStartOnStation(true);
  RCLCPP_INFO(node_->get_logger(), "[UnDocking] pre_run() -> Preparing UnDocking state");
  state_utils->startOdomCheck();
}

void UnDocking::run(const std::shared_ptr<StateUtils> &state_utils) {
 
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[UnDocking] run() -> Running UnDocking state");
  }

  ROBOT_STATUS ready_check;
  switch (state_utils->getStatusID()) {
  case ROBOT_STATUS::READY:
    if( state_utils->getPrepareOdomFlag() ){
      enableLinearTargetMoving();
      ready_check = ROBOT_STATUS::START;
      state_utils->setStatusID(ready_check);
    }else{
      RCLCPP_INFO(node_->get_logger(), "[Undocking] wait odom_ready");
    }
    break;
  case ROBOT_STATUS::START :
    if(end_linear_target){
      ready_check = ROBOT_STATUS::COMPLETE;
      state_utils->setStatusID(ready_check);
    }  
    break;

  case ROBOT_STATUS::PAUSE :
    RCLCPP_INFO(node_->get_logger(), "[Undocking] Running PAUSE Status");
    break;

  case ROBOT_STATUS::COMPLETE :    
    state_utils->setCmdGlobalLocalizationMode(false);

    if(linear_target_timer_)
    {
      RCLCPP_INFO(node_->get_logger(), "[Undocking] close linear target timer");
      linear_target_timer_.reset();
    }

    if(state_utils->getReserveDocking()){ //undocking완료 후 reserve 도킹 상태 확인하여 분기 전환.
      RCLCPP_INFO(node_->get_logger(), "[Undocking] Reserved Docking!!! state docking transition in progress");
      state_utils->setReserveDocking(false);
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::OFFSTATION_TRY_DOCKING);
    } else{
      if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_AUTO_MAPPING){
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING);
      }else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_MANUAL_MAPPING){
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING);
      }else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_NAVIGATION){
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION);
      } else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_FACTORY_NAVIGATION){
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FACTORY_NAVIGATION);
      } else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_FOLLOWING){
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FOLLOWING);
      } else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_MANUAL_CONTROL){
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_CONTROL);
      }
    }

    break;

  case ROBOT_STATUS::FAIL :
    break;

  default:
    RCLCPP_INFO(node_->get_logger(), "[Undocking] Running UNKNOWN Status");
    break;
  }
}

void UnDocking::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[UnDocking] post_run() -> Exiting UnDocking state");
  stateBase::post_run(state_utils);
  state_utils->stopDriving();
  if(linear_target_timer_)
  {
    linear_target_timer_.reset();
  }
  state_utils->startMonitorOdomReset();
}

void UnDocking::enableLinearTargetMoving()
{
    end_linear_target = false;
    base_odom = state_utils->getCurrentOdom();
    if(!linear_target_timer_){
        linear_target_timer_ = node_->create_wall_timer( std::chrono::milliseconds(20), std::bind(&UnDocking::processLinearMoving, this));
        RCLCPP_INFO(node_->get_logger(), "[UnDocking] enableLinearTargetMoving");
    }else{
      RCLCPP_INFO(node_->get_logger(), "[UnDocking] LinearTargetMoving is already enabled!!");
    }
}
void UnDocking::processLinearMoving()
{
    pose current = state_utils->getCurrentOdom();
    double distance = state_utils->getDistance(base_odom,current);
    double target_distance = state_utils->getUndockingDistance();
    if(distance >= target_distance)
    {
        end_linear_target = true;
        state_utils->publishVelocityCommand(0.0,0.0);
        RCLCPP_INFO(node_->get_logger(), "[UnDocking] moving 0.3m over distance : %f , base X : %f, Y : %f, current X : %f, Y : %f ",
        distance,base_odom.x,base_odom.y,current.x,current.y);
    }else{
        //RCLCPP_WARN(node_->get_logger(), "processLinearMoving...distance : %f",distance);
        state_utils->publishVelocityCommand(0.07,0.0);
    }
}

} // namespace airbot_state