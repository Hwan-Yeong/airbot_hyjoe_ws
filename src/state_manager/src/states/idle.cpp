#include "state_manager/states/idle.hpp"

#define REDOCKING_ONOFF 1
#define REDOCKING_DISTANCE_PARAM 0.1 // 10cm
#define REDOCKING_ANGLE_PARAM 0.05236 //3deg
#define WAIT_REDOCKING_TIME 30 //sec
#define RETRY_DOCKING_CNT 10 // CNT * 100ms = 1sec
namespace airbot_state {

Idle::Idle(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils),first_booting(true) {
}

void Idle::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Idle] pre_run() -> Preparing idle state");
  state_utils->enableArrivedGoalSensorsOffTimer();
  state_utils->stopDriving();
  
  if(!(state_utils->getNodeStatusID() == NODE_STATUS::NAVI || state_utils->getNodeStatusID() == NODE_STATUS::FT_NAVI)){
    state_utils->send_node_goal(NODE_STATUS::IDLE);
    bChkNode = true;
  }else{
    bChkNode = false;
  }

  if(state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::STOP_ONSTATION){
    if(state_utils->isInpectionMode()){
      bChkRedocking = false;
      RCLCPP_INFO(node_->get_logger(), "[Idle] INSPECTION MODE[%d] OFFSTATION DETECT BUT BLOCK RE-DOCKING", state_utils->isInpectionMode());
    }else{
      #if REDOCKING_ONOFF > 0
      RCLCPP_INFO(node_->get_logger(), "[Idle] OFFSTATION DETECT START-CHECK RE-DOCKING");
      bChkRedocking = true;
      bSetBaseOdom = false;
      state_utils->startOdomCheck();
      try_docking_starttime = std::chrono::steady_clock::now();
      #else
      RCLCPP_INFO(node_->get_logger(), "[Idle] OFFSTATION DETECT BUT BLOCK RE-DOCKING");
      bChkRedocking = false;
      #endif
    }
  }else{
    bChkRedocking = false;
  }
  #if USE_LIDAR_STATE_CHECK == 0
  state_utils->publishScanMonitor(false);
  #endif
  state_utils->publishTFMonitor(false);
}

void Idle::run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::run(state_utils);
  if (first_booting) {
    stateBase::pre_run(state_utils);
    RCLCPP_INFO(node_->get_logger(), "[Idle] run() -> running Idle state");
    state_utils->setStatusID(ROBOT_STATUS::READY);
    first_booting = false;
    state_utils->publishKeepOutEnable(false);
    state_utils->publishSenSorManagerOff();
  }

  if( isFirstRunning() ){
    //hjkim : state_manager respawn 시 다른 노드들과 상태동기화를 위해 부팅 후 idle 상태에서 로봇 상태 메세지 전송을 위해 추가
    state_utils->publishRobotState(ROBOT_STATE::IDLE, ROBOT_STATUS::READY);
    RCLCPP_INFO(node_->get_logger(), "[Idle] run() -> running Idle state");
  }

  switch (state_utils->getStatusID()) {
    case ROBOT_STATUS::READY:
      /*hjkim : kill node check*/
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
      /*hjkim : kill node check*/
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
      RCLCPP_INFO(node_->get_logger(), "[IDLE] Running UNKNOWN Status");
      break;
  }


  
   //상시 체크사항
  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[Idle] Robot get on Docking Station!!!");
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
    return;
  }

  /*hjkim : offstation re-docking check*/
  if(bChkRedocking){
    double runtime = state_utils->getSteadyClockRunningSeconds(try_docking_starttime);
    pose currentOdom = state_utils->getCurrentOdom();

    if(!bSetBaseOdom && state_utils->getPrepareOdomFlag()){
      baseOdom = currentOdom;
      bSetBaseOdom = true;
      RCLCPP_INFO(node_->get_logger(), "[checkRedocking] BASE ODOM SET OK x,y,theta[%.2f, %.2f, %.2f]", baseOdom.x, baseOdom.y, RAD2DEG(baseOdom.theta));
    }

    if(bSetBaseOdom){
      double diff_pose_x = (currentOdom.x - tempOdom.x);
      double diff_pose_y = (currentOdom.y - tempOdom.y);
      double diff_pose_theta = state_utils->getAngle(tempOdom.theta,currentOdom.theta);
      double distance = state_utils->getDistance(baseOdom, currentOdom);
      double angle = state_utils->getAngle(baseOdom.theta, currentOdom.theta);

      if(fabs(diff_pose_x) > REDOCKING_DISTANCE_PARAM || fabs(diff_pose_y) > REDOCKING_DISTANCE_PARAM || fabs(diff_pose_theta) > REDOCKING_ANGLE_PARAM){
        RCLCPP_INFO(node_->get_logger(), "[checkRedocking] detect moving dist[%.2f](M) angle[%.2f](DEG) / odom(%.3f, %.3f, %.1f(deg))",
        distance,RAD2DEG(angle),currentOdom.x, currentOdom.y, RAD2DEG(currentOdom.theta));
        tempOdom = currentOdom;
      }
    }

    if(runtime >= WAIT_REDOCKING_TIME){
      bChkRedocking = false;
      try_docking_cnt = 0;
      if(bSetBaseOdom){
        double distance = state_utils->getDistance(baseOdom, currentOdom);
        double angle = state_utils->getAngle(baseOdom.theta, currentOdom.theta);
        if(distance <= REDOCKING_DISTANCE_PARAM && fabs(angle) <= REDOCKING_ANGLE_PARAM){
          state_utils->setCheckTryDocking(true);
          RCLCPP_INFO(node_->get_logger(), "[checkRedocking] runtime[%.2f] START CHECK RE-DOCKING moving dist[%.2f](M) angle[%.2f](DEG) FROM STATION base[%.2f, %.2f, %.2f] current[%.2f, %.2f, %.2f] ",
          runtime,distance,RAD2DEG(angle),baseOdom.x, baseOdom.y, RAD2DEG(baseOdom.theta), currentOdom.x, currentOdom.y, RAD2DEG(currentOdom.theta));
        }else{
          RCLCPP_INFO(node_->get_logger(), "[checkRedocking] runtime[%.2f] CAN`T NOT RE-DOCKING OVER moving dist[%.2f](M) angle[%.2f](DEG) FROM STATION base[%.2f, %.2f, %.2f] current[%.2f, %.2f, %.2f] ",
          runtime,distance,RAD2DEG(angle),baseOdom.x, baseOdom.y, RAD2DEG(baseOdom.theta),currentOdom.x, currentOdom.y, RAD2DEG(currentOdom.theta));
        }
      }else{
        RCLCPP_INFO(node_->get_logger(), "[checkRedocking] runtime[%.2f] BASE ODOM NOT SET NEED TO CHECK ODOM CALLBACK", runtime);
      }
    }
  }
  /*hjkim : offstation re-docking check*/

  /*hjkim : try docking*/
  if(state_utils->getCheckTryDocking()){
    #if REDOCKING_ONOFF > 0
    if(tryDockingChecker()){
      state_utils->setCheckTryDocking(false);
      RCLCPP_INFO(node_->get_logger(), "[checkRedocking] CheckTryDocking End");
    }
    #else
    state_utils->setCheckTryDocking(false);
    RCLCPP_INFO(node_->get_logger(), "[checkRedocking] RE-DOCKING IS BLOCKED flag off");
    #endif
  }
  /*hjkim : try docking*/
}

void Idle::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->setCheckTryDocking(false);
  RCLCPP_INFO(node_->get_logger(), "[Idle] post_run() -> exiting %s state ", enumToString(state_utils->getStateID()).c_str());
  state_utils->stopDriving();
}

bool Idle::tryDockingChecker(){
  bool ret = false;
  bool bCheckCenterReceiver = false;
  bool bCheckCenterSignal = false;
  uint8_t receiver = state_utils->getStationReceiverStatus();
  uint8_t short_signal = state_utils->getStationShortSignal();

  if(receiver & (RECEIVER_LEFT_CENTER_DOCK_SIGNAL | RECEIVER_RIGHT_CENTER_DOCK_SIGNAL)){
    bCheckCenterReceiver = true;
  }
  
  if(short_signal & (SHORT_SIG_LEFT_CENTER|SHORT_SIG_CENTER|SHORT_SIG_RIGHT_CENTER)){
    bCheckCenterSignal = true;
  }
  RCLCPP_INFO(node_->get_logger(), "[Idle] CheckTryDocking check Short Signal receiver[%02x] short_signal[%02x]", receiver,short_signal);
    
  if(bCheckCenterReceiver && bCheckCenterSignal)
  {
    if(++try_docking_cnt > RETRY_DOCKING_CNT)
    {
      state_utils->startMonitorOdomReset();
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::OFFSTATION_TRY_DOCKING);  
      try_docking_cnt = 0;
      ret = true;
      RCLCPP_INFO(node_->get_logger(), "[Idle] CheckTryDocking START TRY-DOCKING");
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[Idle] CheckTryDocking No short_signal -> try_docking_cnt[%d]", try_docking_cnt);
    try_docking_cnt = 0;
    ret = true;
  }
 
  return ret;
}

} // namespace airbot_state