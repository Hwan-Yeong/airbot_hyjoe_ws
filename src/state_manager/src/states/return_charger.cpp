#include "state_manager/states/return_charger.hpp"
// #include "state_manager/states/state_base.hpp"

#define USE_JSLLOC 1

namespace airbot_state {

ReturnCharger::ReturnCharger(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {

}

void ReturnCharger::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(),"[ReturnCharger] Preparing ReturnCharger STATE");
  bNaviNodeRecovery = false;
  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  state_utils->setReadyMoving(READY_MOVING::IDLE);
  setReadyNavigation(READY_NAVIGATION::CHECK_SENSOR);
  return_pose = state_utils->getInitPose();
  state_utils->startSensorMonitor();
  state_utils->setMovingPauseFlag(false);
  state_utils->cancelPreviousGoal();
  state_utils->resetTryMoveTargetCount();
  altgoal_retry_count = 0; //hjkim : 상태 전환 시 대체목적지 재시도 count 초기화
}

void ReturnCharger::setReadyNavigation(READY_NAVIGATION set)
{
  if(readyNavi != set){
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] setReadyNavigation %s", enumToString(set).c_str());
  }
  readyNavi = set;
}

void ReturnCharger::run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::run(state_utils);
  ROBOT_STATUS ready_check;
  
  if(state_utils->getStatusID() != ROBOT_STATUS::READY){
    if(state_utils->getOnstationStatus()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Robot get on Docking Station!!!");
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
      state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
      return;
    }

    if(bNaviNodeRecovery){
      if(!state_utils->getNavibringUpRecoveryPause()){
        bNaviNodeRecovery = false;
        state_utils->publishEmergencyStop(false);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Node Recovery Finished!");
      }
      state_utils->publishVelocityCommand(0.0,0.0);
      return;
    }

    if(state_utils->getMoveChargerFlag() && !state_utils->getSensorRecoveryPause() && !state_utils->getNavibringUpRecoveryPause())
    {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] MoveChargerFlag is true");
      altgoal_retry_count = 0; //hjkim : 이미 복귀 중 복귀명령 다시 내려오는 경우 대체목적지  재시도 count 초기화
      state_utils->setMovingStateID(NAVI_STATE::READY);
      state_utils->setStatusID(ROBOT_STATUS::START);
      state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
    }else if(state_utils->getNavibringUpRecoveryPause()){
      bNaviNodeRecovery = true;
      if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI || state_utils->getNodeStatusID() == NODE_STATUS::FT_NAVI){
        state_utils->reserveMapLoadatferNavRecovery();
      }
      state_utils->publishLifeCycleOff();
      state_utils->publishEmergencyStop(true);
      state_utils->setMovingStateID(NAVI_STATE::PAUSE);
      state_utils->stopDriving();
      return;
    }
  }

  if(state_utils->isNeedToRunMapLoadAfterRecovery()){
    int map_type = 0; //default map (airbot_map)
    if(state_utils->getNodeStatusID() == NODE_STATUS::FT_NAVI){
      map_type = 1; //factory map
    }
    state_utils->mapLoadAfterNavRecoveryProcess(map_type);
    return;
  }

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    if( state_utils->getStopDrivingFlag() ){
      ready_check = processNavigationReady();
      state_utils->setStatusID(ready_check);
    }
    break;
  case ROBOT_STATUS::START :
    if(state_utils->getMovingStateID() == NAVI_STATE::MOVE_GOAL && state_utils->getSensorRecoveryPause()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Go to pause due to Sensor Recovery");
      state_utils->setStatusID(ROBOT_STATUS::RECOVERY_PAUSE);
    }
    else if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE || state_utils->getMovingPauseFlag()){
      if(state_utils->getMovingPauseFlag()){
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] MoveChargerFlag is true");
      }else{
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] NAVI STATE PAUSE--> STATE[%s]", enumToString(state_utils->getMovingStateID()).c_str());
      }
	  
      if( state_utils->getStopDrivingFlag() ){ // stop 확인 후 resume동작.
        state_utils->setMovingPauseFlag(false);
        state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
        state_utils->setMovingStateID(NAVI_STATE::READY); //resume
      } else{
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] RESUME return_charger but.. Wait Stop Driving");
      }
    }

    if( state_utils->getStopDrivingFlag() && state_utils->getMovingStateID() == NAVI_STATE::READY){
      processMoveTarget();
    }
    break;
  case ROBOT_STATUS::PAUSE :
    if (state_utils->getMovingStateID() != NAVI_STATE::PAUSE) {
      state_utils->pauseNavigation();
    }else if(state_utils->getMovingStateID() == NAVI_STATE::READY){
      state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
      state_utils->setStatusID( ROBOT_STATUS::START);
    }
    break;
  case ROBOT_STATUS::COMPLETE :
    break;
  case ROBOT_STATUS::FAIL :
    break;      
  case ROBOT_STATUS::RECOVERY_PAUSE :
    if (state_utils->getMovingStateID() != NAVI_STATE::PAUSE) {
      state_utils->pauseNavigation();
    } else if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE ){
      if(!state_utils->getSensorRecoveryPause()){
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Sensor Recovery Finished!");
        state_utils->setStatusID(ROBOT_STATUS::START);
      }
    }
    break; 
  default:
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Running UNKNOWN Status");
    break;
  }
}

void ReturnCharger::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->publishEmergencyStop(false);
  state_utils->stopDriving();
  RCLCPP_INFO(node_->get_logger(),"[ReturnCharger] Exiting ReturnCharger STATE");
  state_utils->setMovingPauseFlag(false);
  stopMonitorReturnCharger();
  state_utils->publishLifeCycleOff();
  if(state_utils->isStartLocalization()){
    state_utils->stopLocalizationMonitor();
  }

  //hyjoe : return charger 중 정지명령 or 도킹명령(시퀀스상 내려올 일 없음)시 map 저장 안하도록 주석 처리
  // if (state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_WORKING || state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_DOCKING) {
  //   if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING ||
  //       state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {
  //     state_utils->map_saver();
  //   }else{
  //     RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] post_run mapping not save");
  //   }
  // }
}

ROBOT_STATUS ReturnCharger::processNavigationReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  switch (readyNavi)
  {
  case READY_NAVIGATION::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      #if USE_JSLLOC > 0
      if(!(state_utils->getNodeStatusID() == NODE_STATUS::NAVI || state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING)){
        setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
      }else{
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }
      #else
      if(state_utils->getRecoveryRebootflag()){
        setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Recoevery_return_charger Navigation Node On");
      }else{
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }
      #endif
      }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "lidar error");
      ret = ROBOT_STATUS::FAIL;
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "tof error");
      ret = ROBOT_STATUS::FAIL;
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "camera Error");
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      #if USE_JSLLOC > 0
      if(!(state_utils->getNodeStatusID() == NODE_STATUS::NAVI || state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING)){
        setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
      }else{
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }
      #else
      if(state_utils->getRecoveryRebootflag()){
        setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Recoevery_return_charger Navigation Node On");
      }else{
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }
      #endif
      
      //ret = ROBOT_STATUS::FAIL;
    }
    break;
  case READY_NAVIGATION::LAUNCH_NODE :
    state_utils->send_node_goal(NODE_STATUS::NAVI);
    setReadyNavigation(READY_NAVIGATION::CHECK_NODE);
    break;
  case READY_NAVIGATION::CHECK_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
#if USE_JSLLOC > 0
      local_mode_retry_cnt = 0;
      state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
      setReadyNavigation(READY_NAVIGATION::START_LOCALIZATION);
      localization_start_time = std::chrono::steady_clock::now();
#else
      setReadyNavigation(READY_NAVIGATION::COMPLETE);
#endif
    }else if(node_result < 0){
      setReadyNavigation(READY_NAVIGATION::STOP_NODE);
    }
    break;
  case READY_NAVIGATION::STOP_NODE :
    state_utils->send_node_goal(NODE_STATUS::IDLE);
    setReadyNavigation(READY_NAVIGATION::CHECK_STOP_NODE);
    break;
  case READY_NAVIGATION::CHECK_STOP_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
    }else if(node_result < 0){
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }
    break;
  case READY_NAVIGATION::START_LOCALIZATION :
    if(state_utils->getLocalizationMode() != LOCALIZATION_MODE::VOID)
    {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] complete localization mode change cmd[%s] current-mode[%s] ",
      state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      setReadyNavigation(READY_NAVIGATION::COMPLETE);
    }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
      if(++local_mode_retry_cnt >= 3){
        local_mode_retry_cnt = 0;
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] retry 3 times localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }else{
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        localization_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] retry localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }
    }
    break;
  case READY_NAVIGATION::COMPLETE :
    state_utils->setMovingStateID(NAVI_STATE::READY);
    if( state_utils->getReservePause() ){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Go to PAUSE STATUS due to pause reservation!" );
      ret = ROBOT_STATUS::PAUSE; //reserve pause 인 경우 Pause진입.
    } else{
      state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
      ret = ROBOT_STATUS::START;
    }
    break;
  case READY_NAVIGATION::FAIL :
    ret = ROBOT_STATUS::FAIL;
    break;    
  default:
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] processNavigationReady readyState Error!! : %d", static_cast<int>(readyNavi));
    break;
  }

  return ret;
}

int8_t ReturnCharger::localizationChecker()
{
  int8_t ret = 0;
  if(state_utils->isStartLocalization()){
    // double wait_localize_time = node_->now().seconds()-state_utils->getLocalizationStartTime();
    if(state_utils->getLocalizationComplete()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] localization Done ");
      ret = 1;
    }else if(state_utils->isLocalizationError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] localization Error ");
      ret = -1;
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] skip localization");
    ret = 1;
  }
  return ret;
}

void ReturnCharger::processMoveTarget()
{  
  int localize_result = 0;
  
  switch (state_utils->getReadyMoving())
  {
  case READY_MOVING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Auto Mapping -> skip localization node[%s]", enumToString(state_utils->getNodeStatusID()).c_str());
        state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
      }else{
        if(state_utils->isSkipLocalization()){
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Sensor is Already On Skip Localization");
          state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
        }else{
          #if USE_JSLLOC > 0
          local_mode_retry_cnt = 0;
          state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
          state_utils->setReadyMoving(READY_MOVING::CHECK_LOCALIZATION_MODE);
          localization_start_time = std::chrono::steady_clock::now();
          #else
          state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
          #endif
        }
      }
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] lidar Error");
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] tof Error");
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] camera Error");
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Auto Mapping -> skip localization node[%s]", enumToString(state_utils->getNodeStatusID()).c_str());
        state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
      }else{
        if(state_utils->isSkipLocalization()){
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Sensor is Already On Skip Localization");
          state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
        }else{
          #if USE_JSLLOC > 0
          local_mode_retry_cnt = 0;
          state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
          state_utils->setReadyMoving(READY_MOVING::CHECK_LOCALIZATION_MODE);
          localization_start_time = std::chrono::steady_clock::now();
          #else
          state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
          #endif
        }
      }
      //state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }
    break;
  #if USE_JSLLOC > 0
  case READY_MOVING::CHECK_LOCALIZATION_MODE :
    if(state_utils->getLocalizationMode() != LOCALIZATION_MODE::VOID)
    {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] complete localization mode change cmd[%s] current-mode[%s] ",
      state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
    }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
      if(++local_mode_retry_cnt >= 3){
        local_mode_retry_cnt = 0;
        state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] retry 3 times localization mode change cmd[%s] current-mode[%s] ", 
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }else{
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        localization_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] retry localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }
    }
   break;
  #endif
  case READY_MOVING::REQUEST_POSE_ESTIMATE :
    state_utils->setCheckAmclAfterLocalization();
    if(state_utils->getRecoveryRebootflag()){
      state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::RECOVERY_POSE);
    }else{
      state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
    }
    state_utils->setReadyMoving(READY_MOVING::CHECK_POSE_ESTIMATE);
    break;
  case READY_MOVING::CHECK_POSE_ESTIMATE :
    localize_result = localizationChecker();
    if(localize_result > 0){
      //hjkim : 로컬 완료 시 복구 폴더 삭제 ( 로컬을 위한 데이터 이므로 복구 후 로컬완료하면 해당 데이터는 더이상 필요 없음) : 로컬 하지 못했는데 충전기 인식되는 상태는 on station 에서 삭제함.
      if(state_utils->getRecoveryRebootflag()){
        state_utils->removeRecoveryDirectory();
        state_utils->setRecoveryRebootflag(false);
        RCLCPP_INFO(node_->get_logger(),"[ReturnCharger] Recovery Reboot Flag Clear");
      }
      if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI){
        state_utils->publishKeepOutEnable(true);
        keepout_enable_cnt = 0;
        state_utils->setReadyMoving(READY_MOVING::CHECK_KEEPOUT_STATE);
      }else{
        state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
      }
      
    }
    else if(localize_result < 0){
      if( detectDockShortSig() ){ //local fail이지만 short sig 감지시 docking전환.
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Return charger [localization] fail!!!-> but detect dock short signal");
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING);
        state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
      } else{
        state_utils->publishLocalizationFailErrorStatus(true);
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
      }
    }
    break;
    case READY_MOVING::CHECK_KEEPOUT_STATE :
      if (state_utils->getKeepoutState() == KEEPOUT_STATE::KEEPOUT_ENABLE) {
        state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
      } else if (state_utils->getSteadyClockRunningSeconds(state_utils->getKeepOutEnableStartTime()) > 3) {
        if (keepout_enable_cnt++ < 3) {
          state_utils->publishKeepOutEnable(true);
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] KeepOut Enable retry %d", keepout_enable_cnt);
        } else {
          state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] KeepOut Enable Fail 3 Times");
        }
      }
      break;
    case READY_MOVING::REQUEST_MANEUVER_ON :
      state_utils->enableManeuverCommand(true);
      state_utils->setReadyMoving(READY_MOVING::CHECK_MANEUVER);
    break;
    case READY_MOVING::CHECK_MANEUVER :
      if(state_utils->isManeuverDone()){
        state_utils->publishClearCostMap();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        state_utils->publishSenSorManagerOn();
        state_utils->setReadyMoving(READY_MOVING::CHECK_PREVIOUS_GOAL);
      }else if(state_utils->isManeuverCommunicateError() || state_utils->isPerceptionCommunicateError()){
        if(state_utils->isManeuverCommunicateError()){
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Maneuver Communication Error");
        }else{
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Perception Communication Error");
        }
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
        return;
      }
    break;
  case READY_MOVING::CHECK_PREVIOUS_GOAL :
  //hjkim 250725 : check goal cancel 에 대한 추가적임 검토 필요. ( nav2 action client 통합 버전 적용 이전으로 되돌림)
    state_utils->setReadyMoving(READY_MOVING::SEND_GOAL);
    break;	
  case READY_MOVING::SEND_GOAL :
    startMonitorReturnCharger();
    state_utils->moveToTarget(return_pose.x, return_pose.y,return_pose.theta);
    state_utils->setReadyMoving(READY_MOVING::COMPLETE);
  break;  
  case READY_MOVING::COMPLETE :
    break;
  default:
    break;
  };
}

void ReturnCharger::stopMonitorReturnCharger() { reset_timerNaviStatus(); }

void ReturnCharger::startMonitorReturnCharger() {
  if(!nav_status_timer_){
    nav_status_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ReturnCharger::monitor_returnCharger, this));
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] create returnChargerMonitor");
  }else{
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] returnChargerMonitor is allready running");
  }
}

void ReturnCharger::monitor_returnCharger() {
  NODE_STATUS node_status = state_utils->getNodeStatusID();
  NAVI_STATE navi_state = state_utils->getMovingStateID();
  if (navi_state == NAVI_STATE::ARRIVED_GOAL) {
    if ((node_status == NODE_STATUS::AUTO_MAPPING && state_utils->isAutoMappingComplete()) || node_status == NODE_STATUS::MANUAL_MAPPING) {
      state_utils->map_saver();
    }else{
      RCLCPP_INFO(node_->get_logger(), "[monitor_returnCharger] ARRIVED_GOAL mapping not save NODE_STATUS : %s",enumToString(node_status).c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "[monitor_returnCharger] Proceed with docking.");
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER);
    dock_pose_estimate = false;
    state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
    stopMonitorReturnCharger();
  }else if(navi_state == NAVI_STATE::ALTERNATE_GOAL){
    pose robot = state_utils->getRobotPose();
    double distance = state_utils->getDistance(robot,return_pose);
    if(distance < 0.65){ //250903 KKS : 1st alternative_goal is changed in 0.3m
      if ((node_status == NODE_STATUS::AUTO_MAPPING && state_utils->isAutoMappingComplete()) || node_status == NODE_STATUS::MANUAL_MAPPING) {
        state_utils->map_saver();
      }
      state_utils->resetTryMoveTargetCount();
      state_utils->stopDriving();
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER);
      state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
      RCLCPP_INFO(node_->get_logger(), "[monitor_returnCharger] ALTERNATE_GOAL ARRIVED --> START DOCKING robot(%.2f, %.2f) target(%.2f, %.2f) distance : %f",robot.x,robot.y,return_pose.x,return_pose.y,distance);
      stopMonitorReturnCharger();
    }else{
      if(altgoal_retry_count >= 3){
          RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Return charger [altgoal retry] fail!!!-> but detect dock short signal");
          //hjkim 251126 : 도킹에서 라이다 센서를 이용한 충돌방지 로직을 적용하며, 멀리서 도킹을 시도를 하더라도 충돌없이 충전기로 들어갈 수 있도록 개선 -> 복귀 실패 시 도킹으로 전환하도록 수정.
          if ((node_status == NODE_STATUS::AUTO_MAPPING && state_utils->isAutoMappingComplete()) || node_status == NODE_STATUS::MANUAL_MAPPING) {
            state_utils->map_saver();
          }
          state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING); //short signal이 들어오면 retry count초과여도 docking으로 전환.
          state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
          #if 0
          if( detectDockShortSig() ){
            RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Return charger [altgoal retry] fail!!!-> but detect dock short signal");
            if ((node_status == NODE_STATUS::AUTO_MAPPING && state_utils->isAutoMappingComplete()) || node_status == NODE_STATUS::MANUAL_MAPPING) {
              state_utils->map_saver();
            }
            state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING); //short signal이 들어오면 retry count초과여도 docking으로 전환.
            state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
          } else{ //short sinal이 없으면 정상 에러 처리.
            //hjkim : 복귀 실패 시 stopDriving 누락으로 인한 saveLastPose 실행 안되어, 다음 이동 명령 시 localization 위치 오차 발생
            state_utils->resetTryMoveTargetCount();
            state_utils->stopDriving();
            state_utils->setSkipLocalization(false);
            state_utils->publishMoveFailError();
            state_utils->setStatusID(ROBOT_STATUS::FAIL);
            if(state_utils->isReservedSaveMapFile()){
              state_utils->startSaveMapFile();
            }
            RCLCPP_INFO(node_->get_logger(), "[monitor_returnCharger] ALTERNATE_GOAL can`t move to dock retry 3 times over try-count[%d] robot(%.2f, %.2f) target(%.2f, %.2f) distance : %.2f",
            altgoal_retry_count,robot.x,robot.y,return_pose.x,return_pose.y,distance);
            altgoal_retry_count = 0;
            stopMonitorReturnCharger();
          }
          #endif
      }else{
        state_utils->setMovingStateID(NAVI_STATE::READY);
        state_utils->publishClearCostMap();
        state_utils->moveToTarget(return_pose.x, return_pose.y,return_pose.theta);
        RCLCPP_INFO(node_->get_logger(), "[monitor_returnCharger] ALTERNATE_GOAL retry move to dock try-count[%d] robot(%.2f, %.2f) target(%.2f, %.2f) distance : %.2f",
        altgoal_retry_count,robot.x,robot.y,return_pose.x,return_pose.y,distance);
      }
      altgoal_retry_count++;
    }
  }else if (navi_state == NAVI_STATE::FAIL){
    //hyjoe : Navigation Fail 시 map 저장 안하도록 주석 처리
    // // Goal failed or aborted after retry 3 times
    // if (node_status == NODE_STATUS::AUTO_MAPPING || node_status == NODE_STATUS::MANUAL_MAPPING) {
    //   state_utils->map_saver();
    // }else{
    //   RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] NAVI_STATE::FAIL mapping not save NODE_STATUS : %s",enumToString(node_status).c_str());
    // }
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] NAVI_STATE::FAIL mapping not save NODE_STATUS : %s",enumToString(node_status).c_str());
    //hjkim 251126 : 도킹에서 라이다 센서를 이용한 충돌방지 로직을 적용하며, 멀리서 도킹을 시도를 하더라도 충돌없이 충전기로 들어갈 수 있도록 개선 -> 복귀 실패 시 도킹으로 전환하도록 수정.
    if ((node_status == NODE_STATUS::AUTO_MAPPING && state_utils->isAutoMappingComplete()) || node_status == NODE_STATUS::MANUAL_MAPPING) {
        state_utils->map_saver();
    }

    pose robot = state_utils->getRobotPose();
    double dx = robot.x - return_pose.x;
    double dy = robot.y - return_pose.y;
    double angle_diff = state_utils->normalize_angle(std::atan2(dy, dx) - return_pose.theta);
    double distance = 100;
    if (std::fabs(angle_diff) <= 1.57079632) {
      distance = std::sqrt(dx*dx + dy*dy);
    }
    RCLCPP_WARN(node_->get_logger(), "[ReturnCharger] failed robot(%.2f, %.2f) target(%.2f, %.2f) distance : %f",robot.x,robot.y,return_pose.x,return_pose.y,distance);
    if(distance <= state_utils->getReturnChargerTryDockingDistanceThreshold() || detectDockShortSig()){ //FAIL인데 전방 2.0 이내 또는 short signal이 감지 된경우 docking전환.
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Return charger [move goal] fail!!!-> try docking [distance: %f | detectDockShortSig: %d | detectDockLongSig: %d]", distance, detectDockShortSig(), detectDockLongSig());
      if ((node_status == NODE_STATUS::AUTO_MAPPING && state_utils->isAutoMappingComplete()) || node_status == NODE_STATUS::MANUAL_MAPPING) {
        state_utils->map_saver();
      }
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING);
      state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
    }else{
      // 26.02.05 [hyjoe] : 배터리 15% 이하인 경우, 복귀 실패하면 다음 복귀 시 global 모드로 relocalization 수행하도록 플래그 세팅
      if (state_utils->getBatteryPercentage() <= 15) {
        state_utils->setCmdGlobalLocalizationMode(true);
        state_utils->setSkipLocalization(false);
      }
      //hjkim : 복귀 실패 시 stopDriving 누락으로 인한 saveLastPose 실행 안되어, 다음 이동 명령 시 localization 위치 오차 발생
      state_utils->resetTryMoveTargetCount();
      state_utils->stopDriving();
      // state_utils->setSkipLocalization(false);
      state_utils->publishMoveFailError();
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
      if(state_utils->isReservedSaveMapFile()){
        state_utils->startSaveMapFile();
      }
    }

    dock_pose_estimate = false;
    stopMonitorReturnCharger();
  } else {
    // RCLCPP_WARN(node_->get_logger(), "Navigation idle, stop monitor ");
  }
}

void ReturnCharger::reset_timerNaviStatus() {
    if (nav_status_timer_) {
      nav_status_timer_.reset();
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] reset_timerNaviStatus");
    } else {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] nav_status_timer_ is allready reset ");
    }
}

bool ReturnCharger::detectDockShortSig(){
  bool ret = false;
  int8_t short_signal = state_utils->getStationShortSignal();
  if(short_signal & (SHORT_SIG_LEFT|SHORT_SIG_LEFT_CENTER|SHORT_SIG_CENTER|SHORT_SIG_RIGHT_CENTER|SHORT_SIG_RIGHT)){
    ret = true;
  }
  return ret;
}

bool ReturnCharger::detectDockLongSig(){
  bool ret = false;
  int8_t long_signal = state_utils->getStationLongSignal();
  if(long_signal & (SHORT_SIG_LEFT|SHORT_SIG_LEFT_CENTER|SHORT_SIG_CENTER|SHORT_SIG_RIGHT_CENTER|SHORT_SIG_RIGHT)){
    ret = true;
  }
  return ret;
}

} // namespace airbot_state