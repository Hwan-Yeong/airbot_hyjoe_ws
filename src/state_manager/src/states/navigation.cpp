#include "state_manager/states/navigation.hpp"

// ****스테이트의 공통 데이터 관리.
namespace airbot_state {

Navigation::Navigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {}

void Navigation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Navigation] pre_run() -> Preparing Navigation state");

  retry_localization = 0;
  bNaviNodeRecovery = false;
  state_utils->setMovingPauseFlag(false);
  state_utils->setMovingStateID( NAVI_STATE::IDLE);
  state_utils->setReadyMoving(READY_MOVING::IDLE);
  setReadyNavigation(READY_NAVIGATION::CHECK_SENSOR);
  bSkipInitPoseLocalization = false;
  state_utils->cancelPreviousGoal();
  state_utils->resetTryMoveTargetCount();
  #if USE_LIDAR_STATE_CHECK == 0
  state_utils->publishScanMonitor(true);
  #endif
  state_utils->publishTFMonitor(true);
  if(state_utils->getMapInfoChanged()){
    state_utils->reserveSavingMapFile();
  }
}

void Navigation::setReadyNavigation(READY_NAVIGATION set)
{
  if(readyNavi != set){
    RCLCPP_INFO(node_->get_logger(), "[Navigation] setReadyNavigation %s", enumToString(set).c_str());
  }
  readyNavi = set;
}
void Navigation::run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::run(state_utils);
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[Navigation] run() -> Running Navigation state");
  }
  //1. sensor On (before move-target)
  //2. poseEstimate (before move-target) - pub(current.pose); 
  //3. pub TargetPose to Navi
  //4. monitor NaviState
  //5. Sensor Off (after Goal-Arrived)
  ROBOT_STATUS ready_check;

  if(state_utils->getStatusID() != ROBOT_STATUS::READY){// NavigationReady가 완료되어 ROBOT_STATUS::START이후로 동작시작. 
    if(state_utils->getOnstationStatus()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] Robot get on Docking Station!!!");
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
      return;
    }

    if(bNaviNodeRecovery){
      if(!state_utils->getNavibringUpRecoveryPause()){
        bNaviNodeRecovery = false;
        state_utils->publishEmergencyStop(false);
        RCLCPP_INFO(node_->get_logger(), "[Navigation] Node Recovery Finished!");
      }
      state_utils->publishVelocityCommand(0.0,0.0);
      return;
    }

    if(state_utils->isReservedMoving() && !state_utils->getReservePause() && !state_utils->getSensorRecoveryPause() && !state_utils->getNavibringUpRecoveryPause()){
      movingData = state_utils->getTargetPosition();
      state_utils->setMovingStateID(NAVI_STATE::READY);
      state_utils->setStatusID(ROBOT_STATUS::START);
      state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
      RCLCPP_INFO(node_->get_logger(), "[Navigation] reserved moving start position[%.2f, %.2f, %.2f]", movingData.target_position.x, movingData.target_position.y, movingData.target_position.theta);
    }else if(state_utils->getNavibringUpRecoveryPause()){
      bNaviNodeRecovery = true;
      state_utils->reserveMapLoadatferNavRecovery();
      state_utils->publishLifeCycleOff();
      state_utils->publishEmergencyStop(true);
      state_utils->setMovingStateID(NAVI_STATE::PAUSE);
      state_utils->stopDriving();
      RCLCPP_INFO(node_->get_logger(), "[Navigation] Go to pause due to Navibring Up Recovery");
      return;
    }
  }

  if(state_utils->isNeedToRunMapLoadAfterRecovery()){
    state_utils->mapLoadAfterNavRecoveryProcess(0);
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
      RCLCPP_INFO(node_->get_logger(), "[Navigation] Go to pause due to Sensor Recovery");
      state_utils->setStatusID(ROBOT_STATUS::RECOVERY_PAUSE);
    }
    else if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE || state_utils->getMovingPauseFlag()){
      if( state_utils->getRotatePauseFlag() ){
        state_utils->setMovingStateID(NAVI_STATE::START_ROTAION);
        state_utils->setMovingPauseFlag(false);
        state_utils->setRotatePauseFlag(false);
      } else{
        if( state_utils->getStopDrivingFlag() ){
          state_utils->setMovingPauseFlag(false);
          state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
          state_utils->setMovingStateID(NAVI_STATE::READY); //resume
        } else{
          RCLCPP_INFO(node_->get_logger(), "[Navigation] RESUME navigation but.. Wait Stop Driving");
        }
      }
    }else if(state_utils->getMovingStateID() == NAVI_STATE::FAIL){
      state_utils->publishMoveFailError();
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
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
        RCLCPP_INFO(node_->get_logger(), "[Navigation] Sensor Recovery Finished!");
        state_utils->setStatusID(ROBOT_STATUS::START);
      }
    }
    break;
  default:
    RCLCPP_INFO(node_->get_logger(), "[Navigation] Running UNKNOWN Status");
    break;
  }
}

void Navigation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->publishEmergencyStop(false);
  state_utils->stopDriving();
  RCLCPP_INFO(node_->get_logger(), "[Navigation] post_run() -> Exiting Navigation state");
  if (state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_WORKING) {
    state_utils->send_node_goal(NODE_STATUS::IDLE);
  }
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
  state_utils->publishLifeCycleOff();
  if(state_utils->isStartLocalization()){
    state_utils->stopLocalizationMonitor();
  }
  state_utils->setMovingPauseFlag(false);
}

void Navigation::processMoveTarget() {  
  int localize_result = 0;
  switch (state_utils->getReadyMoving())
  {
  case READY_MOVING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      if(state_utils->isSkipLocalization()){
        RCLCPP_INFO(node_->get_logger(), "[Navigation] Sensor is Already On Skip Localization");
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
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] lidar Error");
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] tof Error");
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }else if(state_utils->isCamreaError()){
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      if(state_utils->isSkipLocalization()){
        RCLCPP_INFO(node_->get_logger(), "[Navigation] Sensor is Already On Skip Localization");
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
      RCLCPP_INFO(node_->get_logger(), "[Navigation] CameraError");
      //state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }
    break;
  #if USE_JSLLOC > 0
  case READY_MOVING::CHECK_LOCALIZATION_MODE :
    if(state_utils->getLocalizationMode() != LOCALIZATION_MODE::VOID)
    {
      RCLCPP_INFO(node_->get_logger(), "[Navigation] complete localization mode change cmd[%s] current-mode[%s] ",
      state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
    }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
      if(++local_mode_retry_cnt >= 3){
        local_mode_retry_cnt = 0;
        state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
        RCLCPP_INFO(node_->get_logger(), "[Navigation] retry 3time localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }else{
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        localization_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[Navigation] retry localization mode change cmd[%s] current-mode[%s] ",
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
        RCLCPP_INFO(node_->get_logger(),"[Navigation] Recovery Reboot Flag Clear");
      }
      RCLCPP_INFO(node_->get_logger(), "[Navigation] localization success");
      state_utils->publishKeepOutEnable(true);
      keepout_enable_cnt = 0;
      state_utils->setReadyMoving(READY_MOVING::CHECK_KEEPOUT_STATE);//state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
    } 
    else if(localize_result < 0) {
      RCLCPP_INFO(node_->get_logger(), "[Navigation] localization fail result : %d", localize_result);
      state_utils->publishLocalizationFailErrorStatus(true);
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }
    break;
    case READY_MOVING::CHECK_KEEPOUT_STATE :
      if(state_utils->getKeepoutState() == KEEPOUT_STATE::KEEPOUT_ENABLE){
        state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
      }
      else if(state_utils->getSteadyClockRunningSeconds(state_utils->getKeepOutEnableStartTime()) > 3){
        if(keepout_enable_cnt++ < 3){
          state_utils->publishKeepOutEnable(true);
          RCLCPP_INFO(node_->get_logger(), "[Navigation] KeepOut Enable retry %d", keepout_enable_cnt);
        }else{
          state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
          RCLCPP_INFO(node_->get_logger(), "[Navigation] KeepOut Enable Fail 3 Times");
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
          RCLCPP_INFO(node_->get_logger(), "[Navigation] Maneuver Communication Error");
        }else{
          RCLCPP_INFO(node_->get_logger(), "[Navigation] Perception Communication Error");
        }
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
        return;
      }
    break;
  case READY_MOVING::CHECK_PREVIOUS_GOAL :
      state_utils->setReadyMoving(READY_MOVING::SEND_GOAL);
    break;
  case READY_MOVING::SEND_GOAL :
    state_utils->moveToTarget(movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
    state_utils->setReadyMoving(READY_MOVING::COMPLETE);
    break;
  case READY_MOVING::COMPLETE :
    break;  
  default:
    break;
  };
}

ROBOT_STATUS Navigation::processNavigationReady() {
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int localize_result = 0;
  int node_result = 0;
  switch (readyNavi)
  {
    case READY_NAVIGATION::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyNavigation(READY_NAVIGATION::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] lidar Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] tof Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] CameraError");
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      setReadyNavigation(READY_NAVIGATION::CHECK_ODOM_RESET);
      //ret = ROBOT_STATUS::FAIL;
      //setReadyNavigation(READY_NAVIGATION::FAIL);
    }
    break;     
  case READY_NAVIGATION::CHECK_ODOM_RESET :
    if(!state_utils->isStartOdomReset()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] not start odom reset, skip check odom reset");
      bSkipInitPoseLocalization = true;
      setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
    }else if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] odom reset Error");
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }else if(state_utils->getOdomResetDone()){
      setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
    }
    break;
  case READY_NAVIGATION::LAUNCH_NODE :
    if(state_utils->getNodeStatusID() != NODE_STATUS::NAVI){
      state_utils->send_node_goal(NODE_STATUS::NAVI);
      setReadyNavigation(READY_NAVIGATION::CHECK_NODE);
    }else{
      if(bSkipInitPoseLocalization){
        RCLCPP_INFO(node_->get_logger(), "[Navigation] init-pose localization skip cause it hasn`t reset odom");
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }else{
#if USE_JSLLOC > 0
        local_mode_retry_cnt = 0;
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        setReadyNavigation(READY_NAVIGATION::START_LOCALIZATION);
        localization_start_time = std::chrono::steady_clock::now();
#else
        state_utils->setCheckAmclAfterLocalization();
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
        setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
#endif 
      }
    }
    
    break;
  case READY_NAVIGATION::CHECK_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      if(bSkipInitPoseLocalization){
        RCLCPP_INFO(node_->get_logger(), "[Navigation] init-pose localization skip cause it hasn`t reset odom");
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }else{
#if USE_JSLLOC > 0
        local_mode_retry_cnt = 0;
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        setReadyNavigation(READY_NAVIGATION::START_LOCALIZATION);
        localization_start_time = std::chrono::steady_clock::now();
#else
        state_utils->setCheckAmclAfterLocalization();
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
        setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
#endif 
      }
    }else if(node_result < 0){
      setReadyNavigation(READY_NAVIGATION::STOP_NODE);
    }
    break;
  case READY_NAVIGATION::START_LOCALIZATION :
    if(state_utils->getLocalizationMode() != LOCALIZATION_MODE::VOID) {
      state_utils->setCheckAmclAfterLocalization();
      state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
      setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
      RCLCPP_INFO(node_->get_logger(), "[Navigation] complete localization mode change cmd[%s] current-mode[%s] ",
      state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
    }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
      if(++local_mode_retry_cnt >= 3){
        local_mode_retry_cnt = 0;
        state_utils->setCheckAmclAfterLocalization();
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
        setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
        RCLCPP_INFO(node_->get_logger(), "[Navigation] retry 3time localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }else{
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        localization_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[Navigation] retry localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }
    }
    break;  
  case READY_NAVIGATION::CHECK_LOCALIZATION :
    localize_result = localizationChecker();
    if(localize_result > 0){
      if(state_utils->getRecoveryRebootflag()){
        state_utils->removeRecoveryDirectory();
        state_utils->setRecoveryRebootflag(false);
        RCLCPP_INFO(node_->get_logger(),"[Navigation] Recovery Reboot Flag Clear");
      }
      check_nav_active_start_time_ = std::chrono::steady_clock::now();
      setReadyNavigation(READY_NAVIGATION::CHECK_NAV_ACTIVE);
    } 
    else if(localize_result < 0) {
      if(++retry_localization >= state_utils->getLocalErrorCount()){
        RCLCPP_INFO(node_->get_logger(), "[Navigation] localization error count[%u]over", retry_localization);
        retry_localization = 0;
        state_utils->publishLocalizationFailErrorStatus(true);
        setReadyNavigation(READY_NAVIGATION::FAIL);
      }else{
        RCLCPP_INFO(node_->get_logger(), "[Navigation] localization fail re-start Node count[%u]", retry_localization);
        setReadyNavigation(READY_NAVIGATION::STOP_NODE);
      }
    }
    break;
  case READY_NAVIGATION::CHECK_NAV_ACTIVE :
    if( state_utils->getNaviNodeActive() ){
      setReadyNavigation(READY_NAVIGATION::COMPLETE);
      RCLCPP_WARN(node_->get_logger(), "[Navigation] CHECK_NAV_ACTIVE DONE. NAV2 ACTIVATED.");
    } else {
      if (state_utils->getSteadyClockRunningSeconds(check_nav_active_start_time_) >= 20.0) {
        RCLCPP_WARN(node_->get_logger(), "[Navigation] CHECK_NAV_ACTIVE timeout. Transitioning to STOP_NODE.");
        setReadyNavigation(READY_NAVIGATION::STOP_NODE);
      }

      RCLCPP_INFO(node_->get_logger(), "[Navigation] Waiting for Navi node to become active... (%.1fs)",
                  state_utils->getSteadyClockRunningSeconds(check_nav_active_start_time_));
    }
    break;
  case READY_NAVIGATION::COMPLETE :
    if( state_utils->getReservePause() ){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] Go to PAUSE STATUS due to pause reservation!" );
      ret = ROBOT_STATUS::PAUSE; //reserve pause 인 경우 Pause진입.
    } else{
      state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
      ret = ROBOT_STATUS::START;
    }
    break;
  case READY_NAVIGATION::FAIL :
    ret = ROBOT_STATUS::FAIL;
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
  default:
    RCLCPP_INFO(node_->get_logger(), "[Navigation] processNavigationReady readyState Error!! : %d", static_cast<int>(readyNavi));
    break;
  }

  return ret;
}

int8_t Navigation::localizationChecker() {
  int8_t ret = 0;
  if(state_utils->isStartLocalization()){
    // double wait_localize_time = node_->now().seconds()-state_utils->getLocalizationStartTime();
    if(state_utils->getLocalizationComplete()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] localization Done ");
      ret = 1;
    }else if(state_utils->isLocalizationError()){
      RCLCPP_INFO(node_->get_logger(), "[Navigation] localization Error ");
      ret = -1;
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[Navigation] skip localization");
    ret = 1;
  }

  return ret;
}

} // namespace airbot_state