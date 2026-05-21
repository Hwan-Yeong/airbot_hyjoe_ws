#include "state_manager/states/auto_mapping.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

AutoMapping::AutoMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void AutoMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Preparing auto_mapping state");
  rclcpp::QoS wm_qos_profile = rclcpp::QoS(1).reliable().transient_local();
  explore_status_sub = node_->create_subscription<std_msgs::msg::UInt8>("/explore_status", wm_qos_profile, std::bind(&AutoMapping::explore_status_callback, this, std::placeholders::_1));
  mapping_start_time = std::numeric_limits<double>::max();

  setReadyMapping(READY_MAPPING::CHECK_SENSOR);
  if(state_utils->getRobotCMDID().robot_cmd != REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING){
    RCLCPP_INFO(node_->get_logger(), "[WARNING] mapping start on station, but robot_cmd is [%s]", enumToString(state_utils->getRobotCMDID().robot_cmd).c_str());
  }

  state_utils->cancelPreviousGoal(); //hjkim : auto-mapping과 client 충돌날까봐 이전골이 있으면 취소. 방어코드..
  #if USE_LIDAR_STATE_CHECK == 0
  state_utils->publishScanMonitor(true);
  #endif
  state_utils->publishTFMonitor(true);
  state_utils->setAutoMappingCompleteState(false);
}

void AutoMapping::run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::run(state_utils);
  ROBOT_STATUS ready_check;
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] run() -> running AutoMapping state");
  }

  if(state_utils->getStatusID() != ROBOT_STATUS::READY){
    if(state_utils->getOnstationStatus()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping]Robot on Docking Station!!!");
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
      return;
    }

    if(state_utils->getNavibringUpRecoveryPause()){
      if(!bNaviNodeRecovery){
        state_utils->publishLifeCycleOff();
        bNaviNodeRecovery = true;
        state_utils->publishEmergencyStop(true);
        state_utils->publishMappingStop();
        RCLCPP_INFO(node_->get_logger(), "[AutoMapping] stop mapping for Navibring-Up Recovery");
      }
      state_utils->publishVelocityCommand(0.0,0.0);
      return;
    }else{
      if(bNaviNodeRecovery){
        bNaviNodeRecovery = false;
        state_utils->publishEmergencyStop(false);
        state_utils->publishMappingStart();
         RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Node Recovery Finished! -> start mapping");
      }
    }

    if( state_utils->getSensorRecoveryPause() && state_utils->getStatusID() == ROBOT_STATUS::START && 
        explore_status == EXPLORE_STATUS::EXPLORE_RUNNING ){ //tof reset flag인데, explore가 동작중(STATUS::START && EXPLORE_STATUS::EXPLORE_RUNNING)
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Go to pause due to Sensor Recovery [HW-RESET]");
      state_utils->setStatusID(ROBOT_STATUS::RECOVERY_PAUSE); // pause동작 explore 멈춤
    } else if(  !state_utils->getSensorRecoveryPause() && state_utils->getStatusID() == ROBOT_STATUS::RECOVERY_PAUSE ){// reset 아닌 경우
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Sensor Recovery Finished! resume working");
      state_utils->setStatusID(ROBOT_STATUS::START); // explore 재시작.
    }

  }
  
  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    if( state_utils->getStopDrivingFlag() ){
      ready_check = processMappingReady();
      state_utils->setStatusID(ready_check);
      if(ready_check == ROBOT_STATUS::START){
        setExploreStatus(EXPLORE_STATUS::EXPLORE_VOID);
      }else if(ready_check == ROBOT_STATUS::FAIL){
        RCLCPP_INFO(node_->get_logger(), "[AutoMapping] mapping fail");
      }
    } else{
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] READY auto mapping but.. Wait Stop Driving");
    }
    break;
  case ROBOT_STATUS::START :
    if( state_utils->getStopDrivingFlag() ){
      if(runAutoMapping()){
        state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
      }
    } else{
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] START auto mapping but.. Wait Stop Driving");
    }
    break;
  case ROBOT_STATUS::RECOVERY_PAUSE :
  case ROBOT_STATUS::PAUSE :
    if(bUdateStatus && explore_status == EXPLORE_STATUS::EXPLORE_RUNNING){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] mapping Pause");
      state_utils->stopDriving();
      state_utils->publishMappingStop();
    }
    break;
  case ROBOT_STATUS::COMPLETE :
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING);
    state_utils->setAutoMappingCompleteState(true);
    break;
  case ROBOT_STATUS::FAIL :
    break;      
  default:
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Running UNKNOWN Status");
    break;
  }
}

void AutoMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->publishEmergencyStop(false);
  state_utils->stopDriving();
  state_utils->publishMappingStop();
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
  state_utils->publishLifeCycleOff();
  if( state_utils->getMappingResumeToErrorFlag() ){
    RCLCPP_INFO(node_->get_logger(), "[Error] Reset MappingResumeToErrorFlag post process auto mapping");
    state_utils->setMappingResumeToErrorFlag(false);
  }
  //hjkim : 자동매핑 중 stop_working (매핑취소)의 경우 맵을 저장하지 않도록 수정
  //hyjoe : 매핑 정상 종료시에만 저장하도록 주석처리
  // if(state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::START_ONSTATION ){
  //   state_utils->map_saver();
  // }else{
  //   RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] mapping not save");
  // }

  reset_subExploreStatus();
  RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] post_run() -> exiting auto_mapping state");
}

///////////////function in AutoMapping
bool AutoMapping::runAutoMapping() {
  bool ret = false;
  switch (explore_status)
  {
    case EXPLORE_STATUS::EXPLORE_NOT_READY:
    if(bUdateStatus){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] explore is not ready yet");
    }
    break;  
  case EXPLORE_STATUS::EXPLORE_IDLE:
    if(bUdateStatus){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] explore idle start auto-mapping");
      state_utils->publishMappingStart();
    }
    break;
  case EXPLORE_STATUS::EXPLORE_CANCEL:
    if(bUdateStatus){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] explore cancel start auto-mapping");
      state_utils->enableManeuverCommand(true);
      state_utils->publishMappingStart();
    }
    break;
    case EXPLORE_STATUS::EXPLORE_RUNNING:
    if(bUdateStatus){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] explore running...");
    }
    break;
    case EXPLORE_STATUS::EXPLORE_FINISH:
    if(bUdateStatus){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] explore complete");
      ret = true;
    }
    break;
    case EXPLORE_STATUS::EXPLORE_ERROR:
    if(bUdateStatus){
      bUdateStatus = false; 
      RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] explore Error!! Node is dead or exception case");
    }
    break;
  default:
    break;
  }
  return ret;  
}

ROBOT_STATUS AutoMapping::processMappingReady() {
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  switch (ready_mapping)
  {

  // CHECK_SENSOR: 센서가 준비되었는지 확인하는 상태
  case READY_MAPPING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyMapping(READY_MAPPING::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] lidar Error");
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] tof Error");
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] CameraError");
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      setReadyMapping(READY_MAPPING::CHECK_ODOM_RESET);//setReadyMapping(READY_MAPPING::FAIL);
    }
    break;     

  // CHECK_ODOM_RESET: odom reset이 필요한지 확인하는 상태
  case READY_MAPPING::CHECK_ODOM_RESET :
    if(!state_utils->isStartOdomReset()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] odom-reset not be started");
      if( state_utils->getMappingResumeToErrorFlag() ){
        RCLCPP_INFO(node_->get_logger(), "[AutoMapping] resume auto-mapping. skip odom reset for resume mapping");
        setReadyMapping(READY_MAPPING::CHECK_NODE_LAUNCH);
      } else{
        state_utils->startMonitorOdomReset(); //hjkim : in case odom-reset not be started(not be started on statation) -> odom-reset start
      }
    }else if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] odom reset Error");
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->getOdomResetDone()){
      state_utils->send_node_goal(NODE_STATUS::AUTO_MAPPING);
      setReadyMapping(READY_MAPPING::CHECK_NODE_LAUNCH);
    }
    break;
    
    // STOP_NODE: 매핑 노드가 중지되었는지 확인하는 상태
  case READY_MAPPING::STOP_NODE:
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Stopping mapping-related nodes (send IDLE)");
    state_utils->send_node_goal(NODE_STATUS::IDLE);
    setReadyMapping(READY_MAPPING::CHECK_STOP_NODE);
    break;
    
    // CHECK_STOP_NODE: 매핑 노드가 정상적으로 중지되었는지 확인하는 상태
    // node_result > 0 -> CHECK_NODE_LAUNCH
  // node_result < 0 -> FAIL
  case READY_MAPPING::CHECK_STOP_NODE:
    node_result = state_utils->getNodeClientStatus();
    if (node_result > 0) { // node_result = 1
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Mapping nodes successfully stopped.");
      if (current_retry_ <= max_retry_attempts_) {
        RCLCPP_WARN(node_->get_logger(), "[AutoMapping] Retry attempt %d/%d", current_retry_, max_retry_attempts_);
        restartLaunchProcess(state_utils);
        setReadyMapping(READY_MAPPING::CHECK_NODE_LAUNCH);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "[AutoMapping] Retry exceeded max attempts (%d)", max_retry_attempts_);
        setReadyMapping(READY_MAPPING::FAIL);
      }
    } else if(node_result < 0) { // node_result = -1
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] stop node fail");
      setReadyMapping(READY_MAPPING::FAIL);
    }      
    break;
    
  // CHECK_NODE_LAUNCH: 매핑 노드가 실행 중인지 확인하는 상태
  case READY_MAPPING::CHECK_NODE_LAUNCH:
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] AutoMapping Node Launched");
      state_utils->publishKeepOutEnable(false);
      keepout_enable_cnt = 0;
      setReadyMapping(READY_MAPPING::CHECK_KEEPOUT_STATE);//setReadyMapping(READY_MAPPING::REQUEST_MANEUVER_ON);
    } else if (node_result < 0) {
      if (current_retry_ < max_retry_attempts_) {
        setReadyMapping(READY_MAPPING::STOP_NODE);
      }else {
        RCLCPP_WARN(node_->get_logger(), "[AutoMapping] Mapping nodes failed to launch, status: %d", node_result);
        setReadyMapping(READY_MAPPING::FAIL);
      }
    }
    break;
	
	case READY_MAPPING::CHECK_KEEPOUT_STATE:
	  if (state_utils->getKeepoutState() == KEEPOUT_STATE::KEEPOUT_DISABLE) {
	    setReadyMapping(READY_MAPPING::REQUEST_MANEUVER_ON);
	  } else if (state_utils->getSteadyClockRunningSeconds(state_utils->getKeepOutEnableStartTime()) > 3) {
	    if (keepout_enable_cnt++ < 3) {
	      state_utils->publishKeepOutEnable(false);
	      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] KeepOut Enable retry %d", keepout_enable_cnt);
	    } else {
	      setReadyMapping(READY_MAPPING::REQUEST_MANEUVER_ON);
	      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] KeepOut Enable Fail 3 Times");
	    }
	  }
	  break;
  // REQUEST_MANEUVER_ON: 매뉴버 명령을 요청하는 상태
  case READY_MAPPING::REQUEST_MANEUVER_ON:
    state_utils->enableManeuverCommand(true);
    setReadyMapping(READY_MAPPING::CHECK_MANEUVER);
    break;

  // CHECK_MANEUVER: 매뉴버 명령이 완료되었는지 확인하는 상태
  case READY_MAPPING::CHECK_MANEUVER:
    if(state_utils->isManeuverDone()){
      state_utils->publishClearCostMap();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      state_utils->publishSenSorManagerOn();
      setReadyMapping(READY_MAPPING::COMPLETE);
    }else if(state_utils->isManeuverCommunicateError() || state_utils->isPerceptionCommunicateError()){
      if(state_utils->isManeuverCommunicateError()){
        RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Maneuver Communication Error");
      }else{
        RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Perception Communication Error");
      }
      setReadyMapping(READY_MAPPING::FAIL);
    }
    break;
  case READY_MAPPING::COMPLETE:
    //state_utils->resetInitPose(); hjkim : 맵핑 시작 시 충전기위치와 초기위치(복귀좌표) 초기화 -> 매핑 완료 후 맵 저장 시 초기화 하도록 수정
    if( state_utils->getReservePause() ){
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Go to PAUSE STATUS due to pause reservation!" );
      ret = ROBOT_STATUS::PAUSE; //reserve pause 인 경우 Pause진입.
    } else{
      ret = ROBOT_STATUS::START;
    }
    break;
  case READY_MAPPING::FAIL:
    ret = ROBOT_STATUS::FAIL;
    break;     
  default:
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] processMappingReady readyState Error!! : %d", static_cast<int>(ready_mapping));
    break;
  }

  return ret;
}

void AutoMapping::setReadyMapping(READY_MAPPING set) {
  if(ready_mapping != set){
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] setReadyMapping %s", enumToString(set).c_str());
  }
  ready_mapping = set;
}

void AutoMapping::restartLaunchProcess(const std::shared_ptr<StateUtils> &state_utils) {
  current_retry_++;
  RCLCPP_INFO(node_->get_logger(), "[AUTO_MAPPING] Restarting mapping nodes...");
  state_utils->send_node_goal(NODE_STATUS::AUTO_MAPPING);
}

void AutoMapping::setExploreStatus(EXPLORE_STATUS set) {
  if(explore_status != set){
    if(set == EXPLORE_STATUS::EXPLORE_RUNNING){
      state_utils->publishLifeCycleOff();
      state_utils->publishClearCostMap();
    }
    RCLCPP_INFO(node_->get_logger(), "[AutoMapping] setExploreStatus %s", enumToString(set).c_str());
  }
  explore_status = set;
}

void AutoMapping::explore_status_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
  EXPLORE_STATUS temp_status = static_cast<EXPLORE_STATUS>(msg->data);
  setExploreStatus(temp_status);
  bUdateStatus = true;
}

void AutoMapping::reset_subExploreStatus() {
    if (explore_status_sub) {
      explore_status_sub.reset();
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] reset explore_status_sub");
    } else {
      RCLCPP_INFO(node_->get_logger(), "[AutoMapping] explore_status_sub is allready reset ");
    }
}

} // namespace airbot_state