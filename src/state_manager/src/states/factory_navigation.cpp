#include "state_manager/states/factory_navigation.hpp"

namespace airbot_state {

FactoryNavigation::FactoryNavigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {}

    void FactoryNavigation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
      stateBase::pre_run(state_utils);
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] pre_run() -> Preparing FactoryNavigation state");
      
      retry_localization = 0;
      bNaviNodeRecovery = false;
      state_utils->setMovingStateID( NAVI_STATE::IDLE);
      state_utils->setReadyMoving(READY_MOVING::IDLE);
      setReadyNavigation(READY_NAVIGATION::CHECK_SENSOR);
      state_utils->setMovingPauseFlag(false);
      if(state_utils->getRobotCMDID().robot_cmd != REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FACTORY_NAVIGATION){
          RCLCPP_INFO(node_->get_logger(), "[WARNING] factory navi start on station, but robot_cmd is [%s]", enumToString(state_utils->getRobotCMDID().robot_cmd).c_str());
      }

      state_utils->cancelPreviousGoal();
      state_utils->resetTryMoveTargetCount();
      #if USE_LIDAR_STATE_CHECK == 0
      state_utils->publishScanMonitor(true);
      #endif
      state_utils->publishTFMonitor(true);
    }
    
    void FactoryNavigation::setReadyNavigation(READY_NAVIGATION set)
    {
      if(readyNavi != set){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] setReadyNavigation %s", enumToString(set).c_str());
      }
      readyNavi = set;
    }

    void FactoryNavigation::run(const std::shared_ptr<StateUtils> &state_utils) {
      stateBase::run(state_utils);
      if( isFirstRunning() ){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] run() -> Running Navigation state");
      }
      //1. sensor On (before move-target)
      //2. poseEstimate (before move-target) - pub(current.pose); 
      //3. pub TargetPose to Navi
      //4. monitor NaviState
      //5. Sensor Off (after Goal-Arrived)
      ROBOT_STATUS ready_check;

      if(state_utils->getStatusID() != ROBOT_STATUS::READY && state_utils->getOnstationStatus()){// NavigationReady가 완료되어 ROBOT_STATUS::START이후로 동작시작. 
        RCLCPP_INFO(node_->get_logger(), "[Navigation] Robot get on Docking Station!!!");
        state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
        return;
      }

      if(bNaviNodeRecovery){
        if(!state_utils->getNavibringUpRecoveryPause()){
          bNaviNodeRecovery = false;
          state_utils->publishEmergencyStop(false);
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Node Recovery Finished!");
        }
        state_utils->publishVelocityCommand(0.0,0.0);
        return;
      }

      if(state_utils->isReservedMoving() && state_utils->getStatusID() != ROBOT_STATUS::READY && !state_utils->getReservePause() && !state_utils->getSensorRecoveryPause() && !state_utils->getNavibringUpRecoveryPause()){
        movingData = state_utils->getTargetPosition();
        state_utils->setMovingStateID(NAVI_STATE::READY);
        state_utils->setStatusID(ROBOT_STATUS::START);
        state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] reserved moving start position[%.2f, %.2f, %.2f]", movingData.target_position.x, movingData.target_position.y, movingData.target_position.theta);
      }else if(state_utils->getNavibringUpRecoveryPause()){
        bNaviNodeRecovery = true;
        state_utils->reserveMapLoadatferNavRecovery();
        state_utils->publishLifeCycleOff();
        state_utils->publishEmergencyStop(true);
        state_utils->setMovingStateID(NAVI_STATE::PAUSE);
        state_utils->stopDriving();
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Go to pause due to Navibring Up Recovery");
        return;
      }

      if(state_utils->isNeedToRunMapLoadAfterRecovery()){
        state_utils->mapLoadAfterNavRecoveryProcess(1);
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
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Go to pause due to Sensor Recovery");
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
              RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] RESUME factory navigation but.. Wait Stop Driving");
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
        }else if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE ){
          if(!state_utils->getSensorRecoveryPause()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Sensor Recovery Finished!");
            state_utils->setStatusID(ROBOT_STATUS::START);
          }
        }
        break;      
      default:
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Running UNKNOWN Status");
        break;
      }
    }
    
    void FactoryNavigation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
      stateBase::post_run(state_utils);
      state_utils->publishEmergencyStop(false);
      state_utils->stopDriving();
      if(!state_utils->getFactoryMode()){
        state_utils->send_node_goal(NODE_STATUS::IDLE);
      }
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] post_run() -> Exiting FactoryNavigation state");
      state_utils->stopMonitorOdom();
      state_utils->stopSensorMonitor();
      state_utils->publishLifeCycleOff();
      if(state_utils->isStartLocalization()){
        state_utils->stopLocalizationMonitor();
      }
      state_utils->setMovingPauseFlag(false);
    }

    void FactoryNavigation::processMoveTarget()
    {  
      int localize_result = 0;
      switch (state_utils->getReadyMoving())
      {
      case READY_MOVING::CHECK_SENSOR :
        if(state_utils->isSensorReady()){
          if(state_utils->isSkipLocalization()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Sensor is Already On Skip Localization");
            state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
          }else{
            localization_start_time = std::chrono::steady_clock::now();
            local_mode_retry_cnt = 0;
            state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
          }
        }else if(state_utils->isLidarError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] LidarError");
          state_utils->setStatusID(ROBOT_STATUS::FAIL);
        }else if(state_utils->isToFError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] ToFError");
          state_utils->setStatusID(ROBOT_STATUS::FAIL);
        }else if(state_utils->isCamreaError()){
          //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
          if(state_utils->isSkipLocalization()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Sensor is Already On Skip Localization");
            state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
          }else{
            localization_start_time = std::chrono::steady_clock::now();
            local_mode_retry_cnt = 0;
            state_utils->setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
          }
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] CameraError");
          //state_utils->setStatusID(ROBOT_STATUS::FAIL);
        }
        break;
      case READY_MOVING::REQUEST_POSE_ESTIMATE :
#if USE_JSLLOC > 0
      //hjkim : 공정모드에서 이미 팩토리 네비를 켠상태에서 혹시나 global localization on 되어있으면... off
      if(state_utils->getLocalizationMode() == LOCALIZATION_MODE::LOCAL_ONLY){
          state_utils->setCheckAmclAfterLocalization();
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
          state_utils->setReadyMoving(READY_MOVING::CHECK_POSE_ESTIMATE);
        }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
          if(++local_mode_retry_cnt >= 3){
            local_mode_retry_cnt = 0;
            state_utils->setCheckAmclAfterLocalization();
            state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
            state_utils->setReadyMoving(READY_MOVING::CHECK_POSE_ESTIMATE);
          }else{
            state_utils->setCmdGlobalLocalizationMode(false);          
            state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
            localization_start_time = std::chrono::steady_clock::now();
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] retry localization mode change current-mode[%s] ", enumToString(state_utils->getLocalizationMode()).c_str());
          }
        }
#else
      state_utils->setCheckAmclAfterLocalization();
      state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
      state_utils->setReadyMoving(READY_MOVING::CHECK_POSE_ESTIMATE);  
#endif 
        break;
      case READY_MOVING::CHECK_POSE_ESTIMATE :
      localize_result = localizationChecker();
      if(localize_result > 0){
        state_utils->setReadyMoving(READY_MOVING::REQUEST_MANEUVER_ON);
      } 
      else if(localize_result < 0){
        state_utils->publishLocalizationFailErrorStatus(true);
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
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
          state_utils->setReadyMoving(READY_MOVING::CHECK_PREVIOUS_GOAL);
          if(state_utils->isManeuverCommunicateError()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Maneuver Communication Error");
          }else{
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Perception Communication Error");
          }
        }
        break;
	  case READY_MOVING::CHECK_PREVIOUS_GOAL :
    //hjkim 250725 : check goal cancel 에 대한 추가적임 검토 필요. ( nav2 action client 통합 버전 적용 이전으로 되돌림)
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
    
    ROBOT_STATUS FactoryNavigation::processNavigationReady()
    {
      ROBOT_STATUS ret = ROBOT_STATUS::READY;
      int localize_result = 0;
      int node_result = 0;
      switch (readyNavi)
      {
      case READY_NAVIGATION::CHECK_SENSOR :
        if(state_utils->isSensorReady()){
          setReadyNavigation(READY_NAVIGATION::CHECK_ODOM_RESET);
        }else if(state_utils->isLidarError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] lidar Error");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }else if(state_utils->isToFError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] tof Error");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }else if(state_utils->isCamreaError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] CameraError");
          //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
          setReadyNavigation(READY_NAVIGATION::CHECK_ODOM_RESET);
          //ret = ROBOT_STATUS::FAIL;
          //setReadyNavigation(READY_NAVIGATION::FAIL);
        }
        break;     
      case READY_NAVIGATION::CHECK_ODOM_RESET :
        if(!state_utils->isStartOdomReset()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] odom-reset not be started");
          state_utils->startMonitorOdomReset(); //hjkim : in case odom-reset not be started(not be started on statation) -> odom-reset start
        }else if(state_utils->isOdomResetError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] odom reset Error");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }else if(state_utils->getOdomResetDone()){
          setReadyNavigation(READY_NAVIGATION::LAUNCH_NODE);
        }
        break;
      case READY_NAVIGATION::CHECK_NODE :
        node_result = state_utils->getNodeClientStatus();
        if(node_result > 0){
#if USE_JSLLOC > 0
          local_mode_retry_cnt = 0;
          state_utils->setCmdGlobalLocalizationMode(false);          
          state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
          setReadyNavigation(READY_NAVIGATION::START_LOCALIZATION);
          localization_start_time = std::chrono::steady_clock::now();
#else
        if( state_utils->getMapServerNodeActive() ){
            if (state_utils->setMapChange(1)) {
              RCLCPP_INFO(node_->get_logger(), "Map parameter change request sent successfully.");
              setReadyNavigation(READY_NAVIGATION::MAP_CHANGE);
            } else {
              RCLCPP_ERROR(node_->get_logger(), "Failed to send map parameter change request.");
              ret = ROBOT_STATUS::FAIL;
              setReadyNavigation(READY_NAVIGATION::FAIL);
            }
          }
#endif
        }else if(node_result < 0){
          setReadyNavigation(READY_NAVIGATION::STOP_NODE);
        }
        break;
#if USE_JSLLOC > 0
      case READY_NAVIGATION::START_LOCALIZATION :
      //251001 KKS : factory는 항상 local_only인지 확인한다.
        if(state_utils->getLocalizationMode() == LOCALIZATION_MODE::LOCAL_ONLY){
          state_utils->setCheckAmclAfterLocalization();
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
          setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
        }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
          if(++local_mode_retry_cnt >= 3){
            local_mode_retry_cnt = 0;
            state_utils->setCheckAmclAfterLocalization();
            state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
            setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] retry 3time localization mode change current-mode[%s] ", enumToString(state_utils->getLocalizationMode()).c_str());
          }else{
            state_utils->setCmdGlobalLocalizationMode(false);
            state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
            localization_start_time = std::chrono::steady_clock::now();
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] retry localization mode change current-mode[%s] ", enumToString(state_utils->getLocalizationMode()).c_str());
          }
        }
        break;
#else
      case READY_NAVIGATION::MAP_CHANGE : // map 변경.
        if( state_utils->getSetMapChangeDone() ){
          state_utils->setCheckAmclAfterLocalization();
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
          setReadyNavigation(READY_NAVIGATION::CHECK_LOCALIZATION);
        }
        break;
#endif
      case READY_NAVIGATION::CHECK_LOCALIZATION :
        localize_result = localizationChecker();
        if(localize_result > 0){
          setReadyNavigation(READY_NAVIGATION::NAVI_PARAM_CHECK);
          paramset_test = false;
        } 
         else if(localize_result < 0) {
          if(++retry_localization >= state_utils->getLocalErrorCount()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] localization error count[%u]over", retry_localization);
            retry_localization = 0;
            state_utils->publishLocalizationFailErrorStatus(true);
            setReadyNavigation(READY_NAVIGATION::FAIL);
          }else{
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] localization fail re-start Node count[%u]", retry_localization);
            setReadyNavigation(READY_NAVIGATION::STOP_NODE);
          }
        }
        break;  

      case READY_NAVIGATION::NAVI_PARAM_CHECK: //navi param 변경.
        if (!stabilization_timer_&& !paramset_test) {
            stabilization_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
            // 타이머가 설정 시간 만족시 내용 진행
            this->stabilization_timer_->cancel();
            this->stabilization_timer_.reset();
            
            // 안정화 시간 후, 파라미터 설정 시작
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] System stabilized. Now setting navigation parameters.");
            if(state_utils->getNaviNodeActive() ){
              state_utils->setMapParameters(1);
              paramset_test = true;
            } else{ //lifecycle activate 문제 발생 확인 로그.
              RCLCPP_WARN(node_->get_logger(), "[FactoryNavigation] Not all navigation nodes are active yet. Retrying parameter set.");
            }
            });
        }
        if(state_utils->getSetMapParametersDone()){
          setReadyNavigation(READY_NAVIGATION::COMPLETE);
        }
        break;
      case READY_NAVIGATION::COMPLETE :
        if( state_utils->getReservePause() ){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Go to PAUSE STATUS due to pause reservation!" );
          ret = ROBOT_STATUS::PAUSE; //reserve pause 인 경우 Pause진입.
        } else{
          state_utils->setReadyMoving(READY_MOVING::CHECK_SENSOR);
          ret = ROBOT_STATUS::START;
        }  
        break;
      case READY_NAVIGATION::FAIL :
        ret = ROBOT_STATUS::FAIL;
        break;
      case READY_NAVIGATION::LAUNCH_NODE :
        state_utils->send_node_goal(NODE_STATUS::NAVI);
        setReadyNavigation(READY_NAVIGATION::CHECK_NODE);
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
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] process FactoryNavigation Ready readyState Error!! : %d", static_cast<int>(readyNavi));
          break;
      }
      
      return ret;
    }
    
    int8_t FactoryNavigation::localizationChecker()
    {
      int8_t ret = 0;
      if(state_utils->isStartLocalization()){
        // double wait_localize_time = node_->now().seconds()-state_utils->getLocalizationStartTime();
        if(state_utils->getLocalizationComplete()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] localization Done ");
          ret = 1;
        }else if(state_utils->isLocalizationError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] localization Error ");
          ret = -1;
        }
      }else{
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] skip localization");
        ret = 1;
      }
    
      return ret;
    }
    
} // namespace airbot_state