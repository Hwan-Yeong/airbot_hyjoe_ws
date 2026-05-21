#include "state_manager/states/manual_control.hpp"

namespace airbot_state {

ManualControl::ManualControl(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliable();

  rclcpp::QoS qos_best_effort_profile = rclcpp::QoS(5)
                            .best_effort()
                            .durability_volatile();

  streaming_cmd_pub = node_->create_publisher<follow_msgs::msg::UiClient>("/streaming/ui/client", qos_profile);
}

void ManualControl::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[MANUAL_CONTROL] Preparing MANUAL_CONTROL state");
  state_utils->startSensorMonitor();
  state_utils->startManualControlMonitor();
  state_utils->setMovingStateID(NAVI_STATE::MANUAL_CONTROL);
  setReadyManualControl(READY_MANUAL_CONTROL::CHECK_SENSOR);
  retry_localization = 0;
  bNeedsInitPoseLocalization = false;
  pause_flag = false;
}

void ManualControl::run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::run(state_utils);
  ROBOT_STATUS ready_check;
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[MANUAL_CONTROL] run() -> running MANUAL_CONTROL state");
  }

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    if( state_utils->getStopDrivingFlag() ){
      ready_check = processManualControlReady();
      state_utils->setStatusID(ready_check);
    }
    break;
  case ROBOT_STATUS::START :
    break;
  case ROBOT_STATUS::PAUSE :
    break;
  case ROBOT_STATUS::COMPLETE :
    // complete 없음.
    break;
  case ROBOT_STATUS::FAIL :
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] manual_control fail");
    break;
  default:
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] Running UNKNOWN Status");
    break;
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[ManualControl]Robot on Docking Station!!!");
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
  }
}

void ManualControl::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[MANUAL_CONTROL] post_run() -> exiting MANUAL_CONTROL state");
  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  state_utils->stopManualControlMonitor();
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
}

///////////////function in ManualControl

ROBOT_STATUS ManualControl::processManualControlReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  int localize_result = 0;
  switch (ready_working)
  {
    case READY_MANUAL_CONTROL::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyManualControl(READY_MANUAL_CONTROL::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] lidar Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyManualControl(READY_MANUAL_CONTROL::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] tof Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyManualControl(READY_MANUAL_CONTROL::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] CameraError");
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      setReadyManualControl(READY_MANUAL_CONTROL::CHECK_ODOM_RESET);
      //ret = ROBOT_STATUS::FAIL;
      //setReadyManualControl(READY_MAPPING::FAIL);
    }
    break;
  case READY_MANUAL_CONTROL::CHECK_ODOM_RESET :
    if(!state_utils->isStartOdomReset()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] odom-reset not be started");
      bNeedsInitPoseLocalization = false;
      setReadyManualControl(READY_MANUAL_CONTROL::LAUNCH_NODE);
    }else if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] odom reset Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyManualControl(READY_MANUAL_CONTROL::FAIL);
    }else if(state_utils->getOdomResetDone()){
      bNeedsInitPoseLocalization = true;
      setReadyManualControl(READY_MANUAL_CONTROL::LAUNCH_NODE);
    }
    break;
  case READY_MANUAL_CONTROL::LAUNCH_NODE :
    /*hjkim260113 : 네비게이션 노드 실행 시 skipLocal 플래그가 false가 되어 로컬을 실행하기 때문에, 
    네비게이션 노드가 이미 켜져있는 경우에는 실행하지 않도록 해야함.*/
    if(state_utils->getNodeStatusID() != NODE_STATUS::NAVI){
      state_utils->send_node_goal(NODE_STATUS::NAVI);
      setReadyManualControl(READY_MANUAL_CONTROL::CHECK_NODE);
    }else{
      setReadyManualControl(READY_MANUAL_CONTROL::READY_LOCALZIATION);
    }
    break;
  case READY_MANUAL_CONTROL::CHECK_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      setReadyManualControl(READY_MANUAL_CONTROL::READY_LOCALZIATION);
    }else if(node_result < 0){
      setReadyManualControl(READY_MANUAL_CONTROL::STOP_NODE);
    }
    break;
  case READY_MANUAL_CONTROL::READY_LOCALZIATION:
     /* hjkim260113 : 라이다를 끄거나, 네비게이션 노드를 켜라고 명령을 던지면 state_utils->send_node_goal(NODE_STATUS::NAVI)
     state_utils->isSkipLocalization() 가 false로 변경되어 로컬을 실행하게 됨.
     true 케이스는 로컬을 완료한 시점으로 로컬을 한번도 잡은적이 없으면 오돔 reset과 상관없이 로컬을 실행하면 됨.
     오돔 reset을 한적이 없기때문에 로컬을 실행하지 않는 경우는 init_pose로 로컬을 하는 경우에만 해당함.*/
      if(state_utils->isSkipLocalization()){
        RCLCPP_INFO(node_->get_logger(), "[ManualControl] init-pose localization skip cause it hasn`t reset odom");
        setReadyManualControl(READY_MANUAL_CONTROL::COMPLETE);
      }else{
#if USE_JSLLOC > 0
          local_mode_retry_cnt = 0;
          state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
          setReadyManualControl(READY_MANUAL_CONTROL::START_LOCALIZATION);
          localization_start_time = std::chrono::steady_clock::now();
#else
          state_utils->setCheckAmclAfterLocalization();
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
          setReadyManualControl(READY_MANUAL_CONTROL::CHECK_LOCALIZATION);
#endif
      } 
      break;
    case READY_MANUAL_CONTROL::START_LOCALIZATION :
    if(state_utils->getLocalizationMode() != LOCALIZATION_MODE::VOID) {
      state_utils->setCheckAmclAfterLocalization();
      /*hjkim260113 : 로컬 실행 시 아래 3가지 경우에 대해 로컬을 진행해야함. 네비게이션의 경우 네비게이션 ready와 move_targer 두개의 루틴에서 나눠서 아래 로컬을 진행하지만
      팔로우 모드의 경우 별도 이동 명령이 없기 때문에 ready 단계에서 아래 3개의 로컬을 모두 실행할 수 있어야함.
      getRecoveryRebootflag()의 경우 복귀 실패 시 재부팅 후 마지막 위치를 저장한 좌표로 로컬실행을 하기위함이고,
      언도킹 후 로컬을 실행할 때는 init_pose로 그외 로컬은 마지막 저장위치를 기반으로 로컬을 실행한다.*/
      if(state_utils->getRecoveryRebootflag()){
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::RECOVERY_POSE);
      }else if(bNeedsInitPoseLocalization){
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
      }else{
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
      }
      setReadyManualControl(READY_MANUAL_CONTROL::CHECK_LOCALIZATION);
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] complete localization mode change cmd[%s] current-mode[%s] ",
      state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
    }else if(state_utils->getSteadyClockRunningSeconds(localization_start_time) >= 5){
      if(++local_mode_retry_cnt >= 3){
        local_mode_retry_cnt = 0;
        state_utils->setCheckAmclAfterLocalization();
        if(state_utils->getRecoveryRebootflag()){
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::RECOVERY_POSE);
        }else if(bNeedsInitPoseLocalization){
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
        }else{
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
        }
        setReadyManualControl(READY_MANUAL_CONTROL::CHECK_LOCALIZATION);
        RCLCPP_INFO(node_->get_logger(), "[ManualControl] retry 3time localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }else{
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        localization_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[ManualControl] retry localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }
    }
    break;  
  case READY_MANUAL_CONTROL::CHECK_LOCALIZATION :
    localize_result = localizationChecker();
    if(localize_result > 0){
      if(state_utils->getRecoveryRebootflag()){
        state_utils->removeRecoveryDirectory();
        state_utils->setRecoveryRebootflag(false);
        RCLCPP_INFO(node_->get_logger(),"[ManualControl] Recovery Reboot Flag Clear");
      }
      setReadyManualControl(READY_MANUAL_CONTROL::COMPLETE);
    } 
    else if(localize_result < 0) {
      if(++retry_localization >= state_utils->getLocalErrorCount()){
        RCLCPP_INFO(node_->get_logger(), "[ManualControl] localization error count[%u]over", retry_localization);
        retry_localization = 0;
        state_utils->publishLocalizationFailErrorStatus(true);
        setReadyManualControl(READY_MANUAL_CONTROL::FAIL);
      }else{
        RCLCPP_INFO(node_->get_logger(), "[ManualControl] localization fail re-start Node count[%u]", retry_localization);
        setReadyManualControl(READY_MANUAL_CONTROL::STOP_NODE);
      }
    }
    break;  
  case READY_MANUAL_CONTROL::COMPLETE :
    publishStreamingCmd(true);
    ret = ROBOT_STATUS::START;
    break;
  case READY_MANUAL_CONTROL::FAIL :
    ret = ROBOT_STATUS::FAIL;
    break;
  case READY_MANUAL_CONTROL::STOP_NODE :
    state_utils->send_node_goal(NODE_STATUS::IDLE);
    setReadyManualControl(READY_MANUAL_CONTROL::CHECK_STOP_NODE);
    break;
  case READY_MANUAL_CONTROL::CHECK_STOP_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      setReadyManualControl(READY_MANUAL_CONTROL::LAUNCH_NODE);
    }else if(node_result < 0){
      setReadyManualControl(READY_MANUAL_CONTROL::FAIL);
    }
    break;         
  default:
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] processNavigationReady readyState Error!! : %d", static_cast<int>(ready_working));
    break;
  }

  return ret;
}

void ManualControl::setReadyManualControl(READY_MANUAL_CONTROL set)
{
  if(ready_working != set){
    pre_ready_working = ready_working;
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] setReadyManualControl %s", enumToString(set).c_str());
  }
  ready_working = set;
}

int8_t ManualControl::localizationChecker() {
  int8_t ret = 0;
  if(state_utils->isStartLocalization()){
    if(state_utils->getLocalizationComplete()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] localization Done ");
      ret = 1;
    }else if(state_utils->isLocalizationError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualControl] localization Error ");
      ret = -1;
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[ManualControl] skip localization");
    ret = 1;
  }

  return ret;
}

void ManualControl::publishStreamingCmd(const bool &cmd) {
  auto req_following_cmd_msg = follow_msgs::msg::UiClient();
  int cmd_value = cmd == true ? 1 : 4;
  req_following_cmd_msg.cmd = static_cast<int>(cmd);
  req_following_cmd_msg.direction = false;
  req_following_cmd_msg.offset_x = 0.0;
  req_following_cmd_msg.offset_y = 0.0;
  req_following_cmd_msg.max_speed = 0.0;
  streaming_cmd_pub->publish( req_following_cmd_msg );
  return;
}

} // namespace airbot_state