#include "state_manager/states/following.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

Following::Following(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliable();

  rclcpp::QoS qos_best_effort_profile = rclcpp::QoS(5)
                            .best_effort()
                            .durability_volatile();

  following_cmd_pub = node_->create_publisher<follow_msgs::msg::UiClient>("/follow/ui/client", qos_profile);
  following_state_sub_ = node_->create_subscription<follow_msgs::msg::Detect>("/follow/detectuser", qos_best_effort_profile, std::bind(&Following::followingStateCallback, this, std::placeholders::_1));
}

void Following::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[FOLLOWING] Preparing FOLLOWING state");
  state_utils->startSensorMonitor();
  state_utils->setMovingStateID(NAVI_STATE::FOLLOWING);
  setReadyFollowing(READY_FOLLOWING::CHECK_SENSOR);
  pub_cnt = 0;
  retry_localization = 0;
  bNeedsInitPoseLocalization = false;
  pause_flag = false;
  // state_utils->setCmdGlobalLocalizationMode(true);
}

void Following::run(const std::shared_ptr<StateUtils> &state_utils) {
  ROBOT_STATUS ready_check;
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[FOLLOWING] run() -> running FOLLOWING state");
  }

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    if( state_utils->getStopDrivingFlag() ){
      ready_check = processFollowingReady();
      state_utils->setStatusID(ready_check);
    }
    break;
  case ROBOT_STATUS::START :
    // FollowMe 동작 중 PAUSE/RESUME 사양 없음.(26/01/30)
    // if( pause_flag ){
    //   publishFollowCmd(FOLLOW_CMD::FOLLOW_RESUME);
    //   RCLCPP_INFO(node_->get_logger(), "[Following] RESUME FOLLOWING - SEND : %s", enumToString(FOLLOW_CMD::FOLLOW_RESUME).c_str());
    //   pause_flag = false;
    // }
    break;
  case ROBOT_STATUS::PAUSE :
    // FollowMe 동작 중 PAUSE/RESUME 사양 없음.(26/01/30)
    // if( !pause_flag ){
    //   publishFollowCmd(FOLLOW_CMD::FOLLOW_PAUSE);
    //   RCLCPP_INFO(node_->get_logger(), "[Following] PAUSE FOLLOWING - SEND : %s", enumToString(FOLLOW_CMD::FOLLOW_PAUSE).c_str());
    //   pause_flag = true;
    // }
    break;
  case ROBOT_STATUS::COMPLETE :
    // complete 없음.
    break;
  case ROBOT_STATUS::FAIL :
    RCLCPP_INFO(node_->get_logger(), "[Following] follow fail");
    break;
  default:
    RCLCPP_INFO(node_->get_logger(), "[Following] Running UNKNOWN Status");
    break;
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[Following]Robot on Docking Station!!!");
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::START_ONSTATION);
  }
}

void Following::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[FOLLOWING] post_run() -> exiting FOLLOWING state");
  //state_utils->saveLastPosition();
  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  handleFollowFinish(); //stopDriving();
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
}

///////////////function in Following

ROBOT_STATUS Following::processFollowingReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  int localize_result = 0;
  switch (ready_working)
  {
    case READY_FOLLOWING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyFollowing(READY_FOLLOWING::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[Following] lidar Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyFollowing(READY_FOLLOWING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[Following] tof Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyFollowing(READY_FOLLOWING::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[Following] CameraError");
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      setReadyFollowing(READY_FOLLOWING::CHECK_ODOM_RESET);
      //ret = ROBOT_STATUS::FAIL;
      //setReadyFollowing(READY_MAPPING::FAIL);
    }
    break;
  case READY_FOLLOWING::CHECK_ODOM_RESET :
    if(!state_utils->isStartOdomReset()){
      RCLCPP_INFO(node_->get_logger(), "[Following] odom-reset not be started");
      bNeedsInitPoseLocalization = false;
      setReadyFollowing(READY_FOLLOWING::LAUNCH_FOLLOW_NODE);
    }else if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[Following] odom reset Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyFollowing(READY_FOLLOWING::FAIL);
    }else if(state_utils->getOdomResetDone()){
      bNeedsInitPoseLocalization = true;
      setReadyFollowing(READY_FOLLOWING::LAUNCH_FOLLOW_NODE);
    }
    break;
  case READY_FOLLOWING::LAUNCH_FOLLOW_NODE :
    state_utils->send_node_goal(NODE_STATUS::FOLLOWING);
    setReadyFollowing(READY_FOLLOWING::CHECK_NODE);
    break;
  case READY_FOLLOWING::LAUNCH_LOCAL_NODE :
    /*hjkim260113 : 네비게이션 노드 실행 시 skipLocal 플래그가 false가 되어 로컬을 실행하기 때문에, 
    네비게이션 노드가 이미 켜져있는 경우에는 실행하지 않도록 해야함.*/
    if(state_utils->getNodeStatusID() != NODE_STATUS::NAVI){
      state_utils->send_node_goal(NODE_STATUS::NAVI);
      setReadyFollowing(READY_FOLLOWING::CHECK_NODE);
    }else{
      setReadyFollowing(READY_FOLLOWING::READY_LOCALZIATION);
    }
    break;
  case READY_FOLLOWING::CHECK_NODE :
    node_result = state_utils->getNodeClientStatus();
    if( pre_ready_working  == READY_FOLLOWING::LAUNCH_FOLLOW_NODE){
      if(node_result > 0){
        setReadyFollowing(READY_FOLLOWING::LAUNCH_LOCAL_NODE);
      } else if(node_result < 0){
        setReadyFollowing(READY_FOLLOWING::FAIL);
      }
    } else{
      if(node_result > 0){
        setReadyFollowing(READY_FOLLOWING::READY_LOCALZIATION);
      }else if(node_result < 0){
        setReadyFollowing(READY_FOLLOWING::STOP_NODE);
      }
    }
    break;
    case READY_FOLLOWING::READY_LOCALZIATION:
     /* hjkim260113 : 라이다를 끄거나, 네비게이션 노드를 켜라고 명령을 던지면 state_utils->send_node_goal(NODE_STATUS::NAVI)
     state_utils->isSkipLocalization() 가 false로 변경되어 로컬을 실행하게 됨.
     true 케이스는 로컬을 완료한 시점으로 로컬을 한번도 잡은적이 없으면 오돔 reset과 상관없이 로컬을 실행하면 됨.
     오돔 reset을 한적이 없기때문에 로컬을 실행하지 않는 경우는 init_pose로 로컬을 하는 경우에만 해당함.*/
      if(state_utils->isSkipLocalization()){
        RCLCPP_INFO(node_->get_logger(), "[Following] init-pose localization skip cause it hasn`t reset odom");
        setReadyFollowing(READY_FOLLOWING::COMPLETE);
      }else{
#if USE_JSLLOC > 0
          local_mode_retry_cnt = 0;
          state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
          setReadyFollowing(READY_FOLLOWING::START_LOCALIZATION);
          localization_start_time = std::chrono::steady_clock::now();
#else
          state_utils->setCheckAmclAfterLocalization();
          state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
          setReadyFollowing(READY_FOLLOWING::CHECK_LOCALIZATION);
#endif
      } 
      break;
    case READY_FOLLOWING::START_LOCALIZATION :
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
      setReadyFollowing(READY_FOLLOWING::CHECK_LOCALIZATION);
      RCLCPP_INFO(node_->get_logger(), "[Following] complete localization mode change cmd[%s] current-mode[%s] ",
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
        setReadyFollowing(READY_FOLLOWING::CHECK_LOCALIZATION);
        RCLCPP_INFO(node_->get_logger(), "[Following] retry 3time localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }else{
        state_utils->publishLocalizationMode(state_utils->getCmdGlobalLocalizationMode());
        localization_start_time = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "[Following] retry localization mode change cmd[%s] current-mode[%s] ",
        state_utils->getCmdGlobalLocalizationMode() ? "GLOBAL_ALLOWED" : "LOCAL_ONLY" ,enumToString(state_utils->getLocalizationMode()).c_str());
      }
    }
    break;  
  case READY_FOLLOWING::CHECK_LOCALIZATION :
    localize_result = localizationChecker();
    if(localize_result > 0){
      if(state_utils->getRecoveryRebootflag()){
        state_utils->removeRecoveryDirectory();
        state_utils->setRecoveryRebootflag(false);
        RCLCPP_INFO(node_->get_logger(),"[Following] Recovery Reboot Flag Clear");
      }
      setReadyFollowing(READY_FOLLOWING::COMPLETE);
    } 
    else if(localize_result < 0) {
      if(++retry_localization >= state_utils->getLocalErrorCount()){
        RCLCPP_INFO(node_->get_logger(), "[Following] localization error count[%u]over", retry_localization);
        retry_localization = 0;
        state_utils->publishLocalizationFailErrorStatus(true);
        setReadyFollowing(READY_FOLLOWING::FAIL);
      }else{
        RCLCPP_INFO(node_->get_logger(), "[Following] localization fail re-start Node count[%u]", retry_localization);
        setReadyFollowing(READY_FOLLOWING::STOP_NODE);
      }
    }
    break;  
  case READY_FOLLOWING::COMPLETE :
    // FollowMe 동작 중 PAUSE/RESUME 사양 없음.(26/01/30)
    // if( state_utils->getReservePause() ){
    //   RCLCPP_INFO(node_->get_logger(), "[Following] Go to PAUSE STATUS due to pause reservation!" );
    //   ret = ROBOT_STATUS::PAUSE; //reserve pause 인 경우 Pause진입.
    // } else{
    publishFollowCmd(FOLLOW_CMD::FOLLOW_START);
    ret = ROBOT_STATUS::START;
    // }
    break;
  case READY_FOLLOWING::FAIL :
    ret = ROBOT_STATUS::FAIL;
    break;
  case READY_FOLLOWING::STOP_NODE :
    state_utils->send_node_goal(NODE_STATUS::IDLE);
    setReadyFollowing(READY_FOLLOWING::CHECK_STOP_NODE);
    break;
  case READY_FOLLOWING::CHECK_STOP_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      setReadyFollowing(READY_FOLLOWING::LAUNCH_LOCAL_NODE);
    }else if(node_result < 0){
      setReadyFollowing(READY_FOLLOWING::FAIL);
    }
    break;         
  default:
    RCLCPP_INFO(node_->get_logger(), "[Following] processNavigationReady readyState Error!! : %d", static_cast<int>(ready_working));
    break;
  }

  return ret;
}

void Following::setReadyFollowing(READY_FOLLOWING set)
{
  if(ready_working != set){
    pre_ready_working = ready_working;
    RCLCPP_INFO(node_->get_logger(), "[Following] setReadyFollowing %s", enumToString(set).c_str());
  }
  ready_working = set;
}

void Following::handleFollowFinish() {
  state_cmd tmp_cmd;
  tmp_cmd = state_utils->getRobotCMDID();
  RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish [ Current status : %s ]|[ SOC CMD : %s ]|[ ROBOT CMD : %s ]", enumToString(FOLLOWING_STATE(follow_status)).c_str(), enumToString(tmp_cmd.soc_cmd).c_str(), enumToString(tmp_cmd.robot_cmd).c_str());

  if(tmp_cmd.soc_cmd == REQUEST_SOC_CMD::VOID && tmp_cmd.robot_cmd == REQUEST_ROBOT_CMD::VOID) // 둘다 VOID인 경우.
  {
    RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish [UN-NORMAL FINISHED] VOID command received. but SEND : FOLLOW_STOP_SOC");
    publishFollowCmd(FOLLOW_CMD::FOLLOW_STOP_SOC);//SOC 명령이 상위요청이므로 STOP SOC전달.
    return;
  } else{ // SOC 또는 ROBOT CMD가 있을때.
    if(tmp_cmd.soc_cmd != REQUEST_SOC_CMD::VOID){ //SOC 명령에 의한 경우.

      if(tmp_cmd.soc_cmd == REQUEST_SOC_CMD::START_FOLLOWING){ //시작 명령에 의해 온 경우 follow해야함 stop보내지 않음.
        RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish [UN-NORMAL FINISHED] SOC COMMAND : %s but not send stop.", enumToString(tmp_cmd.soc_cmd).c_str());
        return;
      }

      if(tmp_cmd.robot_cmd != REQUEST_ROBOT_CMD::VOID){ //SOC 명령이지만 ROBOT_CMD가 있는 경우.
        publishFollowCmd(FOLLOW_CMD::FOLLOW_STOP_SOC); //SOC 명령이 상위요청이므로 STOP SOC전달.
        RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish [UN-NORMAL FINISHED] SOC COMMAND : %s, ROBOT COMMAND : %s, but SEND : FOLLOW_STOP_SOC", enumToString(tmp_cmd.soc_cmd).c_str(), enumToString(tmp_cmd.robot_cmd).c_str());
        return;
      } else{ // 정상적인 SOC명령인 경우.
        publishFollowCmd(FOLLOW_CMD::FOLLOW_STOP_SOC);
        RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish -> FINISHED BY [SOC COMMAND] : %s || SEND : %s", enumToString(tmp_cmd.soc_cmd).c_str(), enumToString(FOLLOW_CMD::FOLLOW_STOP_SOC).c_str());
        return;
      }
    } else{ //SOC CMD VOID
      //SOC 명령이 없고, ROBOT_CMD만 있는 경우.(SOC CMD가 있다면 상위 조건에서 처리됨. ++ robot cmd가 void면 상위에서 처리됨.)
      if( tmp_cmd.robot_cmd == REQUEST_ROBOT_CMD::CANCEL_FOLLOWING_NOT_MOVING ){ //Follow node 자체 종료로 AMR도 자체종료. 전송 없음.
        RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish -> FINISHED BY [FOLLOW NODE] : %s || SEND NOTHING", enumToString(FOLLOWING_STATE(follow_status)).c_str());
        return;
      } else{ //SOC 명령이 없고, ROBOT_CMD가 NOT_MOVING이 아닐때.
        publishFollowCmd(FOLLOW_CMD::FOLLOW_STOP_AMR);
        RCLCPP_INFO(node_->get_logger(), "[Following] handleFollowFinish -> FINISHED BY [ROBOT COMMAND] : %s || SEND : %s", enumToString(tmp_cmd.robot_cmd).c_str(), enumToString(FOLLOW_CMD::FOLLOW_STOP_AMR).c_str());
        return;
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(), "handle following finish Done ");
}

void Following::publishFollowCmd(const FOLLOW_CMD &cmd) {
  auto req_following_cmd_msg = follow_msgs::msg::UiClient();

  if( pub_cnt++ < 15 )
  {
    req_following_cmd_msg.cmd = static_cast<int>(cmd);
    req_following_cmd_msg.direction = false;
    req_following_cmd_msg.offset_x = 0.0;
    req_following_cmd_msg.offset_y = 0.0;
    req_following_cmd_msg.max_speed = 0.0;
    following_cmd_pub->publish( req_following_cmd_msg );
  }
  else{
    pub_cnt = 0;
  }

  return;
}

int8_t Following::localizationChecker() {
  int8_t ret = 0;
  if(state_utils->isStartLocalization()){
    if(state_utils->getLocalizationComplete()){
      RCLCPP_INFO(node_->get_logger(), "[Following] localization Done ");
      ret = 1;
    }else if(state_utils->isLocalizationError()){
      RCLCPP_INFO(node_->get_logger(), "[Following] localization Error ");
      ret = -1;
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[Following] skip localization");
    ret = 1;
  }

  return ret;
}

void Following::followingStateCallback(const follow_msgs::msg::Detect::SharedPtr msg) {
  int receive_status = msg->detect_user;
  if( pre_follow_status != receive_status ){
    follow_status = receive_status;
    RCLCPP_INFO(node_->get_logger(), "Pre Status : %s ==> Cur Status : %s", enumToString(FOLLOWING_STATE(pre_follow_status)).c_str(), enumToString(FOLLOWING_STATE(follow_status)).c_str());
    pre_follow_status = follow_status;

    if( FOLLOWING_STATE(follow_status) == FOLLOWING_STATE::FOLLOWING_CANCELED_NOT_MOVING ){
      state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::CANCEL_FOLLOWING_NOT_MOVING); // follow node 자체 종료시 done으로 IDLE로 보냄.
    }
  }
}


} // namespace airbot_state