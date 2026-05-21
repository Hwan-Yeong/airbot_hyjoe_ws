#include "state_manager/states/manual_mapping.hpp"

namespace airbot_state {

ManualMapping::ManualMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void ManualMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] pre_run() -> Preparing ManualMapping state");
  mapping_start_time = std::numeric_limits<double>::max();
  setReadyMapping(READY_MAPPING::CHECK_SENSOR);
  
  if(state_utils->getRobotCMDID().robot_cmd != REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING){
    RCLCPP_INFO(node_->get_logger(), "[WARNING] mapping start on station, but robot_cmd is [%s]", enumToString(state_utils->getRobotCMDID().robot_cmd).c_str());
  }
  #if USE_LIDAR_STATE_CHECK == 0
  state_utils->publishScanMonitor(true);
  #endif
  state_utils->publishTFMonitor(true);
  bMapSave = false;
}

void ManualMapping::setReadyMapping(READY_MAPPING set)
{
  if(ready_mapping != set){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] setReadyMapping %s", enumToString(set).c_str());
  }
  ready_mapping = set;
}

void ManualMapping::run(const std::shared_ptr<StateUtils> &state_utils) {
  ROBOT_STATUS ready_check;
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] run() -> running ManualMapping state");
  }

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    ready_check = processMappingReady();
    state_utils->setStatusID(ready_check);
    if(ready_check == ROBOT_STATUS::START){
      state_utils->publishRemoteBlock(false); //hjkim250814 : ready manual mapping complete --> unblock remote control
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] mapping start");
    }else if(ready_check == ROBOT_STATUS::FAIL){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] mapping fail");
    }
    break;
  case ROBOT_STATUS::START :
    if(!bMapSave && state_utils->isRunningDocking()){
      state_utils->map_saver();
      bMapSave = true;
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] detect mcu docking start --> save map");
    }
    break;
  case ROBOT_STATUS::PAUSE :
    break;
  case ROBOT_STATUS::COMPLETE :
    state_utils->publishRobotCommand(REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING);
    state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
    break;
  case ROBOT_STATUS::FAIL :

    break;      
  default:
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Running UNKNOWN Status");
    break;
  }

  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping]Robot on Docking Station!!!");
    state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
  }
}

void ManualMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->stopDriving();
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] post_run() -> Exiting ManualMapping state");
  state_utils->stopMonitorOdom();
  state_utils->stopSensorMonitor();
  state_utils->publishRemoteBlock(true); //hjkim250814 : exit manual mapping state --> block remote control
  //hjkim : 수동 매핑 중 stop_working (매핑취소)의 경우 맵을 저장하지 않도록 수정 automapping과 동일하게 ONSTATION으로 인한 종료시에만 저장(QUBER에서 맵을 가져가는 조건)
  if(state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::START_ONSTATION && !bMapSave){
   state_utils->map_saver();
  }
}

///////////////function in ManualMapping

ROBOT_STATUS ManualMapping::processMappingReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int node_result = 0;
  switch (ready_mapping)
  {
  case READY_MAPPING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyMapping(READY_MAPPING::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] lidar Error");
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] tof Error");
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->isCamreaError()){
      //hjkim : camera_error 발생하더라도 이동은 시키도록 수정
      setReadyMapping(READY_MAPPING::CHECK_ODOM_RESET);
      //RCLCPP_INFO(node_->get_logger(), "[ManualMapping] CameraError");
      //setReadyMapping(READY_MAPPING::FAIL);
    }
    break;     
  case READY_MAPPING::CHECK_ODOM_RESET :
    if(!state_utils->isStartOdomReset()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] odom-reset not be started");
      state_utils->startMonitorOdomReset(); //hjkim : in case odom-reset not be started(not be started on statation) -> odom-reset start
    }else if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] odom reset Error");
      setReadyMapping(READY_MAPPING::FAIL);
    }else if(state_utils->getOdomResetDone()){
      state_utils->send_node_goal(NODE_STATUS::MANUAL_MAPPING);
      setReadyMapping(READY_MAPPING::CHECK_NODE_LAUNCH);
    }
    break;
  case READY_MAPPING::CHECK_NODE_LAUNCH :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] ManualMapping Node Launched");
      state_utils->publishKeepOutEnable(false);
      keepout_enable_cnt = 0;
      setReadyMapping(READY_MAPPING::CHECK_KEEPOUT_STATE);//setReadyMapping(READY_MAPPING::COMPLETE);
    }else if(node_result < 0){
      setReadyMapping(READY_MAPPING::FAIL);
    }
    break;
  case READY_MAPPING::CHECK_KEEPOUT_STATE:
	  if (state_utils->getKeepoutState() == KEEPOUT_STATE::KEEPOUT_DISABLE) {
	    state_utils->publishClearCostMap();
      setReadyMapping(READY_MAPPING::COMPLETE);
	  } else if (state_utils->getSteadyClockRunningSeconds(state_utils->getKeepOutEnableStartTime()) > 3) {
	    if (keepout_enable_cnt++ < 3) {
	      state_utils->publishKeepOutEnable(false);
	      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] KeepOut Enable retry %d", keepout_enable_cnt);
	    } else {
	      state_utils->publishClearCostMap();
	      RCLCPP_INFO(node_->get_logger(), "[ManualMapping] KeepOut Enable Fail 3 Times");
        setReadyMapping(READY_MAPPING::COMPLETE);
	    }
	  }
	  break;
  case READY_MAPPING::COMPLETE :
    //state_utils->resetInitPose(); hjkim : 맵핑 시작 시 충전기위치와 초기위치(복귀좌표) 초기화 -> 매핑 완료 후 맵 저장 시 초기화 하도록 수정
    ret = ROBOT_STATUS::START;
    break;  
  case READY_MAPPING::FAIL :
    ret = ROBOT_STATUS::FAIL;
    break;   
  default:
    RCLCPP_INFO(node_->get_logger(), "[ManualMapping] processMappingReady readyState Error!! : %d", static_cast<int>(ready_mapping));
    break;
  }

  return ret;
}

} // namespace airbot_state