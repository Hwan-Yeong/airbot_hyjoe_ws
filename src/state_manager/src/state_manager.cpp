#include "state_manager/state_manager.hpp"

#define USE_JSLLOC 1

using namespace std::chrono_literals;

namespace airbot_state {

StateManager::StateManager() : Node("state_manager") {
  state_utils = std::make_shared<StateUtils>(std::shared_ptr<rclcpp::Node>(this));
  idle = std::make_shared<Idle>(int(ROBOT_STATE::IDLE), std::shared_ptr<rclcpp::Node>(this), state_utils);
  auto_mapping = std::make_shared<AutoMapping>(int(ROBOT_STATE::AUTO_MAPPING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  manual_mapping = std::make_shared<ManualMapping>(int(ROBOT_STATE::MANUAL_MAPPING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  navigation = std::make_shared<Navigation>(int(ROBOT_STATE::NAVIGATION), std::shared_ptr<rclcpp::Node>(this), state_utils);
  return_charger = std::make_shared<ReturnCharger>(int(ROBOT_STATE::RETURN_CHARGER), std::shared_ptr<rclcpp::Node>(this), state_utils);
  docking = std::make_shared<Docking>(int(ROBOT_STATE::DOCKING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  undocking = std::make_shared<UnDocking>(int(ROBOT_STATE::UNDOCKING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  on_station = std::make_shared<OnStation>(int(ROBOT_STATE::ONSTATION), std::shared_ptr<rclcpp::Node>(this), state_utils);
  factory_navigation = std::make_shared<FactoryNavigation>(int(ROBOT_STATE::FACTORY_NAVIGATION), std::shared_ptr<rclcpp::Node>(this), state_utils);
  error = std::make_shared<Error>(int(ROBOT_STATE::ERROR), std::shared_ptr<rclcpp::Node>(this), state_utils);
  following = std::make_shared<Following>(int(ROBOT_STATE::FOLLOWING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  manual_control = std::make_shared<ManualControl>(int(ROBOT_STATE::MANUAL_CONTROL), std::shared_ptr<rclcpp::Node>(this), state_utils);

  states_[ROBOT_STATE::IDLE] = idle;
  states_[ROBOT_STATE::AUTO_MAPPING] = auto_mapping;
  states_[ROBOT_STATE::MANUAL_MAPPING] = manual_mapping;
  states_[ROBOT_STATE::NAVIGATION] = navigation;
  states_[ROBOT_STATE::RETURN_CHARGER] = return_charger;
  states_[ROBOT_STATE::DOCKING] = docking;
  states_[ROBOT_STATE::UNDOCKING] = undocking;
  states_[ROBOT_STATE::ONSTATION] = on_station;
  states_[ROBOT_STATE::FACTORY_NAVIGATION] = factory_navigation;
  states_[ROBOT_STATE::ERROR] = error;
  states_[ROBOT_STATE::FOLLOWING] = following;
  states_[ROBOT_STATE::MANUAL_CONTROL] = manual_control;
  current_state_ = idle;
  setCurrentStateID(ROBOT_STATE::IDLE);

  robot_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>("robot_state_cmd", 10, std::bind(&StateManager::handleRobotCMD, this,std::placeholders::_1));
  soc_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/soc_cmd", 10, std::bind(&StateManager::handleSoCCMD, this, std::placeholders::_1));
  error_list_sub_ = this->create_subscription<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10, std::bind(&StateManager::handleError, this, std::placeholders::_1));
  battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>("/battery_status", 10, std::bind(&StateManager::handleBattery, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateManager::runCurrentState, this));
  RCLCPP_INFO(this->get_logger(), "[StateManager] node initialized.");
}

StateManager::~StateManager() {
  RCLCPP_INFO(this->get_logger(), "[StateManager] node terminated");
}

void StateManager::setState(ROBOT_STATE state_id, ROBOT_STATUS status_id, const state_cmd &cmd_ids) {
  if (states_.find(state_id) == states_.end()) {
    RCLCPP_ERROR(this->get_logger(), "[StateManager] UNDEFINED STATE ID: %d ", int(state_id));
    return;
  }
  state_utils->setRobotCMDID(cmd_ids);
  RCLCPP_INFO(this->get_logger(),
              "[StateManager] EXIT STATE: [%s] -> Transition by SOC:[%s]|ROBOT:[%s] -> RUN STATE: [%s]",
              enumToString(state_utils->getStateID()).c_str(),
              enumToString(state_utils->getRobotCMDID().soc_cmd).c_str(), enumToString(state_utils->getRobotCMDID().robot_cmd).c_str(),
              enumToString(state_id).c_str());

  if (getCurrentStateID() != state_id) {
    // off current state
    if (current_state_) {
      current_state_->post_run(state_utils);
    }
    // change current state
    pre_cmds = cmd_ids;
    current_state_ = states_[state_id];
    current_state_->pre_run(state_utils);
    setCurrentStateID(state_id);
  }
  state_utils->setAllRobotStateIDs(state_id, status_id, cmd_ids);
}

void StateManager::checkTransition( const state_cmd &cmd_ids )
{
  // TRANSITION BY SOC COMMAND
  ROBOT_STATE current_state = getCurrentStateID();
  switch (cmd_ids.soc_cmd) {
  case REQUEST_SOC_CMD::VOID:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
    break;
  case REQUEST_SOC_CMD::START_AUTO_MAPPING:
    switch( current_state ){
    case ROBOT_STATE::IDLE:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ERROR:
      if( state_utils->getOnstationStatus() ){
        state_utils->startSensorMonitor();
        setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else{
        state_utils->startSensorMonitor();
        setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      }
      break;
    default:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;    
    }
    break;
  case REQUEST_SOC_CMD::START_MANUAL_MAPPING:
    switch( current_state ){
    case ROBOT_STATE::IDLE:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ERROR:
      if( state_utils->getOnstationStatus() ){
        state_utils->startSensorMonitor();
        setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else{
        state_utils->startSensorMonitor();
        setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      }
      break;
    default:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_NAVIGATION:
    switch(current_state){
    case ROBOT_STATE::IDLE:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::RETURN_CHARGER:
    case ROBOT_STATE::DOCKING:
    case ROBOT_STATE::FOLLOWING:
    case ROBOT_STATE::MANUAL_CONTROL:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ERROR:
      if( state_utils->getOnstationStatus() ){
        state_utils->startSensorMonitor();
        setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else{
        state_utils->startSensorMonitor();
        setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_RETURN_CHARGER:
    switch(current_state){
    case ROBOT_STATE::AUTO_MAPPING:
    case ROBOT_STATE::MANUAL_MAPPING:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::FACTORY_NAVIGATION:
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
      break;
    #if USE_JSLLOC > 0
    case ROBOT_STATE::IDLE:
    case ROBOT_STATE::ERROR:
    case ROBOT_STATE::FOLLOWING:
    case ROBOT_STATE::MANUAL_CONTROL:
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
      break;
    #else
    case ROBOT_STATE::ERROR:
      if(state_utils->getPreStateID() == ROBOT_STATE::DOCKING){ //S08 도킹불가 또는 도킹상태에서 에러발생한 경우, docking 진행.
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else{
        //Node가 IDLE일 때는 로봇을 부팅, 매핑 후 스테이션 위에 있을 경우.
        //Node가 IDLE일 때 docking으로 보냄.(navi가 켜져있지 않은 상태, global reloc이 없기 떄문)
        if(state_utils->getNodeStatusID() == NODE_STATUS::IDLE){ 
          setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
        } else{
          setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
        }
      }
      break;
    case ROBOT_STATE::IDLE:
      if(state_utils->getRecoveryRebootflag()){
        RCLCPP_INFO(this->get_logger(), "[StateManager] recovery json load success start return charger");
        setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
      }else{
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      }
      break;
    case ROBOT_STATE::FOLLOWING:
    #endif
    case ROBOT_STATE::DOCKING: // DOCKING중인데 start return charger가 온 경우.
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::UNDOCKING://단자 닿아있을 때, return charger오는 것 방지.docking 예약.
      state_utils->setReserveDocking(true); // UNDOCKING 일 때 return charger가 오면 docking예약.
      break;
    default:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_DOCKING:
    switch(current_state){
    case ROBOT_STATE::IDLE:
    case ROBOT_STATE::UNDOCKING:
    case ROBOT_STATE::AUTO_MAPPING:
    case ROBOT_STATE::MANUAL_MAPPING:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::FACTORY_NAVIGATION:
    case ROBOT_STATE::DOCKING:
    case ROBOT_STATE::RETURN_CHARGER:
    case ROBOT_STATE::ERROR:
    case ROBOT_STATE::FOLLOWING:
    case ROBOT_STATE::MANUAL_CONTROL:
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_CHARGING:
  RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
    break;
  case REQUEST_SOC_CMD::PAUSE_WORKING:
    if( state_utils->getMovingStateID() == NAVI_STATE::START_ROTAION){
      state_utils->setRotatePauseFlag(true);
    }
    switch(current_state){
    case ROBOT_STATE::NAVIGATION:
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    case ROBOT_STATE::RETURN_CHARGER:
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    case ROBOT_STATE::FACTORY_NAVIGATION:
      setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    case ROBOT_STATE::DOCKING:
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    case ROBOT_STATE::AUTO_MAPPING:
      setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    // case ROBOT_STATE::FOLLOWING: //following pause 사양 없음.
      // setState(ROBOT_STATE::FOLLOWING, ROBOT_STATUS::PAUSE, cmd_ids);
      // break;
    case ROBOT_STATE::IDLE:
      if ( !state_utils->getFactoryMode() ){
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::PAUSE, cmd_ids);
      }
      break;
    case ROBOT_STATE::ERROR:
      if ( !state_utils->getFactoryMode() ){
        setState(ROBOT_STATE::ERROR, ROBOT_STATUS::PAUSE, cmd_ids);
      }
      break;
    case ROBOT_STATE::UNDOCKING:
      RCLCPP_INFO(this->get_logger(),"[StateManager] RESERVE PAUSE --> PAUSE CMD received during undocking");
      state_utils->setReservePause(true); // UNDOCKING 일 때 pause가 오면 pause예약.
      break;  
    default:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;
    }
    break;
  case REQUEST_SOC_CMD::RESUME_WORKING:
    if( state_utils->getStatusID() == ROBOT_STATUS::PAUSE){
      switch(current_state){
      case ROBOT_STATE::NAVIGATION:
        setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::START, cmd_ids);
        break;
      case ROBOT_STATE::RETURN_CHARGER:
        setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
        break;
      case ROBOT_STATE::FACTORY_NAVIGATION:
        setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::START, cmd_ids);
        break;
      case ROBOT_STATE::DOCKING:
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
        break;
      case ROBOT_STATE::AUTO_MAPPING:
        setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::START, cmd_ids);
        break;
      // case ROBOT_STATE::FOLLOWING: // following resume 사양 없음.
      //   setState(ROBOT_STATE::FOLLOWING, ROBOT_STATUS::START, cmd_ids);
      //   break;
      case ROBOT_STATE::IDLE:
        if ( !state_utils->getFactoryMode() ){
          setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
        }
        break;
      case ROBOT_STATE::ERROR:
        if ( !state_utils->getFactoryMode() ){
          setState(ROBOT_STATE::ERROR, ROBOT_STATUS::START, cmd_ids);
        }
        break;
      default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
        break;
      }
    } else{
      RCLCPP_ERROR(this->get_logger(), "[StateManager] CANNOT RESUME, NOT PAUSE STATUS --> soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
    }
    break;
  case REQUEST_SOC_CMD::START_FACTORY_NAVIGATION:
    switch(current_state){
    case ROBOT_STATE::ONSTATION:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ERROR:
      state_utils->startSensorMonitor();
      if( state_utils->getOnstationStatus() ){
        setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else{
        setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      }  
      break;
    default:
      state_utils->startSensorMonitor();
      setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      break;    
    }
    break;

  case REQUEST_SOC_CMD::START_FOLLOWING:
    state_utils->startSensorMonitor();
    switch(current_state){
      case ROBOT_STATE::ONSTATION:
        setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
        break;
      default:
        setState(ROBOT_STATE::FOLLOWING, ROBOT_STATUS::READY, cmd_ids);
        break;    
      }
    break;
  
  case REQUEST_SOC_CMD::START_MANUAL_CONTROL:
    state_utils->startSensorMonitor();
    switch(current_state){
      case ROBOT_STATE::ONSTATION:
        setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
        break;
      default:
        setState(ROBOT_STATE::MANUAL_CONTROL, ROBOT_STATUS::READY, cmd_ids);
        break;    
      }
    break;
  
  case REQUEST_SOC_CMD::STOP_WORKING:
    switch(current_state){
    case ROBOT_STATE::MANUAL_MAPPING:
    case ROBOT_STATE::AUTO_MAPPING:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::RETURN_CHARGER:
    case ROBOT_STATE::DOCKING:
    case ROBOT_STATE::UNDOCKING:
    case ROBOT_STATE::FACTORY_NAVIGATION:
    case ROBOT_STATE::ERROR:
    case ROBOT_STATE::FOLLOWING:
    case ROBOT_STATE::MANUAL_CONTROL:
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] soc-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.soc_cmd).c_str(), enumToString(current_state).c_str());
      break;    
    }
    break;
  }
  
  if( pre_cmds.robot_cmd != cmd_ids.robot_cmd) // TRANSITION BY ROBOT COMMAND
  {
    switch (cmd_ids.robot_cmd) {
    case REQUEST_ROBOT_CMD::READY_REBOOT:
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::READY, cmd_ids);
      break;
    case REQUEST_ROBOT_CMD::START_ONSTATION:
      releaseError();
      setState(ROBOT_STATE::ONSTATION, ROBOT_STATUS::READY, cmd_ids);
      break;
    case REQUEST_ROBOT_CMD::STOP_ONSTATION:
      if (current_state == ROBOT_STATE::ONSTATION) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING:
      if (current_state == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING:
      if (current_state == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION:
      if (current_state == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FACTORY_NAVIGATION:
      if (current_state == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING:
      if (current_state == ROBOT_STATE::AUTO_MAPPING) {
        setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING:
      if (current_state == ROBOT_STATE::MANUAL_MAPPING) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_DOCKING:
      if (current_state == ROBOT_STATE::DOCKING) {
        setState(ROBOT_STATE::ONSTATION, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER:
      if (current_state == ROBOT_STATE::RETURN_CHARGER) {
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING:
      if (current_state == ROBOT_STATE::RETURN_CHARGER) {
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FOLLOWING:
      if (current_state == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::FOLLOWING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    case REQUEST_ROBOT_CMD::CANCEL_FOLLOWING_NOT_MOVING:
      if (getCurrentStateID() == ROBOT_STATE::FOLLOWING) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::OFFSTATION_TRY_DOCKING:
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_CONTROL:
      if (current_state == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::MANUAL_CONTROL, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      }
      break;
    default:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] robot-cmd : [%s] is not possible in state : [%s]", enumToString(cmd_ids.robot_cmd).c_str(), enumToString(current_state).c_str());
      break;
    }
  }
  pre_cmds = cmd_ids;
}

void StateManager::setCurrentStateID(const ROBOT_STATE data) {
  current_state = data;
}

ROBOT_STATE StateManager::getCurrentStateID() { 
  return current_state;
}

void StateManager::handleSoCCMD(const std_msgs::msg::UInt8::SharedPtr msg) {
  state_cmd cmds;
  const REQUEST_SOC_CMD req_cmd = static_cast<REQUEST_SOC_CMD>(msg->data);
  RCLCPP_INFO(this->get_logger(),"[StateManager] GET CMD FROM SOC(udp_interface) : [%s]",enumToString(req_cmd).c_str());
  cmds.soc_cmd = req_cmd;
  state_utils->setStartOnStation(false); 
  if(cmds.soc_cmd != REQUEST_SOC_CMD::PAUSE_WORKING){ //pause_working을 제외한 soc명령 내려왔을때.
    state_utils->setReservePause(false);// 다른 명령 수행시작으로 판단 reserve_pause clear.
  } else if(state_utils->getReserveDocking() && cmds.soc_cmd != REQUEST_SOC_CMD::START_RETURN_CHARGER){
    state_utils->setReserveDocking(false); // docking이 예약 된 상태에서 start return charger를 제외한 cmd가 왔을때 해제
  }
  checkTransition(cmds);
}

void StateManager::handleRobotCMD(const std_msgs::msg::UInt8::SharedPtr msg) {
  state_cmd cmds;
  const REQUEST_ROBOT_CMD req_cmd = static_cast<REQUEST_ROBOT_CMD>(msg->data);
  RCLCPP_INFO(this->get_logger(),"[StateManager] GET CMD FROM ROBOT : [%s]",enumToString(req_cmd).c_str());
  cmds.robot_cmd = req_cmd;
  checkTransition(cmds);
}

void StateManager::handleError(const robot_custom_msgs::msg::ErrorListArray::SharedPtr msg)
{
  for (const auto& incoming_error : msg->data_array) {
    auto it = std::find_if(error_list.begin(), error_list.end(),
                           [&](const auto& e) { return e.error_code == incoming_error.error_code; }); //skip known error

    if (incoming_error.error_occurred) {
      if (it == error_list.end()) {
        RCLCPP_INFO(this->get_logger(), "[StateManager] Error Detected: [ERROR_CODE = %s]", incoming_error.error_code.c_str());
        error_list.push_back(incoming_error);

        if( incoming_error.error_code.find("S03") != std::string::npos ){
          if( !batterysleep_send_timer_){
            batterysleep_send_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateManager::handleBatterySleepTimer, this));
          }
        } else if(incoming_error.error_code.find("S06-1") != std::string::npos){
          manage_error_list.push_back(incoming_error);
        } else if(incoming_error.error_code.find("S07") != std::string::npos || incoming_error.error_code.find("S07-1") != std::string::npos){
          //hjkim : 들림에러와 전도에러 발생 시 global localization on
          state_utils->setCmdGlobalLocalizationMode(true);
        }
      }
    } else {
      if (it != error_list.end()) {
        RCLCPP_INFO(this->get_logger(), "[StateManager] Error Cleared: [ERROR_CODE = %s]", incoming_error.error_code.c_str());
        error_list.erase(it); //release error ---> erase from error_list 
      }
    }
  }

  // 2. 심각한 에러가 있는지 검사
  for (const auto& error : error_list) {
    if (error.error_code.find("S05") != std::string::npos ||
        error.error_code.find("S02") != std::string::npos ||
        error.error_code.find("S03") != std::string::npos ||
        error.error_code.find("S10-2") != std::string::npos || //S10-2에서 fan을 돌리며 동작시킴 에러로 넘어가지 않음.
        error.error_code.find("E08") != std::string::npos ||
        error.error_code.find("F09-2") != std::string::npos ||
        error.error_code.find("F09-3") != std::string::npos ||
        error.error_code.find("S12-") != std::string::npos ||
        error.error_code.find("S13-") != std::string::npos) { 
      continue; // 무시할 에러
    }

    // 심각한 에러 발생 → 상태 전환
    if (getCurrentStateID() != ROBOT_STATE::ERROR) {
      //error_occured = true;
      state_cmd cmds;
      cmds.robot_cmd = REQUEST_ROBOT_CMD::ERROR;
      setState(ROBOT_STATE::ERROR, ROBOT_STATUS::READY,cmds);
      error_list.clear();
      break; // 한 개만 있어도 상태 전환
    }
  }
}

void StateManager::handleBattery(robot_custom_msgs::msg::BatteryStatus::SharedPtr msg) {
  battery_percent = msg->battery_percent;
}

void StateManager::handleBatterySleepTimer(){
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  current_time = clock.now().seconds();
  if( !battery_error_init_time ){
    battery_error_init_time = true;
    // Subscribe to battery status only when starting battery error monitoring
    RCLCPP_INFO(this->get_logger(), "[handleBatterySleepTimer] START Publish batterySleep timer");
  }

  if( state_utils->getOnstationStatus() ){
    RCLCPP_INFO(this->get_logger(), "[handleBatterySleepTimer] Cancel BatterySleep publish time checker due to ONSTATION");
    battery_error_init_time = false;
    battery_percent_cnt_init_time = false;
    batterysleep_send_timer_->cancel();
    batterysleep_send_timer_.reset();
    return;
  } else{
    if(battery_percent <= 3 && battery_percent > 0){
      if( !battery_percent_cnt_init_time ){
        battery_percent_cnt_init_time = true;
        error_start_time = current_time;
        RCLCPP_INFO(this->get_logger(), "[handleBatterySleepTimer] Battery under 3 percent. Publish batterySleep in a few seconds.");
      }
      if(current_time - error_start_time >= 10.0){
        RCLCPP_INFO(this->get_logger(), "[handleBatterySleepTimer] Publish batterySleep. Battery under 3 percent.");
        state_utils->publishBatterySleep();
        battery_error_init_time = false;
        battery_percent_cnt_init_time = false;
        batterysleep_send_timer_.reset();
      }
    }
  }
}

void StateManager::releaseError(){
  if (!manage_error_list.empty() && current_state == ROBOT_STATE::ERROR) {
    for (auto it = manage_error_list.begin(); it != manage_error_list.end(); ) {
      if (it->error_code.find("S06-1") != std::string::npos) { // state_manager에서 발생시키는 에러중에 S06-1을 확인하면 에러 해제 보냄.
        state_utils->publishLocalizationFailErrorStatus(false);
        it = manage_error_list.erase(it); // Remove the error after handling
      } else {
        ++it;
      }
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "[StateManager] manage_error_list is empty, skipping releaseError.");
  }
}

void StateManager::runCurrentState() {
  try {
    if (current_state_) {
      current_state_->run(state_utils);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "std::exception : %s", e.what());
  }
}

} // namespace airbot_state
