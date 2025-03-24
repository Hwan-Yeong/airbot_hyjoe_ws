#include "state_manager/state_manager.hpp"

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
  current_state_ = idle;
  setCurrentStateID(ROBOT_STATE::IDLE);

  robot_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>("robot_state_cmd", 10, std::bind(&StateManager::handleRobotCMD, this,std::placeholders::_1));
  soc_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/soc_cmd", 10, std::bind(&StateManager::handleSoCCMD, this, std::placeholders::_1));
  error_list_sub_ = this->create_subscription<robot_custom_msgs::msg::ErrorListArray>("/error_list", 10, std::bind(&StateManager::handleError, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateManager::runCurrentState, this));
  RCLCPP_INFO(this->get_logger(), "[StateManager] Node initialized.");
}

StateManager::~StateManager() {
  RCLCPP_INFO(this->get_logger(), "[StateManager] Node shutting down.");
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
  switch (cmd_ids.soc_cmd) {
  case REQUEST_SOC_CMD::VOID:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
    break;
  case REQUEST_SOC_CMD::START_AUTO_MAPPING:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::IDLE:
      setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      break;    
    }
    break;
  case REQUEST_SOC_CMD::START_MANUAL_MAPPING:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::IDLE:
      setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_NAVIGATION:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::IDLE:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::RETURN_CHARGER:
    case ROBOT_STATE::DOCKING:
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_RETURN_CHARGER:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::IDLE:
    case ROBOT_STATE::UNDOCKING:
    case ROBOT_STATE::AUTO_MAPPING:
    case ROBOT_STATE::MANUAL_MAPPING:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::FACTORY_NAVIGATION:
    case ROBOT_STATE::DOCKING:
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible -> Robot already return_charger or onstation");
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_DOCKING:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::IDLE:
    case ROBOT_STATE::UNDOCKING:
    case ROBOT_STATE::AUTO_MAPPING:
    case ROBOT_STATE::MANUAL_MAPPING:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::FACTORY_NAVIGATION:
    case ROBOT_STATE::DOCKING:
    case ROBOT_STATE::RETURN_CHARGER:
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible -> Robot already return_charger or onstation");
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_CHARGING:
    RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
    break;
  case REQUEST_SOC_CMD::PAUSE_NAVIGATION:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::NAVIGATION:
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    case ROBOT_STATE::RETURN_CHARGER:
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    case ROBOT_STATE::FACTORY_NAVIGATION:
      setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::PAUSE, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible -> cannot pause");
      break;
    }
    break;
  case REQUEST_SOC_CMD::RESUME_NAVIGATION:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::NAVIGATION:
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::START, cmd_ids);
      break;
    case ROBOT_STATE::RETURN_CHARGER:
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
      break;
    case ROBOT_STATE::FACTORY_NAVIGATION:
      setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::START, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible -> Robot not paused");
      break;
    }
    break;
  case REQUEST_SOC_CMD::START_FACTORY_NAVIGATION:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::IDLE:
      setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      break;
    case ROBOT_STATE::ONSTATION:
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::READY, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      break;    
    }
    break;
  case REQUEST_SOC_CMD::STOP_WORKING:
    switch( getCurrentStateID() ){
    case ROBOT_STATE::MANUAL_MAPPING:
    case ROBOT_STATE::AUTO_MAPPING:
    case ROBOT_STATE::NAVIGATION:
    case ROBOT_STATE::RETURN_CHARGER:
    case ROBOT_STATE::DOCKING:
    case ROBOT_STATE::UNDOCKING:
    case ROBOT_STATE::ONSTATION:
    case ROBOT_STATE::FACTORY_NAVIGATION:
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      break;    
    }
    break;
  }
  
  if( pre_cmds.robot_cmd != cmd_ids.robot_cmd) // TRANSITION BY ROBOT COMMAND
  {
    switch (cmd_ids.robot_cmd) {
    case REQUEST_ROBOT_CMD::START_ONSTATION:
      setState(ROBOT_STATE::ONSTATION, ROBOT_STATUS::START, cmd_ids);
      break;
    case REQUEST_ROBOT_CMD::STOP_ONSTATION:
      if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING:
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING:
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION:
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FACTORY_NAVIGATION:
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::FACTORY_NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING:
      if (getCurrentStateID() == ROBOT_STATE::AUTO_MAPPING) {
        setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::READY, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING:
      if (getCurrentStateID() == ROBOT_STATE::MANUAL_MAPPING) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_DOCKING:
      if (getCurrentStateID() == ROBOT_STATE::DOCKING) {
        setState(ROBOT_STATE::ONSTATION, ROBOT_STATUS::START, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER:
      if (getCurrentStateID() == ROBOT_STATE::RETURN_CHARGER) {
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;
    case REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING:
      if (getCurrentStateID() == ROBOT_STATE::RETURN_CHARGER) {
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
      } else {
        RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
      }
      break;  
    default:
      RCLCPP_ERROR(this->get_logger(), "[StateManager] Scenario is not possible");
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
  if( getCurrentStateID() != ROBOT_STATE::ERROR ) { //에러발생.
    state_cmd cmds;
    int array_size = msg->data_array.size();
    for (int i=0; i<array_size; ++i) {
      const auto& error = msg->data_array[i];

      bool isSameRank = false;
      for (const auto& existing_error : error_list) {
          if (existing_error.error_code == error.error_code) {
              isSameRank = true;
              break;
          }
      }
      if (!isSameRank) {
          RCLCPP_INFO(this->get_logger(), "[StateManager] Error Detected by error_manager : [RANK = %d / ERROR_CODE = %s]",
                      error.rank, error.error_code.c_str());
        error_list.push_back(error);
      }
    }

    for(const auto& occur_error : error_list)
    {
      if(occur_error.error_code.find("S05") != std::string::npos){ //이동불가는 에러발생 제외.
        continue;
      }
      else{
        error_occured = true;
        cmds.robot_cmd = REQUEST_ROBOT_CMD::ERROR;
        setState(ROBOT_STATE::ERROR, ROBOT_STATUS::READY, cmds);
        error_list.clear();
        break;
      }
    }
  }
}

void StateManager::runCurrentState() {
  try {
    if (current_state_) {
      current_state_->run(state_utils);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}

} // namespace airbot_state
