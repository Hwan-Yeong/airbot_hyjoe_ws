#include "state_manager/state_manager.hpp"

using namespace std::chrono_literals;

namespace airbot_state {

StateManager::StateManager() : Node("state_manager") {
  state_utils = std::make_shared<StateUtils>(std::shared_ptr<rclcpp::Node>(this));
  idle = std::make_shared<Idle>(int(ROBOT_STATE::IDLE), std::shared_ptr<rclcpp::Node>(this), state_utils);
  on_station = std::make_shared<OnStation>(int(ROBOT_STATE::ONSTATION), std::shared_ptr<rclcpp::Node>(this), state_utils);
  undocking = std::make_shared<UnDocking>(int(ROBOT_STATE::UNDOCKING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  docking = std::make_shared<Docking>(int(ROBOT_STATE::DOCKING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  navigation = std::make_shared<Navigation>(int(ROBOT_STATE::NAVIGATION), std::shared_ptr<rclcpp::Node>(this), state_utils);
  auto_mapping = std::make_shared<AutoMapping>(int(ROBOT_STATE::AUTO_MAPPING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  manual_mapping = std::make_shared<ManualMapping>(int(ROBOT_STATE::MANUAL_MAPPING), std::shared_ptr<rclcpp::Node>(this), state_utils);
  return_charger = std::make_shared<ReturnCharger>(int(ROBOT_STATE::RETURN_CHARGER), std::shared_ptr<rclcpp::Node>(this), state_utils);
  factory_navigation = std::make_shared<Navigation>(int(ROBOT_STATE::FACTORY_NAVIGATION), std::shared_ptr<rclcpp::Node>(this), state_utils);
  states_[ROBOT_STATE::IDLE] = idle;
  states_[ROBOT_STATE::ONSTATION] = on_station;
  states_[ROBOT_STATE::UNDOCKING] = undocking;
  states_[ROBOT_STATE::DOCKING] = docking;
  states_[ROBOT_STATE::NAVIGATION] = navigation;
  states_[ROBOT_STATE::AUTO_MAPPING] = auto_mapping;
  states_[ROBOT_STATE::MANUAL_MAPPING] = manual_mapping;
  states_[ROBOT_STATE::RETURN_CHARGER] = return_charger;
  states_[ROBOT_STATE::FACTORY_NAVIGATION] = factory_navigation;
  current_state_ = idle;
  setCurrentStateID(ROBOT_STATE::IDLE);

  robot_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>("robot_state_cmd", 10, std::bind(&StateManager::handleRobotCMD, this,std::placeholders::_1));
  soc_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>("/soc_cmd", 10, std::bind(&StateManager::handleSoCCMD, this, std::placeholders::_1));
  req_target_sub_ = this->create_subscription<robot_custom_msgs::msg::Position>("/move_target", 10,std::bind(&StateManager::target_callback, this, std::placeholders::_1));

  robot_state_pub_ = this->create_publisher<robot_custom_msgs::msg::RobotState>("/state_datas", 10);
  navi_state_pub_ = this->create_publisher<robot_custom_msgs::msg::NaviState>("/navi_datas", 10);
  node_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/node_status", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateManager::runCurrentState, this));
  RCLCPP_INFO(this->get_logger(), "StateManager Node initialized.");
}

StateManager::~StateManager() {
  RCLCPP_INFO(this->get_logger(), "State Manager shutting down.");
}

void StateManager::setState(ROBOT_STATE state_id, ROBOT_STATUS status_id, const state_cmd &cmd_ids) {
  if (states_.find(state_id) == states_.end()) {
    RCLCPP_WARN(this->get_logger(), "State ID %d not found!", int(state_id));
    return;
  }
  state_utils->setRobotCMDID(cmd_ids);
  if (getCurrentStateID() != state_id) {
    // off current state
    if (current_state_) {
      current_state_->post_run(state_utils);
    }
    // change current state
    pre_cmds = cmd_ids;
    current_state_ = states_[state_id];
    current_state_->pre_run(state_utils);
    RCLCPP_INFO(this->get_logger(), "Changed to state ID: %d", int(state_id));
    setCurrentStateID(state_id);
  }
  state_utils->setAllRobotStateIDs(state_id, status_id, cmd_ids);
  
  RCLCPP_INFO(this->get_logger(),
              "CURRENT STATE: [%s] -> NEXT STATE: [%s] -> Transition SOC:[%s]| ROBOT:[%s] / STATUS: [%s]",
              enumToString(state_utils->getPreStateID()).c_str(),
              enumToString(state_utils->getStateID()).c_str(),
              enumToString(state_utils->getRobotCMDID().soc_cmd).c_str(),enumToString(state_utils->getRobotCMDID().robot_cmd).c_str(),
              enumToString(state_utils->getStatusID()).c_str() );
}

void StateManager::handleSoCCMD(const std_msgs::msg::UInt8::SharedPtr msg) {
  state_cmd cmds;
  const REQUEST_SOC_CMD req_cmd = static_cast<REQUEST_SOC_CMD>(msg->data);
  RCLCPP_INFO(this->get_logger(),">>>>>>>>GET CMD FROM SOC : [%s]",enumToString(req_cmd).c_str());
  cmds.soc_cmd = req_cmd;
  // cmds.robot_cmd = REQUEST_ROBOT_CMD::VOID;
  checkTransition(cmds);
}
void StateManager::handleRobotCMD(const std_msgs::msg::UInt8::SharedPtr msg) {
  state_cmd cmds;
  const REQUEST_ROBOT_CMD req_cmd = static_cast<REQUEST_ROBOT_CMD>(msg->data);
  RCLCPP_INFO(this->get_logger(),">>>>>>>>GET CMD FROM ROBOT : [%s]",enumToString(req_cmd).c_str());
  // cmds.soc_cmd = REQUEST_SOC_CMD::VOID;
  cmds.robot_cmd = req_cmd;
  checkTransition(cmds);
}

void StateManager::checkTransition( const state_cmd &cmd_ids )
{
  // TRANSITION BY SOC COMMAND
  //cmds.robot_cmd = REQUEST_ROBOT_CMD::VOID;
  if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_AUTO_MAPPING) {
    if (getCurrentStateID() == ROBOT_STATE::IDLE) {
      setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_MANUAL_MAPPING) {
    if (getCurrentStateID() == ROBOT_STATE::IDLE) {
      setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_NAVIGATION) {
    if (getCurrentStateID() == ROBOT_STATE::IDLE) {
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
      setState(ROBOT_STATE::UNDOCKING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_RETURN_CHARGER) {
    if (getCurrentStateID() == ROBOT_STATE::IDLE) {
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::AUTO_MAPPING) {
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::MANUAL_MAPPING) {
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_DOCKING) {
    if (getCurrentStateID() == ROBOT_STATE::IDLE) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::AUTO_MAPPING) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::MANUAL_MAPPING) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else if (getCurrentStateID() == ROBOT_STATE::RETURN_CHARGER) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_CHARGING) {
    if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_CHARGING) {
    if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
      setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_AUTO_MAPPING) {
    if (getCurrentStateID() == ROBOT_STATE::AUTO_MAPPING) {
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_MANUAL_MAPPING || cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_AUTO_MAPPING) {
    if (getCurrentStateID() == ROBOT_STATE::MANUAL_MAPPING) {
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::PAUSE_NAVIGATION) {
    if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::PAUSE, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::RESUME_NAVIGATION) {
    if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_NAVIGATION) {
    if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_RETURN_CHARGER) {
    if (getCurrentStateID() == ROBOT_STATE::RETURN_CHARGER) {
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  } else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_DOCKING) {
    if (getCurrentStateID() == ROBOT_STATE::DOCKING) {
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  }
  else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::START_FACTORY_NAVIGATION) { //reserve code
    if (getCurrentStateID() == ROBOT_STATE::IDLE) {
      setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  }
  else if (cmd_ids.soc_cmd == REQUEST_SOC_CMD::STOP_FACTORY_NAVIGATION) { //reserve code
    if (getCurrentStateID() == ROBOT_STATE::NAVIGATION) {
      setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
    } else {
      // fail - abort command
    }
  }
  
  if( pre_cmds.robot_cmd != cmd_ids.robot_cmd) // TRANSITION BY ROBOT COMMAND
  {
    //cmds.soc_cmd = REQUEST_SOC_CMD::VOID;
    if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::START_ONSTATION) {
      if (getCurrentStateID() == ROBOT_STATE::IDLE) {
        setState(ROBOT_STATE::ONSTATION, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    }  
    else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::STOP_ONSTATION) {
      if (getCurrentStateID() == ROBOT_STATE::ONSTATION) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING) {
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::AUTO_MAPPING, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING) {
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::MANUAL_MAPPING, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION) {
      if (getCurrentStateID() == ROBOT_STATE::UNDOCKING) {
        setState(ROBOT_STATE::NAVIGATION, ROBOT_STATUS::READY, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING) {
      if (getCurrentStateID() == ROBOT_STATE::AUTO_MAPPING) {
        setState(ROBOT_STATE::RETURN_CHARGER, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::DONE_DOCKING) {
      if (getCurrentStateID() == ROBOT_STATE::DOCKING) {
        setState(ROBOT_STATE::ONSTATION, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER) {
      if (getCurrentStateID() == ROBOT_STATE::RETURN_CHARGER) {
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER) {
      if (getCurrentStateID() == ROBOT_STATE::RETURN_CHARGER) {
        setState(ROBOT_STATE::DOCKING, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    } else if (cmd_ids.robot_cmd == REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING) {
      if (getCurrentStateID() == ROBOT_STATE::MANUAL_MAPPING) {
        setState(ROBOT_STATE::IDLE, ROBOT_STATUS::START, cmd_ids);
      } else {
        // fail - abort command
      }
    }
  }
  pre_cmds = cmd_ids;
}

void StateManager::setCurrentStateID(const ROBOT_STATE data) {
  current_state = data;
}
ROBOT_STATE StateManager::getCurrentStateID() { return current_state; }

void StateManager::runCurrentState() {
  try {
    if (current_state_) {
      current_state_->run(state_utils);
      //for navigation
      if( bsetMovetarget && getCurrentStateID() == ROBOT_STATE::NAVIGATION )
      {
        bsetMovetarget = false;
        current_state_->reserveTarget(target_pose);
      }
      // send node_status
      auto req_state_msg = robot_custom_msgs::msg::RobotState();
      req_state_msg.state = int(state_utils->getStateID());
      req_state_msg.status = int(state_utils->getStatusID());
      robot_state_pub_->publish(req_state_msg);

      auto node_status_msg = std_msgs::msg::UInt8();
      node_status_msg.data = u_int8_t(state_utils->getNodeStatusID());
      node_status_pub_->publish(node_status_msg);

      // send navi_state
      auto req_navi_msg = robot_custom_msgs::msg::NaviState();
      req_navi_msg.state = int(state_utils->getMovingStateID());
      req_navi_msg.fail_reason = int(state_utils->getMovingFailID());
      navi_state_pub_->publish(req_navi_msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}

void StateManager::target_callback(
    const robot_custom_msgs::msg::Position::SharedPtr msg) {
  try {
    target_pose.x = msg->x;
    target_pose.y = msg->y;
    target_pose.theta = msg->theta;
    bsetMovetarget = true;
    RCLCPP_INFO(this->get_logger(), "target_callback x : %f , y : %f, theta : %f ", target_pose.x,target_pose.y,target_pose.theta);
  } catch (const std::exception &e) {
    RCLCPP_INFO(this->get_logger(), "Exception: %s", e.what());
  }
}
} // namespace airbot_state
