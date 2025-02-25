#include "state_manager/states/auto_mapping.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

AutoMapping::AutoMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void AutoMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Preparing AutoMapping STATE");

  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  bSavedMap = false;
  mapping_start_time = std::numeric_limits<double>::max();
  if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
      exitMappingNode();
  }else if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI){
      exitNavigationNode();
  }
  
  if( state_utils->getRobotCMDID().robot_cmd != REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING ){
    state_utils->startMonitorOdomReset();
  }
  
  state_utils->publishAllSensorOn();
}

void AutoMapping::run(const std::shared_ptr<StateUtils> &state_utils) {

  if(state_utils->isLidarSensorOK())
  {
    if (state_utils->getOdomResetDone()){
      runAutoMapping();
    }
  }
}

void AutoMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->saveLastPosition();
  RCLCPP_INFO(node_->get_logger(), "[AutoMapping] Exiting AutoMapping STATE");
  if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_AUTO_MAPPING ){
    map_saver();
    exitMappingNode();
  }
  req_robot_cmd_pub_.reset();
  state_utils->stopMonitorOdom();
}

///////////////function in AutoMapping
void AutoMapping::runAutoMapping() {
  reset_subExploreFinish();
  // ******Auto mapping Launch files******
  if (state_utils->startProcess("ros2 launch explore explore_all.launch.py",
                   "/home/airbot/mapping_pid.txt")) {
    mapping_start_time = node_->now().seconds();
    state_utils->setNodeStatusID( NODE_STATUS::AUTO_MAPPING );
    explore_finish_sub = node_->create_subscription<std_msgs::msg::Empty>(
        "/explore_finish", 10,
        std::bind(&AutoMapping::explore_finish_callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "AutoMapping Run");
    // stopMonitorOdom(); // Stop Monitoring after reset
  } else {
    RCLCPP_INFO(node_->get_logger(), "runAutoMapping FAIL");
  }
}

void AutoMapping::explore_finish_callback(const std_msgs::msg::Empty::SharedPtr) {
    // finish generate change state msg
    auto robot_cmd_msg = std_msgs::msg::UInt8();
    robot_cmd_msg.data = int( REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING );
    req_robot_cmd_pub_->publish(robot_cmd_msg);
    RCLCPP_INFO(node_->get_logger(), "Explore mapping completed ");
    reset_subExploreFinish();
}

void AutoMapping::reset_subExploreFinish() {
    if (explore_finish_sub) {
      explore_finish_sub.reset();
      RCLCPP_INFO(node_->get_logger(), "reset_subExploreFinish");
    } else {
      RCLCPP_INFO(node_->get_logger(), "explore_finish_sub is allready reset ");
    }
}

void AutoMapping::exitMappingNode() {
  if (state_utils->stopProcess("/home/airbot/mapping_pid.txt")) {
    state_utils->setNodeStatusID( NODE_STATUS::IDLE );
    RCLCPP_INFO(node_->get_logger(), "exit Mapping Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Mapping Node");
    state_utils->setNodeStatusID( NODE_STATUS::IDLE );
  }
  // rclcpp::Rate waitStop(1000);
  // waitStop.sleep();
}

void AutoMapping::exitNavigationNode() {
  if (state_utils->stopProcess("/home/airbot/navigation_pid.txt")) {
    RCLCPP_INFO(node_->get_logger(), "exit Navigation Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Navigation Node");
  }
  // rclcpp::Rate waitStop(1000);
  // waitStop.sleep();
}

void AutoMapping::map_saver() {
  int map_save_result = std::system(
      "ros2 run nav2_map_server map_saver_cli -f /home/airbot/test1_map "
      "--ros-args -p save_map_timeout:=10.0");
  int map_save_cnt = 0;

  while (map_save_result != 0) {
    if (map_save_cnt++ < 10) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Map save command failed with error code, trying again: %d",
                   map_save_result);
      map_save_result = std::system(
          "ros2 run nav2_map_server map_saver_cli -f /home/airbot/test1_map "
          "--ros-args -p save_map_timeout:=10.0");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Map save command failed over 10times");
      break;
    }
  }

  if (map_save_result == 0) {
    RCLCPP_INFO(node_->get_logger(), "Map save command executed successfully.");
    // API_FileDelete(MapEditFile);
    bSavedMap = true;
    //**Saving the copy the map****
    const std::string original_map_file = "/home/airbot/test1_map.pgm";
    const std::string copy_map_file =
        "/home/airbot/OriginalMap.pgm"; // Path for the duplicate map

    try {
      // Use std::filesystem to copy the original map file to a new name
      std::filesystem::copy(original_map_file, copy_map_file,
                            std::filesystem::copy_options::overwrite_existing);
      RCLCPP_INFO(node_->get_logger(),
                  "Map successfully saved as OriginalMap.pgm");
    } catch (const std::filesystem::filesystem_error &e) {
      RCLCPP_INFO(node_->get_logger(), "OriginalMap.pgm save fail ");
      return;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Map save fail");
  }
}

} // namespace airbot_state
