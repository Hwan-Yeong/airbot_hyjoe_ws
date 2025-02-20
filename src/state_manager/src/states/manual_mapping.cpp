#include "state_manager/states/manual_mapping.hpp"

namespace airbot_state {

ManualMapping::ManualMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
}

void ManualMapping::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Preparing ManualMapping STATE");
  bSavedMap = false;
  mapping_start_time = std::numeric_limits<double>::max();
  // if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING){
  //     exitMappingNode();
  // }else if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI){
  //     exitNavigationNode();
  // }
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  station_data_sub = node_->create_subscription<robot_custom_msgs::msg::StationData>(
      "/station_data", 10,
      std::bind(&ManualMapping::stationData_callback, this, std::placeholders::_1));
  
  exitMappingNode();
  exitNavigationNode();
  if( state_utils->getRobotCMDID().robot_cmd != REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING ){
    state_utils->startMonitorOdomReset();
  }
  state_utils->publishAllSensorOn();
}

void ManualMapping::run(const std::shared_ptr<StateUtils> &state_utils) {
//   RCLCPP_INFO(node_->get_logger(),
//               "[ManualMapping] Running ManualMapping with shared data: ");
  //   performManualMappingTasks();
  if (state_utils->getOdomResetDone()){
    runManualMapping();
  }
}

void ManualMapping::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[ManualMapping] Exiting ManualMapping state");
  station_data_sub.reset();
  req_robot_cmd_pub_.reset();
  state_utils->stopMonitorOdom();
  map_saver();
  exitMappingNode();
}

///////////////function in ManualMapping
void ManualMapping::runManualMapping() {
  RCLCPP_INFO(node_->get_logger(), "Manual Mapping Start");
  if (startProcess("ros2 launch airbot_slam slam.launch.py",
                   "/home/airbot/mapping_pid.txt")) {
    mapping_start_time = node_->now().seconds();
    state_utils->setNodeStatusID( NODE_STATUS::MANUAL_MAPPING );
    RCLCPP_INFO(node_->get_logger(), "ManualMapping Run");
    // stopMonitorOdom(); // Stop monitoring Odom status

  } else {
    // reqStatus = REQUEST_STATUS::FAIL;
    RCLCPP_INFO(node_->get_logger(), "ManualMapping FAIL");
  }
}

void ManualMapping::exitMappingNode() {
  if (stopProcess("/home/airbot/mapping_pid.txt")) {
    state_utils->setNodeStatusID( NODE_STATUS::IDLE );
    RCLCPP_INFO(node_->get_logger(), "exit Mapping Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Mapping Node");
    state_utils->setNodeStatusID( NODE_STATUS::IDLE );
  }
  // rclcpp::Rate waitStop(1000);
  // waitStop.sleep();
}
void ManualMapping::exitNavigationNode()
{
    if(stopProcess("/home/airbot/navigation_pid.txt")){
        RCLCPP_INFO(node_->get_logger(), "exit Navigation Node");
    }else{
        RCLCPP_INFO(node_->get_logger(), "Fail - kill Navigation Node");
    }
    // rclcpp::Rate waitStop(1000);
    // waitStop.sleep();
    
}

void ManualMapping::map_saver() {
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


////Process check

bool ManualMapping::startProcess(const std::string& command, const std::string& pidFilePath) {
    int result = std::system(("setsid bash -c '" + command + "' > /dev/null 2>&1 & echo $! > " + pidFilePath).c_str());
    if (result == 0) {
        std::ifstream pidFile(pidFilePath);
        if (pidFile.is_open()) {
            std::string pid;
            std::getline(pidFile, pid);
            pidFile.close();
            if (!pid.empty()) {
                RCLCPP_INFO(node_->get_logger(), "Process started successfully with PID: %s", pid.c_str());
                return true;
            }
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "Failed to start the process.");
    return false;
}


bool ManualMapping::stopProcess(const std::string& pidFilePath)
{
    std::ifstream pidFile(pidFilePath);
    std::string pid;
    if (pidFile.is_open()) {
        std::getline(pidFile, pid);
        pidFile.close();
        if (!pid.empty()) {
            pid_t processGroupID = std::stoi(pid);
            if (kill(-processGroupID, SIGINT) == 0) {
                sleep(1);  // Wait for process to handle SIGINT
                if (kill(-processGroupID, 0) == -1) {  // Process no longer exists
                    RCLCPP_INFO(node_->get_logger(), "Process terminated successfully.");
                    return true;
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Process did not terminate, sending SIGKILL.");
                    kill(-processGroupID, SIGKILL);
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send SIGINT.");
            }
        }
    }
    return false;
}

void ManualMapping::stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg) {
  try {
    docking_status = msg->docking_status;
    if (docking_status & 0x70) { // charging
      if (docking_status & 0x60) {
      RCLCPP_INFO(node_->get_logger(), "Robot Status Charging, Docking Status %u", docking_status);}
      auto req_state_msg = std_msgs::msg::UInt8();
      req_state_msg.data = int(REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING);
      req_robot_cmd_pub_->publish(req_state_msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}

} // namespace airbot_state
