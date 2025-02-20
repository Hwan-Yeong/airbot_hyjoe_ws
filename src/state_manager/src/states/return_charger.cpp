#include "state_manager/states/return_charger.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

ReturnCharger::ReturnCharger(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {
  // req_nomotion_local_pub_ =
  //     this->create_publisher<std_msgs::msg::Empty>("/localization/request",
  //     1);
}

void ReturnCharger::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(),
              "[ReturnCharger] Preparing ReturnCharger STATE");
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  target_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose");
  
  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  moveToDock(0.0, 0.0, 1.0);
  publishTargetPosition(0.0, 0.0, 1.0);
}

void ReturnCharger::run(const std::shared_ptr<StateUtils> &state_utils) {
  // RCLCPP_INFO(node_->get_logger(),
  //             "[ReturnCharger] Running ReturnCharger with shared data: ");
  // moveToDock(0.0, 0.0, 1.0);
  // publishTargetPosition(0.0, 0.0, 1.0);
  
}

void ReturnCharger::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(),
              "[ReturnCharger] Exiting ReturnCharger STATE");
  req_robot_cmd_pub_.reset();
  if (state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_RETURN_CHARGER || state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_DOCKING) {
    if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING ||
        state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {
      map_saver();
      exitMappingNode();
    } else{
      // exitNavigationNode();
    }
  }
}

///////////////function in ReturnCharger
void ReturnCharger::moveToDock(double x, double y, double theta) {
  startMonitorReturnCharger();

  if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    state_utils->setMovingStateID( NAVI_STATE::FAIL);
    state_utils->setMovingFailID(NAVI_FAIL_REASON::SERVER_NO_ACTION);
    return;
  }

  state_utils->publishClearCostMap();
  auto nearDockGoal = nav2_msgs::action::NavigateToPose::Goal();

  nearDockGoal.pose.pose.position.x = x;
  nearDockGoal.pose.pose.position.y = y;
  nearDockGoal.pose.pose.position.z = 1.0;

  // Convert theta (yaw) to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  nearDockGoal.pose.pose.orientation = tf2::toMsg(q);

  nearDockGoal.pose.header.frame_id = "map";
  nearDockGoal.pose.header.stamp = node_->now();

  // Send goal and wait for result
  auto docking_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  docking_goal_options.feedback_callback =
      [this](auto, const std::shared_ptr<
                       const nav2_msgs::action::NavigateToPose::Feedback>
                       feedback) {
        // RCLCPP_INFO(node_->get_logger(), "Feedback received: Distance
        // remaining to goal: %.2f", feedback->distance_remaining);

        // You can update the robot_status based on feedback if needed
        if (feedback->distance_remaining > 0.20) {
            state_utils->setMovingStateID( NAVI_STATE::MOVE_GOAL);
        }
      };

  docking_goal_options.result_callback = [this](const auto &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(node_->get_logger(), "Goal reached successfully.");
      state_utils->setMovingStateID(NAVI_STATE::ARRIVED_GOAL); // node_->robot_status = 2; //
      // Update goal_status to indicate success
      break;
    case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_INFO(node_->get_logger(), "Goal was aborted.");
      //state_utils->setMovingStateID(NAVI_STATE::FAIL); // node_->robot_status = 4;  // STOP
      //state_utils->setMovingFailID(NAVI_FAIL_REASON::GOAL_ABORT);
      break;
    case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_INFO(node_->get_logger(), "Goal was canceled.");
      state_utils->setMovingStateID(NAVI_STATE::PAUSE); // node_->robot_status = 3;  // FAIL
      // STATUS
      break;
    default:
    RCLCPP_INFO(node_->get_logger(), "Unknown result code.");
      state_utils->setMovingStateID(NAVI_STATE::FAIL); // node_->robot_status = 3;  // FAIL
      state_utils->setMovingFailID(NAVI_FAIL_REASON::UNKWON);
      break;
    }
  };
  client_->async_send_goal(nearDockGoal, docking_goal_options);
}

void ReturnCharger::publishTargetPosition(double x, double y, double theta) {
  // Set the target pose
  geometry_msgs::msg::Pose req_target_position;
  req_target_position.position.x = x;
  req_target_position.position.y = y;
  req_target_position.position.z = 0.0; // Assuming a 2D plane, z = 0

  // Convert theta (yaw) to a quaternion for the orientation
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, theta);
  req_target_position.orientation.z = quaternion.z();
  req_target_position.orientation.w = quaternion.w();

  // Publish the target pose
  target_pose_pub_->publish(req_target_position);
}

void ReturnCharger::stopMonitorReturnCharger() { reset_timerNaviStatus(); }

void ReturnCharger::startMonitorReturnCharger() {
  try {
    nav_status_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ReturnCharger::monitor_returnCharger, this));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}

void ReturnCharger::monitor_returnCharger() {
  if (state_utils->getMovingStateID() == NAVI_STATE::ARRIVED_GOAL) {
    // Goal reached successfully
    if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING ||
        state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {

      map_saver();
      exitMappingNode();
    }

    RCLCPP_INFO(node_->get_logger(), "Proceed with docking.");
    // finish generate change state msg
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER);
    req_robot_cmd_pub_->publish(req_state_msg);
    //
    dock_pose_estimate = false;
    stopMonitorReturnCharger();
  } else if (state_utils->getMovingStateID() == NAVI_STATE::FAIL) {
    // Goal failed or aborted
    if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING ||
        state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {
      map_saver();
      exitMappingNode();
    }
    RCLCPP_WARN(node_->get_logger(), "Navigation failed, docking aborted.");
    // finish generate change state msg
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER);
    req_robot_cmd_pub_->publish(req_state_msg);
    //
    dock_pose_estimate = false;
    stopMonitorReturnCharger();
  } else {
    // RCLCPP_WARN(node_->get_logger(), "Navigation idle, stop monitor ");
  }
}

void ReturnCharger::reset_timerNaviStatus() {
  try {
    std::lock_guard<std::mutex> lock(nav_status_timer_mutex_);
    if (nav_status_timer_) {
      RCLCPP_INFO(node_->get_logger(), "reset_timerNaviStatus");
      nav_status_timer_.reset();
      RCLCPP_INFO(node_->get_logger(), "reset_timerNaviStatus - end ");
    } else {
      RCLCPP_INFO(node_->get_logger(), "nav_status_timer_ is allready reset ");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception: %s", e.what());
  }
}
//////////////saving map
void ReturnCharger::map_saver() {
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

void ReturnCharger::exitMappingNode() {
  if (stopProcess("/home/airbot/mapping_pid.txt")) {
    RCLCPP_INFO(node_->get_logger(), "exit Mapping Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Mapping Node");
  }  
  state_utils->setNodeStatusID(NODE_STATUS::IDLE);
  // rclcpp::Rate waitStop(1000);
  // waitStop.sleep();
}

void ReturnCharger::exitNavigationNode() {
  if (stopProcess("/home/airbot/navigation_pid.txt")) {
    RCLCPP_INFO(node_->get_logger(), "exit Navigation Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Navigation Node");
  }
  state_utils->setNodeStatusID( NODE_STATUS::IDLE );
}

bool ReturnCharger::stopProcess(const std::string &pidFilePath) {
  std::ifstream pidFile(pidFilePath);
  std::string pid;
  if (pidFile.is_open()) {
    std::getline(pidFile, pid);
    pidFile.close();
    if (!pid.empty()) {
      pid_t processGroupID = std::stoi(pid);
      if (kill(-processGroupID, SIGINT) == 0) {
        sleep(1); // Wait for process to handle SIGINT
        if (kill(-processGroupID, 0) == -1) { // Process no longer exists
          RCLCPP_INFO(node_->get_logger(), "Process terminated successfully.");
          return true;
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "Process did not terminate, sending SIGKILL.");
          kill(-processGroupID, SIGKILL);
        }
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send SIGINT.");
      }
    }
  }
  return false;
}
} // namespace airbot_state
