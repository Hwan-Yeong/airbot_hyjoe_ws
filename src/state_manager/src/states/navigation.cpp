#include "state_manager/states/navigation.hpp"

namespace airbot_state {

Navigation::Navigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {}

void Navigation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Navigation] Preparing Navigation STATE");
  target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
  // req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  //req_target_sub_ = node_->create_subscription<robot_custom_msgs::msg::Position>("/move_target", 10,std::bind(&Navigation::target_callback, this, std::placeholders::_1));
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  waitLaunchNode = false;
  ready_navigation = false;
  navi_started = false;
  state_utils->publishLidarOn(); ////state_utils->publishAllSensorOn();
  if( state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION ){
    state_utils->publishLocalizeEmpty();
  //state_utils->publishLocalizePose();
  }
  state_utils->setMovingStateID( NAVI_STATE::IDLE);
  state_utils->setStatusID(ROBOT_STATUS::READY );
}

void Navigation::run(const std::shared_ptr<StateUtils> &state_utils) {
  //1. sensor On (before move-target)
  //2. poseEstimate (before move-target) - pub(current.pose); 
  //3. pub TargetPose to Navi
  //4. monitor NaviState
  //5. Sensor Off (after Goal-Arrived)
  
  if( state_utils->getStatusID() == ROBOT_STATUS::READY )
  {
    double wait_localize_time = node_->now().seconds()-state_utils->getLocalizationStartTime();
    if(!navi_started){
      if(state_utils->getNodeStatusID() != NODE_STATUS::NAVI){
        if(startNavigation()){
          navi_started = true;
          waitLaunchNode = true;
        }else{
          state_utils->setStatusID(ROBOT_STATUS::FAIL);
        }
      }else{
        navi_started = true;
      }
    }
    
    if(waitLaunchNode){
      waitLaunchNode = false;
      waitNodeLaunching();
    }

    if (navi_started &&  !waitLaunchNode){
      
      if((state_utils->getOdomResetDone()) || (!state_utils->isStartOdomReset())){
        if((state_utils->getLocalizationComplete()) || (!state_utils->isStartLocalization())){
          state_utils->setStatusID(ROBOT_STATUS::START);
        }else if(wait_localize_time >= 30){
          state_utils->setStatusID(ROBOT_STATUS::FAIL);
        }
      }
      state_utils->setNodeStatusID( NODE_STATUS::NAVI );
    }
  } 
  else if( state_utils->getStatusID() == ROBOT_STATUS::START ){

    if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE )
    {
      state_utils->setMovingStateID(NAVI_STATE::IDLE); //resume
    }
	
    if (set_movetarget) {
      moveToTarget(target_pose.x, target_pose.y, target_pose.theta);
      publishTargetPosition(target_pose.x, target_pose.y, target_pose.theta);
      set_movetarget = false;
    }
    // RCLCPP_INFO(node_->get_logger(), "[Navigation] >>>>>>>>> START STATUS");//test log
  }
  else if( state_utils->getStatusID() == ROBOT_STATUS::PAUSE ){
    if (state_utils->getMovingStateID() != NAVI_STATE::PAUSE) {
      pauseNavigation();
      set_movetarget = true;
      // RCLCPP_INFO(node_->get_logger(), "[Navigation] >>>>>>>>> PAUSE STATUS");//test log
    }
  }
  else if( state_utils->getStatusID() == ROBOT_STATUS::COMPLETE)
  {
    state_utils->setStatusID( ROBOT_STATUS::START);
    //sensor off??
    //RCLCPP_INFO(node_->get_logger(), "[Navigation] >>>>>>>>> COMPLETE STATUS");//test log
    // if( set_movetarget ){ // new target received ..
    //   moveToTarget(target_pose.x, target_pose.y, target_pose.theta);
    //   publishTargetPosition(target_pose.x, target_pose.y, target_pose.theta);
    //   state_utils->setStatusID( ROBOT_STATUS::START);
    //   set_movetarget = false;
    // }
  }
  else if( state_utils->getStatusID() == ROBOT_STATUS::FAIL)
  {
    //fail scenario??
  }
  
}

void Navigation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[Navigation] Exiting Navigation STATE");
  navi_started = false;
  //req_target_sub_.reset();
  target_pose_pub_.reset();
  if (state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_NAVIGATION) {
    exitNavigationNode();
  }
  state_utils->stopMonitorOdom();
}

///////////////function in navigation

// NAVIGATION FUNCTION


bool Navigation::startProcess(const std::string &command,
                              const std::string &pidFilePath) {
  int result = std::system(("setsid bash -c '" + command +
                            "' > /dev/null 2>&1 & echo $! > " + pidFilePath)
                               .c_str());
  if (result == 0) {
    std::ifstream pidFile(pidFilePath);
    if (pidFile.is_open()) {
      std::string pid;
      std::getline(pidFile, pid);
      pidFile.close();
      if (!pid.empty()) {
        RCLCPP_INFO(node_->get_logger(),
                    "Process started successfully with PID: %s", pid.c_str());
        return true;
      }
    }
  }
  RCLCPP_ERROR(node_->get_logger(), "Failed to start the process.");
  return false;
}

bool Navigation::stopProcess(const std::string &pidFilePath) {
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

void Navigation::exitNavigationNode() {
  if (stopProcess("/home/airbot/navigation_pid.txt")) {
    RCLCPP_INFO(node_->get_logger(), "exit Navigation Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Navigation Node");
  }
  state_utils->setNodeStatusID( NODE_STATUS::IDLE );
}

bool Navigation::startNavigation() {
  bool ret = false;

  state_cmd command = state_utils->getRobotCMDID();
  
  if(command.soc_cmd == REQUEST_SOC_CMD::START_FACTORY_NAVIGATION){
      if (startProcess("ros2 launch airbot_navigation factory_navigation.launch.py", "/home/airbot/navigation_pid.txt")){
          nav_node_start_time = node_->get_clock()->now();
          RCLCPP_INFO(node_->get_logger(), "Factory Navigation Start Success");
          ret = true;
      }else{
          //reqStatus = REQUEST_STATUS::FAIL;
          RCLCPP_INFO(node_->get_logger(), "Factory Navigation Node Start FAIL");
      }
  }else{ //if(command.soc_cmd == REQUEST_SOC_CMD::START_NAVIGATION){
      if (startProcess("ros2 launch airbot_navigation navigation.launch.py","/home/airbot/navigation_pid.txt")) {
        nav_node_start_time = node_->get_clock()->now();
        RCLCPP_INFO(node_->get_logger(), "Navigation Node Start Success");
        ret = true;
      }else {
        // reqStatus = REQUEST_STATUS::FAIL;
        RCLCPP_INFO(node_->get_logger(), "Navigation Node Start FAIL");
      }
  }
  // else{
  //     RCLCPP_INFO(node_->get_logger(), "startNavigation Command Error : ");
  // }

  // state_utils->startMonitorOdomReset();
  // }

  return ret;
}

void Navigation::pauseNavigation() {
  RCLCPP_INFO(node_->get_logger(), "PAUSE Signal");

  // Check if the goal handle exists and is in an active state
  if (future_goal_handle_) {
    auto status = future_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
      client_->async_cancel_goal(future_goal_handle_);
      state_utils->setMovingStateID(NAVI_STATE::PAUSE);
      RCLCPP_INFO(node_->get_logger(), "Navigation paused");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Cannot pause, goal is not active. Status: %d", status);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Cannot pause, no valid goal handle");
  }
}

void Navigation::waitNodeLaunching() {
  // wait nav bringup time
  bool ret = false;
  rclcpp::Duration checkTime = node_->get_clock()->now() - nav_node_start_time;
  int waitSec = 10 - (int)checkTime.seconds();

  if (waitSec > 0) {
    std::this_thread::sleep_for(std::chrono::seconds(waitSec));
    RCLCPP_INFO(node_->get_logger(), "START_NAVIGATION wait sec : %d ",
                waitSec);
  }
}

void Navigation::moveToTarget(double x, double y, double theta) {
  if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Action server not available after waiting");
    state_utils->setMovingStateID(NAVI_STATE::FAIL);
    state_utils->setMovingFailID(NAVI_FAIL_REASON::SERVER_NO_ACTION);
    state_utils->setStatusID(ROBOT_STATUS::FAIL);
    return;
  }

  
  // 2025-01-02, clabil 강성준 선임연구원
  // request costmap clear
  state_utils->publishClearCostMap();
  goal_msg = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();
  // Prepare the goal message
  goal_msg->pose.pose.position.x = x;
  goal_msg->pose.pose.position.y = y;
  goal_msg->pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  goal_msg->pose.pose.orientation = tf2::toMsg(q);
  goal_msg->pose.header.frame_id = "map";
  goal_msg->pose.header.stamp = node_->now();

  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback =
      [this](auto, const std::shared_ptr<
                       const nav2_msgs::action::NavigateToPose::Feedback>
                       feedback) {
        // RCLCPP_INFO(node_->get_logger(), "Feedback received: Distance
        // remaining to goal: %.2f", feedback->distance_remaining);
        if (feedback->distance_remaining > 0.10) {
          state_utils->setMovingStateID(NAVI_STATE::MOVE_GOAL);
        }
      };

  send_goal_options.result_callback = [this](const auto &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Goal reached successfully.");
      state_utils->setMovingStateID(NAVI_STATE::ARRIVED_GOAL); // node_->robot_status = 2;// Update
      state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
      break;
    case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_INFO(node_->get_logger(), "Goal was aborted.");
      state_utils->setMovingStateID(NAVI_STATE::FAIL); // node_->robot_status = 4;  // STOP STATUS
      state_utils->setMovingFailID(NAVI_FAIL_REASON::GOAL_ABORT);
      break;
    case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_INFO(node_->get_logger(), "Goal was canceled.");
      state_utils->setMovingStateID(NAVI_STATE::PAUSE); // node_->robot_status = 3;  // FAIL
                                           // STATUS
      break;
    default:
    RCLCPP_INFO(node_->get_logger(), "Unknown result code.");
      state_utils->setMovingStateID(NAVI_STATE::FAIL); // node_->robot_status = 4;  // STOP STATUS
      state_utils->setMovingFailID(NAVI_FAIL_REASON::UNKWON);
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
      break;
    }
  };

  send_goal_options.goal_response_callback =
      [this](std::shared_ptr<
             rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by the server.");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by the server.");
          future_goal_handle_ =
              goal_handle; // Store the handle for pause/resume
        }
      };

  client_->async_send_goal(*goal_msg, send_goal_options);
}

void Navigation::publishTargetPosition(double x, double y, double theta) {
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
  RCLCPP_INFO(node_->get_logger(),
              "[Navigation] Running Navigation with shared data: [%d]",
              __LINE__);
  // Publish the target pose
  target_pose_pub_->publish(req_target_position);
}

void Navigation::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  amcl.header = msg->header;
  amcl.pose = msg->pose;
}

} // namespace airbot_state
