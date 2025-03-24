#include "state_manager/states/return_charger.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

ReturnCharger::ReturnCharger(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {

}

void ReturnCharger::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(),"[ReturnCharger] Preparing ReturnCharger STATE");
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
  if(future_goal_handle_)
  {
    future_goal_handle_.reset();
  }

  state_utils->setMovingStateID(NAVI_STATE::IDLE);
  state_utils->setStatusID(ROBOT_STATUS::READY);
  setReadyMoving(READY_MOVING::IDLE);
  if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || state_utils->getNodeStatusID() == NODE_STATUS::NAVI){
    setReadyNavigation(READY_NAVIGATION::CHECK_SENSOR);
  }else{
    setReadyNavigation(READY_NAVIGATION::LAUCN_NODE);
  }
  return_pose = state_utils->getInitPose();
  state_utils->startSensorMonitor();
  retry_move_target = 0;
  bSendGoal = false;
}

void ReturnCharger::setReadyNavigation(READY_NAVIGATION set)
{
  if(readyNavi != set){
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] setReadyNavigation %s", enumToString(set).c_str());
  }
  readyNavi = set;
}

void ReturnCharger::setReadyMoving(READY_MOVING set)
{
  if(readyMoving != set){
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] setReadyMoving %s", enumToString(set).c_str());
  }
  readyMoving = set;
}

void ReturnCharger::run(const std::shared_ptr<StateUtils> &state_utils) {
  ROBOT_STATUS ready_check;
  
  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Robot get on Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
    req_robot_cmd_pub_->publish(req_state_msg);
    return;
  }

  if(state_utils->getStatusID() != ROBOT_STATUS::READY ){
    if( state_utils->getMoveChargerFlag())
    {
      if( state_utils->getMovingStateID() != NAVI_STATE::ARRIVED_GOAL && state_utils->getMovingStateID() != NAVI_STATE::FAIL) {
        new_targetcall_flag = true;
        retry_move_target = 0;
        bSendGoal = false;
        state_utils->setMovingStateID(NAVI_STATE::READY);
        setReadyMoving(READY_MOVING::COMPLETE); 
      } else{
        retry_move_target = 0;
        bSendGoal = false;
        state_utils->setMovingStateID(NAVI_STATE::READY);
        setReadyMoving(READY_MOVING::CHECK_SENSOR);
      }
    }
  }

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    ready_check = processNavigationReady();
    state_utils->setStatusID(ready_check);
    break;
  case ROBOT_STATUS::START :
    if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE ){
      retry_move_target = 0;
      bSendGoal = false;
      setReadyMoving(READY_MOVING::CHECK_SENSOR);
      state_utils->setMovingStateID(NAVI_STATE::READY); //resume
    }
    if(state_utils->getMovingStateID() == NAVI_STATE::READY){
      processMoveTarget();
    }
    break;
  case ROBOT_STATUS::PAUSE :
    if (state_utils->getMovingStateID() != NAVI_STATE::PAUSE) {
      pauseReturnCharger();
    }else if(state_utils->getMovingStateID() == NAVI_STATE::READY){
      state_utils->setStatusID( ROBOT_STATUS::START);
    }
    break;
  case ROBOT_STATUS::COMPLETE :
    if(state_utils->getMovingStateID() == NAVI_STATE::READY){
      state_utils->setStatusID( ROBOT_STATUS::START);
    }
    break;
  case ROBOT_STATUS::FAIL :
    if(state_utils->getMovingStateID() == NAVI_STATE::READY){
      state_utils->setStatusID( ROBOT_STATUS::START);
    }
    break;      
  default:
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Running UnKown Status");
    break;
  }
}

void ReturnCharger::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  state_utils->saveLastPosition();
  RCLCPP_INFO(node_->get_logger(),
              "[ReturnCharger] Exiting ReturnCharger STATE");
  req_robot_cmd_pub_.reset();
  state_utils->stopSensorMonitor();
  stopMonitorReturnCharger();
  state_utils->publishSenSorManagerOff();
  state_utils->publishManeuverOff();
  if (state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_WORKING || state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_DOCKING) {
    if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING ||
        state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {
      map_saver();
      state_utils->send_node_goal(NODE_STATUS::IDLE);
    } else{
      // exitNavigationNode();
    }
  }
  if(future_goal_handle_ && state_utils->getMovingStateID() == NAVI_STATE::MOVE_GOAL)
  {
    client_->async_cancel_goal(future_goal_handle_);
    future_goal_handle_.reset();
  }
  client_.reset();
}

ROBOT_STATUS ReturnCharger::processNavigationReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int localize_result = 0;
  int node_result = 0;
  switch (readyNavi)
  {
  case READY_NAVIGATION::LAUCN_NODE :
    state_utils->send_node_goal(NODE_STATUS::NAVI);
    setReadyNavigation(READY_NAVIGATION::CHECK_NODE);
    break;
  case READY_NAVIGATION::CHECK_NODE :
    node_result = state_utils->getNodeClientStatus();
    if(node_result > 0){
      setReadyNavigation(READY_NAVIGATION::CHECK_SENSOR);
    }else if(node_result < 0){
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }
    break;
  case READY_NAVIGATION::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING){
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }else{
        setReadyNavigation(READY_NAVIGATION::REQUEST_POSE_ESTIMATE);
      }
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "lidar error");
      ret = ROBOT_STATUS::FAIL;
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "tof error");
      ret = ROBOT_STATUS::FAIL;
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "camera Error");
      ret = ROBOT_STATUS::FAIL;
    }
    break;
  case READY_NAVIGATION::REQUEST_POSE_ESTIMATE :
      state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
      setReadyNavigation(READY_NAVIGATION::CHECK_POSE_ESTIMATE);
    break;
  case READY_NAVIGATION::CHECK_POSE_ESTIMATE :
      localize_result = localizationChecker();
      if(localize_result > 0){
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
      }else if(localize_result < 0){
        setReadyNavigation(READY_NAVIGATION::FAIL);
      }
    break;
  case READY_NAVIGATION::COMPLETE :
    state_utils->setMovingStateID(NAVI_STATE::READY);  
    setReadyMoving(READY_MOVING::COMPLETE);
    ret = ROBOT_STATUS::START;  
    break;
  case READY_NAVIGATION::FAIL :
    ret = ROBOT_STATUS::FAIL;
    break;    
  default:
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] processNavigationReady readyState Error!! : %d", static_cast<int>(readyNavi));
    break;
  }

  return ret;
}

int8_t ReturnCharger::localizationChecker()
{
  int8_t ret = 0;
  if(state_utils->isStartLocalization()){
    // double wait_localize_time = node_->now().seconds()-state_utils->getLocalizationStartTime();
    if(state_utils->getLocalizationComplete()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] localization Done ");
      ret = 1;
    }else if(state_utils->isLocalizationError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] localization Error ");
      ret = -1;
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] skip localization");
    ret = 1;
  }
  return ret;
}

void ReturnCharger::processMoveTarget()
{  
  int localize_result = 0;
  switch (readyMoving)
  {
  case READY_MOVING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      if(state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING){
        setReadyMoving(READY_MOVING::COMPLETE);
      }else{
        setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
      }
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] lidar Error");
      setReadyMoving(READY_MOVING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] tof Error");
      setReadyMoving(READY_MOVING::FAIL);
    }else if(state_utils->isCamreaError()){
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] camera Error");
      setReadyMoving(READY_MOVING::FAIL);
    }
    break;
  case READY_MOVING::REQUEST_POSE_ESTIMATE :
    state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::SAVED_POSE);
    setReadyMoving(READY_MOVING::CHECK_POSE_ESTIMATE);
    break;
  case READY_MOVING::CHECK_POSE_ESTIMATE :
    localize_result = localizationChecker();
    if(localize_result > 0){
      setReadyMoving(READY_MOVING::COMPLETE);
    }
    else if(localize_result < 0) setReadyMoving(READY_MOVING::FAIL);
    break;
  case READY_MOVING::COMPLETE :
  if(!bSendGoal){
    state_utils->publishClearCostMap();
    state_utils->publishSenSorManagerOn();
    state_utils->publishManeuverOn();
    moveToDock();
    publishTargetPosition(return_pose.x, return_pose.y,return_pose.theta);
  }else if(state_utils->getMovingStateID() == NAVI_STATE::READY && node_->now().seconds()-send_goal_start_time >= 10){
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Send Goal after 10 sec over no action");
  }
    
    break;
  case READY_MOVING::FAIL :
    state_utils->setStatusID(ROBOT_STATUS::FAIL);
    break;   
  default:
    break;
  };
}


void ReturnCharger::pauseReturnCharger() {
  RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] PAUSE Signal");

  // Check if the goal handle exists and is in an active state
  if (future_goal_handle_) {
    auto status = future_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
        status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
      client_->async_cancel_goal(future_goal_handle_);
      state_utils->setMovingStateID(NAVI_STATE::PAUSE);
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] ReturnCharger paused");
    } else {
      RCLCPP_WARN(node_->get_logger(), "[ReturnCharger] Cannot pause, goal is not active. Status: %d", status);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "[ReturnCharger] Cannot pause, no valid goal handle");
  }
}

///////////////function in ReturnCharger
void ReturnCharger::moveToDock() {
  startMonitorReturnCharger();

  if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "[ReturnCharger] Action server not available after waiting");
    state_utils->setMovingStateID( NAVI_STATE::FAIL,NAVI_FAIL_REASON::SERVER_NO_ACTION);
    state_utils->publishMoveFailError();
    return;
  }

  if (new_targetcall_flag && future_goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Cancelling previous goal due to new target.");

    auto cancel_callback = [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
        if (response) {
            if (response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
                RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Previous goal successfully canceled due to new target.");
            } else {
                RCLCPP_WARN(node_->get_logger(), "[ReturnCharger] Failed to cancel previous goal: %d", response->return_code);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "[ReturnCharger] Cancel goal response is null.");
        }
        new_targetcall_flag = false;
        future_goal_handle_.reset();
    };

    client_->async_cancel_goal(future_goal_handle_, std::bind(cancel_callback, std::placeholders::_1));

    return;
  }

  state_utils->publishClearCostMap();
  goal_msg = std::make_shared<nav2_msgs::action::NavigateToPose::Goal>();

  goal_msg->pose.pose.position.x = return_pose.x;
  goal_msg->pose.pose.position.y = return_pose.y;
  goal_msg->pose.pose.position.z = 0.0;

  // Convert theta (yaw) to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, return_pose.theta);
  goal_msg->pose.pose.orientation = tf2::toMsg(q);
  goal_msg->pose.header.frame_id = "map";
  goal_msg->pose.header.stamp = node_->now();

  // Send goal and wait for result
  send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback = [this](const auto &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Goal reached successfully.");
      state_utils->setMovingStateID(NAVI_STATE::ARRIVED_GOAL); // node_->robot_status = 2; //
      // Update goal_status to indicate success
      break;
    case rclcpp_action::ResultCode::ABORTED:
      if(++retry_move_target > state_utils->getMoveGoalRetryCount()){
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Goal was aborted retry count : %u",retry_move_target);
        state_utils->saveLastPosition();
        state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::GOAL_ABORT);
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
        state_utils->publishMoveFailError();
      }else{
        state_utils->publishClearCostMap();
        client_->async_send_goal(*goal_msg, send_goal_options);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Goal was aborted retry send goal X : %f, Y : %f, retry : %u",return_pose.x,return_pose.y,retry_move_target);
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Goal was canceled.");
      state_utils->saveLastPosition();
      state_utils->setMovingStateID(NAVI_STATE::PAUSE); // node_->robot_status = 3;  // FAIL
      // STATUS
      break;
    default:
      if(++retry_move_target > state_utils->getMoveGoalRetryCount()){
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Unknown result code retry count : %u",retry_move_target);
        state_utils->saveLastPosition();
        state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::UNKWON);
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
        state_utils->publishMoveFailError();
      }else{
        state_utils->publishClearCostMap();
        client_->async_send_goal(*goal_msg, send_goal_options);
        RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Unknown result code retry retry send goal X : %f, Y : %f, retry : %u",return_pose.x,return_pose.y,retry_move_target);
      }
      break;
    }
  };

  send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>goal_handle) {
    if (!goal_handle) {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Goal was rejected by the server.");
      state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::GOAL_REJECT);
      state_utils->publishMoveFailError();
    } else {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Goal accepted by the server.");
      future_goal_handle_ = goal_handle; // Store the handle for pause/resume
      state_utils->setMovingStateID(NAVI_STATE::MOVE_GOAL);
    }
  };
  client_->async_send_goal(*goal_msg, send_goal_options);
  bSendGoal = true;
  send_goal_start_time = node_->now().seconds();
  RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] send goal X : %f, Y : %f, Theta : %f",return_pose.x,return_pose.y,return_pose.theta);
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
    nav_status_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ReturnCharger::monitor_returnCharger, this));
}

void ReturnCharger::monitor_returnCharger() {
  if (state_utils->getMovingStateID() == NAVI_STATE::ARRIVED_GOAL) {
    // Goal reached successfully
    if (state_utils->getNodeStatusID() == NODE_STATUS::AUTO_MAPPING ||
        state_utils->getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING) {

      map_saver();
      state_utils->send_node_goal(NODE_STATUS::IDLE);
    }

    RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] Proceed with docking.");
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
      state_utils->send_node_goal(NODE_STATUS::IDLE);
    }

    pose robot = state_utils->getRobotPose();
    double distance = state_utils->getDistance(robot,return_pose);
    RCLCPP_WARN(node_->get_logger(), "[ReturnCharger] failed robot X : %f, Y : %f charger X : %f, Y: %f, distance : %f",robot.x,robot.y,return_pose.x,return_pose.y,distance);
    if(distance < 1.0){
      auto req_state_msg = std_msgs::msg::UInt8();
      req_state_msg.data = int(REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING);
      req_robot_cmd_pub_->publish(req_state_msg);
    }else{
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
    }

    dock_pose_estimate = false;
    stopMonitorReturnCharger();
  } else {
    // RCLCPP_WARN(node_->get_logger(), "Navigation idle, stop monitor ");
  }
}

void ReturnCharger::reset_timerNaviStatus() {
    if (nav_status_timer_) {
      nav_status_timer_.reset();
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] reset_timerNaviStatus");
    } else {
      RCLCPP_INFO(node_->get_logger(), "[ReturnCharger] nav_status_timer_ is allready reset ");
    }
}
//////////////saving map
void ReturnCharger::map_saver() {
  int map_save_result = std::system(
      "ros2 run nav2_map_server map_saver_cli -f /home/airbot/app_rw/map/airbot_map_00 "
      "--ros-args -p save_map_timeout:=10.0");
  int map_save_cnt = 0;

  while (map_save_result != 0) {
    if (map_save_cnt++ < 10) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[ReturnCharger] Map save command failed with error code, trying again: %d",
                   map_save_result);
      map_save_result = std::system(
          "ros2 run nav2_map_server map_saver_cli -f /home/airbot/app_rw/map/airbot_map_00 "
          "--ros-args -p save_map_timeout:=10.0");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[ReturnCharger] Map save command failed over 10times");
      break;
    }
  }
}

} // namespace airbot_state
