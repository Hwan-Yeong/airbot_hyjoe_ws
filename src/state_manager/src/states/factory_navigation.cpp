#include "state_manager/states/factory_navigation.hpp"

namespace airbot_state {

FactoryNavigation::FactoryNavigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {}

void FactoryNavigation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Preparing FactoryNavigation STATE");
  target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
  rotation_move_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);

  req_target_sub_ = node_->create_subscription<robot_custom_msgs::msg::Position>("/move_target", 10,std::bind(&FactoryNavigation::target_callback, this, std::placeholders::_1));
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  state_utils->setMovingStateID( NAVI_STATE::IDLE);
  state_utils->setStatusID(ROBOT_STATUS::READY);
  setReadyMoving(READY_MOVING::IDLE);
  setReadyNavigation(READY_NAVIGATION::LAUCN_NODE);
  state_utils->startSensorMonitor();
}

void FactoryNavigation::setReadyNavigation(READY_NAVIGATION set)
{
  if(readyNavi != set){
    RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] setReady FactoryNavigation %s", enumToString(set).c_str());
  }
  readyNavi = set;
}
void FactoryNavigation::setReadyMoving(READY_MOVING set)
{
  if(readyMoving != set){
    RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] setReadyMoving %s", enumToString(set).c_str());
  }
  readyMoving = set;
}
void FactoryNavigation::run(const std::shared_ptr<StateUtils> &state_utils) {
  //1. sensor On (before move-target)
  //2. poseEstimate (before move-target) - pub(current.pose); 
  //3. pub TargetPose to Navi
  //4. monitor NaviState
  //5. Sensor Off (after Goal-Arrived)
  ROBOT_STATUS ready_check;
  
  if(state_utils->getOnstationStatus()){
    RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Robot get on Docking Station!!!");
    auto req_state_msg = std_msgs::msg::UInt8();
    req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
    req_robot_cmd_pub_->publish(req_state_msg);
    return;
  }

  switch (state_utils->getStatusID())
  {
  case ROBOT_STATUS::READY :
    ready_check = processNavigationReady();
    if(ready_check == ROBOT_STATUS::START){
      state_utils->setNodeStatusID( NODE_STATUS::NAVI );
    }
    state_utils->setStatusID(ready_check);
    break;
    case ROBOT_STATUS::START :
    if( state_utils->getMovingStateID() == NAVI_STATE::PAUSE ){
      state_utils->setMovingStateID(NAVI_STATE::READY); //resume
    }

    if(state_utils->getMovingStateID() == NAVI_STATE::READY){
      processMoveTarget();
    }
    break;
    case ROBOT_STATUS::PAUSE :
    if (state_utils->getMovingStateID() != NAVI_STATE::PAUSE) {
      pauseNavigation();
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
    RCLCPP_INFO(node_->get_logger(), "[Navigation] Running UnKown Status");
    break;
  }
}

void FactoryNavigation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::post_run(state_utils);
  state_utils->saveLastPosition();
  exitNavigationNode();
  RCLCPP_INFO(node_->get_logger(), "[Navigation] Exiting Navigation STATE");
  req_robot_cmd_pub_.reset();
  target_pose_pub_.reset();
  state_utils->stopMonitorOdom();
}

///////////////function in navigation

// NAVIGATION FUNCTION

void FactoryNavigation::exitNavigationNode() {
  if (state_utils->stopProcess("/home/airbot/navigation_pid.txt")) {
    RCLCPP_INFO(node_->get_logger(), "exit Navigation Node");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Fail - kill Navigation Node");
  }
  state_utils->setNodeStatusID( NODE_STATUS::IDLE );
}

bool FactoryNavigation::startNavigation() {
  bool ret = false;

  if (state_utils->startProcess("ros2 launch airbot_navigation factory_navigation.launch.py", "/home/airbot/navigation_pid.txt")){
      nav_node_start_time = node_->get_clock()->now();
      RCLCPP_INFO(node_->get_logger(), "Factory Navigation Start Success");
      ret = true;
  }else{
      //reqStatus = REQUEST_STATUS::FAIL;
      RCLCPP_INFO(node_->get_logger(), "Factory Navigation Node Start FAIL");
  }
  return ret;
}

void FactoryNavigation::pauseNavigation() {
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

void FactoryNavigation::waitNodeLaunching() {
  // wait nav bringup time
  rclcpp::Duration checkTime = node_->get_clock()->now() - nav_node_start_time;
  int waitSec = 5 - (int)checkTime.seconds();

  if (waitSec > 0) {
    std::this_thread::sleep_for(std::chrono::seconds(waitSec));
    RCLCPP_INFO(node_->get_logger(), "START_NAVIGATION wait sec : %d ",
                waitSec);
  }
}

void FactoryNavigation::moveToTarget(double x, double y, double theta) {
  
  if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Action server not available after waiting");
    state_utils->setMovingStateID(NAVI_STATE::FAIL);
    state_utils->setMovingFailID(NAVI_FAIL_REASON::SERVER_NO_ACTION);
    state_utils->setStatusID(ROBOT_STATUS::FAIL);
    return;
  }

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

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

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

  send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>goal_handle) {
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "Goal was rejected by the server.");
          state_utils->setMovingStateID(NAVI_STATE::FAIL);
          state_utils->setMovingFailID(NAVI_FAIL_REASON::GOAL_REJECT);
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by the server.");
          future_goal_handle_ = goal_handle; // Store the handle for pause/resume
          state_utils->setMovingStateID(NAVI_STATE::MOVE_GOAL);
        }
      };

  client_->async_send_goal(*goal_msg, send_goal_options);
}

void FactoryNavigation::processMoveTarget()
{  
  int localize_result = 0;
  switch (readyMoving)
  {
  case READY_MOVING::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyMoving(READY_MOVING::REQUEST_POSE_ESTIMATE);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "LidarError");
      setReadyMoving(READY_MOVING::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "ToFError");
      setReadyMoving(READY_MOVING::FAIL);
    }
    break;
  case READY_MOVING::REQUEST_POSE_ESTIMATE :
    state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
    setReadyMoving(READY_MOVING::CHECK_POSE_ESTIMATE);
    break;
  case READY_MOVING::CHECK_POSE_ESTIMATE :
  localize_result = localizationChecker();
  if(localize_result > 0) setReadyMoving(READY_MOVING::COMPLETE);
  else if(localize_result < 0) setReadyMoving(READY_MOVING::FAIL);
    break;
  case READY_MOVING::COMPLETE :
    state_utils->publishClearCostMap();
    moveToTarget(movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
    publishTargetPosition(movingData.target_position.x, movingData.target_position.y, movingData.target_position.theta);
    break;
  case READY_MOVING::FAIL :
    state_utils->setStatusID(ROBOT_STATUS::FAIL);
    break;   
  default:
    break;
  };
}

ROBOT_STATUS FactoryNavigation::processNavigationReady()
{
  ROBOT_STATUS ret = ROBOT_STATUS::READY;
  int localize_result = 0;
  int node_result = 0;
  switch (readyNavi)
  {
  case READY_NAVIGATION::LAUCN_NODE :
    if(naviNodeLauncher()){
      setReadyNavigation(READY_NAVIGATION::CHECK_NODE);
    }else{
      setReadyNavigation(READY_NAVIGATION::FAIL);
      ret = ROBOT_STATUS::FAIL;
    }
    break;
    case READY_NAVIGATION::CHECK_NODE :
    node_result = naviNodeChecker();
    if(node_result > 0){
      setReadyNavigation(READY_NAVIGATION::CHECK_SENSOR);
    }else if(node_result < 0){
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }
    break;
    case READY_NAVIGATION::CHECK_SENSOR :
    if(state_utils->isSensorReady()){
      setReadyNavigation(READY_NAVIGATION::CHECK_ODOM_RESET);
    }else if(state_utils->isLidarError()){
      RCLCPP_INFO(node_->get_logger(), "LidarError");
      ret = ROBOT_STATUS::FAIL;
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }else if(state_utils->isToFError()){
      RCLCPP_INFO(node_->get_logger(), "ToFError");
      ret = ROBOT_STATUS::FAIL;
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }
    break;     
  case READY_NAVIGATION::CHECK_ODOM_RESET :
    if(state_utils->isOdomResetError()){
      RCLCPP_INFO(node_->get_logger(), "odom reset Error");
      ret = ROBOT_STATUS::FAIL;
      setReadyNavigation(READY_NAVIGATION::FAIL);
    }
    else if(resetOdomChecker()){
      setReadyNavigation(READY_NAVIGATION::REQUEST_POSE_ESTIMATE);
    }
    break;
  case READY_NAVIGATION::REQUEST_POSE_ESTIMATE :
      state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
      setReadyNavigation(READY_NAVIGATION::CHECK_POSE_ESTIMATE);
    break;
  case READY_NAVIGATION::CHECK_POSE_ESTIMATE :
      localize_result = localizationChecker();
      if(localize_result > 0){
        setReadyNavigation(READY_NAVIGATION::COMPLETE);
        ret = ROBOT_STATUS::START;
      }else if(localize_result < 0){
        state_utils->publishAllSensorOff();
        setReadyNavigation(READY_NAVIGATION::FAIL);
        ret = ROBOT_STATUS::FAIL;
      }
    break;    
  default:
  RCLCPP_INFO(node_->get_logger(), "processNavigationReady readyState Error!! : %d", static_cast<int>(readyNavi));
    break;
  }

  return ret;
}

bool FactoryNavigation::naviNodeLauncher()
{
  bool ret = false;
  if(state_utils->getNodeStatusID() == NODE_STATUS::NAVI){
    RCLCPP_INFO(node_->get_logger(), "Node status already Navi skip node launch ");
    ret = true;
  }else{
    if(startNavigation()){
	      RCLCPP_INFO(node_->get_logger(), "Navi node launch Success");
	      ret = true;
		}else{
      ret = false;
      RCLCPP_INFO(node_->get_logger(), "Navi node launch Fail");
    }
  }
  return ret;
}

int FactoryNavigation::naviNodeChecker()
{
  int ret = 0;
  double wait_navi_launch = node_->now().seconds()-node_start_time;
  RCLCPP_INFO(node_->get_logger(), "[Navigation] Navigation NODE ALL RUNNING -> launch time %f sec", wait_navi_launch);
  if(state_utils->isValidateNode(NODE_STATUS::NAVI, node_start_time)){
      waitNodeLaunching();
      RCLCPP_INFO(node_->get_logger(), "Navi node launch Complete time : %f", wait_navi_launch);
      ret = 1;
  }else if(wait_navi_launch >= 30){
    RCLCPP_INFO(node_->get_logger(), "Navi node launch Fail time : %f", wait_navi_launch);
    ret = -1;
  }
  //else{
  //   RCLCPP_INFO(node_->get_logger(), "Navi node launch Fail time : %f", wait_navi_launch);
  // }
  return ret;
}

bool FactoryNavigation::resetOdomChecker()
{
  bool ret = false;
  if(!state_utils->isStartOdomReset()){
    RCLCPP_INFO(node_->get_logger(), "skip odom reset Checker ");
    ret = true;
  }
  else if(state_utils->getOdomResetDone()){
    RCLCPP_INFO(node_->get_logger(), "odom reset Done ");
    ret = true;
  } 

  return ret;
}

int8_t FactoryNavigation::localizationChecker()
{
  int8_t ret = 0;
  if(state_utils->isStartLocalization()){
    // double wait_localize_time = node_->now().seconds()-state_utils->getLocalizationStartTime();
    if(state_utils->getLocalizationComplete()){
      RCLCPP_INFO(node_->get_logger(), "localization Done ");
      ret = 1;
    }else if(state_utils->isLocalizationError()){
      RCLCPP_INFO(node_->get_logger(), "localization Error ");
      ret = -1;
    }
  }else{
    RCLCPP_INFO(node_->get_logger(), "skip localization");
    ret = 1;
  }

  return ret;
}


void FactoryNavigation::publishTargetPosition(double x, double y, double theta) {
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

void FactoryNavigation::target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg) {
  state_utils->setStartOnStation(false);
  movingData.bStartMoving = true;
  movingData.target_position.x = msg->x;
  movingData.target_position.y = msg->y;
  movingData.target_position.theta = msg->theta;
  state_utils->setMovingStateID(NAVI_STATE::READY);
  setReadyMoving(READY_MOVING::CHECK_SENSOR);
  RCLCPP_INFO(node_->get_logger(), "target_callback x : %f , y : %f, theta : %f ", movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
}

bool FactoryNavigation::startRotation(int type, double targetAngle)
{
    bool ret = false;
    pose robotPose = state_utils->getRobotPose();
    //movingState = NAVI_STATE::IDLE;
    rotation.type = type;
    if(type == 0){
        rotation.target = robotPose.theta+targetAngle;
        ret = true;
    }else if(type==1){
        rotation.target = targetAngle;
        ret = true;
    }else if(type==2 || type==3){
        rotation.accAngle = 0.0;
        rotation.preTheta = normalize_angle(robotPose.theta);
        ret = true;
    }else{
        RCLCPP_INFO(node_->get_logger(), "Request Rotate type error : %d", type);
    }

    if(ret){
        startRotateMonitor();
    }

    return ret;
}

//Timer to checck Odom value to do One point rotation

void FactoryNavigation::reset_Rotationtimer()
{
    rotation.progress = false;
    if(rotation_target_timer_){
        RCLCPP_INFO(node_->get_logger(), "reset_Rotationtimer");
        rotation_target_timer_.reset();
        RCLCPP_INFO(node_->get_logger(), "reset_Rotationtimer - end ");
    }else{
        RCLCPP_INFO(node_->get_logger(), "odom_target_timeris allready reset ");
    }
}

void FactoryNavigation::startRotateMonitor()
{
    if(!rotation.progress){
        rotation_target_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FactoryNavigation::progressRotation, this));
        rotation.progress = true;
    }else{
        RCLCPP_INFO(node_->get_logger(), "already Rotation progressing");
    }
    
}

void FactoryNavigation::stopMonitorRotate()
{
    reset_Rotationtimer();
}

void FactoryNavigation::progressRotation()
{
    // If navigation is enabled, use amcl_pose
    pose current = state_utils->getRobotPose();
    if(rotation.type == 0 || rotation.type == 1){
        progressRotationTarget(rotation.target,current.theta);
    }else{
        int direction = (rotation.type == 3) ? 1 : -1;
        progressRotation360(direction,current.theta);
    }
}

int FactoryNavigation::checkRotationDirection(double diff)
{
    return (diff > 0) ? 1 : -1;
}

bool FactoryNavigation::checkRotationTarget(double diff)
{
    bool ret = false;
    if (std::fabs(diff) < 0.175){
        RCLCPP_INFO(node_->get_logger(), "Target angle reached"); 
        ret = true;
    }
    return ret;
}

void FactoryNavigation::progressRotationTarget(double target, double current)
{
    double v = 0, w = 0;
    double nomalize_target = normalize_angle(target);
    double nomalize_current = normalize_angle(current);
    double angle_diff = normalize_angle(nomalize_target-nomalize_current);

    if(checkRotationTarget(angle_diff)){
        publishVelocityCommand(v,w);
        //movingState = NAVI_STATE::COMPLETE_ROTATION;//node_->robot_status = 6;
        stopMonitorRotate();
        return;
    }
    
    int direction = checkRotationDirection(angle_diff);
    w = direction * 0.3; // Adjust speed if needed
    publishVelocityCommand(v,w);
}

void FactoryNavigation::progressRotation360(int direction, double current)
{
    double v = 0, w = 0;
    double nomalizeTheta = normalize_angle(current);
    double delta_theta = normalize_angle(nomalizeTheta - rotation.preTheta);

    rotation.accAngle += std::fabs(delta_theta);
    rotation.preTheta = nomalizeTheta;

    if (rotation.accAngle >= 2*M_PI){
        RCLCPP_INFO(node_->get_logger(), "Completed 360-degree rotation");
        publishVelocityCommand(v,w);
        // movingState = NAVI_STATE::COMPLETE_ROTATION;//node_->robot_status = 6;
        stopMonitorRotate();
        return;
    }

    w =  direction * 0.3;
    publishVelocityCommand(v,w);
}

double FactoryNavigation::normalize_angle(double angle) {
    // Normalize angle to the range [-p, p]
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void FactoryNavigation::publishVelocityCommand(double v, double w)
{
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = w;
    rotation_move_pub_->publish(cmd_msg);
    // RCLCPP_INFO(node_->get_logger(), "publishVelocityCommand V : %f, W : %f ", v,w);
}


} // namespace airbot_state
