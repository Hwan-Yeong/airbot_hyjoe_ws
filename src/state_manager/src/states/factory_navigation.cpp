#include "state_manager/states/factory_navigation.hpp"

namespace airbot_state {

FactoryNavigation::FactoryNavigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : stateBase(actionID, node, utils) {}

    void FactoryNavigation::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
      stateBase::pre_run(state_utils);
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] pre_run() -> Preparing FactoryNavigation state");
    
      rotation_sub_ = node_->create_subscription<robot_custom_msgs::msg::MoveNRotation>("/rotation", 10,std::bind(&FactoryNavigation::rotation_callback, this, std::placeholders::_1));
      target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 10);
      req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
    
      client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
    
      state_utils->setMovingStateID( NAVI_STATE::IDLE);
      state_utils->setStatusID(ROBOT_STATUS::READY);
      setReadyMoving(READY_MOVING::IDLE);
      setReadyNavigation(READY_NAVIGATION::LAUCN_NODE);
      state_utils->startSensorMonitor();
      future_goal_handle_.reset();
    }
    
    void FactoryNavigation::setReadyNavigation(READY_NAVIGATION set)
    {
      if(readyNavi != set){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] setReadyNavigation %s", enumToString(set).c_str());
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
      if( isFirstRunning() ){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] run() -> Running Navigation state");
      }
      //1. sensor On (before move-target)
      //2. poseEstimate (before move-target) - pub(current.pose); 
      //3. pub TargetPose to Navi
      //4. monitor NaviState
      //5. Sensor Off (after Goal-Arrived)
      ROBOT_STATUS ready_check;
      
      // if(state_utils->getOnstationStatus()){
      //   RCLCPP_INFO(node_->get_logger(), "[Navigation] Robot get on Docking Station!!!");
      //   auto req_state_msg = std_msgs::msg::UInt8();
      //   req_state_msg.data = int(REQUEST_ROBOT_CMD::START_ONSTATION);
      //   req_robot_cmd_pub_->publish(req_state_msg);
      //   return;
      // }
      if(state_utils->getStatusID() != ROBOT_STATUS::READY ){// NavigationReady가 완료되어 ROBOT_STATUS::START이후로 동작시작.
        movingData = state_utils->getTargetPosition(); //getTargetPosition을 받으면 state_utils의 bstartmoving이 토글로 false됨.
        if( movingData.bStartMoving ) // 새 target을 받으면 bstartmoving이 true상태로 센서 체크로 넘어감.
        {
          if(rotation.progress){
            stopMonitorRotate();
          }
          if( state_utils->getMovingStateID() != NAVI_STATE::ARRIVED_GOAL && state_utils->getMovingStateID() != NAVI_STATE::FAIL) {// result가 명확하게 나온상황은 제외한다.
            new_targetcall_flag = true; //abort skip flag.
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
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Running UnKown Status");
        break;
      }
    }
    
    void FactoryNavigation::post_run(const std::shared_ptr<StateUtils> &state_utils) {
      stateBase::post_run(state_utils);
      state_utils->saveLastPosition();
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] post_run() -> Exiting FactoryNavigation state");
      req_robot_cmd_pub_.reset();
      target_pose_pub_.reset();
      if (state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::STOP_WORKING) {
        state_utils->send_node_goal(NODE_STATUS::IDLE);
      }
      state_utils->stopMonitorOdom();
      state_utils->stopSensorMonitor();
      state_utils->disableArrivedGoalSensorsOffTimer();
      state_utils->publishSenSorManagerOff();
      state_utils->publishManeuverOff();
      if(future_goal_handle_)
      {
        clearMoveTarget();
      }
    }
    
    ///////////////function in navigation
    
    // NAVIGATION FUNCTION
    
    void FactoryNavigation::pauseNavigation() {
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Pause move to target");
      // Check if the goal handle exists and is in an active state
      if (future_goal_handle_) {
        auto status = future_goal_handle_->get_status();
        if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING ||
            status == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
          client_->async_cancel_goal(future_goal_handle_);
          state_utils->setMovingStateID(NAVI_STATE::PAUSE);
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] FactoryNavigation paused");
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "[FactoryNavigation] Cannot pause, goal is not active. Status: %d", status);
        }
      } else {
        RCLCPP_WARN(node_->get_logger(), "[FactoryNavigation] Cannot pause, no valid goal handle");
      }
    }
    
    
    void FactoryNavigation::moveToTarget(double x, double y, double theta) {
      
      if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[FactoryNavigation] Action server not available after waiting");
        state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::SERVER_NO_ACTION);
        state_utils->setStatusID(ROBOT_STATUS::FAIL);
        state_utils->publishMoveFailError();
        state_utils->publishSenSorManagerOff();
        state_utils->publishManeuverOff();
        return;
      }
    
      // Cancel the previous goal if a new target is received
      if (new_targetcall_flag && future_goal_handle_) {
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Cancelling previous goal due to new target.");
    
        auto cancel_callback = [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
            if (response) {
                if (response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
                    RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Previous goal successfully canceled due to new target.");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[FactoryNavigation] Failed to cancel previous goal: %d", response->return_code);
                }
            } else {
                RCLCPP_WARN(node_->get_logger(), "[FactoryNavigation] Cancel goal response is null.");
            }
            new_targetcall_flag = false; // Reset the flag after attempting to cancel
            future_goal_handle_.reset(); // Release the handle to prevent reuse
        };
    
        client_->async_cancel_goal(future_goal_handle_, std::bind(cancel_callback, std::placeholders::_1));
    
        // Don't send the new goal until the cancellation of the old one is complete!
        return; // Very important: Exit so we don't immediately send a new goal
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
    
      send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
      send_goal_options.result_callback = [this](const auto &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Goal reached successfully.");
          if(state_utils->getPathPlanDestination()){
            //state_utils->publishAlternativeDestinationError();
            state_utils->setMovingStateID(NAVI_STATE::ALTERNATE_GOAL);
          }else{
            state_utils->setMovingStateID(NAVI_STATE::ARRIVED_GOAL);
          }
          state_utils->publishSenSorManagerOff();
          state_utils->publishManeuverOff();
          break;
        case rclcpp_action::ResultCode::ABORTED:
          if(++retry_move_target > state_utils->getMoveGoalRetryCount()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Goal was aborted retry count : %u",retry_move_target);
            state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::GOAL_ABORT);
            state_utils->publishMoveFailError();
            state_utils->publishSenSorManagerOff();
            state_utils->publishManeuverOff();
          }else{
            state_utils->publishClearCostMap();
            client_->async_send_goal(*goal_msg, send_goal_options);
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Goal was aborted retry send goal X : %f, Y : %f, retry : %u",movingData.target_position.x,movingData.target_position.y,retry_move_target);
          }
          
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Goal was canceled.");
          break;
        default:
          if(++retry_move_target > state_utils->getMoveGoalRetryCount()){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Unknown result code retry count : %u",retry_move_target);
            state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::UNKWON);
            state_utils->setStatusID(ROBOT_STATUS::FAIL);
            state_utils->publishMoveFailError();
            state_utils->publishSenSorManagerOff();
            state_utils->publishManeuverOff();
          }else{
            state_utils->publishClearCostMap();
            client_->async_send_goal(*goal_msg, send_goal_options);
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Unknown result code retry send goal X : %f, Y : %f, retry : %u",movingData.target_position.x,movingData.target_position.y,retry_move_target);
          }
          break;
        }
        future_goal_handle_.reset(); // Clear the goal handle after completion/abortion
      };
    
      send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
        if (!goal_handle) {
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Goal was rejected by the server");
          state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::GOAL_REJECT);
          state_utils->publishMoveFailError();
          state_utils->publishSenSorManagerOff();
          state_utils->publishManeuverOff();
          #if 0
          if(++retry_move_target > state_utils->getMoveGoalRetryCount()){
            RCLCPP_INFO(node_->get_logger(), "[Navigation] Goal was rejected by the server retry Over count : %u",retry_move_target);
            state_utils->setMovingStateID(NAVI_STATE::FAIL,NAVI_FAIL_REASON::GOAL_REJECT);
            state_utils->publishMoveFailError();
          }else{
            state_utils->publishClearCostMap();
            client_->async_send_goal(*goal_msg, send_goal_options);
            RCLCPP_INFO(node_->get_logger(), "[Navigation] Goal was rejected by the server retry send goal X : %f, Y : %f, retry : %u",movingData.target_position.x,movingData.target_position.y,retry_move_target);
          }
          #endif
        } else {
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Goal accepted by the server.");
          future_goal_handle_ = goal_handle; // Store the handle for pause/resume
          state_utils->setMovingStateID(NAVI_STATE::MOVE_GOAL);
        }
      };
    
      client_->async_send_goal(*goal_msg, send_goal_options);
      bSendGoal = true;
      send_goal_start_time = node_->now().seconds();
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] send goal X : %f, Y : %f, Theta : %f",x,y,theta);
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
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] LidarError");
          setReadyMoving(READY_MOVING::FAIL);
        }else if(state_utils->isToFError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] ToFError");
          setReadyMoving(READY_MOVING::FAIL);
        }else if(state_utils->isCamreaError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] CameraError");
          setReadyMoving(READY_MOVING::FAIL);
        }
        break;
      case READY_MOVING::REQUEST_POSE_ESTIMATE :
        state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::ROBOT_POSE);
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
          state_utils->publishManeuverOn();
          state_utils->publishSenSorManagerOn();
          moveToTarget(movingData.target_position.x,movingData.target_position.y,movingData.target_position.theta);
          publishTargetPosition(movingData.target_position.x, movingData.target_position.y, movingData.target_position.theta);
        }else if(state_utils->getMovingStateID() == NAVI_STATE::READY && node_->now().seconds()-send_goal_start_time >= 10){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Send Goal after 10 sec over no action");
        }
        
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
        state_utils->send_node_goal(NODE_STATUS::FT_NAVI);
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
          setReadyNavigation(READY_NAVIGATION::CHECK_ODOM_RESET);
        }else if(state_utils->isLidarError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] lidar Error");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }else if(state_utils->isToFError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] tof Error");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }else if(state_utils->isCamreaError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] CameraError");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }
        break;     
      case READY_NAVIGATION::CHECK_ODOM_RESET :
        if(state_utils->isOdomResetError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] odom reset Error");
          ret = ROBOT_STATUS::FAIL;
          setReadyNavigation(READY_NAVIGATION::FAIL);
        }
        else if(resetOdomChecker()){
          setReadyNavigation(READY_NAVIGATION::REQUEST_POSE_ESTIMATE);
        }
        break;
      case READY_NAVIGATION::REQUEST_POSE_ESTIMATE :
          if(state_utils->getRobotCMDID().robot_cmd == REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION){
            state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::INIT_POSE);
          }else{
            state_utils->startLocalizationMonitor(LOCALIZATION_TYPE::ROBOT_POSE);
          }
          setReadyNavigation(READY_NAVIGATION::CHECK_POSE_ESTIMATE);
        break;
      case READY_NAVIGATION::CHECK_POSE_ESTIMATE :
          localize_result = localizationChecker();
          if(localize_result > 0){
            setReadyNavigation(READY_NAVIGATION::COMPLETE);
            ret = ROBOT_STATUS::START;
          }else if(localize_result < 0){
            setReadyNavigation(READY_NAVIGATION::FAIL);
            ret = ROBOT_STATUS::FAIL;
          }
        break;    
      default:
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] process FactoryNavigation Ready readyState Error!! : %d", static_cast<int>(readyNavi));
        break;
      }
    
      return ret;
    }
    
    bool FactoryNavigation::resetOdomChecker()
    {
      bool ret = false;
      if(!state_utils->isStartOdomReset()){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] skip odom reset Checker ");
        ret = true;
      }
      else if(state_utils->getOdomResetDone()){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] odom reset Done ");
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
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] localization Done ");
          ret = 1;
        }else if(state_utils->isLocalizationError()){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] localization Error ");
          ret = -1;
        }
      }else{
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] skip localization");
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
                  "[FactoryNavigation] Running FactoryNavigation with shared data: [%d]",
                  __LINE__);
      // Publish the target pose
      target_pose_pub_->publish(req_target_position);
    }
    
    void FactoryNavigation::setRotationTarget(bool immediately,uint8_t type, double targetAngle)
    {
        pose robotPose = state_utils->getRobotPose();
        rotation.immediately = immediately;
        rotation.type = type;
        if(type == 0){
          rotation.target = robotPose.theta+targetAngle;
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation Diff : %f",rotation.target);
        }else if(type==1){
          rotation.target = targetAngle;
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation Pose : %f",rotation.target);
        }else if(type==2){
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] there is no Auto Rotation ");
        }else if(type == 3 || type == 4){
          if(type == 3){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation Left 360 Deg");
          }else{
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation Right 360 Deg");
          }  
          rotation.accAngle = 0.0;
          rotation.preTheta = normalize_angle(robotPose.theta);
        }else{
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] no rotation ");
        }
    }
    
    void FactoryNavigation::startRotation(bool immediately ,uint8_t type, double targetAngle)
    {
        setRotationTarget(immediately,type, targetAngle);
        startRotateMonitor();
    }
    
    //Timer to checck Odom value to do One point rotation
    
    void FactoryNavigation::reset_Rotationtimer()
    {
        rotation.progress = false;
        if(rotation_target_timer_){
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] reset_Rotationtimer");
            rotation_target_timer_.reset();
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] reset_Rotationtimer - end ");
        }else{
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] odom_target_timeris allready reset ");
        }
    }
    
    void FactoryNavigation::startRotateMonitor()
    {
        rotation_start_time = node_->now().seconds();
        state_utils->setMovingStateID(NAVI_STATE::START_ROTAION);
        if(!rotation.progress){
            rotation_target_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FactoryNavigation::progressRotation, this));
            rotation.progress = true;
        }else{
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] already Rotation progressing");
        }
    }
    
    void FactoryNavigation::stopMonitorRotate()
    {
      reset_Rotationtimer();
    }
    
    void FactoryNavigation::progressRotation()
    {
      pose current = state_utils->getRobotPose();
      double runTime = node_->now().seconds()-rotation_start_time;
    
      if(rotation.immediately || (!rotation.immediately && runTime >= 2.0)){
        if(rotation.type == 0 || rotation.type == 1){
          progressRotationTarget(rotation.target,current.theta);
        }else if(rotation.type == 3 || rotation.type == 4){
          int direction = (rotation.type == 3) ? 1 : -1;
          progressRotation360(direction,current.theta);
        }else{
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation type error : %d", rotation.type);
          state_utils->setStatusID(ROBOT_STATUS::FAIL);
          stopMonitorRotate();
        }
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
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Target angle reached"); 
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
        double runTime = node_->now().seconds()-rotation_start_time;

        if(checkRotationTarget(angle_diff)){
            state_utils->setMovingStateID(NAVI_STATE::ROTATION_COMPLETE);
            state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
            state_utils->publishVelocityCommand(v,w);
            stopMonitorRotate();
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation Complete angle_diff : %f, target : %f, current : %f, runTime : %f ",angle_diff,nomalize_target,nomalize_current,runTime);
            return;
        }else{
          RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] progressRotationTarget angle_diff : %f, target : %f, current : %f, runTime : %f ",angle_diff,nomalize_target,nomalize_current,runTime);
        }
        
        int direction = checkRotationDirection(angle_diff);
        w = direction * 0.3; // Adjust speed if needed
        state_utils->publishVelocityCommand(v,w);
    }
    
    void FactoryNavigation::progressRotation360(int direction, double current)
    {
        double v = 0, w = 0;
        double nomalizeTheta = normalize_angle(current);
        double delta_theta = normalize_angle(nomalizeTheta - rotation.preTheta);
        double runTime = node_->now().seconds()-rotation_start_time;
        rotation.accAngle += std::fabs(delta_theta);
        rotation.preTheta = nomalizeTheta;
    
        if (rotation.accAngle >= 2*M_PI){
            state_utils->setMovingStateID(NAVI_STATE::ROTATION_COMPLETE);
            state_utils->setStatusID(ROBOT_STATUS::COMPLETE);
            state_utils->publishVelocityCommand(v,w);
            stopMonitorRotate();
            RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Rotation Complete accAngle : %f, runTime : %f ",rotation.accAngle,runTime);
            return;
        }
    
        w =  direction * 0.3;
        state_utils->publishVelocityCommand(v,w);
    }
    
    double FactoryNavigation::normalize_angle(double angle) {
        // Normalize angle to the range [-p, p]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    void FactoryNavigation::rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg) {
      RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] rotation_callback ");
      if(state_utils->getMovingStateID() == NAVI_STATE::MOVE_GOAL ){
        RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Warnning!!! NAVI Is Moving can`t Rotation");
      }else{
        state_utils->startSensorMonitor();
        startRotation(true,msg->type, msg->theta);
      }
    }
    
    
    void FactoryNavigation::clearMoveTarget(){
      auto cancel_callback = [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
        if (response) {
            if (response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE) {
                RCLCPP_INFO(node_->get_logger(), "[FactoryNavigation] Previous goal successfully canceled due to new target.");
            } else {
                RCLCPP_WARN(node_->get_logger(), "[FactoryNavigation] Failed to cancel previous goal: %d", response->return_code);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "[FactoryNavigation] Cancel goal response is null.");
        }
        future_goal_handle_.reset(); // Release the handle to prevent reuse
        client_.reset();
      };
    
      client_->async_cancel_goal(future_goal_handle_, std::bind(cancel_callback, std::placeholders::_1));
    }

} // namespace airbot_state
