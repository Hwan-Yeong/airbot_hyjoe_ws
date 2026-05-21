#include "state_manager/states/state_base.hpp"
// ****스테이트의 공통 동작 수행.
namespace airbot_state {

stateBase::stateBase(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : node_(node), state_utils(utils), id(actionID), first_running(true)
{
  rotation_sub_ = node_->create_subscription<robot_custom_msgs::msg::MoveNRotation>("/rotation", 10,std::bind(&stateBase::rotation_callback, this, std::placeholders::_1));
  scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
    std::bind(&stateBase::scan_callback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "[stateBase] created rotation_sub_ [id :%s]", enumToString((ROBOT_STATE)id).c_str());
}

void stateBase::pre_run(const std::shared_ptr<StateUtils> &) {
  first_running = true;
  get_start_rotate = false;
  odom_retry_count = 0;
  wait_odom_time = 0.0;
  rotate_state = ROTATE_STATE::ROTATE_IDLE;
}

void stateBase::run(const std::shared_ptr<StateUtils> &) {
  first_running = false;

  if( get_start_rotate ){
    RCLCPP_INFO(node_->get_logger(), "[stateBase] get_start_rotate [id :%s] rotate_state %d ", enumToString((ROBOT_STATE)id).c_str(), static_cast<int>(rotate_state));
    switch( rotate_state ){
    case ROTATE_STATE::ROTATE_IDLE:
      break;
    case ROTATE_STATE::ROTATE_ODOM_SET:
      state_utils->startOdomCheck();
      odom_set_start_time = std::chrono::steady_clock::now();
      rotate_state = ROTATE_STATE::ROTATE_ODOM_CHECK;
      state_utils->setMovingStateID(NAVI_STATE::START_ROTAION);
      break;
    case ROTATE_STATE::ROTATE_ODOM_CHECK:
      wait_odom_time =  state_utils->getSteadyClockRunningSeconds(odom_set_start_time);
      if( !state_utils->getPrepareOdomFlag() ){ // odom이 아직 들어오지 않은 경우.
        
        if( wait_odom_time > 0.5 ){ //500ms 이상 odom이 들어오지 않았으 떄
          if( odom_retry_count > 3 ){
            RCLCPP_ERROR(node_->get_logger(), "Failed to get odom data waittime[%.2f] / count[%d] Aborting rotation.",wait_odom_time,odom_retry_count);
            state_utils->setMovingStateID(NAVI_STATE::ROTATION_COMPLETE);// odom diable 된 경우 rotation 끝을 위에 알려줌.(SOC이동중 방지)
            rotate_state = ROTATE_STATE::ROTATE_IDLE;
            get_start_rotate = false;
            odom_retry_count = 0;
          } else{ //odomcallback distable 후 retry
            RCLCPP_ERROR(node_->get_logger(), "Failed to get odom data.. preceed retry odom enable waittime[%.2f] / count %d",wait_odom_time, odom_retry_count);
            odom_retry_count++;
            rotate_state = ROTATE_STATE::ROTATE_ODOM_SET;
          }
        } else {
          RCLCPP_DEBUG(node_->get_logger(), "waiting for odom data...");
        }
      } else{
        odom_retry_count = 0;
        odom_set_start_time = std::chrono::steady_clock::now();
        wait_odom_time = 0.0;
        if( state_utils->getStopDrivingFlag()){
          state_utils->startSensorMonitor();
          if(state_utils->getFactoryMode() || state_utils->getStateID() == ROBOT_STATE::MANUAL_CONTROL){
            rotate_state = ROTATE_STATE::ROTATE_START; //rotate 시작
          } else{
            rotate_state = ROTATE_STATE::ROTATE_LIDAR_CHECK; // lidar check
          }
        }
      }
      break;
    case ROTATE_STATE::ROTATE_LIDAR_CHECK:
      if( state_utils->isLidarSensorOK() ){
        rotate_state = ROTATE_STATE::ROTATE_START; //rotate 시작
      }
      #if USE_LIDAR_STATE_CHECK == 0
      else {
        RCLCPP_INFO(node_->get_logger(),
          "[stateBase] Waiting for LiDAR state to be ON [front : %s, back : %s, scan hz OK: %s]",
          state_utils->lidar_front_state_on ? "ON" : "OFF",
          state_utils->lidar_back_state_on  ? "ON" : "OFF",
          state_utils->bLidarSensorOK       ? "STABLE" : "UNSTABLE"
        );
      }
      #endif
      break;
    case ROTATE_STATE::ROTATE_START:
      if( state_utils->getPrepareOdomFlag() ){ //odom 수신 확인 시
        startRotation(true,rotate_msg->type, rotate_msg->theta); //rotation 세팅 후 rotation timer 시작.
        get_start_rotate = false;
      } else {
        state_utils->setMovingStateID(NAVI_STATE::ROTATION_COMPLETE); // odom diable 된 경우 rotation 끝을 위에 알려줌.(SOC이동중 방지)
      }
      break;
    }
  }
    
  if(state_utils->isReservedMoving() || state_utils->getManualControlFlag() ){ //타겟이 새로 들어오거나, manual control제어가 들어온경우. --> 회전 멈춤.
    if(rotation.progress){
      state_utils->setRotatePauseFlag(false); // 타겟이 새로 온경우 해제.
      RCLCPP_INFO(node_->get_logger(), "[stateBase] ReservedMoving [id :%s] rotate_state %d ", enumToString((ROBOT_STATE)id).c_str(), static_cast<int>(rotate_state));
      stopMonitorRotate();
    }
  }
}

void stateBase::post_run(const std::shared_ptr<StateUtils> &){
  RCLCPP_INFO(node_->get_logger(), "[stateBase] post_run [id :%s] rotate_state %d ", enumToString((ROBOT_STATE)id).c_str(), static_cast<int>(rotate_state));
  odom_retry_count = 0;
  wait_odom_time = 0.0;
  stopMonitorRotate(); // state 전환 시 rotation 중지.
  state_utils->setRotatePauseFlag(false); // state전환시 rotation pause 초기화.
}

bool stateBase::isFirstRunning(){ 
  bool ret = false;
  if(first_running){
    first_running = false;
    ret = true;
  }
  return ret;
}

void stateBase::rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg) {
  if( state_utils->getStateID() == (ROBOT_STATE)id ){
    RCLCPP_INFO(node_->get_logger(), "[stateBase][id :%s] rotation_callback ", enumToString((ROBOT_STATE)id).c_str());
    if(state_utils->getMovingStateID() == NAVI_STATE::MOVE_GOAL ){
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Warnning!!! NAVI Is Moving can`t Rotation");
    }else{
      if( state_utils->isReservedMoving()){
        state_utils->setReservedMoving(false); // 타겟이 있지만 rotation 명령이 들어오면 disable( 이동해야 할 타겟이 있는 상태에서 회전가능한 시나리오는 Lookatme 뿐.)
      }
      stopMonitorRotate();
      
      if( state_utils->getRotatePauseFlag() && state_utils->getMovingStateID() == NAVI_STATE::PAUSE){ //rotation pause상태에서 다른 rotation 명령이 들어오면.
        state_utils->setStateID(state_utils->getStateID(), ROBOT_STATUS::START); //status를 start로 변경.
        state_utils->setMovingStateID(NAVI_STATE::START_ROTAION);
      } else{
        state_utils->setRotatePauseFlag(false); // 새로운 rotation이 들어오면 초기화.
      }
      get_start_rotate = true;
      rotate_state = ROTATE_STATE::ROTATE_ODOM_SET;
      rotate_msg = msg;
    }
  }
}

void stateBase::startRotation(bool immediately ,uint8_t type, double targetAngle) {
  setRotationTarget(immediately,type, targetAngle);
  startRotateMonitor();
}

void stateBase::setRotationTarget(bool immediately,uint8_t type, double targetAngle) {
  pose robotPose;
  if( state_utils->getNodeStatusID() == NODE_STATUS::IDLE || state_utils->getFactoryMode()){
    robotPose = state_utils->getCurrentOdom();
  } else{
    robotPose = state_utils->getRobotPose();
  }
  rotation.immediately = immediately;
  rotation.type = type;
  if(type == 0){
    rotation.target = robotPose.theta+targetAngle; 
    RCLCPP_INFO(node_->get_logger(), "[stateBase] SET Rotation [Diff] Current degree: [%f] || target degree: [%f] -> angle diff : [degree : %f / radian : %f ]", robotPose.theta*180.0f /M_PI, targetAngle*180.0f /M_PI, rotation.target*180.0f /M_PI, rotation.target );
  }else if(type==1){
    rotation.target = targetAngle;
    RCLCPP_INFO(node_->get_logger(), "[stateBase] SET Rotation [Pose] Current degree: [%f] || TARGET : [degree : %f / radian : %f ]", robotPose.theta*180.0f /M_PI, targetAngle*180.0f /M_PI, rotation.target);
  }else if(type==2){
    RCLCPP_INFO(node_->get_logger(), "[stateBase] Rotation Auto ");
    startSearchOpenSpace();
  }else if(type == 3 || type == 4){
    if(type == 3){
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Rotation Left 360 Deg");
    }else{
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Rotation Right 360 Deg");
    }  
    rotation.accAngle = 0.0;
    rotation.preTheta = normalize_angle(robotPose.theta);
  }else{
    RCLCPP_INFO(node_->get_logger(), "[stateBase] no rotation ");
  }
}

void stateBase::startRotateMonitor() {
  bSearched = false;
  bReceivedScan = false;
  auto_rotation_scan_msg = sensor_msgs::msg::LaserScan();
  rotation_start_time = std::chrono::steady_clock::now();
  debug_rotation_time = 0;
  if(!rotation.progress){
      rotation_target_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&stateBase::progressRotation, this));
      rotation.progress = true;
  }else{
      RCLCPP_INFO(node_->get_logger(), "[stateBase] already Rotation progressing");
  }
}

void stateBase::progressRotation() {
  pose current;
  if( state_utils->getNodeStatusID() == NODE_STATUS::IDLE || state_utils->getFactoryMode() ){
    current = state_utils->getCurrentOdom();
  } else{
    current = state_utils->getRobotPose();
  }
  double runTime = state_utils->getSteadyClockRunningSeconds(rotation_start_time);

  if(rotation.immediately || (!rotation.immediately && runTime >= 2.0)){
    if(rotation.type == 0 || rotation.type == 1){
      progressRotationTarget(rotation.target,current.theta);
    }else if(rotation.type == 2){
      if (bReceivedScan && !isSearchedCompleteOpenSpace()) {
        calculateScanOpenSpace();
      }
      if (isSearchedCompleteOpenSpace()) {
        double targetHeading = getOpenSpaceHeading();
        progressRotationTarget(targetHeading,current.theta);
      }else{ // [25.09.17 hyjoe]: scan received 실패 or 계산 실패 시 time out 5초 지나면 현재 각도 기준 +10deg 만큼 회전시키고 회전 동작 종료.
        if(runTime >= 5.0) {
          pose robot_pose = state_utils->getRobotPose();
          bSearched = true;
          openspaceRad = robot_pose.theta + 10.0 * M_PI/180.0;
          RCLCPP_INFO(node_->get_logger(), "[stateBase] bReceivedScan : %d, targetRad : %f, robot theta : %f, time out : %f",bReceivedScan,openspaceRad,robot_pose.theta,runTime);
          disableSearchOpenSpace();
        }else{
          RCLCPP_INFO(node_->get_logger(), "[stateBase] waiting calculate target Radian....");
        }
      }
    }else if(rotation.type == 3 || rotation.type == 4){
      int direction = (rotation.type == 3) ? 1 : -1;
      progressRotation360(direction,current.theta);
    }else{
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Rotation type error : %d", rotation.type);
      state_utils->setStatusID(ROBOT_STATUS::FAIL);
      state_utils->enableArrivedGoalSensorsOffTimer();
      stopMonitorRotate();
    }
  }
}

void stateBase::progressRotationTarget(double target, double current) {
  double v = 0, w = 0;
  double nomalize_target = normalize_angle(target);
  double nomalize_current = normalize_angle(current);
  double angle_diff = normalize_angle(nomalize_target-nomalize_current);
  double runTime = state_utils->getSteadyClockRunningSeconds(rotation_start_time);

  if(checkRotationTarget(angle_diff)){
      state_utils->setMovingStateID(NAVI_STATE::ROTATION_COMPLETE);
      state_utils->publishVelocityCommand(v,w);
      stopMonitorRotate();
      if (state_utils->getStateID() != ROBOT_STATE::MANUAL_CONTROL){
        state_utils->enableArrivedGoalSensorsOffTimer();
      }
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Rotation Complete Target : [%.1f]->current : [%.1f], angle_diff : [%.1f], runTime : %.1fsec", nomalize_target*180.0f /M_PI, nomalize_current*180.0f /M_PI,angle_diff*180.0f /M_PI, runTime);
      return;
  }else if(runTime-debug_rotation_time >= 1.0){
    RCLCPP_INFO(node_->get_logger(), "[stateBase] progressRotationTarget Target : [%.1f]->current : [%.1f], angle_diff : [%.1f], runTime : %.1fsec", nomalize_target*180.0f /M_PI, nomalize_current*180.0f /M_PI,angle_diff*180.0f /M_PI, runTime);
    debug_rotation_time = runTime;
  }
  
  int direction = checkRotationDirection(angle_diff);
  w = direction * 0.3; // Adjust speed if needed
  if( state_utils->getRotatePauseFlag() ){
    state_utils->publishVelocityCommand(0.0,0.0);
  } else{
    state_utils->publishVelocityCommand(v,w);
  }
}

double stateBase::normalize_angle(double angle) {
  // Normalize angle to the range [-p, p]
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool stateBase::checkRotationTarget(double diff) {
  bool ret = false;
  if (std::fabs(diff) < 0.0875){
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Target angle reached"); 
      ret = true;
  }
  return ret;
}

void stateBase::stopMonitorRotate() {
  bSearched = false;
  bReceivedScan = false;
  auto_rotation_scan_msg = sensor_msgs::msg::LaserScan();
  rotate_state = ROTATE_STATE::ROTATE_IDLE;
  disableSearchOpenSpace();
  reset_Rotationtimer();
  state_utils->stopDriving();
}

int stateBase::checkRotationDirection(double diff) {
  return (diff > 0) ? 1 : -1;
}

void stateBase::reset_Rotationtimer()
{
    rotation.progress = false;
    if(rotation_target_timer_){
        RCLCPP_INFO(node_->get_logger(), "[stateBase] reset_Rotationtimer");
        rotation_target_timer_.reset();
        state_utils->publishVelocityCommand(0.0,0.0);
        RCLCPP_INFO(node_->get_logger(), "[stateBase] reset_Rotationtimer - end ");
    }else{
        RCLCPP_INFO(node_->get_logger(), "[stateBase] odom_target_timeris allready reset ");
    }
}

void stateBase::progressRotation360(int direction, double current) {
  double v = 0, w = 0;
  double nomalizeTheta = normalize_angle(current);
  double delta_theta = normalize_angle(nomalizeTheta - rotation.preTheta);
  double runTime =  state_utils->getSteadyClockRunningSeconds(rotation_start_time);
  rotation.accAngle += std::fabs(delta_theta);
  rotation.preTheta = nomalizeTheta;
  RCLCPP_INFO(node_->get_logger(), "[stateBase] progressRotation360 : nomalizeTheta[%.1f]->delta_theta : [%.1f], accAngle : [%.1f],  rotation.preTheta : %.1f", nomalizeTheta*180.0f /M_PI, delta_theta*180.0f /M_PI,rotation.accAngle*180.0f /M_PI, rotation.preTheta*180.0f /M_PI);
  if (rotation.accAngle >= 2*M_PI - std::fabs(delta_theta)){
      state_utils->setMovingStateID(NAVI_STATE::ROTATION_COMPLETE);
      state_utils->publishVelocityCommand(v,w);
      if (state_utils->getStateID() != ROBOT_STATE::MANUAL_CONTROL){
        state_utils->enableArrivedGoalSensorsOffTimer();
      }
      stopMonitorRotate();
      RCLCPP_INFO(node_->get_logger(), "[stateBase] Rotation Complete accAngle : %f, runTime : %f ",rotation.accAngle,runTime);
      return;
  }
  if( state_utils->getFactoryMode()){
    w = direction * state_utils->getDirectVelocityW();
  } else{
    w =  direction * 0.3;
  }
  if( state_utils->getRotatePauseFlag() ){
    state_utils->publishVelocityCommand(0.0,0.0);
  } else{
    state_utils->publishVelocityCommand(v,w);
  }
}

void stateBase::startSearchOpenSpace() {
  RCLCPP_INFO(node_->get_logger(), "[stateBase] startSearchOpenSpace in state [id :%s]", enumToString((ROBOT_STATE)id).c_str());
  if(!scan_sub){ //만약에 init에서 sub이 생성되지 않았다면. sub 생성.
    RCLCPP_INFO(node_->get_logger(), "[stateBase] startSearchOpenSpace -> create scan_sub for openspace rotation");
    scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
    std::bind(&stateBase::scan_callback, this, std::placeholders::_1));
  }
  scan_start = true;
}

void stateBase::disableSearchOpenSpace() {
  RCLCPP_INFO(node_->get_logger(), "[stateBase] disableSearchOpenSpace");
  scan_start = false;
}

void stateBase::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // RCLCPP_INFO(node_->get_logger(), "[stateBase] Received scan with %zu ranges", msg->ranges.size());
  if( scan_start ){
    auto_rotation_scan_msg = *msg;
    bReceivedScan = true;
  }  
}

void stateBase::calculateScanOpenSpace() {
  if (!bReceivedScan) {
    RCLCPP_INFO(node_->get_logger(), "[stateBase] scan message not updated yet");
    return;
  }

  std::vector<std::vector<float>> sections(num_sections);
  float angle_increment = auto_rotation_scan_msg.angle_increment;
  float angle_min = auto_rotation_scan_msg.angle_min;
  float max_dist = 3.0;

  // 데이터를 섹션별로 분류
  for (size_t i = 0; i < auto_rotation_scan_msg.ranges.size(); ++i)
  {
      float angle = angle_min + i * angle_increment;
      float distance = auto_rotation_scan_msg.ranges[i];

      if (distance > 0.0)
      {
          int section_index =
              static_cast<int>(std::floor((angle + M_PI) / (M_PI / (num_sections / 2)))) % num_sections;
          sections[section_index].push_back(distance > max_dist ? 0 : distance);
      }
  }

  // 각 섹션의 평균 거리 계산, sum이 0이면 inf로 계산
  std::vector<float> section_averages(num_sections, 0.0);
  for (int i = 0; i < num_sections; ++i)
  {
      if (!sections[i].empty())
      {
          float sum = 0.0;
          int count = 0;
          for (float d : sections[i])
          {
              if (d != 0)
              {
                  sum += d;
                  count++;
              }
          }
          section_averages[i] = sum > 0 ? sum / count : std::numeric_limits<float>::infinity();
      }
      else
      {
          section_averages[i] = std::numeric_limits<float>::infinity();
      }
  }

  int max_index = -1;
  float min_heading_diff = std::numeric_limits<float>::infinity();
  bool has_inf = false;
  float max_distance = 0.0;

  for (int i = 0; i < num_sections; ++i)
  {
      float section_rad = -M_PI + (i * M_PI / (num_sections / 2)) + (M_PI / num_sections);
      //float heading_diff = std::abs(section_rad - robot_pose.theta);

      if (section_averages[i] == std::numeric_limits<float>::infinity())
      {
          has_inf = true;
          if (section_rad < min_heading_diff)
          {
              min_heading_diff = section_rad;
              max_index = i;
              max_distance = std::numeric_limits<float>::infinity();
          }
      }

      if(!has_inf && section_averages[i] > max_distance)
      {
          max_distance = section_averages[i];
          max_index = i;
      }
  }

  if(max_index != -1){
    bSearched = true;
    pose robot_pose = state_utils->getRobotPose();
    double scan_base = -M_PI + (max_index * M_PI / (num_sections / 2)) + (M_PI / num_sections);
    openspaceRad = robot_pose.theta + scan_base;
    RCLCPP_INFO(node_->get_logger(), "[stateBase] openspaceRad : %f, robot theta : %f, scan theta : %f",openspaceRad,robot_pose.theta,scan_base);
    disableSearchOpenSpace();
  } else {
    RCLCPP_INFO(node_->get_logger(), "[stateBase] No valid open space found in this scan.");
  }
}

bool stateBase::isSearchedCompleteOpenSpace() {
  return bSearched;
}

double stateBase::getOpenSpaceHeading() {
  return openspaceRad;
}


}  // namespace airbot_state