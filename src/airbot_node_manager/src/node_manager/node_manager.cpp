#include "node_manager/node_manager.hpp"
//everyAIRbot!4938
using namespace std::chrono_literals;

#define DEBUG_LOG 0

NodeManager::NodeManager() : Node("airbot_node_manager") {

  // pub_timer_ = this->create_wall_timer( 100ms, std::bind(&NodeManager::publishNodeStatus, this));

  manage_node_action_server_ = rclcpp_action::create_server<robot_custom_msgs::action::ManageNode>(this,"manage_node",
    std::bind(&NodeManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NodeManager::handle_cancel, this, std::placeholders::_1),
    std::bind(&NodeManager::handle_accepted, this, std::placeholders::_1));

  task = MANAGE_STATE::CHECK_AND_KILL_NODE;
  current_goal_handle_ = nullptr;

}

NodeManager::~NodeManager() {}

// void NodeManager::publishNodeStatus() {

// }

void NodeManager::setNodeStatusID(const NODE_STATUS &id){
  node_status_id = id;
}

NODE_STATUS NodeManager::getNodeStatusID(){
  return node_status_id;
}


bool NodeManager::isValidateNode(const NODE_STATUS &node, double start_time) {
  std::vector<std::string> require_nodes;
  if( node ==  NODE_STATUS::AUTO_MAPPING){
    require_nodes = {
      "/behavior_server",
      "/bt_navigator",
      "/bt_navigator_navigate_to_pose_rclcpp_node",
      "/controller_server",
      "/explore_node",
      "/planner_server",
      "/robot_pose_publisher_node",
      "/slam_toolbox",
      "/smoother_server",
      "/velocity_smoother",
      "/warmup_server_node",
      "/waypoint_follower"
    };
  } else if( node ==  NODE_STATUS::MANUAL_MAPPING){
    require_nodes = {
      "/robot_pose_publisher_node",
      "/slam_toolbox"
    };
  } else if( node ==  NODE_STATUS::NAVI){
    require_nodes = {
      "/amcl",
      "/behavior_server",
      "/bt_navigator",
      "/bt_navigator_navigate_to_pose_rclcpp_node",
      "/global_costmap/global_costmap",
      "/local_costmap/local_costmap",
      "/controller_server",
      "/planner_server",
      "/map_server",
      "/lifecycle_manager",
      "/nav2_container",
      "/smoother_server",
      "/velocity_smoother",
      "/waypoint_follower"
    };
  }

  static std::vector<std::string> temp_require_nodes;

  if( temp_require_nodes.empty() )
  {
    temp_require_nodes = require_nodes;
  }

  std::vector<std::string> running_nodes = this->get_node_names();
  for (std::size_t i = 0; i < (std::size_t)temp_require_nodes.size(); i++) {
    bool found = false;
    for (const auto& running_node : running_nodes) {
      if (running_node.find(temp_require_nodes[i]) != std::string::npos) {
        found = true;
        double wait_navi_launch = this->now().seconds()-start_time;
        RCLCPP_INFO(this->get_logger(), "[isValidProcess] Required [%s] node launched [%s]: time %f", enumToString(node).c_str(),running_node.c_str(), wait_navi_launch);
        break;
      }
    }
    if (!found) {
      // RCLCPP_ERROR(this->get_logger(), "[isValidProcess] Required [%s] node not found: %s", enumToString(node).c_str(),temp_require_nodes[i].c_str());
      return false;
    } else{
      RCLCPP_ERROR(this->get_logger(), "[isValidProcess] %s | remain check list size : %d ",temp_require_nodes[i].c_str(), static_cast<int>(temp_require_nodes.size()));
      temp_require_nodes.erase(temp_require_nodes.begin() + i);
    }
  }
      
  if( temp_require_nodes.empty() )
  {
    return true;
  }
  return false;
}

bool NodeManager::nodePIDChecker(const NODE_STATUS &node ){
  std::string pidFilePath;
  if( node ==  NODE_STATUS::AUTO_MAPPING){
    pidFilePath = "/home/airbot/mapping_pid.txt";
  } else if( node ==  NODE_STATUS::MANUAL_MAPPING){
    pidFilePath = "/home/airbot/mapping_pid.txt";
  } else if( node ==  NODE_STATUS::NAVI || node == NODE_STATUS::FT_NAVI){
    pidFilePath = "/home/airbot/navigation_pid.txt";
  }
  std::ifstream pidFile(pidFilePath);
  std::string pid;
  if(pidFile.is_open())
  {
    std::getline(pidFile, pid);
    pidFile.close();
    if (!pid.empty())
    {
      pid_t processGroupID = std::stoi(pid);
      if (kill(-processGroupID, 0) == 0) {
        return true;
      } else
      {
        return false;
      }
    }
  }
  return false;
}

bool NodeManager::startProcess(const std::string& command, const std::string& pidFilePath) {
  int result = std::system(("setsid bash -c '" + command + "' > /dev/null 2>&1 & echo $! > " + pidFilePath).c_str());
  if (result == 0) {
      std::ifstream pidFile(pidFilePath);
      if (pidFile.is_open()) {
          std::string pid;
          std::getline(pidFile, pid);
          pidFile.close();
          if (!pid.empty()) {
              RCLCPP_INFO(this->get_logger(), "Process started successfully with PID: %s", pid.c_str());
              return true;
          }
      }
  }
  RCLCPP_ERROR(this->get_logger(), "Failed to start the process.");
  return false;
}

bool NodeManager::stopProcess(const std::string &pidFilePath) {
  std::ifstream pidFile(pidFilePath);
  std::string pid;
  int rtn_sigint = 0;
  int rtn_sigkill = 0;

  if (pidFile.is_open()) {
    std::getline(pidFile, pid);
    pidFile.close();
    if (!pid.empty()) {
      pid_t processGroupID = std::stoi(pid);
      rtn_sigint = kill(-processGroupID, SIGTERM);
      if (rtn_sigint == 0) {
        int wait_count = 0; // Wait for process to handle SIGINT
        while (kill(-processGroupID, 0) == 0 && wait_count < 5) {
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
          wait_count++;
        }
        
        if (kill(-processGroupID, 0) == -1) { // Process no longer exists
          RCLCPP_INFO(this->get_logger(), "[stopProcess]Process terminated successfully.");
          return true;
        } else {
          rtn_sigkill = kill(-processGroupID, SIGKILL);
          if (rtn_sigkill == 0) {
            RCLCPP_WARN(this->get_logger(), "[stopProcess]Process did not terminate, retry sending SIGKILL.");
            return true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "[stopProcess]Failed to send SIGKILL[%d]", rtn_sigkill);
            return false;
          }
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "[stopProcess]Failed to send SIGINT[%d]", rtn_sigint);
      }
    }
  }
  return false;
}

bool NodeManager::startNavigation() {
  bool ret = false;

  if (startProcess("ros2 launch airbot_navigation navigation.launch.py","/home/airbot/navigation_pid.txt")) {
    node_start_time = this->now().seconds();
    RCLCPP_INFO(this->get_logger(), "[startNavigation]Navigation Node Start Success");
    ret = true;
  }else {
    RCLCPP_INFO(this->get_logger(), "[startNavigation]Navigation Node Start FAIL");
  }
  return ret;
}

bool NodeManager::startFactoryNavigation() {
  bool ret = false;

  if (startProcess("ros2 launch airbot_navigation factory_navigation.launch.py", "/home/airbot/navigation_pid.txt")){
    node_start_time = this->now().seconds();
    RCLCPP_INFO(this->get_logger(), "Factory Navigation Start Success");
    ret = true;
  }else{
      //reqStatus = REQUEST_STATUS::FAIL;
      RCLCPP_INFO(this->get_logger(), "Factory Navigation Node Start FAIL");
  }
  return ret;
}

bool NodeManager::startManualMapping() {
  bool ret = false;
  RCLCPP_INFO(this->get_logger(), "Manual Mapping Start");
  if (startProcess("ros2 launch airbot_slam slam.launch.py", "/home/airbot/mapping_pid.txt")) {
    node_start_time = this->now().seconds();
    RCLCPP_INFO(this->get_logger(), "[startManualMapping]ManualMapping Run");
    ret = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "[startManualMapping]ManualMapping FAIL");
  }
  return ret;
}

bool NodeManager::startAutoMapping() {
  // ******Auto mapping Launch files******
  bool ret = false;
  if (startProcess("ros2 launch explore explore_all.launch.py", "/home/airbot/mapping_pid.txt")) {
    node_start_time = this->now().seconds();
    RCLCPP_INFO(this->get_logger(), "[startAutoMapping]AutoMapping Run");
    ret = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "[startAutoMapping]runAutoMapping FAIL");
  }
  return ret;
}

int NodeManager::manageNodeStatus( const NODE_STATUS &require_node, std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle )
{
  int ret = 0;
  auto feedback = std::make_shared<robot_custom_msgs::action::ManageNode::Feedback>();
  ///
  switch(task)
  {
  case MANAGE_STATE::CHECK_AND_KILL_NODE: //// CHECK AND KILLING NODE PART ==> 노드 체크 후 종료.
  feedback->progress = 0;
  goal_handle->publish_feedback(feedback);
  if(require_node == NODE_STATUS::AUTO_MAPPING) //auto mapping 요청
  {
    if( getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING )
    {
      if (stopProcess("/home/airbot/mapping_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]exit Mapping Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill Manual Mapping Node");
      }
    } else if( getNodeStatusID() == NODE_STATUS::NAVI )
    {
      if (stopProcess("/home/airbot/navigation_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]exit NAVIGATION Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill NAVIGATION Node");
      }
    }else if( getNodeStatusID() == NODE_STATUS::IDLE )
    {
      task = MANAGE_STATE::LAUNCH_NODE;
    }
  }
  else if(require_node == NODE_STATUS::MANUAL_MAPPING)//manual mapping 요청
  {
    if( getNodeStatusID() == NODE_STATUS::AUTO_MAPPING )
    {
      if (stopProcess("/home/airbot/mapping_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]exit AUTO_MAPPING Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill AUTO_MAPPING Node");
      }
    } else if( getNodeStatusID() == NODE_STATUS::NAVI )
    {
      if (stopProcess("/home/airbot/navigation_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "exit NAVIGATION Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill NAVIGATION Node");
      }
    }else if( getNodeStatusID() == NODE_STATUS::IDLE )
    {
      task = MANAGE_STATE::LAUNCH_NODE;
    } else {
      ret = 1;
    }
  }
  else if(require_node == NODE_STATUS::NAVI)//navi 요청
  {
    if( getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING)
    {
      if (stopProcess("/home/airbot/mapping_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]exit Mapping Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill Mapping Node");
      }
    } else if( getNodeStatusID() == NODE_STATUS::IDLE )
    {
      task = MANAGE_STATE::LAUNCH_NODE;
    } else {
      RCLCPP_INFO(this->get_logger(), "[manageNodeStatus] Already Navi Node Running");
      ret = 1;
    }
  }
  else if(require_node == NODE_STATUS::FT_NAVI)//navi 요청
  {
    if( getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING)
    {
      if (stopProcess("/home/airbot/mapping_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]exit Mapping Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill Mapping Node");
      }
    }
    else if( getNodeStatusID() == NODE_STATUS::NAVI )
    {
      if (stopProcess("/home/airbot/navigation_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "exit NAVIGATION Node");
        task = MANAGE_STATE::LAUNCH_NODE; // launch 단계.
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill NAVIGATION Node");
      }
    } 
    else if( getNodeStatusID() == NODE_STATUS::IDLE )
    {
      task = MANAGE_STATE::LAUNCH_NODE;
    }
  }
  else // idle 요청
  {
    if( getNodeStatusID() == NODE_STATUS::AUTO_MAPPING || getNodeStatusID() == NODE_STATUS::MANUAL_MAPPING)
    {
      if (stopProcess("/home/airbot/mapping_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "exit Mapping Node");
        ret = 1;
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill Mapping Node");
        ret = -1;
      }
    } else if( getNodeStatusID() == NODE_STATUS::NAVI ) {
      if (stopProcess("/home/airbot/navigation_pid.txt")) {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]exit Navigation Node");
        ret = 1;
      } else {
        RCLCPP_INFO(this->get_logger(), "[manageNodeStatus]Fail - kill Navigation Node");
      }
    } else {
      if( nodePIDChecker(NODE_STATUS::AUTO_MAPPING) || nodePIDChecker(NODE_STATUS::MANUAL_MAPPING) ){
        stopProcess("/home/airbot/mapping_pid.txt");
        ret = 1;
      } else if ( nodePIDChecker(NODE_STATUS::NAVI) || nodePIDChecker(NODE_STATUS::FT_NAVI)){
        stopProcess("/home/airbot/navigation_pid.txt");
        ret = 1;
      } else {
        ret = 1;
      }
    }
  }
  break;

  case  MANAGE_STATE::LAUNCH_NODE: // LAUCN PART ==> launch node 원하는 mode의 노드를 실행.
  feedback->progress = 1;
  goal_handle->publish_feedback(feedback);
  if(require_node == NODE_STATUS::AUTO_MAPPING)
  {
    if(startAutoMapping()){
      task = MANAGE_STATE::CHECK_NODE_LIST;
    }else{
      RCLCPP_INFO(this->get_logger(), "[manageNodeStatus] auto mapping node launch Fail");
      ret = -1;
    }
  }
  else if(require_node == NODE_STATUS::MANUAL_MAPPING)
  {
    if(startManualMapping()){
      task = MANAGE_STATE::CHECK_NODE_LIST;
    }else{
      RCLCPP_INFO(this->get_logger(), "[manageNodeStatus] manual mapping node launch Fail");
      ret = -1;
    }
  }
  else if(require_node == NODE_STATUS::NAVI)
  {
    if(startNavigation()){
      task = MANAGE_STATE::CHECK_NODE_LIST;
    }else{
      RCLCPP_INFO(this->get_logger(), "[manageNodeStatus] navi node launch Fail");
      ret = -1;
    }
  }
  else if(require_node == NODE_STATUS::FT_NAVI)
  {
    if(startFactoryNavigation()){
      task = MANAGE_STATE::CHECK_NODE_LIST;
    }else{
      RCLCPP_INFO(this->get_logger(), "[manageNodeStatus] ft_navi node launch Fail");
      ret = -1;
    }
  }
  break;
  case MANAGE_STATE::CHECK_NODE_LIST: // CHECKING PART ===> launch한 노드들이 잘 켜졌는지 확인.
  feedback->progress = 2;
  goal_handle->publish_feedback(feedback);
  double wait_navi_launch = this->now().seconds()-node_start_time;
  if(require_node == NODE_STATUS::AUTO_MAPPING)
  {
    if(isValidateNode(NODE_STATUS::AUTO_MAPPING, node_start_time)){
      RCLCPP_INFO(this->get_logger(), "AUTO_MAPPING node launch Complete time : %f", wait_navi_launch);
      ret = 1;
    }else if(wait_navi_launch >= 30){
      RCLCPP_INFO(this->get_logger(), "AUTO_MAPPING node launch Fail time : %f", wait_navi_launch);
      ret = -1;
    }
  }
  else if(require_node == NODE_STATUS::MANUAL_MAPPING)
  {
    if(isValidateNode(NODE_STATUS::MANUAL_MAPPING, node_start_time)){
      RCLCPP_INFO(this->get_logger(), "MANUAL_MAPPING node launch Complete time : %f", wait_navi_launch);
      ret = 1;
    }else if(wait_navi_launch >= 30){
      RCLCPP_INFO(this->get_logger(), "MANUAL_MAPPING node launch Fail time : %f", wait_navi_launch);
      ret = -1;
    }
  }
  else if(require_node == NODE_STATUS::NAVI || require_node == NODE_STATUS::FT_NAVI)
  {
    //RCLCPP_INFO(this->get_logger(), "[Navigation] Navigation NODE ALL RUNNING -> launch time %f sec", wait_navi_launch);
    if(isValidateNode(NODE_STATUS::NAVI, node_start_time)){
      RCLCPP_INFO(this->get_logger(), "Navi node launch Complete time : %f", wait_navi_launch);
      ret = 1;
    }else if(wait_navi_launch >= 30){
      RCLCPP_INFO(this->get_logger(), "Navi node launch Fail time : %f", wait_navi_launch);
      ret = -1;
    }
  }
  break;
  }
  if( ret != 0 )
  {
    task = MANAGE_STATE::CHECK_AND_KILL_NODE;
    return ret;
  }
  return ret;
}

rclcpp_action::GoalResponse NodeManager::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const robot_custom_msgs::action::ManageNode::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with require_node: %s", enumToString((NODE_STATUS)goal->require_node).c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NodeManager::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  if(current_goal_handle_){
    if(current_goal_handle_ != goal_handle)
    {
       RCLCPP_WARN(this->get_logger(), "Received request to cancel goal wrong goal");
       return rclcpp_action::CancelResponse::REJECT;
    }
  }
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NodeManager::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle)
{
  using namespace std::placeholders;
  // current_goal_handle_ = goal_handle;
  task = MANAGE_STATE::CHECK_AND_KILL_NODE;
  std::thread{std::bind(&NodeManager::execute, this, _1), goal_handle}.detach();
}

void NodeManager::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle)
{
  current_goal_handle_ = goal_handle;
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<robot_custom_msgs::action::ManageNode::Feedback>();
  auto result = std::make_shared<robot_custom_msgs::action::ManageNode::Result>();

  NODE_STATUS require_node;
  switch (goal->require_node)
  {
  case 0: 
    require_node = NODE_STATUS::IDLE; 
  break;
  case 1:
    require_node = NODE_STATUS::AUTO_MAPPING;
  break;
  case 2: 
    require_node = NODE_STATUS::MANUAL_MAPPING;
  break;
  case 3:
    require_node = NODE_STATUS::NAVI;
  break;
  case 4:
    require_node = NODE_STATUS::FT_NAVI;
  break;
  default :
    require_node = NODE_STATUS::IDLE;
  break;
  }


  // manageNodeStatus 호출.
  int manage_result = 0;

  while(rclcpp::ok()){
    if (current_goal_handle_ != goal_handle) { //새로운 goal이 들어오면 이전 goal 취소.
      RCLCPP_INFO(this->get_logger(), "A new goal has been received. Stopping the current goal silently.");
      break;  
    }

    if(goal_handle->is_canceling()){
        result->result = 0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        current_goal_handle_.reset();
        return;
    }
    manage_result = manageNodeStatus(require_node,goal_handle);
    if(manage_result != 0){
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if(manage_result == 1){
      result->result = 1;
      goal_handle->succeed(result);
      setNodeStatusID(require_node);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }else if(manage_result == -1){
      result->result = 0;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Goal Aborted");
  }
  
}