#include "state_manager/states/undocking.hpp"
// #include "state_manager/states/state_base.hpp"

namespace airbot_state {

UnDocking::UnDocking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
: stateBase(actionID, node, utils) {
}

void UnDocking::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  stateBase::pre_run(state_utils);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  req_robot_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state_cmd",10);
  bUndockStart = false;
  state_utils->setStartOnStation(true);
  RCLCPP_INFO(node_->get_logger(), "[UnDocking] pre_run() -> Preparing UnDocking state");
  state_utils->enableOdomcallback();
}

void UnDocking::run(const std::shared_ptr<StateUtils> &state_utils) {
 
  if( isFirstRunning() ){
    RCLCPP_INFO(node_->get_logger(), "[UnDocking] run() -> Running UnDocking state");
  }

  auto undock_state_msg = std_msgs::msg::UInt8();
  ROBOT_STATUS ready_check;
  switch (state_utils->getStatusID()) {
  case ROBOT_STATUS::READY:
    if( state_utils->getPrepareOdomFlag() ){
      enableLinearTargetMoving();
      ready_check = ROBOT_STATUS::START;
      state_utils->setStatusID(ready_check);
    }
    break;
  case ROBOT_STATUS::START :
    if( bUndockStart )
    {
      ready_check = ROBOT_STATUS::COMPLETE;
      state_utils->setStatusID(ready_check);
    }  
    break;

  case ROBOT_STATUS::PAUSE :
    RCLCPP_INFO(node_->get_logger(), "[Undocking] Running PAUSE Status");
    break;

  case ROBOT_STATUS::COMPLETE :
    if(linear_target_timer_)
    {
      RCLCPP_INFO(node_->get_logger(), "[Undocking] close linear target timer");
      linear_target_timer_.reset();
      state_utils->disableOdomcallback();
    }
    if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_AUTO_MAPPING){
      undock_state_msg.data = int(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING);
    }else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_MANUAL_MAPPING){
      undock_state_msg.data = int(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING);
    }else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_NAVIGATION){
      undock_state_msg.data = int(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION);
    }
    req_robot_cmd_pub_->publish(undock_state_msg);
    break;

  case ROBOT_STATUS::FAIL :
    RCLCPP_INFO(node_->get_logger(), "[Undocking] Running FAIL Status");
    break;

  default:
    RCLCPP_INFO(node_->get_logger(), "[Undocking] Running UnKown Status");
    break;
  }
}

void UnDocking::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[UnDocking] post_run() -> Exiting UnDocking state");
  stateBase::post_run(state_utils);
  req_robot_cmd_pub_.reset();
  cmd_vel_pub_.reset();
  if(linear_target_timer_)
  {
    linear_target_timer_.reset();
  }
  state_utils->startMonitorOdomReset();
}

void UnDocking::enableLinearTargetMoving()
{
    end_linear_target = false;
    base_odom = state_utils->getCurrentOdom();
    if(!linear_target_timer_){
        linear_target_timer_ = node_->create_wall_timer( std::chrono::milliseconds(20), std::bind(&UnDocking::processLinearMoving, this));
        RCLCPP_INFO(node_->get_logger(), "[UnDocking] enableLinearTargetMoving");
    }else{
      RCLCPP_INFO(node_->get_logger(), "[UnDocking] LinearTargetMoving is already enabled!!");
    }
}
void UnDocking::processLinearMoving()
{
    pose current = state_utils->getCurrentOdom();
    double distance = state_utils->getDistance(base_odom,current);
    if(distance >= 0.3)
    {
        end_linear_target = true;
        publishVelocityCommand(0.0,0.0);
        RCLCPP_INFO(node_->get_logger(), "[UnDocking] moving 0.3m over distance : %f , base X : %f, Y : %f, current X : %f, Y : %f ",
        distance,base_odom.x,base_odom.y,current.x,current.y);
        bUndockStart = true;
    }else{
        //RCLCPP_WARN(node_->get_logger(), "processLinearMoving...distance : %f",distance);
        publishVelocityCommand(0.07,0.0);
    }
}

void UnDocking::publishVelocityCommand(double v, double w)
{
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = v;
    cmd_msg.angular.z = w;
    cmd_vel_pub_->publish(cmd_msg);
    // RCLCPP_INFO(node_->get_logger(), "publishVelocityCommand V : %f, W : %f ", v,w);
}


} // namespace airbot_state
