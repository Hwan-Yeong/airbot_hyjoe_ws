#include "state_manager/states/state_base.hpp"

namespace airbot_state {

stateBase::stateBase(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : id(actionID), node_(node), state_utils(utils), first_running(true), bStatusChange(false)
{}

void stateBase::pre_run(const std::shared_ptr<StateUtils> &state_utils) {
  first_running = true; 
  // cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void stateBase::run(const std::shared_ptr<StateUtils> &state_utils) {
  first_running = false;
  
}

void stateBase::post_run(const std::shared_ptr<StateUtils> &state_utils){
  // auto cmd_msg = geometry_msgs::msg::Twist(); //state들에서 동작중에 취소하면 동작 유지되어 정지 cmd전달.
  // cmd_msg.linear.x = 0.0;
  // cmd_msg.angular.z = 0.0;
  // cmd_vel_pub_->publish(cmd_msg);
  // cmd_vel_pub_.reset();
}

bool stateBase::isFirstRunning(){ 
  bool ret = false;
  if(first_running){
    first_running = false;
    ret = true;
  }
  return ret;
}

void stateBase::reserveTarget(pose pose_data) {
  target_pose.x = pose_data.x;
  target_pose.y =pose_data.y;
  target_pose.theta = pose_data.theta;
  if (pre_target_pose.x != target_pose.x &&
      pre_target_pose.y != target_pose.y) {
    set_movetarget = true;
    pre_target_pose = target_pose;
    state_utils->setMovingStateID(NAVI_STATE::IDLE);
  }
}

}  // namespace airbot_state