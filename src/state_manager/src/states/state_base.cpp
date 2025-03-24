#include "state_manager/states/state_base.hpp"

namespace airbot_state {

stateBase::stateBase(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils)
    : node_(node), state_utils(utils), id(actionID), first_running(true)
{}

void stateBase::pre_run(const std::shared_ptr<StateUtils> &) {
  first_running = true; 
}

void stateBase::run(const std::shared_ptr<StateUtils> &) {
  first_running = false;
  
}

void stateBase::post_run(const std::shared_ptr<StateUtils> &){
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

}  // namespace airbot_state