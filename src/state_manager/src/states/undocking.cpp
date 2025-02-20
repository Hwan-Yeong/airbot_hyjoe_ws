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
  RCLCPP_INFO(node_->get_logger(), "[UnDocking] Preparing UnDocking STATE");
  state_utils->publishMultiTofOff();
}

void UnDocking::run(const std::shared_ptr<StateUtils> &state_utils) {
 //RCLCPP_INFO(node_->get_logger(), "[UnDocking] Running UnDocking with shared data: ");

  if (bUndockStart) {
    bUndockStart = false;
    // finish generate change state msg
    auto undock_state_msg = std_msgs::msg::UInt8();
    if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_AUTO_MAPPING){
      undock_state_msg.data = int(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING);
    }else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_MANUAL_MAPPING){
      undock_state_msg.data = int(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING);
    }else if(state_utils->getRobotCMDID().soc_cmd == REQUEST_SOC_CMD::START_NAVIGATION){
      undock_state_msg.data = int(REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION);
    }
    req_robot_cmd_pub_->publish(undock_state_msg);
  } else {
    unDockingMove(0.3);
  }

  
}

void UnDocking::post_run(const std::shared_ptr<StateUtils> &state_utils) {
  RCLCPP_INFO(node_->get_logger(), "[UnDocking] Exiting UnDocking STATE");
  stateBase::post_run(state_utils);
  req_robot_cmd_pub_.reset();
  cmd_vel_pub_.reset();
  state_utils->startMonitorOdomReset();
}

void UnDocking::unDockingMove(const float dist) {
  RCLCPP_ERROR(node_->get_logger(), "undocking move - start");
  auto moveFwd = geometry_msgs::msg::Twist();
  moveFwd.linear.x = 0.05; // Move forward at 0.05 m/s
  moveFwd.angular.z = 0.0; // No rotation
  // Distance to move
  double distance = dist;
  // Velocity: 0.2 m/s
  double velocity = 0.05;
  // Time needed to move 30 cm
  double time_to_move = distance / velocity; // Time in seconds
  rclcpp::Rate rate(10); // Publish at 10 Hz (10 times per second)
  int num_iterations =
      time_to_move * 10; // Number of iterations required to move 30 cm
  for (int i = 0; i < num_iterations; ++i) {
    cmd_vel_pub_->publish(moveFwd);
    rate.sleep(); // Sleep to maintain the loop rate
  }
  // Stop the robot after moving the required distance
  moveFwd.linear.x = 0.0;
  cmd_vel_pub_->publish(moveFwd);
  sleep(3); // hjkim 250110 - wait for stop undocking move
  bUndockStart = true;
  dock_pose_estimate = false;
  RCLCPP_ERROR(node_->get_logger(), "undocking move - after 6 + 3 sec");
}

} // namespace airbot_state
