#ifndef RETURNCHARGER_HPP_
#define RETURNCHARGER_HPP_

#include "state_manager/states/state_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace airbot_state {

class ReturnCharger : public stateBase {
public:
  ReturnCharger(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // return charger function
  void publishTargetPosition(double x, double y, double theta);
  void moveToDock();
  void startMonitorReturnCharger();
  void stopMonitorReturnCharger();
  void reset_timerNaviStatus();
  void monitor_returnCharger();

  void exitMappingNode();
  void map_saver();

  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();
  int8_t localizationChecker();

  void pauseReturnCharger();

  void setReadyNavigation(READY_NAVIGATION set);
  void setReadyMoving(READY_MOVING set);

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle_;
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> goal_msg;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;
  rclcpp::TimerBase::SharedPtr nav_status_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dock_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  
  rclcpp::Time nav_node_start_time;
  double node_start_time;
  bool dock_pose_estimate = false;
  std_msgs::msg::UInt8 dock_cmd_;
  uint8_t retry_move_target;
  bool new_targetcall_flag = false;
  bool bSendGoal;
  double send_goal_start_time;

  pose return_pose;  

  MOVING_DATA movingData;
  NAVI_STATE movingState;
  READY_NAVIGATION readyNavi;
  READY_MOVING  readyMoving;

};

} // namespace airbot_state

#endif // RETURNCHARGER_HPP_
