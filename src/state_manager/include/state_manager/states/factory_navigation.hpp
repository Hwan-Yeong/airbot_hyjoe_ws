#ifndef FACTORY_NAVIGATION_HPP_
#define FACTORY_NAVIGATION_HPP_

#include "state_manager/states/state_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_custom_msgs/msg/position.hpp"


namespace airbot_state {

class FactoryNavigation : public stateBase {
  
public:
  FactoryNavigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // navigation behavior
  void rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg);
  void moveToTarget(double x, double y, double theta);
  void publishTargetPosition(double x, double y, double theta);

  void pauseNavigation();
  void resumeNavigation();

  ///Rotation
  int checkRotationDirection(double diff);
  bool checkRotationTarget(double diff);
  void progressRotationTarget(double target, double current);
  void progressRotation360(int direction, double current);
  void reset_Rotationtimer();
  void startRotateMonitor();
  void progressRotation();
  void stopMonitorRotate();
  void startRotation(bool emmediately,uint8_t type, double targetAngle);
  double normalize_angle(double angle);
  void clearMoveTarget();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::MoveNRotation>::SharedPtr rotation_sub_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> goal_msg;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
  rclcpp::TimerBase::SharedPtr rotation_target_timer_;
  rclcpp::Time nav_node_start_time;
  geometry_msgs::msg::PoseWithCovarianceStamped amcl;
  
  rotationData rotation;
  MOVING_DATA movingData;
  NAVI_STATE movingState;
  READY_NAVIGATION readyNavi;
  READY_MOVING  readyMoving;

  bool new_targetcall_flag = false;
  
  double node_start_time;
  bool bSendGoal;
  double send_goal_start_time;
  double rotation_start_time;
  uint8_t retry_move_target;

  void setReadyNavigation(READY_NAVIGATION set);
  void setReadyMoving(READY_MOVING set);

  void setRotationTarget(bool emmediately,uint8_t type, double targetAngle);

  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();

  bool resetOdomChecker();
  int8_t localizationChecker();

};

} // namespace airbot_state

#endif // FACTORY_NAVIGATION_HPP_
