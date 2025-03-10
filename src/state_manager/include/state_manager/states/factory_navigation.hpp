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
  void moveToTarget(double x, double y, double theta);
  void publishTargetPosition(double x, double y, double theta);
  void waitNodeLaunching();
  bool startNavigation();
  void pauseNavigation();
  void resumeNavigation();
  
  void map_saver();
  void exitMappingNode();
  void exitNavigationNode();
  void target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg);
  ///Rotation
  int checkRotationDirection(double diff);
  bool checkRotationTarget(double diff);
  void progressRotationTarget(double target, double current);
  void progressRotation360(int direction, double current);
  void reset_Rotationtimer();
  void startRotateMonitor();
  void progressRotation();
  void stopMonitorRotate();
  bool startRotation(int type, double targetAngle);
  double normalize_angle(double angle);
  void publishVelocityCommand(double v, double w);

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;
  rclcpp::Subscription<robot_custom_msgs::msg::Position>::SharedPtr req_target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotation_move_pub_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> goal_msg;
  rclcpp::TimerBase::SharedPtr rotation_target_timer_;
  rclcpp::Time nav_node_start_time;

  geometry_msgs::msg::PoseWithCovarianceStamped amcl;
  
  rotationData rotation;
  MOVING_DATA movingData;
  NAVI_STATE movingState;
  READY_NAVIGATION readyNavi;
  READY_MOVING  readyMoving;
  
  void setReadyNavigation(READY_NAVIGATION set);
  void setReadyMoving(READY_MOVING set);
  
  bool bSavedMap;
  double node_start_time;
  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();
  bool naviNodeLauncher();
  int naviNodeChecker();
  bool resetOdomChecker();
  int8_t localizationChecker();

};

} // namespace airbot_state

#endif // FACTORY_NAVIGATION_HPP_
