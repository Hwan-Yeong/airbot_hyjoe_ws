#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

#include "state_manager/states/state_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_custom_msgs/msg/position.hpp"
#include "state_manager/utils/navi_defines.hpp"
#include <cmath>


namespace airbot_state {

class Navigation : public stateBase {
  
public:
  Navigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // navigation behavior
  void rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg);
  void moveToTarget(double x, double y, double theta);
  void publishTargetPosition(double x, double y, double theta);
  void waitNodeLaunching();
  bool startNavigation();
  void pauseNavigation();
  void resumeNavigation();

  void exitNavigationNode();
  ///Rotation
  int checkRotationDirection(double diff);
  bool checkRotationTarget(double diff);
  void progressRotationTarget(double target, double current);
  void progressRotation360(int direction, double current);
  void reset_Rotationtimer();
  void startRotateMonitor();
  void progressRotation();
  void stopMonitorRotate();
  bool startRotation(uint8_t type, double targetAngle);
  double normalize_angle(double angle);
  void publishVelocityCommand(double v, double w);
  void clearMoveTarget();

  void startSearchOpenSpace();
  void disableSearchOpenSpace();
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool isSearchedCompleteOpenSpace();
  double getOpenSpaceHeading();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::MoveNRotation>::SharedPtr rotation_sub_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotation_vel_pub_;
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

  bool new_targetcall_flag = false;
  
  double node_start_time;
  bool bSearched = false;
  double openspaceRad;

  void setReadyNavigation(READY_NAVIGATION set);
  void setReadyMoving(READY_MOVING set);

  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();
  bool naviNodeLauncher();
  int naviNodeChecker();
  bool resetOdomChecker();
  int8_t localizationChecker();

};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
