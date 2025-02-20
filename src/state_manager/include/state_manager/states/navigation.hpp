#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

#include "state_manager/states/state_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "robot_custom_msgs/msg/position.hpp"

namespace airbot_state {

class Navigation : public stateBase {
public:
  Navigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

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

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;
  NAVI_STATE movingState;

  rclcpp::Subscription<robot_custom_msgs::msg::Position>::SharedPtr req_target_sub_;
  
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> goal_msg;

  rclcpp::Time nav_node_start_time;

  geometry_msgs::msg::PoseWithCovarianceStamped amcl;

  bool ready_navigation;

  bool bsetMovetarget;
  bool navi_started;
  bool waitLaunchNode;
  
  void map_saver();
  void exitMappingNode();
  void exitNavigationNode();
  bool bSavedMap;

  ////process check
  bool startProcess(const std::string &command, const std::string &pidFilePath);
  bool stopProcess(const std::string &pidFilePath);

};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
