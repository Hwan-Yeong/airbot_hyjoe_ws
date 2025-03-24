#ifndef NODE_MANAGER_HPP
#define NODE_MANAGER_HPP

#include <memory>
#include <functional>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_custom_msgs/action/manage_node.hpp"
#include "std_msgs/msg/bool.hpp"

enum class MANAGE_STATE {
  CHECK_AND_KILL_NODE,
  LAUNCH_NODE,
  CHECK_NODE_LIST
};

enum class NODE_STATUS {
  IDLE,
  AUTO_MAPPING,
  MANUAL_MAPPING,
  NAVI,
  FT_NAVI,
};

inline std::string enumToString(NODE_STATUS in) {
  std::string out;
  switch (in) {
  case NODE_STATUS::IDLE:
    out = std::string("IDLE");
    break;
  case NODE_STATUS::AUTO_MAPPING:
    out = std::string("AUTO_MAPPING");
    break;
  case NODE_STATUS::MANUAL_MAPPING:
    out = std::string("MANUAL_MAPPING");
    break;
  case NODE_STATUS::NAVI:
    out = std::string("NAVI");
    break;
  case NODE_STATUS::FT_NAVI:
    out = std::string("FT_NAVI");
    break;
  }
  return out;
};

class NodeManager : public rclcpp::Node {
public:
  NodeManager();
  ~NodeManager();

private:
  // for node manager
  void setNodeStatusID(const NODE_STATUS &id) ;
  NODE_STATUS getNodeStatusID();
  int manageNodeStatus( const NODE_STATUS &require_node, std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle );
  bool nodePIDChecker(const NODE_STATUS &node );
  bool isValidateNode(const NODE_STATUS &node, double start_time = 0.0);

  bool startProcess(const std::string &command, const std::string &pidFilePath);
  bool stopProcess(const std::string &pidFilePath);
  bool startNavigation();
  bool startFactoryNavigation();
  bool startManualMapping();
  bool startAutoMapping();
  // void publishNodeStatus();
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle);
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle);

private:
  NODE_STATUS node_status_id = NODE_STATUS::IDLE;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  MANAGE_STATE task;
  double node_start_time;
  rclcpp_action::Server<robot_custom_msgs::action::ManageNode>::SharedPtr manage_node_action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> current_goal_handle_;
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const robot_custom_msgs::action::ManageNode::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_custom_msgs::action::ManageNode>> goal_handle);
};

#endif // NODE_MANAGER_HPP
