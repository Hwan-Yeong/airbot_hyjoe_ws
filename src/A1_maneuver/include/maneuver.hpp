#ifndef __A1_MANEUVER_H__
#define __A1_MANEUVER_H__

#include "/home/airbot/airbot_ws/install/state_manager/include/state_manager/utils/state_defines.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "states.hpp"
#include <chrono>
#include <cmath>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_custom_msgs/msg/motor_status.hpp>
#include <robot_custom_msgs/msg/navi_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace airbot_state;
namespace A1::maneuver {

class ManeuverNode : public rclcpp::Node {
public:
  ManeuverNode();
  ~ManeuverNode() override = default;

private:
  ACTION_STATE action_state_;
  STOP_STATE stop_state_;
  NAVI_STATE navi_state_;
  MotorStatus motor_status_{};
  // int32_t drop_off_datas_;
  int32_t idle_state_cnt_;
  int32_t idle_state_limit_;
  rclcpp::Time back_start_time_;
  rclcpp::TimerBase::SharedPtr timer_{};

  rclcpp::Subscription<robot_custom_msgs::msg::NaviState>::SharedPtr
      navi_state_subs_;
  rclcpp::Subscription<robot_custom_msgs::msg::MotorStatus>::SharedPtr
      motor_status_subs_;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
  //     perception_dropoff_subs_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr action_stop_subs;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void
  navi_state_callback(const robot_custom_msgs::msg::NaviState::SharedPtr msg);
  void motor_status_callback(
      const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
  void drop_off_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void action_stop_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback(void);
  void robot_move_stop(void);
  void robot_move_back(const double speed);
  bool is_action_idle(void);
  bool is_stop_state(void);
};
} // namespace A1::maneuver

#endif
