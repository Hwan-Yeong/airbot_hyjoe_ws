#include "maneuver.hpp"

namespace A1::maneuver {

ManeuverNode::ManeuverNode() : Node("A1_maneuver") {

  action_state_ = ACTION_STATE::IDLE;
  navi_state_ = NAVI_STATE::IDLE;
  stop_state_ = STOP_STATE::NO_STOP;
  // drop_off_datas_ = 0;
  idle_state_cnt_ = 0;
  idle_state_limit_ = 5;

  this->declare_parameter("timer_ms", 100);
  int timer_ms = this->get_parameter("timer_ms").as_int();

  this->declare_parameter("idle_state_ms", 500);
  int idle_state_ms = this->get_parameter("idle_state_ms").as_int();
  idle_state_limit_ = idle_state_ms / timer_ms;

  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.best_effort();
  // From UDP_Interface /navi_datas topic
  navi_state_subs_ =
      this->create_subscription<robot_custom_msgs::msg::NaviState>(
          "/navi_datas", qos_profile,
          std::bind(&ManeuverNode::navi_state_callback, this,
                    std::placeholders::_1));

  motor_status_subs_ =
      this->create_subscription<robot_custom_msgs::msg::MotorStatus>(
          "/motor_status", qos_profile,
          std::bind(&ManeuverNode::motor_status_callback, this,
                    std::placeholders::_1));

  action_stop_subs = this->create_subscription<std_msgs::msg::Int32>(
      "/perception/action/stop", qos_profile,
      std::bind(&ManeuverNode::action_stop_callback, this,
                std::placeholders::_1));

  // 로봇 움직임 제어
  cmd_vel_nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed", qos_profile,
      std::bind(&ManeuverNode::cmd_vel_nav_callback, this,
                std::placeholders::_1));

  // cmd_vel_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_vel",
  // 1);
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(timer_ms),
                              std::bind(&ManeuverNode::timer_callback, this));
}

void ManeuverNode::cmd_vel_nav_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {

  // 이상 없는 상태일 떄 nav2의 명령어 전달
  if (is_action_idle()) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.linear.y = msg->linear.y;
    cmd_vel.linear.z = msg->linear.z;
    cmd_vel.angular.x = msg->angular.x;
    cmd_vel.angular.y = msg->angular.y;
    cmd_vel.angular.z = msg->angular.z;
    cmd_vel_pub_->publish(cmd_vel);
  }
}

void ManeuverNode::navi_state_callback(
    const robot_custom_msgs::msg::NaviState::SharedPtr msg) {
  NAVI_STATE current_state = static_cast<NAVI_STATE>(msg->state);
  if (navi_state_ != NAVI_STATE::FAIL && current_state == NAVI_STATE::FAIL) {
    action_state_ = ACTION_STATE::BACK;
    back_start_time_ = this->get_clock()->now();

    RCLCPP_INFO(
        this->get_logger(), "%s():%d: Navi State: %s, Fail_reason: %s",
        __FUNCTION__, __LINE__, enumToString(current_state).c_str(),
        enumToString(static_cast<NAVI_FAIL_REASON>(msg->fail_reason)).c_str());
  }
  navi_state_ = current_state;
}

/***
 0 : STOP_STATE::NO_STOP
 1 : STOP_STATE::DROP_OFF
 2 : STOP_STATE::ONE_D_TOF
 */
void ManeuverNode::action_stop_callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
    if (is_action_idle() && motor_status_.isMoveToFoward()) {
    RCLCPP_INFO(this->get_logger(), "%s():%d: action stop: %s", __FUNCTION__,
                __LINE__,
                enumToString(static_cast<STOP_STATE>(msg->data)).c_str());
    if (msg->data == 1) {
      action_state_ = ACTION_STATE::STOP;
      stop_state_ = STOP_STATE::DROP_OFF;
    } else if (msg->data == 2) {
      action_state_ = ACTION_STATE::STOP;
      stop_state_ = STOP_STATE::ONE_D_TOF;
    } else {
      action_state_ = ACTION_STATE::IDLE;
      stop_state_ = STOP_STATE::NO_STOP;
    }
  }
}

void ManeuverNode::motor_status_callback(
    const robot_custom_msgs::msg::MotorStatus::SharedPtr msg) {
  this->motor_status_.setLeftRpm(msg->left_motor_rpm);
  this->motor_status_.setRightRpm(msg->right_motor_rpm);
}

void ManeuverNode::timer_callback() {
  // // 이동 상태가 아닐경우 명령어 전달 x
  // if (navi_state_ != NAVI_STATE::MOVE_GOAL) {
  //   return;
  // }

  rclcpp::Time current_time = this->get_clock()->now();
  // 뒤로가기 상태
  if (action_state_ == ACTION_STATE::BACK) {
    if (current_time - back_start_time_ < rclcpp::Duration::from_seconds(1.5)) {
      robot_move_back(-0.1);
    } else {
      action_state_ = ACTION_STATE::IDLE;
      stop_state_ = STOP_STATE::NO_STOP;
      robot_move_stop();
    }
  } else if (action_state_ == ACTION_STATE::STOP && is_stop_state()) {
    bool isMinRpm = this->motor_status_.isLowRpm();
    bool isRotate = this->motor_status_.isRotate();

    if (isMinRpm || isRotate) {
      idle_state_cnt_++;
      if (idle_state_cnt_ > idle_state_limit_) {
        idle_state_cnt_ = 0;
        if (stop_state_ == STOP_STATE::DROP_OFF) {
          // 단차 감지는 즉각 정지
          action_state_ = ACTION_STATE::IDLE;
          stop_state_ = STOP_STATE::NO_STOP;
        } else if (stop_state_ == STOP_STATE::ONE_D_TOF) {
          // 1D Tof 감지는 뒤로 이동
          action_state_ = ACTION_STATE::BACK;
          back_start_time_ = this->get_clock()->now();
        }
      }
    } else {
      robot_move_stop();
    }
  }
}
bool ManeuverNode::is_action_idle() {
  return action_state_ == ACTION_STATE::IDLE && !is_stop_state();
}

bool ManeuverNode::is_stop_state() {
  return stop_state_ != STOP_STATE::NO_STOP;
}

void ManeuverNode::robot_move_stop(void) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

void ManeuverNode::robot_move_back(double speed) {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = speed;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

} // namespace A1::maneuver