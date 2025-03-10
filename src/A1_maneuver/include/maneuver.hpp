#ifndef __A1_MANEUVER_H__
#define __A1_MANEUVER_H__

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <any>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_custom_msgs/msg/motor_status.hpp>
#include <robot_custom_msgs/msg/navi_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>

#include "/home/airbot/airbot_ws/install/state_manager/include/state_manager/utils/state_defines.hpp"
#include "motor_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "states.hpp"

using namespace airbot_state;
namespace A1::maneuver
{

class ManeuverNode : public rclcpp::Node
{
   public:
    ManeuverNode();
    ~ManeuverNode() override = default;

   private:
    ACTION_STATE action_state_;
    STOP_STATE stop_state_;
    NAVI_STATE navi_state_;
    MotorStatus motor_status_{};

    double current_velocity_;
    double velocity_scaling_factor_;

    bool back_object_detect_{false};
    bool front_escape_detect_{false};

    int32_t back_ms_1d_;
    int32_t back_ms_abort_;
    int32_t wait_ms_drop_off_;
    int32_t wait_ms_normal_;

    float lidar_front_escape_dist_;
    float lidar_back_abort_dist_;

    rclcpp::TimerBase::SharedPtr timer_{};

    rclcpp::Time action_start_time_;
    rclcpp::Duration action_process_time_{rclcpp::Duration::from_seconds(0)};
    std::unordered_map<std::string, std::any> subscribers{};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void init_subs(void);
    void navi_state_callback(const robot_custom_msgs::msg::NaviState::SharedPtr msg);
    void motor_status_callback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    void drop_off_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void action_stop_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void scan_back_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan_front_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback(void);

    void robot_action_stop(void);
    void robot_action_back(const double speed);
    void robot_action_front(const double speed);

    void robot_back(const double process_time_ms);
    void robot_wait(const double process_time_ms);
    void robot_front(const double process_time_ms);

    bool is_action_idle(void);
    bool is_stop_state(void);
};
}  // namespace A1::maneuver

#endif
