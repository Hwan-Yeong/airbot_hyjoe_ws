#ifndef __A1_MANEUVER_H__
#define __A1_MANEUVER_H__

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <any>
#include <chrono>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <robot_custom_msgs/msg/motor_status.hpp>
#include <robot_custom_msgs/msg/navi_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>

// #include "state_manager/utils/state_defines.hpp"
#include "/home/airbot/airbot_ws/install/state_manager/include/state_manager/utils/state_defines.hpp"

#include "motor_status.hpp"
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

    bool use_maneuver_action_{true};

    double current_velocity_;
    double velocity_scaling_factor_;

    bool back_object_detect_{false};
    bool front_escape_detect_{false};

    int32_t back_ms_1d_;
    int32_t back_ms_abort_;
    int32_t wait_ms_drop_off_;
    int32_t wait_ms_normal_;

    float lidar_front_escape_dist_;
    float lidar_front_escape_angle_;
    float lidar_back_escape_dist_;
    float lidar_back_escape_angle_;

    rclcpp::TimerBase::SharedPtr timer_{};

    rclcpp::Time action_start_time_;
    rclcpp::Duration action_process_time_{rclcpp::Duration::from_seconds(0)};
    std::unordered_map<std::string, std::any> subscribers{};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    void init_subs(void);
    void init_params(void);

    void navi_state_callback(const robot_custom_msgs::msg::NaviState::SharedPtr msg);
    void motor_status_callback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
    void bottom_ir_callback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void action_stop_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void collision_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void maneuver_use_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void timer_callback(void);
    void pub_states(void);

    geometry_msgs::msg::Twist robot_action_default(void);
    void robot_action_back(geometry_msgs::msg::Twist& cmd_vel, const double speed);
    void robot_action_front(geometry_msgs::msg::Twist& cmd_vel, const double speed);

    void robot_state(const double process_time_ms, const ACTION_STATE state);

    bool is_action_idle(void);
    bool is_stop_state(void);

    void state_log(ACTION_STATE a_before, ACTION_STATE a_after, STOP_STATE b_before, STOP_STATE b_after);
};
}  // namespace A1::maneuver

#endif
