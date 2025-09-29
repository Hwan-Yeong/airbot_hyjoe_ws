#ifndef __A1_MANEUVER_H__
#define __A1_MANEUVER_H__

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 2025.07.31 Clabil 강성준, pcl lib RAD2DEG 중복으로 인한 build warning 제거
#ifdef RAD2DEG
#undef RAD2DEG
#endif

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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <robot_custom_msgs/msg/motor_status.hpp>
#include <robot_custom_msgs/msg/navi_state.hpp>
#include <robot_custom_msgs/msg/tof_data.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "motor_status.hpp"
#include "state_manager/utils/state_defines.hpp"

#include "escape_candidate.hpp"
#include "position.hpp"
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
    MAIN_STATE main_state_;
    SUB_STATE sub_state_;
    NAVI_STATE navi_state_;
    MAPPING_STATE explore_state_;
    MotorStatus motor_status_{};

    bool use_maneuver_action_{false};
    bool current_maneuver_state{false};

    double current_velocity_;
    double velocity_scaling_factor_;

    bool back_object_detect_{false};
    bool front_escape_detect_{false};

    int32_t back_ms_1d_;
    int32_t back_ms_collision_;

    int32_t wait_ms_drop_off_;
    int32_t wait_ms_normal_;

    int32_t costmap_escape_value_;                    // 92(obstacles > 90)
    int32_t costmap_escape_release_value_;            // 65
    int32_t costmap_escape_candidate_threshold_;      // default 80(0~99)
    float costmap_escape_candidate_min_dist_;         // unit m
    float costmap_escape_candidate_max_dist_;         // unit m
    int32_t costmap_escape_time_multiplier_for_dist;  // unit ms (탈출 거리(m)에 곱해질 시간(ms))
    int32_t costmap_escape_sampling_;                 // sampling cnt
    int32_t costmap_escape_sampling_threshold_;       // sampling cnt threshold
    float costmap_escape_angular_z_speed;
    EscapeCandidate escape_target_;  // destination for escape

    int32_t no_plan_sampling_;            // sampling cnt
    int32_t no_plan_sampling_threshold_;  // sampling cnt threshold
    float no_plan_release_degree_;
    float no_plan_global_timeout_sec_;
    float no_plan_timeout_sec_;
    rclcpp::Time no_plan_start_time_;

    float max_vel_;
    float min_vel_;

    float lidar_front_escape_diagonal_dist_;
    float lidar_front_escape_min_dist_;
    float lidar_front_escape_max_dist_;
    float lidar_front_escape_angle_;
    float lidar_back_escape_dist_;
    float lidar_back_escape_angle_;

    float drop_ir_lidar_collision_front_distance_;
    float drop_ir_lidar_collision_front_angle_;
    float drop_ir_lidar_collision_back_distance_;
    float drop_ir_lidar_collision_back_angle_;

    bool drop_ir_lidar_front_detect_{false};
    bool drop_ir_lidar_back_detect_{false};
    bool is_climb_from_perception{false};

    double force_escape_target_angle_;
    double force_escape_given_yaw_;
    std::chrono::steady_clock::time_point force_escape_start_time_;

    Position robot_pose_{};

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_static_map_;
    nav_msgs::msg::Path::SharedPtr global_plan_;
    nav_msgs::msg::Path::SharedPtr local_plan_;
    robot_custom_msgs::msg::TofData::SharedPtr tof_msg_;
    rclcpp::TimerBase::SharedPtr timer_{};

    std::chrono::steady_clock::time_point last_state_pub_time_;
    std::chrono::steady_clock::time_point action_start_time_;
    std::chrono::milliseconds action_process_time_;
    std::chrono::steady_clock::time_point escape_start_time;
    // rclcpp::Time action_start_time_;
    // rclcpp::Duration action_process_time_{rclcpp::Duration::from_seconds(0)};
    // rclcpp::Time escape_start_time;
    std::unordered_map<std::string, std::any> subscribers{};
    // std::unordered_map<std::string, rclcpp::Time> logTimeMap{};
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> logTimeMap;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_str_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr state_int_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr force_escape_pub_;

    /*
     * functions
     */
    void init_subs(void);
    void init_params(void);

    // 작업이 있는 callback, 단순 callback은 inline function
    void navi_state_callback(const robot_custom_msgs::msg::NaviState::SharedPtr msg);
    // void explore_status_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void bottom_ir_callback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void action_stop_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void collision_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void perception_climb_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void timer_callback(void);
    int8_t getCostXY(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, float x, float y);

    // ROBOT ACTION
    void robot_action_back(geometry_msgs::msg::Twist& cmd_vel, const double speed);
    void robot_action_front(geometry_msgs::msg::Twist& cmd_vel, const double speed);
    void robot_state(const double process_time_ms, const MAIN_STATE state);

    // 상태값 확인 함수
    bool is_main_idle(void);
    bool is_sub_idle(void);
    // bool is_processing(rclcpp::Time current_time);
    bool is_processing(std::chrono::steady_clock::time_point current_time);

    // costmap escape
    void force_escape_turn_action(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish);
    void escape_turn_action(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish);
    void current_postion_costmap_check(void);
    void lethal_escape_process(void);

    // no valid trajectory
    void check_no_local_plan(void);
    void enter_no_local_plan(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish);
    void process_no_local_plan(geometry_msgs::msg::Twist& cmd_vel, bool& is_publish);
    void resolve_no_local_plan();

    bool log_time_diff(std::string name, double interval_ms);
    // for state
    void state_log(MAIN_STATE a_after, SUB_STATE b_after);
    void pub_states(void);
};
}  // namespace A1::maneuver

#endif
