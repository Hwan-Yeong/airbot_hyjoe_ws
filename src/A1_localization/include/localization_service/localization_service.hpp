// 2025-03-11 clabil v1.5
#ifndef __A1_LOCALIZATION_H__
#define __A1_LOCALIZATION_H__
#include <any>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <unordered_map>

class LocalizationService : public rclcpp::Node
{
   public:
    LocalizationService();
    ~LocalizationService() = default;
    void run();

   private:
    long unsigned int min_particle_;
    unsigned int reqeust_time_ms_;
    float particle_check_sec_;
    float fail_check_sec_;
    bool turn_left_ = false;
    bool initpose_req_ = false;
    bool initpose_end_ = false;
    bool print_log_ = false;
    rclcpp::Time particle_time_{};
    rclcpp::Time initpose_start_time_{};
    rclcpp::Time lidar_check_time{};

    // for navigation2
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr initpose_srv_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_global_srv_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_local_srv_;

    std::unordered_map<std::string, std::any> subscribers{};

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr localization_comp_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_comp_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_publisher_;

    void initpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void particlecloud_callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg);
    void goalpose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void costmap_clear_callback(const std_msgs::msg::Empty::SharedPtr msg);
    [[deprecated("2025-02-10, use nomotion_localization_callback() instead")]] void nomotion_callback(
        const std_msgs::msg::Empty::SharedPtr msg);
    void nomotion_with_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback(void);
    void init_subscribers(void);

    bool checking_lidar(rclcpp::Time now);
    void checking_particle(rclcpp::Time now);
    void call_clear_costmap_services(void);
};
#endif  //__A1_LOCALIZATION_H__
