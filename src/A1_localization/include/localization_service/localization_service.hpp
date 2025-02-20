// 2025-02-10 clabil v1.3
#ifndef __A1_LOCALIZATION_H__
#define __A1_LOCALIZATION_H__
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"

class LocalizationService : public rclcpp::Node {
public:
  LocalizationService();
  ~LocalizationService() = default;
  void run();

private:
  long unsigned int min_particle_;
  unsigned int reqeust_time_ms_;
  bool initpose_req_ = false;
  bool initpose_end_ = false;
  bool print_log_ = false;

  // for navigation2
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr initpose_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
      clear_costmap_global_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
      clear_costmap_local_srv_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goalpose_subscription_;
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr
      particlecloud_subscription_;

  // for everybot service(UDP Interface)
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      costmap_clear_req_subscription_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      nomotion_req_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      nomotion_localization_req_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
      localization_comp_publisher_;

  void initpose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void
  particlecloud_callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg);
  void goalpose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void costmap_clear_callback(const std_msgs::msg::Empty::SharedPtr msg);
  // @depreacate, 2025-02-10, use nomotion_localization_callback
  [[deprecated(
      "2025-02-10, use nomotion_localization_callback() instead")]] void
  nomotion_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void nomotion_localization_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void timer_callback(void);
  void call_clear_costmap_services(void);
};
#endif //__A1_LOCALIZATION_H__
