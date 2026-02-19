#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_custom_msgs/msg/tof_data.hpp>
#include <robot_custom_msgs/msg/camera_data_array.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <robot_custom_msgs/msg/abnormal_event_data.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <functional>

enum class SensorType {
  kTofMono,
  kTofMultiLeft,
  kTofMultiRight,
  kCamera,
  kBottomIr,
  kCollisionFront,
  kCollisionRear
};

class RosNode : public rclcpp::Node {
public:
  RosNode();

  void toggleSensor(SensorType type, bool on);
  void toggleSensorManager(bool on);
  bool getSensorState(SensorType type) const;

  // Callback type for Cloud data
  using CloudCallback = std::function<void(const std::string&, const sensor_msgs::msg::PointCloud2::SharedPtr)>;
  void setCloudCallback(CloudCallback cb) { cloud_callback_ = cb; }

private:
  void publishFakeData();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_pub_;
  
  rclcpp::Publisher<robot_custom_msgs::msg::TofData>::SharedPtr tof_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_pub_;

  // Subscribers for visualization
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_mono_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_multi_l_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_multi_r_sub_;

  CloudCallback cloud_callback_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<SensorType, bool> sensor_states_;
  bool sensor_manager_active_ = false;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};
