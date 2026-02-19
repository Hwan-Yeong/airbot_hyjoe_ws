#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_custom_msgs/msg/tof_data.hpp>
#include <robot_custom_msgs/msg/camera_data_array.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <robot_custom_msgs/msg/abnormal_event_data.hpp>
#include <map>

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

private:
  void publishFakeData();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_pub_;
  
  rclcpp::Publisher<robot_custom_msgs::msg::TofData>::SharedPtr tof_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<SensorType, bool> sensor_states_;
  bool sensor_manager_active_ = false;
};
