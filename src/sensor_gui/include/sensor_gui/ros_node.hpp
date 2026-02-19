#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_custom_msgs/msg/tof_data.hpp>
#include <robot_custom_msgs/msg/camera_data_array.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <robot_custom_msgs/msg/abnormal_event_data.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <functional>
#include <atomic>

enum class SensorType {
  kTofMono,
  kTofMultiLeft,
  kTofMultiRight,
  kCamera,
  kBottomIr,
  kCollisionFront,
  kCollisionRear
};

enum class IrIndex {
  kFF = 0, kFL, kFR, kBB, kBL, kBR
};

class RosNode : public rclcpp::Node {
public:
  RosNode();

  void toggleSensor(SensorType type, bool on);
  void toggleSensorManager(bool on);
  bool getSensorState(SensorType type) const;

  // Simulation Parameters
  void setToFDistance(float d) { tof_dist_ = d; }
  void setCameraParams(float dist, float w, float h) {
    cam_dist_ = dist;
    cam_width_ = w;
    cam_height_ = h;
  }
  void setIrState(IrIndex idx, bool state) { ir_states_[static_cast<int>(idx)] = state; }
  void setRobotPose(float x, float y, float yaw) {
    robot_x_ = x;
    robot_y_ = y;
    robot_yaw_ = yaw;
  }

  // Callback type for Cloud data
  using CloudCallback = std::function<void(const std::string&, const sensor_msgs::msg::PointCloud2::SharedPtr)>;
  void setCloudCallback(CloudCallback cb) { cloud_callback_ = cb; }

  std::shared_ptr<tf2_ros::Buffer> getTFBuffer() const { return tf_buffer_; }

private:
  void publishFakeData();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_pub_;
  
  rclcpp::Publisher<robot_custom_msgs::msg::TofData>::SharedPtr tof_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_pub_;

  // Subscribers for visualization
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tof_mono_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> tof_multi_subs_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bottom_ir_pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr collision_f_pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr collision_r_pc_sub_;

  CloudCallback cloud_callback_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::map<SensorType, bool> sensor_states_;
  bool sensor_manager_active_ = false;

  // Simulation Parameters (atomic for thread safety during publish)
  std::atomic<float> tof_dist_{0.5f};
  std::atomic<float> cam_dist_{0.3f};
  std::atomic<float> cam_width_{0.4f};
  std::atomic<float> cam_height_{0.2f};
  std::atomic<bool> ir_states_[6];
  
  std::atomic<float> robot_x_{0.0f};
  std::atomic<float> robot_y_{0.0f};
  std::atomic<float> robot_yaw_{0.0f};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
