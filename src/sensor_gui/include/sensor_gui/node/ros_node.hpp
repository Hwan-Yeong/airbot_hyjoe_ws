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
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <functional>
#include <atomic>
#include "sensor_gui/util/s_curve_profile.hpp"
#include <chrono>

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
  void setTofMonoDist(float d) { tof_mono_dist_ = d; }
  void setTofLeftDist(float d) { tof_left_dist_ = d; }
  void setTofRightDist(float d) { tof_right_dist_ = d; }
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
  void setRobotZ(float z) { robot_z_ = z; }
  void setVelocities(float vx, float vy, float vyaw) {
    target_vx_ = vx;
    target_vy_ = vy;
    target_vyaw_ = vyaw;
  }

  // Linear and Angular Speed Configurations
  void setLinearSpeed(float s)  { linear_speed_  = s; }
  void setAngularSpeed(float s) { angular_speed_ = s; }

  float getRobotX() const { return robot_x_; }
  float getRobotY() const { return robot_y_; }
  float getRobotYaw() const { return robot_yaw_; }
  float getRobotZ() const { return robot_z_; }

  float getTargetVx() const { return target_vx_.load(); }
  float getTargetVyawRad() const { return target_vyaw_.load() * M_PI / 180.0f; }

  float getSmoothedVx() const { return smoothed_vx_.load(); }
  float getSmoothedVyawRad() const { return smoothed_vyaw_rad_.load(); }

  void setUsePhysics(bool use) { use_physics_ = use; }
  bool getUsePhysics() const { return use_physics_; }
  
  // Physics Parameters
  float getRobotMass() const { return robot_mass_; }

  // Callback type for Cloud data
  using CloudCallback = std::function<void(const std::string&, const sensor_msgs::msg::PointCloud2::SharedPtr)>;
  void setCloudCallback(CloudCallback cb) { cloud_callback_ = cb; }

  std::shared_ptr<tf2_ros::Buffer> getTFBuffer() const { return tf_buffer_; }

private:
  void publishFakeData();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
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
  std::atomic<float> tof_mono_dist_{0.5f};
  std::atomic<float> tof_left_dist_{1.0f};
  std::atomic<float> tof_right_dist_{1.0f};
  std::atomic<float> cam_dist_{0.3f};
  std::atomic<float> cam_width_{0.4f};
  std::atomic<float> cam_height_{0.2f};
  std::atomic<bool> ir_states_[6];
  
  std::atomic<float> robot_x_{0.0f};
  std::atomic<float> robot_y_{0.0f};
  std::atomic<float> robot_yaw_{0.0f};
  std::atomic<float> robot_z_{0.045f};

  // Target velocities from /cmd_vel
  std::atomic<float> target_vx_{0.0f};
  std::atomic<float> target_vy_{0.0f};
  std::atomic<float> target_vyaw_{0.0f};

  // Smoothed outputs
  std::atomic<float> smoothed_vx_{0.0f};
  std::atomic<float> smoothed_vyaw_rad_{0.0f};

  // Speed Limits
  float linear_speed_  = 0.5f;
  float angular_speed_ = 60.0f;

  // Smoothing
  airbot::SCurveProfile smoother_vx_;
  airbot::SCurveProfile smoother_vy_;
  airbot::SCurveProfile smoother_vyaw_;
  
  std::chrono::steady_clock::time_point last_time_;
  bool first_update_ = true;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool use_physics_ = false;

  // Parameters
  float robot_mass_ = 10.0f;
};
