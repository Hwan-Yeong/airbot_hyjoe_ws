#include "sensor_gui/ros_node.hpp"


RosNode::RosNode() : Node("sensor_simulator") {
  // Publishers
  sensor_manager_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("cmd_sensor_manager", 10);
  
  tof_pub_ = this->create_publisher<robot_custom_msgs::msg::TofData>("/tof_data", 10);
  camera_pub_ = this->create_publisher<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10);
  bottom_ir_pub_ = this->create_publisher<robot_custom_msgs::msg::BottomIrData>("/bottom_ir_data", 10);
  collision_pub_ = this->create_publisher<robot_custom_msgs::msg::AbnormalEventData>("/collision_detected", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RosNode::publishFakeData, this));

  // Subscribers
  auto create_cloud_sub = [this](const std::string& topic, const std::string& name) {
    return this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, 10, [this, name](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (cloud_callback_) cloud_callback_(name, msg);
      });
  };

  tof_mono_sub_ = create_cloud_sub("/sensor_to_pointcloud/tof/mono", "ToF Mono");

  std::vector<int> left_indices = {0, 3, 6, 17, 20, 23, 28, 31, 44, 47, 49, 52, 55, 57, 60, 63};
  for (int idx : left_indices) {
    std::string topic = "/sensor_to_pointcloud/tof/multi/left/idx_" + std::to_string(idx);
    tof_multi_subs_.push_back(create_cloud_sub(topic, "ToF Multi L " + std::to_string(idx)));
  }

  std::vector<int> right_indices = {1, 4, 7, 16, 19, 22, 24, 27, 40, 43, 48, 51, 54, 56, 59, 62};
  for (int idx : right_indices) {
    std::string topic = "/sensor_to_pointcloud/tof/multi/right/idx_" + std::to_string(idx);
    tof_multi_subs_.push_back(create_cloud_sub(topic, "ToF Multi R " + std::to_string(idx)));
  }

  // Initialize states
  sensor_states_[SensorType::kTofMono] = false;
  sensor_states_[SensorType::kTofMultiLeft] = false;
  sensor_states_[SensorType::kTofMultiRight] = false;
  sensor_states_[SensorType::kCamera] = false;
  sensor_states_[SensorType::kBottomIr] = false;
  sensor_states_[SensorType::kCollisionFront] = false;
  sensor_states_[SensorType::kCollisionRear] = false;

  // Static TF: map -> base_link (ensure RViz has a frame)
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = 0;
  t.transform.translation.y = 0;
  t.transform.translation.z = 0;
  t.transform.rotation.w = 1.0;
  static_broadcaster_->sendTransform(t);
}

void RosNode::toggleSensor(SensorType type, bool on) {
  sensor_states_[type] = on;
  RCLCPP_INFO(this->get_logger(), "Sensor %d toggled %s", static_cast<int>(type), on ? "ON" : "OFF");
}

void RosNode::toggleSensorManager(bool on) {
  sensor_manager_active_ = on;
  auto msg = std_msgs::msg::Bool();
  msg.data = on;
  sensor_manager_cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Sensor Manager CMD published: %s", on ? "ON" : "OFF");
}

bool RosNode::getSensorState(SensorType type) const {
  auto it = sensor_states_.find(type);
  if (it != sensor_states_.end()) return it->second;
  return false;
}

void RosNode::publishFakeData() {
  auto now = this->now();

  // 1. ToF Data
  if (sensor_states_[SensorType::kTofMono] || 
      sensor_states_[SensorType::kTofMultiLeft] || 
      sensor_states_[SensorType::kTofMultiRight]) {
    auto msg = robot_custom_msgs::msg::TofData();
    msg.timestamp = now;
    
    // Fake a plane at 0.5m
    if (sensor_states_[SensorType::kTofMono]) {
      msg.top = 0.5;
    } else {
      msg.top = 0.0;
    }
    if (sensor_states_[SensorType::kTofMultiLeft]) {
      for (int i = 0; i < 16; ++i) {
        msg.bot_left[i] = 0.5;
      }
    } else {
      for (int i = 0; i < 16; ++i) {
        msg.bot_left[i] = 0.0;
      }
    }
    if (sensor_states_[SensorType::kTofMultiRight]) {
      for (int i = 0; i < 16; ++i) {
        msg.bot_right[i] = 0.5;
      }
    } else {
      for (int i = 0; i < 16; ++i) {
        msg.bot_right[i] = 0.0;
      }
    }
    tof_pub_->publish(msg);
  }

  // 2. Camera Data
  if (sensor_states_[SensorType::kCamera]) {
    auto msg = robot_custom_msgs::msg::CameraDataArray();
    msg.timestamp = now;
    msg.num = 1;
    robot_custom_msgs::msg::CameraData obj;
    obj.id = 0; // Cable
    obj.score = 90;
    obj.distance = 0.3;
    obj.x = 0.3;
    obj.y = 0.0;
    obj.width = 0.4;
    obj.height = 0.2;
    obj.theta = 0.0;
    msg.data_array.push_back(obj);
    camera_pub_->publish(msg);
  }

  // 3. Bottom IR
  if (sensor_states_[SensorType::kBottomIr]) {
    auto msg = robot_custom_msgs::msg::BottomIrData();
    msg.timestamp = now;
    // Assume no cliff
    msg.ff = true;
    msg.fl = true;
    msg.fr = true;
    msg.bl = true;
    msg.br = true;
    msg.bb = true;
    bottom_ir_pub_->publish(msg);
  }

  // 4. Collision
  if (sensor_states_[SensorType::kCollisionFront] || 
      sensor_states_[SensorType::kCollisionRear]) {
    auto msg = robot_custom_msgs::msg::AbnormalEventData();
    msg.timestamp = now;
    if (sensor_states_[SensorType::kCollisionFront]) msg.event_trigger = 1;
    else if (sensor_states_[SensorType::kCollisionRear]) msg.event_trigger = -1;
    collision_pub_->publish(msg);
  }
}
