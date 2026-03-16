#include "sensor_gui/node/ros_node.hpp"


RosNode::RosNode() : Node("sensor_simulator") {
  for (int i = 0; i < 6; ++i) ir_states_[i] = true;

  // Publishers
  sensor_manager_cmd_pub_ = this->create_publisher<std_msgs::msg::Bool>("cmd_sensor_manager", 10);
  
  tof_pub_ = this->create_publisher<robot_custom_msgs::msg::TofData>("/tof_data", 10);
  camera_pub_ = this->create_publisher<robot_custom_msgs::msg::CameraDataArray>("/camera_data", 10);
  bottom_ir_pub_ = this->create_publisher<robot_custom_msgs::msg::BottomIrData>("/bottom_ir_data", 10);
  collision_pub_ = this->create_publisher<robot_custom_msgs::msg::AbnormalEventData>("/collision_detected", 10);

  collision_pub_ = this->create_publisher<robot_custom_msgs::msg::AbnormalEventData>("/collision_detected", 10);

  // Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&RosNode::cmdVelCallback, this, std::placeholders::_1));

  // Declare Parameters
  this->declare_parameter<double>("physics.robot_mass", 10.0);
  robot_mass_ = this->get_parameter("physics.robot_mass").as_double();

  robot_z_ = 0.05f;

  smoother_vx_.setLimits(linear_speed_, 0.6, 2.0);
  smoother_vy_.setLimits(linear_speed_, 0.6, 2.0);
  smoother_vyaw_.setLimits(angular_speed_, 143.24, 458.37);

  last_time_ = std::chrono::steady_clock::now();

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

  tof_mono_sub_ = create_cloud_sub("/sensor_to_pointcloud/tof/mono/local", "ToF Mono");

  std::vector<int> left_indices = {0, 3, 6, 17, 20, 23, 28, 31, 44, 47, 49, 52, 55, 57, 60, 63};
  for (int idx : left_indices) {
    std::string topic = "/sensor_to_pointcloud/tof/multi/left/idx_" + std::to_string(idx) + "/local";
    tof_multi_subs_.push_back(create_cloud_sub(topic, "ToF Multi L " + std::to_string(idx)));
  }

  std::vector<int> right_indices = {1, 4, 7, 16, 19, 22, 24, 27, 40, 43, 48, 51, 54, 56, 59, 62};
  for (int idx : right_indices) {
    std::string topic = "/sensor_to_pointcloud/tof/multi/right/idx_" + std::to_string(idx) + "/local";
    tof_multi_subs_.push_back(create_cloud_sub(topic, "ToF Multi R " + std::to_string(idx)));
  }

  camera_pc_sub_ = create_cloud_sub("/sensor_to_pointcloud/camera_object/local", "Camera Object PC");
  bottom_ir_pc_sub_ = create_cloud_sub("/sensor_to_pointcloud/bottom_ir/local", "Bottom IR PC");
  collision_f_pc_sub_ = create_cloud_sub("/sensor_to_pointcloud/collision/front/local", "Collision F PC");
  collision_r_pc_sub_ = create_cloud_sub("/sensor_to_pointcloud/collision/rear/local", "Collision R PC");

  // Initialize states
  sensor_states_[SensorType::kTofMono] = true;
  sensor_states_[SensorType::kTofMultiLeft] = true;
  sensor_states_[SensorType::kTofMultiRight] = true;
  sensor_states_[SensorType::kCamera] = false;
  sensor_states_[SensorType::kBottomIr] = false;
  sensor_states_[SensorType::kCollisionFront] = false;
  sensor_states_[SensorType::kCollisionRear] = false;

  // Dynamic TF: map -> base_link
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // TF Listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

void RosNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  target_vx_ = msg->linear.x;
  target_vy_ = msg->linear.y;
  target_vyaw_ = msg->angular.z * 180.0 / M_PI; // rad to deg for internal calc
}

void RosNode::publishFakeData() {
  auto now = this->now();

  // 0. Publish map -> base_link transform
  // Apply S-Curve Smoothing
  auto now_time = std::chrono::steady_clock::now();
  double dt = 0.1; // fallback
  if (first_update_) {
      first_update_ = false;
      smoother_vx_.reset(0.0, 0.0);
      smoother_vy_.reset(0.0, 0.0);
      smoother_vyaw_.reset(0.0, 0.0);
  } else {
      dt = std::chrono::duration<double>(now_time - last_time_).count();
  }
  last_time_ = now_time;

  smoother_vx_.setLimits(linear_speed_, 0.6, 2.0);
  smoother_vy_.setLimits(linear_speed_, 0.6, 2.0);
  smoother_vyaw_.setLimits(angular_speed_, 143.24, 458.37);

  float vx = static_cast<float>(smoother_vx_.update(target_vx_.load(), dt));
  float vy = static_cast<float>(smoother_vy_.update(target_vy_.load(), dt));
  float vyaw = static_cast<float>(smoother_vyaw_.update(target_vyaw_.load(), dt));
  
  smoothed_vx_.store(vx);
  smoothed_vy_.store(vy);
  smoothed_vyaw_rad_.store(vyaw * M_PI / 180.0f);

  if (!use_physics_) {
      // Velocity Integration
      if (std::abs(vx) > 0.001f || std::abs(vy) > 0.001f || std::abs(vyaw) > 0.001f) {
          float cy = std::cos(robot_yaw_.load() * M_PI / 180.0f);
          float sy = std::sin(robot_yaw_.load() * M_PI / 180.0f);
          
          float dx = (vx * cy - vy * sy) * dt;
          float dy = (vx * sy + vy * cy) * dt;
          float dyaw = vyaw * dt;
          
          robot_x_.store(robot_x_.load() + dx);
          robot_y_.store(robot_y_.load() + dy);
          robot_yaw_.store(robot_yaw_.load() + dyaw);
      }
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = robot_x_.load();
  t.transform.translation.y = robot_y_.load();
  t.transform.translation.z = robot_z_.load();
  
  float yaw = robot_yaw_.load() * M_PI / 180.0f;
  t.transform.rotation.z = std::sin(yaw / 2.0);
  t.transform.rotation.w = std::cos(yaw / 2.0);
  tf_broadcaster_->sendTransform(t);

  // 1. ToF Data
  if (sensor_states_[SensorType::kTofMono] || 
      sensor_states_[SensorType::kTofMultiLeft] || 
      sensor_states_[SensorType::kTofMultiRight]) {
    auto msg = robot_custom_msgs::msg::TofData();
    msg.timestamp = now;
    msg.robot_x = robot_x_.load();
    msg.robot_y = robot_y_.load();
    msg.robot_angle = robot_yaw_.load() * M_PI / 180.0f;
    
    if (sensor_states_[SensorType::kTofMono]) {
      msg.top = tof_mono_dist_.load();
    } else {
      msg.top = 0.0;
    }
    if (sensor_states_[SensorType::kTofMultiLeft]) {
      float d_left = tof_left_dist_.load();
      for (int i = 0; i < 16; ++i) {
        msg.bot_left[i] = d_left;
      }
    } else {
      for (int i = 0; i < 16; ++i) {
        msg.bot_left[i] = 0.0;
      }
    }
    if (sensor_states_[SensorType::kTofMultiRight]) {
      float d_right = tof_right_dist_.load();
      for (int i = 0; i < 16; ++i) {
        msg.bot_right[i] = d_right;
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
    msg.robot_x = robot_x_.load();
    msg.robot_y = robot_y_.load();
    msg.robot_angle = robot_yaw_.load() * M_PI / 180.0f;
    msg.num = 1;
    robot_custom_msgs::msg::CameraData obj;
    obj.id = 0; // Cable
    obj.score = 90;
    obj.distance = cam_dist_.load();
    obj.x = cam_dist_.load();
    obj.y = 0.0;
    obj.width = cam_width_.load();
    obj.height = cam_height_.load();
    obj.theta = 0.0;
    msg.data_array.push_back(obj);
    camera_pub_->publish(msg);
  }

  // 3. Bottom IR
  if (sensor_states_[SensorType::kBottomIr]) {
    auto msg = robot_custom_msgs::msg::BottomIrData();
    msg.timestamp = now;
    msg.robot_x = robot_x_.load();
    msg.robot_y = robot_y_.load();
    msg.robot_angle = robot_yaw_.load() * M_PI / 180.0f;
    msg.ff = ir_states_[0].load();
    msg.fl = ir_states_[1].load();
    msg.fr = ir_states_[2].load();
    msg.bb = ir_states_[3].load();
    msg.bl = ir_states_[4].load();
    msg.br = ir_states_[5].load();
    bottom_ir_pub_->publish(msg);
  }

  // 4. Collision
  if (sensor_states_[SensorType::kCollisionFront] || 
      sensor_states_[SensorType::kCollisionRear]) {
    auto msg = robot_custom_msgs::msg::AbnormalEventData();
    msg.timestamp = now;
    msg.robot_x = robot_x_.load();
    msg.robot_y = robot_y_.load();
    msg.robot_angle = robot_yaw_.load() * M_PI / 180.0f;
    if (sensor_states_[SensorType::kCollisionFront]) msg.event_trigger = 1;
    else if (sensor_states_[SensorType::kCollisionRear]) msg.event_trigger = -1;
    collision_pub_->publish(msg);
  }
}
