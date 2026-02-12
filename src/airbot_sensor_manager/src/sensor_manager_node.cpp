#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_manager_node.hpp"

namespace sensor_manager {

SensorManagerNode::SensorManagerNode() : Node("airbot_sensor_to_pointcloud") {
  this->LoadConfig();
  this->declare_parameter("target_frame", "map");
  this->get_parameter("target_frame", node_target_frame_);
  RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'",
              node_target_frame_.c_str());

  // Initialize Sensor Topic Registry
  sensor_topic_registry_ = {
    //{SensorType, {ConverterName, TopicName}}
      {SensorType::kTofMono, {"tof_mono", "tof/mono"}},
      {SensorType::kTofMultiLeft, {"tof_multi_left", "tof/multi/left"}},
      {SensorType::kTofMultiRight, {"tof_multi_right", "tof/multi/right"}},
      {SensorType::kCamera, {"camera", "camera_object"}},
      {SensorType::kBottomIr, {"bottom_ir", "bottom_ir"}},
      {SensorType::kCollisionFront, {"collision_front", "collision/front"}},
      {SensorType::kCollisionRear, {"collision_rear", "collision/rear"}}};

  // Initialize Multizone ToF Calibrator
  mtof_calibrator_ = std::make_unique<MultizoneTofCalibrator>(
      this->get_logger(), this->LoadMultizoneTofCalibrationParams());
  RCLCPP_INFO(this->get_logger(), "MultizoneTofCalibrator Initialized.");

  // Sensor Manager On/Off Cmd Subscriber
  sensor_manager_cmd_sub_ =
    this->create_subscription<std_msgs::msg::Bool>(
      "cmd_sensor_manager", rclcpp::QoS(3).reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg) {
          RCLCPP_ERROR(this->get_logger(),
                       "cmd_sensor_manager topic is a nullptr message.");
          return;
        }

        this->node_active_cmd_ = msg->data;
        std_msgs::msg::Bool sensor_manager_state_msg;
        sensor_manager_state_msg.data = msg->data;

        if (this->node_active_cmd_) {
          RCLCPP_INFO(this->get_logger(),
                      "[sensor to pointcloud] activeCmdCallback : Active");
          for (int i = 0; i < 3; ++i) {
            node_active_cmd_response_pub_->publish(sensor_manager_state_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
          }
        } else {
          InitializeRuntime();
          PublishEmptyMsg();
          RCLCPP_INFO(this->get_logger(),
                      "[sensor to pointcloud] activeCmdCallback : De-Active");
          for (int i = 0; i < 3; ++i) {
            node_active_cmd_response_pub_->publish(sensor_manager_state_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
          }
        }
      });

  // ToF Msg Subscriber
  tof_sub_ =
    this->create_subscription<robot_custom_msgs::msg::TofData>(
      "/tof_data", rclcpp::SensorDataQoS(),
      [this](robot_custom_msgs::msg::TofData::SharedPtr msg) {
        if (!this->node_active_cmd_) return;
        std::lock_guard<std::mutex> lock(tof_buffer_.mtx);
        tof_buffer_.latest_msg = msg;
        tof_buffer_.receive_time = this->now();
        tof_buffer_.updated.store(true);
      });

  // Camera Msg Subscriber
  camera_sub_ =
    this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
      "/camera_data", rclcpp::SensorDataQoS(),
      [this](robot_custom_msgs::msg::CameraDataArray::SharedPtr msg) {
        if (!this->node_active_cmd_) return;
        std::lock_guard<std::mutex> lock(camera_buffer_.mtx);
        camera_buffer_.latest_msg = msg;
        camera_buffer_.receive_time = this->now();
        camera_buffer_.updated.store(true);
      });

  // Bottom IR Msg Subscriber
  bottom_ir_sub_ =
    this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
      "/bottom_ir_data", rclcpp::SensorDataQoS(),
      [this](robot_custom_msgs::msg::BottomIrData::SharedPtr msg) {
        if (!this->node_active_cmd_) return;
        std::lock_guard<std::mutex> lock(bottom_ir_buffer_.mtx);
        bottom_ir_buffer_.latest_msg = msg;
        bottom_ir_buffer_.receive_time = this->now();
        bottom_ir_buffer_.updated.store(true);
      });

  // Collision Msg Subscriber
  collision_sub_ =
    this->create_subscription<robot_custom_msgs::msg::AbnormalEventData>(
      "/collision_detected", rclcpp::QoS(10).reliable(),
      [this](robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg) {
        if (!this->node_active_cmd_) return;
        std::lock_guard<std::mutex> lock(collision_buffer_.mtx);
        collision_buffer_.latest_msg = msg;
        collision_buffer_.receive_time = this->now();
        collision_buffer_.updated.store(true);
      });

  // Multizone ToF Calibration Cmd Subscriber
  mtof_calibration_cmd_sub_ =
    this->create_subscription<std_msgs::msg::UInt8>(
      "start_tofcalib", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {
        auto calib_cmd_state = static_cast<MToFCalibState>(msg->data);
        mtof_calibrator_->SetCalibrationState(calib_cmd_state);

        switch (calib_cmd_state) {
          case MToFCalibState::kActiveLeft:
          case MToFCalibState::kActiveRight: {
            std::string key = (calib_cmd_state == MToFCalibState::kActiveLeft)
                                  ? "tof_multi_left"
                                  : "tof_multi_right";
            TofSide side = (calib_cmd_state == MToFCalibState::kActiveLeft)
                               ? TofSide::kLeft
                               : TofSide::kRight;
            auto it = converters_.find(key);
            if (it != converters_.end() && it->second) {
              mtof_calibrator_->SetConverter(it->second);
              mtof_calibrator_->SetCalibrationDone(side, false);
            } else {
              RCLCPP_WARN(this->get_logger(),
                          "Failed to find converter for calibration: %s",
                          key.c_str());
              mtof_calibrator_->SetCalibrationState(MToFCalibState::kInactive);
              mtof_calibrator_->SetConverter(nullptr);
            }
            break;
          }
          case MToFCalibState::kInactive:
            mtof_calibrator_->SetCalibrationState(MToFCalibState::kInactive);
            RCLCPP_INFO(this->get_logger(),
                        "multi-ToF Calibration Wrong Cmd : [%d], Set State => "
                        "[%s]",
                        msg->data,
                        EnumToString(mtof_calibrator_->GetCalibrationState())
                            .c_str());
            break;
          default:
            break;
        }
        RCLCPP_INFO(this->get_logger(), "multi-ToF Calibration Cmd : [%s]",
                    EnumToString(mtof_calibrator_->GetCalibrationState())
                        .c_str());
      });

  // PointCloud Publish Timer
  timer_ =
    this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SensorManagerNode::PublishPointcloudTimer, this));

  // Dynamic Parameter Handler (for changing parameters at runtime)
  param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  target_frame_callback_handle_ =
    param_handler_->add_parameter_callback(
      "target_frame", [this](const rclcpp::Parameter& param) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string before = this->node_target_frame_; // before update param
          this->node_target_frame_ = param.as_string(); // after update param
          if (!this->node_target_frame_.empty()) {
            RCLCPP_INFO(this->get_logger(),
                        "[=== Updating target_frame: %s -> %s ===]",
                        before.c_str(), this->node_target_frame_.c_str());
          } else {
            RCLCPP_WARN(this->get_logger(), "target_frame parameter not found!");
          }
        }
      });
}

void SensorManagerNode::LoadConfig() {
  std::string node_params{};
  try {
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("airbot_sensor_manager");
    std::string full_path =
        package_share_directory + "/config/sensor_param.yaml";
    this->config_ =
        YAML::LoadFile(full_path)["airbot_sensor_manager"]["ros__parameters"];
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s",
                 e.what());
    std::string fallback_path =
        "install/airbot_sensor_manager/share/airbot_sensor_manager/config/"
        "sensor_param.yaml";
    this->config_ =
        YAML::LoadFile(fallback_path)["airbot_sensor_manager"]
                      ["ros__parameters"];
  }
}

void SensorManagerNode::Init() {
  mtof_calibrator_->SetCalibrationState(MToFCalibState::kInactive);
  InitializeRuntime();
  InitPublisher(this->config_);
  InitConverters(this->config_["sensors"]);

  self_diagnosis_ = std::make_shared<SelfDiagnosis>(this, converters_);
  self_diagnosis_->RunStartupDiagnosis(this->config_["sensors"]);
}

void SensorManagerNode::InitPublisher(const YAML::Node& config) {
  std::string topic_prefix;
  if (config["output_topic_prefix"] &&
      config["output_topic_prefix"].IsScalar()) {
    topic_prefix = config["output_topic_prefix"].as<std::string>();
  }

  auto create_pc_pub = [this, topic_prefix](const std::string& topic_name) {
    return this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_prefix + topic_name, 10);
  };

  std::ostringstream oss;
  oss << "\n[POINTCLOUD PUBLISHING RATES]\n";

  const YAML::Node& sensor_config = config["sensors"];
  for (const auto& sensor_pair : sensor_config) {
    std::string sensor_name = sensor_pair.first.as<std::string>();
    YAML::Node sensor_config = sensor_pair.second;

    bool is_not_used_publihser =
        (sensor_name == "empty") || (sensor_name == "tof_multi_left") ||
        (sensor_name == "tof_multi_right");
    if (is_not_used_publihser) continue;

    // init publishing rate
    unsigned int rate_ms = 0;
    if (sensor_config["publish_rate_ms"]) {
      const std::string sensor_name = sensor_pair.first.as<std::string>();
      rate_ms = sensor_config["publish_rate_ms"].as<unsigned int>();
      pointcloud_publishing_rate_map_[sensor_name] = rate_ms;
      oss << "  " << sensor_name << " : " << rate_ms << " ms\n";
    }

    // init publisher
    if (sensor_config.IsMap()) {
      bool is_use =
          sensor_config["use"] ? sensor_config["use"].as<bool>() : false;

      if (!is_use) continue;

      if (sensor_config["output_topic_suffix"] &&
          sensor_config["output_topic_suffix"].IsScalar()) {
        std::string base_topic =
            sensor_config["output_topic_suffix"].as<std::string>();
        oss << "    (basic topic name - suffix : " << base_topic << ")\n";
        bool enable_target = sensor_config["enable_target_frame_cloud"]
                                 ? sensor_config["enable_target_frame_cloud"]
                                       .as<bool>()
                                 : true;
        if (enable_target) {
          pointcloud_pubs_[base_topic] = create_pc_pub(base_topic);
        }

        bool enable_tf_cloud = sensor_config["enable_sensor_tf_cloud"]
                                   ? sensor_config["enable_sensor_tf_cloud"]
                                         .as<bool>()
                                   : false;
        if (enable_tf_cloud) {
          std::string local_topic = base_topic + "/local";
          oss << "    (local topic name - suffix : " << local_topic << ")\n";
          pointcloud_pubs_[local_topic] = create_pc_pub(local_topic);
        }
      }
    }
  }
  oss << "----------------------------------------------------";
  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

  bool is_enable_8x8 = sensor_config["tof_multi"]["enable_8x8"]
                           ? sensor_config["tof_multi"]["enable_8x8"].as<bool>()
                           : false;
  if (is_enable_8x8) {
    auto setup_multi_tof = [&](const std::string& side,
                               std::vector<int>& idx_array) {
      const YAML::Node& node = sensor_config[side];
      if (node && node["use"] && node["use"].as<bool>() &&
          node["sub_cell_idx_array"]) {
        std::string base_suffix =
            node["output_idx_topic_suffix"].as<std::string>();
        bool enable_tf_cloud = node["enable_sensor_tf_cloud"]
                                   ? node["enable_sensor_tf_cloud"].as<bool>()
                                   : false;

        for (const auto& idx_node : node["sub_cell_idx_array"]) {
          int index = idx_node.as<int>();
          std::string idx_name = std::to_string(index);

          pointcloud_pubs_[base_suffix + idx_name] =
              create_pc_pub(base_suffix + idx_name);

          if (enable_tf_cloud) {
            std::string local_idx_topic = base_suffix + idx_name + "/local";
            pointcloud_pubs_[local_idx_topic] = create_pc_pub(local_idx_topic);
          }
          idx_array.push_back(index);
        }
      }
    };

    multi_tof_left_sub_cell_idx_array_.clear();
    multi_tof_right_sub_cell_idx_array_.clear();
    setup_multi_tof("tof_multi_left", multi_tof_left_sub_cell_idx_array_);
    setup_multi_tof("tof_multi_right", multi_tof_right_sub_cell_idx_array_);
  }

  // etc Publishers
  // - for notify node status
  node_active_cmd_response_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "sensor_to_pointcloud_active", 10);
  // - for notify completed m-tof calibration result (6 idx) to A1_perception (to
  // make Calibration file)
  mtof_calibration_complete_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "perception/calibration/update", 10);
  // - for notify m-tof calibration status
  mtof_calibration_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
      "tof_calib_state", 10);
  // - for notify m-tof each calibration result (3 idx) to udp_interface (send to
  // Quber SoC)
  mtof_calibration_data_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "tof_calib_data", 10);
}

void SensorManagerNode::InitConverters(const YAML::Node& config) {
  auto pnode =
      std::dynamic_pointer_cast<SensorManagerNode>(this->shared_from_this());

  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  for (const auto& sensor : config) {
    std::string sensor_name = sensor.first.as<std::string>();
    const YAML::Node& sensor_config = sensor.second;

    auto converter = sensor_manager::CloudConverterFactory::Create(
        pnode, sensor_name, sensor_config);
    this->converters_[sensor_name] = converter;

    if (converter != nullptr) {
      auto tf_opt = converter->GetStaticTf();
      if (tf_opt.has_value()) {
        static_transforms.push_back(tf_opt.value());
      }
    }
  }

  if (!static_transforms.empty()) {
    // 1. The sendTransform function accepts a vector argument (this is why
    //    static_transforms is a vector).
    // 2. When called, sendTransform publishes static TF to the dedicated
    //    "/tf_static" topic.
    // 3. The "/tf_static" topic uses Latching QoS. (ROS 2 internally stores
    //    the last message and immediately sends the latest TF data to any
    //    new nodes that subscribe later).
    // 4. Therefore, users do not need to publish static TFs periodically
    //    if sent via sendTransform.
    static_tf_broadcaster_->sendTransform(static_transforms);
    RCLCPP_INFO(this->get_logger(),
                "[InitConverters] Broadcasted %zu static TFs for sensors.",
                static_transforms.size());
  }
}

void SensorManagerNode::InitializeRuntime() {
  this->node_active_cmd_ = false;
  this->tof_buffer_.Reset();
  this->camera_buffer_.Reset();
  this->bottom_ir_buffer_.Reset();
  this->collision_buffer_.Reset();
}

void SensorManagerNode::PublishPointcloudTimer() {
  if ((!this->node_active_cmd_) &&
      (mtof_calibrator_->GetCalibrationState() == MToFCalibState::kInactive)) {
    return;
  }

  auto process_buffer = [&](auto& buffer, auto& msg_copied_out,
                            rclcpp::Time& recv_time_out) -> bool {
    if (buffer.updated.load()) {
      std::lock_guard<std::mutex> lock(buffer.mtx);
      msg_copied_out = std::move(buffer.latest_msg);
      recv_time_out = buffer.receive_time;
      buffer.updated.store(false);
      return (msg_copied_out != nullptr);
    }
    return false;
  };

  tof_buffer_.publishing_cnt_map["tof_mono"] += 10;
  tof_buffer_.publishing_cnt_map["tof_multi"] += 10;
  robot_custom_msgs::msg::TofData::SharedPtr tof_msg_copied;
  rclcpp::Time tof_recv_time;
  if (process_buffer(tof_buffer_, tof_msg_copied, tof_recv_time)) {
    // --- Calibration Interrupt ---
    if (mtof_calibrator_->GetCalibrationState() != MToFCalibState::kInactive) {
      this->RunMultizoneToFCalibration(tof_msg_copied);
      return;
    }

    if (tof_buffer_.publishing_cnt_map["tof_mono"] >=
        pointcloud_publishing_rate_map_["tof_mono"]) {
      PublishPointcloud(SensorType::kTofMono, tof_msg_copied, tof_recv_time);
      tof_buffer_.publishing_cnt_map["tof_mono"] = 0;
    }
    if (tof_buffer_.publishing_cnt_map["tof_multi"] >=
        pointcloud_publishing_rate_map_["tof_multi"]) {
      PublishPointcloud(SensorType::kTofMultiLeft, tof_msg_copied,
                        tof_recv_time);
      PublishPointcloud(SensorType::kTofMultiRight, tof_msg_copied,
                        tof_recv_time);
      tof_buffer_.publishing_cnt_map["tof_multi"] = 0;
    }
  }

  camera_buffer_.publishing_cnt += 10;
  robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copied;
  rclcpp::Time camera_recv_time;
  if (process_buffer(camera_buffer_, camera_msg_copied, camera_recv_time) &&
      (camera_buffer_.publishing_cnt >=
       pointcloud_publishing_rate_map_["camera"])) {
    PublishPointcloud(SensorType::kCamera, camera_msg_copied, camera_recv_time);
    camera_buffer_.publishing_cnt = 0;
  }

  bottom_ir_buffer_.publishing_cnt += 10;
  robot_custom_msgs::msg::BottomIrData::SharedPtr bottom_ir_msg_copied;
  rclcpp::Time bottom_ir_recv_time;
  if (process_buffer(bottom_ir_buffer_, bottom_ir_msg_copied,
                     bottom_ir_recv_time) &&
      (bottom_ir_buffer_.publishing_cnt >=
       pointcloud_publishing_rate_map_["bottom_ir"])) {
    PublishPointcloud(SensorType::kBottomIr, bottom_ir_msg_copied,
                      bottom_ir_recv_time);
    bottom_ir_buffer_.publishing_cnt = 0;
  }

  collision_buffer_.publishing_cnt_map["collision_front"] += 10;
  collision_buffer_.publishing_cnt_map["collision_rear"] += 10;
  robot_custom_msgs::msg::AbnormalEventData::SharedPtr collision_msg_copied;
  rclcpp::Time collision_recv_time;
  if (process_buffer(collision_buffer_, collision_msg_copied,
                     collision_recv_time)) {
    if (collision_buffer_.publishing_cnt_map["collision_front"] >=
            pointcloud_publishing_rate_map_["collision_front"] &&
        collision_msg_copied->event_trigger == 1) {
      PublishPointcloud(SensorType::kCollisionFront, collision_msg_copied,
                        collision_recv_time);
      collision_buffer_.publishing_cnt_map["collision_front"] = 0;
    }
    if (collision_buffer_.publishing_cnt_map["collision_rear"] >=
            pointcloud_publishing_rate_map_["collision_rear"] &&
        collision_msg_copied->event_trigger == -1) {
      PublishPointcloud(SensorType::kCollisionRear, collision_msg_copied,
                        collision_recv_time);
      collision_buffer_.publishing_cnt_map["collision_rear"] = 0;
    }
  }
}

void SensorManagerNode::PublishPointcloud(SensorType sensor_type,
                                          const std::shared_ptr<void> msg_copied,
                                          const rclcpp::Time& receive_time) {
  auto reg_it = sensor_topic_registry_.find(sensor_type);
  if (reg_it == sensor_topic_registry_.end()) {
    RCLCPP_ERROR(this->get_logger(), "SensorType not found in registry.");
    return;
  }
  const auto& config = reg_it->second;
  const std::string& converter_key = config.converter_key;
  const std::string& topic_key = config.topic_key;

  // Check Latency
  unsigned int rate_ms = 0;
  if (sensor_type == SensorType::kTofMono)
    rate_ms = pointcloud_publishing_rate_map_["tof_mono"];
  else if (sensor_type == SensorType::kTofMultiLeft ||
           sensor_type == SensorType::kTofMultiRight)
    rate_ms = pointcloud_publishing_rate_map_["tof_multi"];
  else if (sensor_type == SensorType::kCamera)
    rate_ms = pointcloud_publishing_rate_map_["camera"];
  else if (sensor_type == SensorType::kBottomIr)
    rate_ms = pointcloud_publishing_rate_map_["bottom_ir"];
  else if (sensor_type == SensorType::kCollisionFront)
    rate_ms = pointcloud_publishing_rate_map_["collision_front"];
  else if (sensor_type == SensorType::kCollisionRear)
    rate_ms = pointcloud_publishing_rate_map_["collision_rear"];

  self_diagnosis_->CheckLatency(sensor_type, receive_time, rate_ms);

  auto it = converters_.find(converter_key);
  if (it == converters_.end() || !it->second) {
    RCLCPP_WARN(this->get_logger(),
                "No converter found for key '%s'. Skipping publish.",
                converter_key.c_str());
    return;
  }

  auto cloud_outputs =
      it->second->PcConvert(static_cast<const void*>(msg_copied.get()));

  if (topic_key == "tof/multi/left" || topic_key == "tof/multi/right") {
    this->PublishMultiTofIdxPointcloud(cloud_outputs, topic_key);
    return;
  }

  // 1. Publish Target Frame Cloud
  if (!cloud_outputs.target_frame_clouds.empty()) {
    auto pub_it = pointcloud_pubs_.find(topic_key);
    if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
      pub_it->second->publish(cloud_outputs.target_frame_clouds[0]);
    }
  }

  // 2. Publish Local Frame Cloud
  if (!cloud_outputs.local_frame_clouds.empty()) {
    std::string local_topic_key = topic_key + cloud_outputs.local_topic_suffix;
    auto pub_it = pointcloud_pubs_.find(local_topic_key);
    if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
      pub_it->second->publish(cloud_outputs.local_frame_clouds[0]);
    }
  }
}

void SensorManagerNode::PublishEmptyMsg() {
  for (const auto& [sensor_name, converter] : converters_) {
    if (!converter) continue;

    // 1. Publish for Target Frame
    if (converter->IsEnableTargetFrameCloud()) {
      auto empty_output = converter->PcConvertEmpty(converter->GetTargetFrame());
      if (!empty_output.target_frame_clouds.empty()) {
        const auto& empty_msg = empty_output.target_frame_clouds[0];
        
        auto reg_it = std::find_if(
            sensor_topic_registry_.begin(), sensor_topic_registry_.end(),
            [&](const auto& pair) {
              return pair.second.converter_key == sensor_name;
            });
        
        if (reg_it != sensor_topic_registry_.end()) {
          std::string topic_key = reg_it->second.topic_key;
          
          if (topic_key == "tof/multi/left" || topic_key == "tof/multi/right") {
            std::vector<int> std_sub_cell_idx = (topic_key == "tof/multi/left") ? 
                                                multi_tof_left_sub_cell_idx_array_ :
                                                multi_tof_right_sub_cell_idx_array_;
            for (int idx : std_sub_cell_idx) {
              std::string pub_key = topic_key + "/idx_" + std::to_string(idx);
              auto pub_it = pointcloud_pubs_.find(pub_key);
              if (pub_it != pointcloud_pubs_.end() && pub_it->second &&
                  pub_it->second->get_subscription_count() > 0) {
                pub_it->second->publish(empty_msg);
              }
            }
          } else {
            auto pub_it = pointcloud_pubs_.find(topic_key);
            if (pub_it != pointcloud_pubs_.end() && pub_it->second &&
                pub_it->second->get_subscription_count() > 0) {
              pub_it->second->publish(empty_msg);
            }
          }
        }
      }
    }

    // 2. Publish for Local Frame
    if (converter->IsEnableSensorTfCloud()) {
      auto empty_output = converter->PcConvertEmpty(converter->GetChildFrame());
      if (!empty_output.target_frame_clouds.empty()) {
        const auto& empty_msg = empty_output.target_frame_clouds[0];
        
        auto reg_it = std::find_if(
            sensor_topic_registry_.begin(), sensor_topic_registry_.end(),
            [&](const auto& pair) { return pair.second.converter_key == sensor_name; });
        
        if (reg_it != sensor_topic_registry_.end()) {
          std::string topic_key = reg_it->second.topic_key;
          
          if (topic_key == "tof/multi/left" || topic_key == "tof/multi/right") {
            std::vector<int> std_sub_cell_idx = (topic_key == "tof/multi/left") ? 
                                                multi_tof_left_sub_cell_idx_array_ :
                                                multi_tof_right_sub_cell_idx_array_;
            for (int idx : std_sub_cell_idx) {
              std::string pub_key = topic_key + "/idx_" + std::to_string(idx) + "/local";
              auto pub_it = pointcloud_pubs_.find(pub_key);
              if (pub_it != pointcloud_pubs_.end() && pub_it->second &&
                  pub_it->second->get_subscription_count() > 0) {
                pub_it->second->publish(empty_msg);
              }
            }
          } else {
            std::string local_topic_key = topic_key + "/local";
            auto pub_it = pointcloud_pubs_.find(local_topic_key);
            if (pub_it != pointcloud_pubs_.end() && pub_it->second &&
                pub_it->second->get_subscription_count() > 0) {
              pub_it->second->publish(empty_msg);
            }
          }
        }
      }
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "All Active Publishers published appropriate empty_cloud msgs!");
}

void SensorManagerNode::PublishMultiTofIdxPointcloud(
    const ConverterOutput& output, const std::string& topic_key) {
  const auto& target_clouds = output.target_frame_clouds;
  const auto& local_clouds = output.local_frame_clouds;

  std::vector<int> std_sub_cell_idx =
      (topic_key == "tof/multi/left") ? multi_tof_left_sub_cell_idx_array_
                                      : multi_tof_right_sub_cell_idx_array_;

  for (size_t cloud_idx = 0; cloud_idx < std_sub_cell_idx.size(); ++cloud_idx) {
    int idx = std_sub_cell_idx[cloud_idx];
    std::string idx_name = std::to_string(idx);

    // 1. Publish Target Frame Cloud
    if (cloud_idx < target_clouds.size()) {
      std::string pub_key = (topic_key == "tof/multi/left"
                                 ? "tof/multi/left/idx_"
                                 : "tof/multi/right/idx_") +
                            idx_name;
      auto pub_it = pointcloud_pubs_.find(pub_key);
      if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
        pub_it->second->publish(target_clouds[cloud_idx]);
      }
    }

    // 2. Publish Local Frame Cloud
    if (cloud_idx < local_clouds.size()) {
      std::string local_pub_key = (topic_key == "tof/multi/left"
                                       ? "tof/multi/left/idx_"
                                       : "tof/multi/right/idx_") +
                                  idx_name + "/local";
      auto pub_it = pointcloud_pubs_.find(local_pub_key);
      if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
        pub_it->second->publish(local_clouds[cloud_idx]);
      }
    }
  }
}

TofCalibrationParam SensorManagerNode::LoadMultizoneTofCalibrationParams() {
  TofCalibrationParam cfg;  // default initialized struct

  try {
    // config_["sensors"]["tof_multi"]["calibration"] path check
    if (this->config_["sensors"] && this->config_["sensors"]["tof_multi"] &&
        this->config_["sensors"]["tof_multi"]["calibration"]) {
      const auto& calib_node =
          this->config_["sensors"]["tof_multi"]["calibration"];

      // check existence of each field and assign value (safe parsing)
      if (calib_node["method"])
        cfg.method = calib_node["method"].as<std::string>();

      if (calib_node["sampling_count"])
        cfg.sampling_count = calib_node["sampling_count"].as<int>();

      if (calib_node["pass_min_value"])
        cfg.pass_min_value = calib_node["pass_min_value"].as<float>();

      if (calib_node["pass_max_value"])
        cfg.pass_max_value = calib_node["pass_max_value"].as<float>();

      if (calib_node["pass_diff_th"])
        cfg.pass_diff_th = calib_node["pass_diff_th"].as<float>();

      if (calib_node["time_out_sec"])
        cfg.time_out_sec = calib_node["time_out_sec"].as<float>();

      if (calib_node["data_non_renewal_count"])
        cfg.data_non_renewal_count =
            calib_node["data_non_renewal_count"].as<int>();
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Calibration config path not found. Using default values.");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Error parsing MToF calibration params: %s", e.what());
  }

  return cfg;
}

void SensorManagerNode::RunMultizoneToFCalibration(
    robot_custom_msgs::msg::TofData::SharedPtr tof_msg) {
  MToFCalibResult result = MToFCalibResult::kInactive;
  MToFCalibState state = mtof_calibrator_->GetCalibrationState();
  TofSide side =
      (state == MToFCalibState::kActiveLeft) ? TofSide::kLeft : TofSide::kRight;
  MToFCalibData result_data = MToFCalibData();

  if (state != MToFCalibState::kInactive &&
      !mtof_calibrator_->IsCalibrationDone(side)) {
    result = mtof_calibrator_->Update(result_data, tof_msg, side);
    if (result == MToFCalibResult::kPass) {
      mtof_calibrator_->SetCalibrationDone(side, true);
    }
    std_msgs::msg::UInt8 calib_state_msg;
    calib_state_msg.data = mtof_calibrator_->MakeMTofState(side, result);
    mtof_calibration_state_pub_->publish(calib_state_msg);

    // Send Calib result data to udp_interface (Quber SoC)
    if (result >= MToFCalibResult::kPass) {
      std_msgs::msg::Float32MultiArray msg_arr;
      result_data.SetResult(side, static_cast<float>(result));
      result_data.SetPublishValue(side);
      if (side == TofSide::kLeft) {
        msg_arr.data.assign(result_data.left.pub_data.begin(),
                            result_data.left.pub_data.end());
      } else if (side == TofSide::kRight) {
        msg_arr.data.assign(result_data.right.pub_data.begin(),
                            result_data.right.pub_data.end());
      }
      RCLCPP_INFO(this->get_logger(), "[Publish Data %s] : ",
                  EnumToString(side).c_str());
      mtof_calibration_data_pub_->publish(msg_arr);
    }

    // Send Calib Full-result data to A1_perception (for updating
    // calibration.yaml file)
    if (mtof_calibrator_->IsCalibrationDone(TofSide::kLeft) &&
        mtof_calibrator_->IsCalibrationDone(TofSide::kRight)) {
      std_msgs::msg::Float32MultiArray msg_arr;
      const auto& results = mtof_calibrator_->GetResultArray();
      msg_arr.data.assign(results.begin(), results.end());
      mtof_calibration_complete_pub_->publish(msg_arr);
      RCLCPP_INFO(this->get_logger(),
                  "[Calibration Result: PASS] Publish m-ToF Calibration Data "
                  "to A1_Perception");
      mtof_calibrator_->SetCalibrationDone(TofSide::kLeft, false);
      mtof_calibrator_->SetCalibrationDone(TofSide::kRight, false);
    }
  }
}

}  // namespace sensor_manager
