#include "utils/self_diagnosis.hpp"

#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"

namespace sensor_manager {

SelfDiagnosis::SelfDiagnosis(
    rclcpp::Node* node,
    const std::unordered_map<std::string, CloudConverterPtr>& converters)
    : node_(node), converters_(converters) {}

SelfDiagnosis::~SelfDiagnosis() {}

void SelfDiagnosis::CheckLatency(SensorType sensor_type,
                                 const rclcpp::Time& receive_time,
                                 unsigned int publish_rate_ms) {
  if (receive_time.nanoseconds() == 0) {
    return;  // Invalid time
  }

  rclcpp::Time now = node_->now();
  double latency_ms = (now - receive_time).seconds() * 1000.0;
  double threshold_ms = static_cast<double>(publish_rate_ms) * 5.0;

  if (latency_ms > threshold_ms) {
    std::string sensor_name;
    switch (sensor_type) {
      case SensorType::kTofMono:
        sensor_name = "TOF_MONO";
        break;
      case SensorType::kTofMultiLeft:
        sensor_name = "TOF_MULTI_LEFT";
        break;
      case SensorType::kTofMultiRight:
        sensor_name = "TOF_MULTI_RIGHT";
        break;
      case SensorType::kCamera:
        sensor_name = "CAMERA";
        break;
      case SensorType::kBottomIr:
        sensor_name = "BOTTOM_IR";
        break;
      case SensorType::kCollisionFront:
        sensor_name = "COLLISION_FRONT";
        break;
      case SensorType::kCollisionRear:
        sensor_name = "COLLISION_REAR";
        break;
      case SensorType::kDepthCamera:
        sensor_name = "DEPTH_CAMERA";
        break;
      default:
        sensor_name = "UNKNOWN";
        break;
    }

    RCLCPP_WARN(node_->get_logger(),
                "[SelfDiagnosis] High Latency Detected! Sensor: %s, Latency: "
                "%.2f ms (Threshold: %.2f ms)",
                sensor_name.c_str(), latency_ms, threshold_ms);
  }
}

void SelfDiagnosis::RunStartupDiagnosis(const YAML::Node& config) {
  RCLCPP_INFO(node_->get_logger(),
              "[SelfDiagnosis] Starting Startup Self-Diagnosis...");

  if (!config.IsMap()) {
    RCLCPP_WARN(node_->get_logger(),
                "[SelfDiagnosis] Invalid configuration format.");
    return;
  }

  for (const auto& sensor_pair : config) {
    std::string sensor_name = sensor_pair.first.as<std::string>();
    YAML::Node sensor_config = sensor_pair.second;

    // Skip unused sensors or special keys
    if (sensor_name == "empty" || !sensor_config["use"] ||
        !sensor_config["use"].as<bool>()) {
      continue;
    }

    if (sensor_name == "tof_multi")
      continue;  // Skip group config, check individual left/right

    CheckSingleSensor(sensor_name, sensor_config);
  }

  RCLCPP_INFO(node_->get_logger(),
              "[SelfDiagnosis] Startup Self-Diagnosis Completed.");
}

void SelfDiagnosis::CheckSingleSensor(const std::string& sensor_name,
                                      const YAML::Node& sensor_config) {
  (void)sensor_config;  // Unused for now

  auto it = converters_.find(sensor_name);
  if (it == converters_.end() || !it->second) {
    // Some sensors might not have converters, but we check here.
    // Or if the sensor name usage in config doesn't match converter key.
    // based on SensorManagerNode::initConverters, keys match config keys.
    RCLCPP_WARN(node_->get_logger(),
                "[SelfDiagnosis] No converter found for active sensor: %s",
                sensor_name.c_str());
    return;
  }

  std::shared_ptr<void> dummy_data;

  // Determine sensor type by name string (convention from config/code)
  if (sensor_name == "tof_mono") {
    dummy_data = CreateDummyTofData();
  } else if (sensor_name == "tof_multi_left" ||
             sensor_name == "tof_multi_right") {
    dummy_data = CreateDummyTofData();
  } else if (sensor_name == "camera") {
    dummy_data = CreateDummyCameraData();
  } else if (sensor_name == "bottom_ir") {
    dummy_data = CreateDummyBottomIrData();
  } else if (sensor_name == "collision_front" ||
             sensor_name == "collision_rear") {
    dummy_data = CreateDummyCollisionData();
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[SelfDiagnosis] Skipping unknown sensor type: %s",
                sensor_name.c_str());
    return;
  }

  if (dummy_data) {
    try {
      auto output = it->second->PcConvert(dummy_data.get());
      // If pc_convert runs without throwing exception, we assume success.
      // We can check if output is empty, but empty result is also valid
      // A crash or exception would indicate failure.
      RCLCPP_INFO(node_->get_logger(),
                  "[SelfDiagnosis] %s pipeline check. Result: %s",
                  sensor_name.c_str(),
                  (output.target_frame_clouds.empty() &&
                   output.local_frame_clouds.empty()) ? "FAIL" : "PASS");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SelfDiagnosis] %s pipeline check. Result: FAIL. Exception: %s",
                   sensor_name.c_str(), e.what());
    }
  }
}

std::shared_ptr<void> SelfDiagnosis::CreateDummyTofData() {
  auto msg = std::make_shared<robot_custom_msgs::msg::TofData>();
  msg->timestamp = node_->now();
  // Fill with minimal valid data
  // Fixed size arrays [16] are std::array in ROS2 C++
  msg->top = 1.0;
  msg->bot_left.fill(1.0);  // 1.0m
  msg->bot_right.fill(1.0);
  msg->robot_x = 0.0;
  msg->robot_y = 0.0;
  msg->robot_angle = 0.0;

  return std::static_pointer_cast<void>(msg);
}

std::shared_ptr<void> SelfDiagnosis::CreateDummyCameraData() {
  auto msg = std::make_shared<robot_custom_msgs::msg::CameraDataArray>();
  msg->timestamp = node_->now();
  msg->num = 1;
  msg->data_array.resize(1);
  msg->data_array[0].id = 1;
  msg->data_array[0].score = 1.0;
  msg->data_array[0].x = 1.0;
  msg->data_array[0].y = 1.0;
  msg->data_array[0].theta = 10.0;
  msg->data_array[0].width = 0.3;
  msg->data_array[0].height = 0.3;
  msg->data_array[0].distance = 0.3;
  msg->robot_x = 0.0;
  msg->robot_y = 0.0;
  msg->robot_angle = 0.0;
  return std::static_pointer_cast<void>(msg);
}

std::shared_ptr<void> SelfDiagnosis::CreateDummyBottomIrData() {
  auto msg = std::make_shared<robot_custom_msgs::msg::BottomIrData>();
  msg->timestamp = node_->now();
  // Initialize some ADC values
  msg->ff = true;
  msg->fl = true;
  msg->fr = true;
  msg->bb = true;
  msg->bl = true;
  msg->br = true;
  msg->adc_ff = 100;
  msg->adc_fl = 100;
  msg->adc_fr = 100;
  msg->adc_bb = 100;
  msg->adc_bl = 100;
  msg->adc_br = 100;
  msg->robot_x = 0.0;
  msg->robot_y = 0.0;
  msg->robot_angle = 0.0;
  return std::static_pointer_cast<void>(msg);
}

std::shared_ptr<void> SelfDiagnosis::CreateDummyCollisionData() {
  auto msg = std::make_shared<robot_custom_msgs::msg::AbnormalEventData>();
  msg->timestamp = node_->now();
  msg->event_trigger = 1;  // Simulate front collision
  msg->robot_x = 0.0;
  msg->robot_y = 0.0;
  msg->robot_angle = 0.0;
  return std::static_pointer_cast<void>(msg);
}

}  // namespace sensor_manager
