#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_manager_node.hpp"

#include "cloud_converter/empty_converter.hpp"
#include "cloud_converter/sensors/bottom_ir.hpp"
#include "cloud_converter/sensors/camera.hpp"
#include "cloud_converter/sensors/depth_camera.hpp"
#include "cloud_converter/sensors/tof_mono.hpp"
#include "cloud_converter/sensors/tof_multi.hpp"
#include "cloud_converter/virtual/collision.hpp"

namespace sensor_manager {

CloudConverterPtr CloudConverterFactory::Create(
    SensorManagerNode* node_ptr, const std::string& type,
    const YAML::Node& config) {
  using ConverterCreator = std::function<CloudConverterPtr(
      SensorManagerNode*, const YAML::Node&)>;

  static const std::unordered_map<std::string, ConverterCreator> factory = {
      {"tof_mono", [](auto n, auto c) {
         return std::make_shared<TofMonoCloudConverter>(n, c);
       }},
      // {"tof_multi", [&]() { return nullptr; }},
      {"tof_multi_left", [](auto n, auto c) {
         return std::make_shared<TofMultiLeftCloudConverter>(n, c);
       }},
      {"tof_multi_right", [](auto n, auto c) {
         return std::make_shared<TofMultiRightCloudConverter>(n, c);
       }},
      {"camera", [](auto n, auto c) {
         return std::make_shared<CameraCloudConverter>(n, c);
       }},
      {"bottom_ir", [](auto n, auto c) {
         return std::make_shared<BottomIrCloudConverter>(n, c);
       }},
      {"depth_camera", [](auto n, auto c) {
         return std::make_shared<DepthCameraCloudConverter>(n, c);
       }},
      {"collision_front", [](auto n, auto c) {
         return std::make_shared<CollisionCloudConverter>(n, c);
       }},
      {"collision_rear", [](auto n, auto c) {
         return std::make_shared<CollisionCloudConverter>(n, c);
       }},
      {"empty", [](auto n, auto c) {
         return std::make_shared<EmptyCloudConverter>(n, c);
       }}};

  auto it = factory.find(type);
  if (it != factory.end()) {
    return it->second(node_ptr, config);
  } else {
    RCLCPP_WARN(node_ptr->get_logger(),
      "Unknown Sensor type: %s", type.c_str());
    return nullptr;
    // throw std::runtime_error("Unknown Sensor type: " + type);
  }
}

CloudConverterPtr CloudConverterFactory::Create(
    SensorManagerNode* node_ptr, const YAML::Node& config) {
  if (!config.IsMap() || config.size() != 1) {
    throw std::runtime_error("Invalid sensor config format.");
  }

  auto it = config.begin();
  std::string type = it->first.as<std::string>();
  YAML::Node filter_config = it->second;

  return Create(node_ptr, type, filter_config);
}

}  // namespace sensor_manager
