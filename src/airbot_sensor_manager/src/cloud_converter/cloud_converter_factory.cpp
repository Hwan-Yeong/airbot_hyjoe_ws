#include "cloud_converter/cloud_converter_factory.hpp"

#include "cloud_converter/empty_converter.hpp"
#include "cloud_converter/sensors/bottom_ir.hpp"
#include "cloud_converter/sensors/camera.hpp"
#include "cloud_converter/sensors/tof_mono.hpp"
#include "cloud_converter/sensors/tof_multi.hpp"
#include "cloud_converter/virtual/collision.hpp"

namespace sensor_manager {

CloudConverterPtr CloudConverterFactory::Create(
    std::shared_ptr<SensorManagerNode> node_ptr, const std::string& type,
    const YAML::Node& config) {
  using ConverterCreator =
      std::function<std::shared_ptr<CloudConverterStrategy>()>;
  static const std::unordered_map<std::string, ConverterCreator> factory = {
      {"tof_mono",
       [&]() {
         return std::make_shared<TofMonoCloudConverter>(node_ptr, config);
       }},
      {"tof_multi", [&]() { return nullptr; }},
      {"tof_multi_left",
       [&]() {
         return std::make_shared<TofMultiLeftCloudConverter>(node_ptr, config);
       }},
      {"tof_multi_right",
       [&]() {
         return std::make_shared<TofMultiRightCloudConverter>(node_ptr, config);
       }},
      {"camera",
       [&]() {
         return std::make_shared<CameraCloudConverter>(node_ptr, config);
       }},
      {"bottom_ir",
       [&]() {
         return std::make_shared<BottomIrCloudConverter>(node_ptr, config);
       }},
      {"collision_front",
       [&]() {
         return std::make_shared<CollisionCloudConverter>(node_ptr, config);
       }},
      {"collision_rear",
       [&]() {
         return std::make_shared<CollisionCloudConverter>(node_ptr, config);
       }},
      {"empty", [&]() {
         return std::make_shared<EmptyCloudConverter>(node_ptr, config);
       }}};

  auto it = factory.find(type);
  if (it != factory.end()) {
    return it->second();
  } else {
    throw std::runtime_error("Unknown Sensor type: " + type);
  }
}

CloudConverterPtr CloudConverterFactory::Create(
    std::shared_ptr<SensorManagerNode> node_ptr, const YAML::Node& config) {
  if (!config.IsMap() || config.size() != 1) {
    auto s = YAML::Dump(config);
    throw std::runtime_error("Invalid sensor config format.");
  }

  auto it = config.begin();
  std::string type = it->first.as<std::string>();
  YAML::Node filter_config = it->second;

  return Create(node_ptr, type, filter_config);
}

}  // namespace sensor_manager
