#include "cloud_converter/empty_converter.hpp"
#include "sensor_manager_node.hpp"

namespace sensor_manager {

EmptyCloudConverter::EmptyCloudConverter(
    SensorManagerNode* node_ptr, const YAML::Node& config)
    : CloudConverterStrategy(node_ptr) {
  if (!config.IsMap()) {
    auto s = YAML::Dump(config);
    throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
  }

  // Load Config
  LoadCommonConfig(config);

  // Print Config
  std::ostringstream oss_empty;
  oss_empty << GetCommonConfigInfo("EMPTY");
  oss_empty << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_empty.str().c_str());
}

ConverterOutput EmptyCloudConverter::PcConvertImpl(const void* sensor_msg) {
  (void)sensor_msg;
  ConverterOutput output;

  if (!this->use_converter_ && sensor_msg == nullptr) return output;

  output.target_frame_clouds.push_back(
      this->pointcloud_generator_.GenerateEmptyPointCloud2Message(
          this->target_frame_));

  return output;
}

}  // namespace sensor_manager