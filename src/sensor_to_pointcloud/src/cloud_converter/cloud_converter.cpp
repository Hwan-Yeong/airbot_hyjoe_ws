#include "cloud_converter/cloud_converter.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

CloudConverterStrategy::CloudConverterStrategy(std::shared_ptr<SensorToPointcloudNode> node_ptr_) : node_ptr(node_ptr_)
{
}

std::shared_ptr<SensorToPointcloudNode> CloudConverterStrategy::getNodePtr() const
{
    return node_ptr;
}

CameraCloudConverter::CameraCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node &config)
    : CloudConverterStrategy(node_ptr_)
{
    if (!config.IsMap())
    {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
    }

    this->use_camera_ = config["use"].as<bool>();
    this->pointcloud_resolution_ = config["pointcloud_resolution"].as<double>();

    RCLCPP_INFO(this->node_ptr->get_logger(), "=== Complete to Set Camera PointCloud Converter! ===");
    RCLCPP_INFO(this->node_ptr->get_logger(),
        "use_camera_: %d, pointcloud_resolution_: %.2f",
        this->use_camera_, this->pointcloud_resolution_
    );
}

sensor_msgs::msg::PointCloud2 CameraCloudConverter::pc_convert(const void *sensor_msg)
{
    auto msg = static_cast<const robot_custom_msgs::msg::CameraDataArray*>(sensor_msg);

    sensor_msgs::msg::PointCloud2 pc2;

    return pc2;
}

} // namespace sensor_to_pointcloud
