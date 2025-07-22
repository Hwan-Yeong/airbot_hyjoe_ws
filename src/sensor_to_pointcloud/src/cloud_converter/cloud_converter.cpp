#include "cloud_converter/cloud_converter.hpp"

#include "cloud_converter/cloud_converter_factory.hpp"
#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

CloudConverterStrategy::CloudConverterStrategy(std::shared_ptr<SensorToPointcloudNode> node_ptr_) : node_ptr(node_ptr_)
{
}

sensor_msgs::msg::PointCloud2 CameraCloudConverter::pc_convert(const void* sensor_msg)
{
    auto msg = static_cast<const robot_custom_msgs::msg::CameraDataArray*>(sensor_msg);

    sensor_msgs::msg::PointCloud2 pc2;
    // algorithm
    RCLCPP_INFO(this->node_ptr->get_logger(), "num: %d, data size: %zu", msg->num, msg->data_array.size());
    if (msg->data_array.size() > 0) {
        RCLCPP_INFO(this->node_ptr->get_logger(), "id: %d", msg->data_array[0].id);
    }

    return pc2;
}

} // namespace sensor_to_pointcloud
