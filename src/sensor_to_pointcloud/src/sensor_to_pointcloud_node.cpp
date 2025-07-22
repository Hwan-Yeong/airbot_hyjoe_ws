#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

SensorToPointcloudNode::SensorToPointcloudNode()
: Node("sensor_to_pointcloud_node")
{
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "/camera_data",
        rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::CameraDataArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            latest_camera_msg_ = msg;
        }
    );

    camera_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pointcloud/camera", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SensorToPointcloudNode::publishPointcloudTimer, this)
    );
}

void SensorToPointcloudNode::init(std::shared_ptr<SensorToPointcloudNode> self)
{
    converters_["Camera"] = sensor_to_pointcloud::CloudConverterFactory::create(SensorType::Camera, self);
}

void SensorToPointcloudNode::publishPointcloudTimer()
{
    robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copy;

    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_msg_copy = latest_camera_msg_;
    }

    if (!camera_msg_copy) {
        RCLCPP_DEBUG(this->get_logger(), "No latest camera msg");
        return;
    }

    // 변환기 조회 및 변환 수행
    auto it = converters_.find("Camera");
    if (it == converters_.end() || !it->second) {
        RCLCPP_WARN(this->get_logger(), "No converter for Camera sensor");
        return;
    }

    auto pc2_msg = it->second->pc_convert(static_cast<const void*>(camera_msg_copy.get()));

    // PointCloud2 퍼블리시
    camera_pc_pub_->publish(pc2_msg);
}

} // namespace sensor_to_pointcloud