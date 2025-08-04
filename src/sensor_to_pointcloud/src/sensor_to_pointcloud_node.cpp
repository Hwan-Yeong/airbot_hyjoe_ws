#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

SensorToPointcloudNode::SensorToPointcloudNode() : Node("sensor_to_pointcloud_node")
{
    this->loadConfig();
    this->declare_parameter("target_frame", "map");
    this->get_parameter("target_frame", node_target_frame_);
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", node_target_frame_.c_str());

    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "/camera_data",
        rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::CameraDataArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            latest_camera_msg_ = msg;
            camera_msg_updated_.store(true);
        }
    );

    camera_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pointcloud/camera", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SensorToPointcloudNode::publishPointcloudTimer, this)
    );
}

void SensorToPointcloudNode::loadConfig()
{
    std::string node_params{};
    try
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("sensor_to_pointcloud");
        std::string full_path = package_share_directory + "/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(full_path)["sensor_to_pointcloud"]["ros__parameters"];
    }
    catch (const std::exception& e)
    {
        // fallback (ament_index_cpp::get_package_share_directory()가 제대로 작동하지 않을 경우)
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
        std::string fallback_path = "install/sensor_to_pointcloud/share/sensor_to_pointcloud/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(fallback_path)["sensor_to_pointcloud"]["ros__parameters"];
    }
}

void SensorToPointcloudNode::init()
{
    initConverters(this->config_["sensors"]);
}

void SensorToPointcloudNode::initConverters(const YAML::Node& config)
{
    auto pnode = std::dynamic_pointer_cast<SensorToPointcloudNode>(this->shared_from_this());

    for (const auto& sensor : config) {
        std::string sensor_name = sensor.first.as<std::string>();
        const YAML::Node& sensor_config = sensor.second;
        this->converters_[sensor_name] = sensor_to_pointcloud::CloudConverterFactory::create(pnode, sensor_name, sensor_config);
    }
}

void SensorToPointcloudNode::publishPointcloudTimer()
{
    if (camera_msg_updated_.load()) {
        robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copy;

        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            camera_msg_copy = latest_camera_msg_;
            latest_camera_msg_.reset();
            camera_msg_updated_.store(false);
        }

        if (!camera_msg_copy) {
            RCLCPP_INFO(this->get_logger(), "No latest camera msg");
            return;
        }

        auto it = converters_.find("camera");
        if (it == converters_.end() || !it->second) {
            RCLCPP_INFO(this->get_logger(), "No converter for Camera sensor");
            return;
        }

        auto pc2_msg = it->second->pc_convert(static_cast<const void*>(camera_msg_copy.get()));

        camera_pc_pub_->publish(pc2_msg);
    }
}

} // namespace sensor_to_pointcloud