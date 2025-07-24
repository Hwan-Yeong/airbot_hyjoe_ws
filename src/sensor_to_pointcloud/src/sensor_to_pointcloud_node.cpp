#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

SensorToPointcloudNode::SensorToPointcloudNode() : Node("sensor_to_pointcloud_node")
{
    this->loadConfig();

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

void SensorToPointcloudNode::loadConfig()
{
    std::string node_params{};
    try
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("sensor_to_pointcloud");
        std::string full_path = package_share_directory + "/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(full_path)["sensor_to_pointcloud"];
    }
    catch (const std::exception& e)
    {
        // fallback (ament_index_cpp::get_package_share_directory()가 제대로 작동하지 않을 경우)
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
        std::string fallback_path = "install/sensor_to_pointcloud/share/sensor_to_pointcloud/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(fallback_path)["sensor_to_pointcloud"];
    }

    // 🟡 config 내용 출력
    // RCLCPP_INFO(this->get_logger(), "===== YAML Config Loaded =====");
    // for (YAML::const_iterator it = this->config_.begin(); it != this->config_.end(); ++it)
    // {
    //     const std::string key = it->first.as<std::string>();
    //     const YAML::Node& value = it->second;

    //     if (value.IsScalar())
    //     {
    //         RCLCPP_INFO(this->get_logger(), "%s : %s", key.c_str(), value.as<std::string>().c_str());
    //     }
    //     else
    //     {
    //         std::stringstream ss;
    //         ss << value;
    //         RCLCPP_INFO(this->get_logger(), "%s :\n%s", key.c_str(), ss.str().c_str());
    //     }
    // }
}

void SensorToPointcloudNode::init()
{
    auto pnode = std::dynamic_pointer_cast<SensorToPointcloudNode>(this->shared_from_this());

    this->converters_["Camera"] = sensor_to_pointcloud::CloudConverterFactory::create(pnode, "camera", this->config_);
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