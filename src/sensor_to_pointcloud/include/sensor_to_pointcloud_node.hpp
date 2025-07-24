#pragma once

#include <memory>
#include <unordered_map>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "robot_custom_msgs/msg/camera_data_array.hpp"

#include "cloud_converter/cloud_converter.hpp"
#include "cloud_converter/cloud_converter_factory.hpp"

namespace sensor_to_pointcloud {

class SensorToPointcloudNode : public rclcpp::Node
{
public:
    SensorToPointcloudNode();
    void init();

private:
    void loadConfig();
    void publishPointcloudTimer();
    void initConverters(const YAML::Node &config);

    YAML::Node config_;

    std::unordered_map<std::string, CloudConverterPtr> converters_;

    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Publisher<PointCloudMsg>::SharedPtr camera_pc_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex camera_mutex_;
    robot_custom_msgs::msg::CameraDataArray::SharedPtr latest_camera_msg_;
};

} // namespace sensor_to_pointcloud