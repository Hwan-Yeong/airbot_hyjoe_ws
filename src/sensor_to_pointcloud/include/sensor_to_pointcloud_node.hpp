#pragma once

#include <memory>
#include <unordered_map>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"

#include "cloud_converter/cloud_converter.hpp"
#include "cloud_converter/cloud_converter_factory.hpp"

namespace sensor_to_pointcloud {

using PC2PublisherPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

class SensorToPointcloudNode : public rclcpp::Node
{
public:
    SensorToPointcloudNode();
    void init();
    std::string getTargetFrame() const { return node_target_frame_; }

private:
    void loadConfig();
    void initializeRuntime();
    void initPublisher(const YAML::Node& config);
    void initPublishingRates(const YAML::Node &config);
    void initConverters(const YAML::Node &config);
    void publishPointcloudTimer();
    void publishPointcloud(const std::string& converter_key, const std::string& topic_key, const std::shared_ptr<void> msg_copy);
    void publishEmptyMsg();

    YAML::Node config_;
    std::string node_target_frame_;

    std::unordered_map<std::string, CloudConverterPtr> converters_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_to_pointcloud_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;

    std::unordered_map<std::string, PC2PublisherPtr> pointcloud_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr node_active_cmd_response_pub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> target_frame_callback_handle_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool node_active_cmd_;

    std::unordered_map<std::string, unsigned int> pointcloud_publishing_rate_map_;
    unsigned int camera_publishing_cnt_;

    std::mutex camera_mutex_;
    std::atomic<bool> camera_msg_updated_{false};
    robot_custom_msgs::msg::CameraDataArray::SharedPtr latest_camera_msg_;
};

} // namespace sensor_to_pointcloud