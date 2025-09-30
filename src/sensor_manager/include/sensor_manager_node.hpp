#pragma once

#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"

#include "cloud_converter/cloud_converter.hpp"
#include "cloud_converter/cloud_converter_factory.hpp"

namespace sensor_manager {

using PC2PublisherPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

class SensorManagerNode : public rclcpp::Node
{
public:
    SensorManagerNode();
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
    void publishMultiTofIdxPointcloud(const PointCloudMsgVector& clouds, const std::string& topic_key);

    YAML::Node config_;
    std::string node_target_frame_;

    std::unordered_map<std::string, CloudConverterPtr> converters_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr bottom_ir_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_sub_;

    std::unordered_map<std::string, PC2PublisherPtr> pointcloud_pubs_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr node_active_cmd_response_pub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> target_frame_callback_handle_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool node_active_cmd_;

    std::unordered_map<std::string, unsigned int> pointcloud_publishing_rate_map_;

    std::vector<int> multi_tof_left_sub_cell_idx_array_;
    std::vector<int> multi_tof_right_sub_cell_idx_array_;

    tSensorBuffer<robot_custom_msgs::msg::TofData> tof_buffer_;
    tSensorBuffer<robot_custom_msgs::msg::CameraDataArray> camera_buffer_;
    tSensorBuffer<robot_custom_msgs::msg::BottomIrData> bottom_ir_buffer_;
    tSensorBuffer<robot_custom_msgs::msg::AbnormalEventData> collision_buffer_;
};

} // namespace sensor_manager