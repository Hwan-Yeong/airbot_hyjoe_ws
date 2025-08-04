#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_to_pointcloud_node.hpp"

namespace sensor_to_pointcloud {

SensorToPointcloudNode::SensorToPointcloudNode() : Node("sensor_to_pointcloud_node")
{
    this->loadConfig();
    this->declare_parameter("target_frame", "map");
    this->get_parameter("target_frame", node_target_frame_);
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", node_target_frame_.c_str());

    // Sensor Manager On/Off Cmd Subscriber
    sensor_to_pointcloud_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "cmd_sensor_manager",
        rclcpp::QoS(3).reliable(),
        [this](std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "cmd_sensor_to_pointcloud topic is a nullptr message.");
                return;
            }

            this->node_active_cmd_ = msg->data;
            std_msgs::msg::Bool sensor_manager_state_msg;
            sensor_manager_state_msg.data = msg->data;

            if (this->node_active_cmd_){
                RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : Active");
                for (int i=0; i<3; ++i) {
                    node_active_cmd_response_pub_->publish(sensor_manager_state_msg);
                    rclcpp::sleep_for(std::chrono::milliseconds(1));
                }
            } else {
                publishEmptyMsg();
                RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : De-Active");
                for (int i=0; i<3; ++i) {
                    node_active_cmd_response_pub_->publish(sensor_manager_state_msg);
                    rclcpp::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
    );

    // Camera Msg Subscriber
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "/camera_data",
        rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::CameraDataArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            latest_camera_msg_ = msg;
            camera_msg_updated_.store(true);
        }
    );

    // PointCloud Publishers
    camera_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pointcloud/camera", 10
    );

    // etc Publishers
    node_active_cmd_response_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "sensor_to_pointcloud_active", 10
    );

    // PointCloud Publish Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SensorToPointcloudNode::publishPointcloudTimer, this)
    );

    // Dynamic Parameter Handler
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    target_frame_callback_handle_ = param_handler_->add_parameter_callback(
        "target_frame",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                std::string before = this->node_target_frame_;
                this->node_target_frame_ = param.as_string();
                std::string after;
                if (this->get_parameter("target_frame", after)) {
                    RCLCPP_INFO(this->get_logger(), "[=== Updating target_frame: %s -> %s ===]", before.c_str(), after.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "target_frame parameter not found!");
                }
            }
        }
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
    // initializeOnce();
    initializeRuntime();
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

void SensorToPointcloudNode::initializeRuntime()
{
    this->node_active_cmd_ = false;
}

void SensorToPointcloudNode::publishPointcloudTimer()
{
    if (!this->node_active_cmd_) {
        initializeRuntime();
        return;
    }

    if (camera_msg_updated_.load()) {
        robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copy;

        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            camera_msg_copy = latest_camera_msg_;
            latest_camera_msg_.reset();
            camera_msg_updated_.store(false);
        }

        if (!camera_msg_copy) { return; }

        auto it = converters_.find("camera");
        if (it == converters_.end() || !it->second) {
            RCLCPP_INFO(this->get_logger(), "No converter for Camera sensor");
            return;
        }

        auto pc2_msg = it->second->pc_convert(static_cast<const void*>(camera_msg_copy.get()));

        camera_pc_pub_->publish(pc2_msg);
    }
}

void SensorToPointcloudNode::publishEmptyMsg()
{
    auto it = converters_.find("empty");
    if (it == converters_.end() || !it->second) {
        RCLCPP_INFO(this->get_logger(), "No converter for empty msg");
        return;
    }

    auto empty_msg = it->second->pc_convert(nullptr);

    // for (auto& [name, pub] : pointcloud_pubs_) {
    //     if (pub && pub->get_subscription_count() > 0) {
    //         pub->publish(empty_msg);
    //     }
    // }
    camera_pc_pub_->publish(empty_msg);

    RCLCPP_INFO(this->get_logger(), "All Active Publisher publish empty_cloud msgs!");
}

} // namespace sensor_to_pointcloud