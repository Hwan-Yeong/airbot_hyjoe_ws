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

            if (this->node_active_cmd_) {
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

    // PointCloud Publish Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
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
    initializeRuntime();
    initPublisher(this->config_);
    initPublishingRates(this->config_["sensors"]);
    initConverters(this->config_["sensors"]);
}

void SensorToPointcloudNode::initPublisher(const YAML::Node& config)
{
    std::string topic_prefix;
    if (config["output_topic_prefix"] && config["output_topic_prefix"].IsScalar()) {
        topic_prefix = config["output_topic_prefix"].as<std::string>();
    }

    auto create_pc_pub = [this, topic_prefix](const std::string& topic_name) {
        return this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_prefix + topic_name, 10
        );
    };

    const YAML::Node& sensor_config = config["sensors"];
    for(const auto& sensor_pair : sensor_config) {
        std::string sensor_name = sensor_pair.first.as<std::string>();
        YAML::Node sensor_config = sensor_pair.second;

        bool is_not_used_publihser = (sensor_name == "empty");
        if (is_not_used_publihser) continue;

        if (sensor_config.IsMap()) {
            bool is_use = sensor_config["use"] ? sensor_config["use"].as<bool>() : false;

            if (is_use) {
                if (sensor_config["topic"] && sensor_config["topic"].IsScalar()) {
                    std::string topic_name = sensor_config["topic"].as<std::string>();
                    pointcloud_pubs_[topic_name] = create_pc_pub(topic_name);
                } else {
                    continue;
                }
            }
        }
    }

    bool is_enable_8x8 = sensor_config["tof_multi"]["enable_8x8"] ? sensor_config["tof_multi"]["enable_8x8"].as<bool>() : false;
    if (is_enable_8x8) {
        const YAML::Node& left_node = config["sensors"]["tof_multi_left"];
        const YAML::Node& right_node = config["sensors"]["tof_multi_right"];


        if (left_node && left_node["use"] && left_node["sub_cell_idx_array"] && left_node["sub_cell_idx_array"].IsSequence()) {
            bool use_left = false;
            try { use_left = left_node["use"].as<bool>(); } catch(...) { use_left = false; }
            if (use_left) {
                auto topic_idx = left_node["topic_idx"].as<std::string>();
                for (const auto& idx_node : left_node["sub_cell_idx_array"]) {
                    int index = idx_node.as<int>();
                    pointcloud_pubs_[topic_idx + std::to_string(index)] = create_pc_pub(topic_idx + std::to_string(index));
                }
            }
        }

        if (right_node && right_node["use"] && right_node["sub_cell_idx_array"] && right_node["sub_cell_idx_array"].IsSequence()) {
            bool use_right = false;
            try { use_right = right_node["use"].as<bool>(); } catch(...) { use_right = false; }
            if (use_right) {
                auto topic_idx = right_node["topic_idx"].as<std::string>();
                for (const auto& idx_node : right_node["sub_cell_idx_array"]) {
                    int index = idx_node.as<int>();
                    pointcloud_pubs_[topic_idx + std::to_string(index)] = create_pc_pub(topic_idx + std::to_string(index));
                }
            }
        }
    }

    // etc Publishers
    node_active_cmd_response_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "sensor_to_pointcloud_active", 10
    );
}

void SensorToPointcloudNode::initPublishingRates(const YAML::Node& config)
{
    if (!config.IsMap()) {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid sensor config format.");
        return;
    }

    std::ostringstream oss;
    oss << "\n==== POINTCLOUD CONVERTER PUBLISHING RATE PARAMETERS ====\n";

    for (const auto& sensor_pair : config) {
        unsigned int rate_ms = 0;
        const YAML::Node& sensor_node = sensor_pair.second;
        if (sensor_node["publish_rate_ms"]) {
            const std::string sensor_name = sensor_pair.first.as<std::string>();
            rate_ms = sensor_node["publish_rate_ms"].as<unsigned int>();
            pointcloud_publishing_rate_map_[sensor_name] = rate_ms;
            oss << "  " << sensor_name << " : " << rate_ms << " ms\n";
        }
    }
    oss <<   "==========================================================";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
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
    this->camera_publishing_cnt_ = 0;
}

void SensorToPointcloudNode::publishPointcloudTimer()
{
    if (!this->node_active_cmd_) {
        initializeRuntime();
        return;
    }

    camera_publishing_cnt_ += 10;
    if (camera_msg_updated_.load() && (camera_publishing_cnt_ >= pointcloud_publishing_rate_map_["camera"])) {
        robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copy;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            camera_msg_copy = latest_camera_msg_;
            latest_camera_msg_.reset();
            camera_msg_updated_.store(false);
        }
        if (!camera_msg_copy) {
            return;
        }
        publishPointcloud("camera", "camera_object", camera_msg_copy);
        camera_publishing_cnt_ = 0;
    }
}

/**
 * @brief 센서 데이터 변환 후 메시지 퍼블리싱하는 함수
 * @param converter_key: pointcloud 변환기 식별 기 (string)
 * @param topic_key: 발행할 토픽명 (string)
 * @param msg_copy: 변환할 센서 raw data
 */
void SensorToPointcloudNode::publishPointcloud(const std::string& converter_key, const std::string& topic_key, const std::shared_ptr<void> msg_copy)
{
    auto it = converters_.find(converter_key);
    if (it == converters_.end() || !it->second) {
        RCLCPP_WARN(this->get_logger(),
            "No converter found for key '%s'. Skipping publish.",
            converter_key.c_str()
        );
        return;
    }

    auto pc2_msg = it->second->pc_convert(static_cast<const void*>(msg_copy.get()));

    auto pub_it = pointcloud_pubs_.find(topic_key);
    if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
        pub_it->second->publish(pc2_msg);
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Publisher for topic key '%s' not found. Skipping publish.",
            topic_key.c_str()
        );
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

    for (auto& [name, pub] : pointcloud_pubs_) {
        if (pub && pub->get_subscription_count() > 0) {
            pub->publish(empty_msg);
        }
    }

    RCLCPP_INFO(this->get_logger(), "All Active Publisher publish empty_cloud msgs!");
}

} // namespace sensor_to_pointcloud