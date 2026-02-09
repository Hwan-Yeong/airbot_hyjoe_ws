#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_manager_node.hpp"

namespace sensor_manager {

SensorManagerNode::SensorManagerNode() : Node("airbot_sensor_to_pointcloud")
{
    this->loadConfig();
    this->declare_parameter("target_frame", "map");
    this->get_parameter("target_frame", node_target_frame_);
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", node_target_frame_.c_str());

    // Initialize Sensor Topic Registry
    sensor_topic_registry_ = {
    //  {SensorType                     ,{ConverterName         ,TopicName}}
        {SensorType::TOF_MONO           ,{"tof_mono"            ,"tof/mono"}}       ,
        {SensorType::TOF_MULTI_LEFT     ,{"tof_multi_left"      ,"tof/multi/left"}} ,
        {SensorType::TOF_MULTI_RIGHT    ,{"tof_multi_right"     ,"tof/multi/right"}},
        {SensorType::CAMERA             ,{"camera"              ,"camera_object"}}  ,
        {SensorType::BOTTOM_IR          ,{"bottom_ir"           ,"bottom_ir"}}      ,
        {SensorType::COLLISION_FRONT    ,{"collision_front"     ,"collision/front"}},
        {SensorType::COLLISION_REAR     ,{"collision_rear"      ,"collision/rear"}}
    };

    // Initialize Multizone ToF Calibrator
    mtof_calibrator_ = std::make_unique<MultizoneTofCalibrator>(
        this->get_logger(),
        this->loadMultizoneTofCalibrationParams()
    );
    RCLCPP_INFO(this->get_logger(), "MultizoneTofCalibrator Initialized.");

    // Sensor Manager On/Off Cmd Subscriber
    sensor_manager_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "cmd_sensor_manager",
        rclcpp::QoS(3).reliable(),
        [this](std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "cmd_sensor_manager topic is a nullptr message.");
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
                initializeRuntime();
                publishEmptyMsg();
                RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : De-Active");
                for (int i=0; i<3; ++i) {
                    node_active_cmd_response_pub_->publish(sensor_manager_state_msg);
                    rclcpp::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
    );

    // ToF Msg Subscriber
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "/tof_data",
        rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::TofData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(tof_buffer_.mtx);
            tof_buffer_.latest_msg = msg;
            tof_buffer_.updated.store(true);
        }
    );

    // Camera Msg Subscriber
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "/camera_data",
        rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::CameraDataArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(camera_buffer_.mtx);
            camera_buffer_.latest_msg = msg;
            camera_buffer_.updated.store(true);
        }
    );

    // Bottom IR Msg Subscriber
    bottom_ir_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "/bottom_ir_data",
        rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::BottomIrData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(bottom_ir_buffer_.mtx);
            bottom_ir_buffer_.latest_msg = msg;
            bottom_ir_buffer_.updated.store(true);
        }
    );

    // Collision Msg Subscriber
    collision_sub_ = this->create_subscription<robot_custom_msgs::msg::AbnormalEventData>(
        "/collision_detected",
        rclcpp::QoS(10).reliable(),
        [this](robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(collision_buffer_.mtx);
            collision_buffer_.latest_msg = msg;
            collision_buffer_.updated.store(true);
        }
    );

    // Multizone ToF Calibration Cmd Subscriber
    mtof_calibration_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "start_tofcalib",
        rclcpp::QoS(10).reliable(),
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            auto calib_cmd_state = static_cast<MTOF_CALIB_STATE>(msg->data);
            mtof_calibrator_->setCalibrationState(calib_cmd_state);

            switch (calib_cmd_state) {
            case MTOF_CALIB_STATE::ACTIVE_LEFT:
            case MTOF_CALIB_STATE::ACTIVE_RIGHT: {
                std::string key = (calib_cmd_state == MTOF_CALIB_STATE::ACTIVE_LEFT) ? "tof_multi_left" : "tof_multi_right";
                TOF_SIDE side = (calib_cmd_state == MTOF_CALIB_STATE::ACTIVE_LEFT) ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT;
                auto it = converters_.find(key);
                if (it != converters_.end() && it->second) {
                    mtof_calibrator_->setConverter(it->second);
                    mtof_calibrator_->setCalibrationDone(side, false);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to find converter for calibration: %s", key.c_str());
                    mtof_calibrator_->setCalibrationState(MTOF_CALIB_STATE::INACTIVE);
                    mtof_calibrator_->setConverter(nullptr);
                }
                break; }
            case MTOF_CALIB_STATE::INACTIVE:
                mtof_calibrator_->setCalibrationState(MTOF_CALIB_STATE::INACTIVE);
                RCLCPP_INFO(this->get_logger(),
                    "multi-ToF Calibration Wrong Cmd : [%d], Set State => [%s]",
                    msg->data, enumToString(mtof_calibrator_->getCalibrationState()).c_str()
                );
                break;
            default:
                break;
            }
            RCLCPP_INFO(this->get_logger(),
                "multi-ToF Calibration Cmd : [%s]",
                enumToString(mtof_calibrator_->getCalibrationState()).c_str()
            );
        }
    );

    // PointCloud Publish Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SensorManagerNode::publishPointcloudTimer, this)
    );

    // Dynamic Parameter Handler (for changing parameters at runtime)
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

void SensorManagerNode::loadConfig()
{
    std::string node_params{};
    try
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("airbot_sensor_manager");
        std::string full_path = package_share_directory + "/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(full_path)["airbot_sensor_manager"]["ros__parameters"];
    }
    catch (const std::exception& e)
    {
        // fallback (ament_index_cpp::get_package_share_directory()가 제대로 작동하지 않을 경우)
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
        std::string fallback_path = "install/airbot_sensor_manager/share/airbot_sensor_manager/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(fallback_path)["airbot_sensor_manager"]["ros__parameters"];
    }
}

void SensorManagerNode::init()
{
    mtof_calibrator_->setCalibrationState(MTOF_CALIB_STATE::INACTIVE);
    initializeRuntime();
    initPublisher(this->config_);
    initConverters(this->config_["sensors"]);
}

void SensorManagerNode::initPublisher(const YAML::Node& config)
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

    std::ostringstream oss;
    oss << "\n[POINTCLOUD PUBLISHING RATES]\n";

    const YAML::Node& sensor_config = config["sensors"];
    for(const auto& sensor_pair : sensor_config) {
        std::string sensor_name = sensor_pair.first.as<std::string>();
        YAML::Node sensor_config = sensor_pair.second;

        bool is_not_used_publihser = (sensor_name == "empty") || (sensor_name == "tof_multi_left") || (sensor_name == "tof_multi_right");
        if (is_not_used_publihser) continue;

        // init publishing rate
        unsigned int rate_ms = 0;
        if (sensor_config["publish_rate_ms"]) {
            const std::string sensor_name = sensor_pair.first.as<std::string>();
            rate_ms = sensor_config["publish_rate_ms"].as<unsigned int>();
            pointcloud_publishing_rate_map_[sensor_name] = rate_ms;
            oss << "  " << sensor_name << " : " << rate_ms << " ms\n";
        }

        // init publisher
        if (sensor_config.IsMap()) {
            bool is_use = sensor_config["use"] ? sensor_config["use"].as<bool>() : false;

            if (!is_use) continue;

            if (sensor_config["output_topic_suffix"] && sensor_config["output_topic_suffix"].IsScalar()) {
                std::string base_topic = sensor_config["output_topic_suffix"].as<std::string>();
                oss << "    (basic topic name - suffix : " << base_topic << ")\n";
                bool enable_target = sensor_config["enable_target_frame_cloud"] ? sensor_config["enable_target_frame_cloud"].as<bool>() : true;
                if (enable_target) {
                    pointcloud_pubs_[base_topic] = create_pc_pub(base_topic);
                }

                bool enable_tf_cloud = sensor_config["enable_sensor_tf_cloud"] ? sensor_config["enable_sensor_tf_cloud"].as<bool>() : false;
                if (enable_tf_cloud) {
                    std::string local_topic = base_topic + "/local";
                    oss << "    (local topic name - suffix : " << local_topic << ")\n";
                    pointcloud_pubs_[local_topic] = create_pc_pub(local_topic);
                }
            }
        }
    }
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    bool is_enable_8x8 = sensor_config["tof_multi"]["enable_8x8"] ? sensor_config["tof_multi"]["enable_8x8"].as<bool>() : false;
    if (is_enable_8x8) {
        auto setup_multi_tof = [&](const std::string& side, std::vector<int>& idx_array) {
            const YAML::Node& node = sensor_config[side];
            if (node && node["use"] && node["use"].as<bool>() && node["sub_cell_idx_array"]) {
                std::string base_suffix = node["output_idx_topic_suffix"].as<std::string>();
                bool enable_tf_cloud = node["enable_sensor_tf_cloud"] ? node["enable_sensor_tf_cloud"].as<bool>() : false;

                for (const auto& idx_node : node["sub_cell_idx_array"]) {
                    int index = idx_node.as<int>();
                    std::string idx_name = std::to_string(index);

                    pointcloud_pubs_[base_suffix + idx_name] = create_pc_pub(base_suffix + idx_name);

                    if (enable_tf_cloud) {
                        std::string local_idx_topic = base_suffix + idx_name + "/local";
                        pointcloud_pubs_[local_idx_topic] = create_pc_pub(local_idx_topic);
                    }
                    idx_array.push_back(index);
                }
            }
        };

        multi_tof_left_sub_cell_idx_array_.clear();
        multi_tof_right_sub_cell_idx_array_.clear();
        setup_multi_tof("tof_multi_left", multi_tof_left_sub_cell_idx_array_);
        setup_multi_tof("tof_multi_right", multi_tof_right_sub_cell_idx_array_);
    }

    // etc Publishers
    // - for notify node status
    node_active_cmd_response_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "sensor_to_pointcloud_active", 10
    );
    // - for notify completed m-tof calibration result (6 idx) to A1_perception (to make Calibration file)
    mtof_calibration_complete_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "perception/calibration/update", 10
    );
    // - for notify m-tof calibration status
    mtof_calibration_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "tof_calib_state", 10
    );
    // - for notify m-tof each calibration result (3 idx) to udp_interface (send to Quber SoC)
    mtof_calibration_data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "tof_calib_data", 10
    );
}

void SensorManagerNode::initConverters(const YAML::Node& config)
{
    auto pnode = std::dynamic_pointer_cast<SensorManagerNode>(this->shared_from_this());

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

    for (const auto& sensor : config) {
        std::string sensor_name = sensor.first.as<std::string>();
        const YAML::Node& sensor_config = sensor.second;

        auto converter = sensor_manager::CloudConverterFactory::create(pnode, sensor_name, sensor_config);
        this->converters_[sensor_name] = converter;

        if (converter != nullptr) {
            auto tf_opt = converter->get_static_tf();
            if (tf_opt.has_value()) {
                static_transforms.push_back(tf_opt.value());
            }
        }
    }

    if (!static_transforms.empty()) {
        // 1. sendTransform 함수는 vector type을 인자로 받음 (static_transforms 변수가 vector 타입인 이유)
        // 2. sendTransform 호출시, "/tf_static" 이라는 전용 토픽으로 static tf 전송
        // 3. /tf_static 토픽은 Latching 방식으로 동작. (ros2 내부에서 마지막 발행 메시지를 보관하고 있다가, 새로운 노드가 나중에 생기더라고 그 노드에게 이전에 발행했던 최신 TF 데이터를 즉시 전달)
        // 4. 즉, static tf 를 sendTrasnform 으로 전송하면, 사용자가 주기적으로 tf를 발행해 줄 필요 없음.
        static_tf_broadcaster_->sendTransform(static_transforms);
        RCLCPP_INFO(this->get_logger(), "[initConverters] Broadcasted %zu static TFs for sensors.", static_transforms.size());
    }
}

void SensorManagerNode::initializeRuntime()
{
    this->node_active_cmd_ = false;
    this->tof_buffer_.reset();
    this->camera_buffer_.reset();
    this->bottom_ir_buffer_.reset();
    this->collision_buffer_.reset();
}

void SensorManagerNode::publishPointcloudTimer()
{
    if ((!this->node_active_cmd_) && (mtof_calibrator_->getCalibrationState() == MTOF_CALIB_STATE::INACTIVE)) {
        return;
    }

    auto process_buffer = [&](auto& buffer, auto& msg_copied_out) -> bool {
        if (buffer.updated.load()) {
            std::lock_guard<std::mutex> lock(buffer.mtx);
            msg_copied_out = std::move(buffer.latest_msg);
            buffer.updated.store(false);
            return (msg_copied_out != nullptr);
        }
        return false;
    };

    tof_buffer_.publishing_cnt_map["tof_mono"] += 10;
    tof_buffer_.publishing_cnt_map["tof_multi"] += 10;
    robot_custom_msgs::msg::TofData::SharedPtr tof_msg_copied;
    if (process_buffer(tof_buffer_, tof_msg_copied)) {
        // --- Calibration Interrupt ---
        if (mtof_calibrator_->getCalibrationState() != MTOF_CALIB_STATE::INACTIVE) {
            this->runMultizoneToFCalibration(tof_msg_copied);
            return;
        }

        if (tof_buffer_.publishing_cnt_map["tof_mono"] >= pointcloud_publishing_rate_map_["tof_mono"]) {
            publishPointcloud(SensorType::TOF_MONO, tof_msg_copied);
            tof_buffer_.publishing_cnt_map["tof_mono"] = 0;
        }
        if (tof_buffer_.publishing_cnt_map["tof_multi"] >= pointcloud_publishing_rate_map_["tof_multi"]) {
            publishPointcloud(SensorType::TOF_MULTI_LEFT, tof_msg_copied);
            publishPointcloud(SensorType::TOF_MULTI_RIGHT, tof_msg_copied);
            tof_buffer_.publishing_cnt_map["tof_multi"] = 0;
        }
    }

    camera_buffer_.publishing_cnt += 10;
    robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copied;
    if (process_buffer(camera_buffer_, camera_msg_copied) &&
        (camera_buffer_.publishing_cnt >= pointcloud_publishing_rate_map_["camera"])) {
        publishPointcloud(SensorType::CAMERA, camera_msg_copied);
        camera_buffer_.publishing_cnt = 0;
    }

    bottom_ir_buffer_.publishing_cnt += 10;
    robot_custom_msgs::msg::BottomIrData::SharedPtr bottom_ir_msg_copied;
    if (process_buffer(bottom_ir_buffer_, bottom_ir_msg_copied) &&
        (bottom_ir_buffer_.publishing_cnt >= pointcloud_publishing_rate_map_["bottom_ir"])) {
        publishPointcloud(SensorType::BOTTOM_IR, bottom_ir_msg_copied);
        bottom_ir_buffer_.publishing_cnt = 0;
    }

    collision_buffer_.publishing_cnt_map["collision_front"] += 10;
    collision_buffer_.publishing_cnt_map["collision_rear"] += 10;
    robot_custom_msgs::msg::AbnormalEventData::SharedPtr collision_msg_copied;
    if (process_buffer(collision_buffer_, collision_msg_copied)) {
        if (collision_buffer_.publishing_cnt_map["collision_front"] >= pointcloud_publishing_rate_map_["collision_front"]
            && collision_msg_copied->event_trigger == 1) {
            publishPointcloud(SensorType::COLLISION_FRONT, collision_msg_copied);
            collision_buffer_.publishing_cnt_map["collision_front"] = 0;
        }
        if (collision_buffer_.publishing_cnt_map["collision_rear"] >= pointcloud_publishing_rate_map_["collision_rear"]
            && collision_msg_copied->event_trigger == -1) {
            publishPointcloud(SensorType::COLLISION_REAR, collision_msg_copied);
            collision_buffer_.publishing_cnt_map["collision_rear"] = 0;
        }
    }
}

void SensorManagerNode::publishPointcloud(SensorType sensor_type, const std::shared_ptr<void> msg_copied)
{
    auto reg_it = sensor_topic_registry_.find(sensor_type);
    if (reg_it == sensor_topic_registry_.end()) {
        RCLCPP_ERROR(this->get_logger(), "SensorType not found in registry.");
        return;
    }
    const auto& config = reg_it->second;
    const std::string& converter_key = config.converter_key;
    const std::string& topic_key = config.topic_key;

    auto it = converters_.find(converter_key);
    if (it == converters_.end() || !it->second) {
        RCLCPP_WARN(this->get_logger(),
            "No converter found for key '%s'. Skipping publish.",
            converter_key.c_str()
        );
        return;
    }

    auto cloud_outputs = it->second->pc_convert(static_cast<const void*>(msg_copied.get()));

    if (topic_key == "tof/multi/left" || topic_key == "tof/multi/right") {
        this->publishMultiTofIdxPointcloud(cloud_outputs, topic_key);
        return;
    }

    // 1. Publish Target Frame Cloud
    if (!cloud_outputs.target_frame_clouds.empty()) {
        auto pub_it = pointcloud_pubs_.find(topic_key);
        if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
            pub_it->second->publish(cloud_outputs.target_frame_clouds[0]);
        }
    }

    // 2. Publish Local Frame Cloud
    if (!cloud_outputs.local_frame_clouds.empty()) {
        std::string local_topic_key = topic_key + cloud_outputs.local_topic_suffix;
        auto pub_it = pointcloud_pubs_.find(local_topic_key);
        if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
            pub_it->second->publish(cloud_outputs.local_frame_clouds[0]);
        }
    }
}

void SensorManagerNode::publishEmptyMsg()
{
    // 현재 "map" target_frame 에 대해서만 publish 되도록 되어있음
    // TODO: 각 converter 의 child frame에 대해서도 publish 되도록 수정
    auto it = converters_.find("empty");
    if (it == converters_.end() || !it->second) {
        RCLCPP_INFO(this->get_logger(), "No converter for empty msg");
        return;
    }

    auto empty_msg_outputs = it->second->pc_convert(nullptr);

    if (!empty_msg_outputs.target_frame_clouds.empty()) {
        for (auto& [name, pub] : pointcloud_pubs_) {
            if (pub && pub->get_subscription_count() > 0) {
                pub->publish(empty_msg_outputs.target_frame_clouds[0]);
            }
            // RCLCPP_INFO(this->get_logger(), "CLEAR: %s", name.c_str());
        }
    }

    RCLCPP_INFO(this->get_logger(), "All Active Publisher publish empty_cloud msgs!");
}

void SensorManagerNode::publishMultiTofIdxPointcloud(const ConverterOutput& output, const std::string& topic_key)
{
    const auto& target_clouds = output.target_frame_clouds;
    const auto& local_clouds = output.local_frame_clouds;

    std::vector<int> std_sub_cell_idx = (topic_key == "tof/multi/left") ? multi_tof_left_sub_cell_idx_array_ : multi_tof_right_sub_cell_idx_array_;

    for (size_t cloud_idx = 0; cloud_idx < std_sub_cell_idx.size(); ++cloud_idx) {
        int idx = std_sub_cell_idx[cloud_idx];
        std::string idx_name = std::to_string(idx);

        // 1. Publish Target Frame Cloud
        if (cloud_idx < target_clouds.size()) {
            std::string pub_key = (topic_key == "tof/multi/left" ? "tof/multi/left/idx_" : "tof/multi/right/idx_") + idx_name;
            auto pub_it = pointcloud_pubs_.find(pub_key);
            if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
                pub_it->second->publish(target_clouds[cloud_idx]);
            }
        }

        // 2. Publish Local Frame Cloud
        if (cloud_idx < local_clouds.size()) {
            std::string local_pub_key = (topic_key == "tof/multi/left" ? "tof/multi/left/idx_" : "tof/multi/right/idx_") + idx_name + "/local";
            auto pub_it = pointcloud_pubs_.find(local_pub_key);
            if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
                pub_it->second->publish(local_clouds[cloud_idx]);
            }
        }
    }
}

tTofCalibrationParam SensorManagerNode::loadMultizoneTofCalibrationParams()
{
    tTofCalibrationParam cfg; // 기본값으로 초기화된 구조체 생성

    try {
        // config_["sensors"]["tof_multi"]["calibration"] 경로 확인
        if (this->config_["sensors"] &&
            this->config_["sensors"]["tof_multi"] &&
            this->config_["sensors"]["tof_multi"]["calibration"])
        {
            const auto& calib_node = this->config_["sensors"]["tof_multi"]["calibration"];

            // 각 필드 존재 여부 확인 후 값 할당 (안전한 파싱)
            if (calib_node["method"])
                cfg.method = calib_node["method"].as<std::string>();

            if (calib_node["sampling_count"])
                cfg.sampling_count = calib_node["sampling_count"].as<int>();

            if (calib_node["pass_min_value"])
                cfg.pass_min_value = calib_node["pass_min_value"].as<float>();

            if (calib_node["pass_max_value"])
                cfg.pass_max_value = calib_node["pass_max_value"].as<float>();

            if (calib_node["pass_diff_th"])
                cfg.pass_diff_th = calib_node["pass_diff_th"].as<float>();

            if (calib_node["time_out_sec"])
                cfg.time_out_sec = calib_node["time_out_sec"].as<float>();

            if (calib_node["data_non_renewal_count"])
                cfg.data_non_renewal_count = calib_node["data_non_renewal_count"].as<int>();
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Calibration config path not found. Using default values.");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing MToF calibration params: %s", e.what());
    }

    return cfg;
}

void SensorManagerNode::runMultizoneToFCalibration(robot_custom_msgs::msg::TofData::SharedPtr tof_msg)
{
    MTOF_CALIB_RESULT result = MTOF_CALIB_RESULT::INACTIVE;
    MTOF_CALIB_STATE state = mtof_calibrator_->getCalibrationState();
    TOF_SIDE side = (state == MTOF_CALIB_STATE::ACTIVE_LEFT) ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT;
    MTOF_CALIB_DATA result_data = MTOF_CALIB_DATA();

    if (state != MTOF_CALIB_STATE::INACTIVE && !mtof_calibrator_->isCalibrationDone(side)) {
        result = mtof_calibrator_->update(result_data, tof_msg, side);
        if (result == MTOF_CALIB_RESULT::PASS) {
            mtof_calibrator_->setCalibrationDone(side, true);
        }
        std_msgs::msg::UInt8 calib_state_msg;
        calib_state_msg.data = mtof_calibrator_->makeMTofState(side, result);
        mtof_calibration_state_pub_->publish(calib_state_msg);

        // Send Calib result data to udp_interface (Quber SoC)
        if (result >= MTOF_CALIB_RESULT::PASS) {
            std_msgs::msg::Float32MultiArray msg_arr;
            result_data.setResult(side, static_cast<float>(result));
            result_data.setPublishValue(side);
            if (side == TOF_SIDE::LEFT) {
                msg_arr.data.assign(result_data.left.pub_data.begin(), result_data.left.pub_data.end());
            } else if (side == TOF_SIDE::RIGHT) {
                msg_arr.data.assign(result_data.right.pub_data.begin(), result_data.right.pub_data.end());
            }
            RCLCPP_INFO(this->get_logger(), "[Publish Data %s] : ", enumToString(side).c_str());
            mtof_calibration_data_pub_->publish(msg_arr);
        }

        // Send Calib Full-result data to A1_perception (for updating calibration.yaml file)
        if (mtof_calibrator_->isCalibrationDone(TOF_SIDE::LEFT) && mtof_calibrator_->isCalibrationDone(TOF_SIDE::RIGHT)) {
            std_msgs::msg::Float32MultiArray msg_arr;
            const auto& results = mtof_calibrator_->getResultArray();
            msg_arr.data.assign(results.begin(), results.end());
            mtof_calibration_complete_pub_->publish(msg_arr);
            RCLCPP_INFO(this->get_logger(), "[Calibration Result: PASS] Publish m-ToF Calibration Data to A1_Perception");
            mtof_calibrator_->setCalibrationDone(TOF_SIDE::LEFT, false);
            mtof_calibrator_->setCalibrationDone(TOF_SIDE::RIGHT, false);
        }
    }
}

} // namespace sensor_manager