#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_manager_node.hpp"

namespace sensor_manager {

SensorManagerNode::SensorManagerNode() : Node("sensor_manager_node")
{
    this->loadConfig();
    this->declare_parameter("target_frame", "map");
    this->get_parameter("target_frame", node_target_frame_);
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", node_target_frame_.c_str());

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

            if (calib_cmd_state == MTOF_CALIB_STATE::ACTIVE_LEFT || calib_cmd_state == MTOF_CALIB_STATE::ACTIVE_RIGHT) {
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
            } else {
                mtof_calibrator_->setCalibrationState(MTOF_CALIB_STATE::INACTIVE);
                RCLCPP_INFO(this->get_logger(),
                    "multi-ToF Calibration Wrong Cmd : [%d], Set State => [%s]",
                    msg->data, enumToString(mtof_calibrator_->getCalibrationState()).c_str()
                );
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

    // Dynamic Parameter Handler (for changing parameters in run-time)
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
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("sensor_manager");
        std::string full_path = package_share_directory + "/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(full_path)["sensor_manager"]["ros__parameters"];
    }
    catch (const std::exception& e)
    {
        // fallback (ament_index_cpp::get_package_share_directory()가 제대로 작동하지 않을 경우)
        RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
        std::string fallback_path = "install/sensor_manager/share/sensor_manager/config/sensor_param.yaml";
        this->config_ = YAML::LoadFile(fallback_path)["sensor_manager"]["ros__parameters"];
    }
}

void SensorManagerNode::init()
{
    mtof_calibrator_->setCalibrationState(MTOF_CALIB_STATE::INACTIVE);
    initializeRuntime();
    initPublisher(this->config_);
    initPublishingRates(this->config_["sensors"]);
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

    const YAML::Node& sensor_config = config["sensors"];
    for(const auto& sensor_pair : sensor_config) {
        std::string sensor_name = sensor_pair.first.as<std::string>();
        YAML::Node sensor_config = sensor_pair.second;

        bool is_not_used_publihser = (sensor_name == "empty") || (sensor_name == "tof_multi_left") || (sensor_name == "tof_multi_right");
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

        multi_tof_left_sub_cell_idx_array_.clear();
        multi_tof_right_sub_cell_idx_array_.clear();

        if (left_node && left_node["use"] && left_node["sub_cell_idx_array"] && left_node["sub_cell_idx_array"].IsSequence()) {
            bool use_left = false;
            try { use_left = left_node["use"].as<bool>(); } catch(...) { use_left = false; }
            if (use_left) {
                auto topic_idx = left_node["topic_idx"].as<std::string>();
                for (const auto& idx_node : left_node["sub_cell_idx_array"]) {
                    int index = idx_node.as<int>();
                    pointcloud_pubs_[topic_idx + std::to_string(index)] = create_pc_pub(topic_idx + std::to_string(index));
                    multi_tof_left_sub_cell_idx_array_.push_back(index);
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
                    multi_tof_right_sub_cell_idx_array_.push_back(index);
                }
            }
        }
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

void SensorManagerNode::initPublishingRates(const YAML::Node& config)
{
    if (!config.IsMap()) {
        auto s = YAML::Dump(config);
        throw std::runtime_error("Invalid sensor config format.");
        return;
    }

    std::ostringstream oss;
    oss << "\n[POINTCLOUD PUBLISHING RATES]\n";

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
    oss << "----------------------------------------------------";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void SensorManagerNode::initConverters(const YAML::Node& config)
{
    auto pnode = std::dynamic_pointer_cast<SensorManagerNode>(this->shared_from_this());

    for (const auto& sensor : config) {
        std::string sensor_name = sensor.first.as<std::string>();
        const YAML::Node& sensor_config = sensor.second;
        this->converters_[sensor_name] = sensor_manager::CloudConverterFactory::create(pnode, sensor_name, sensor_config);
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

    tof_buffer_.publishing_cnt_map["tof_mono"] += 10;
    tof_buffer_.publishing_cnt_map["tof_multi"] += 10;
    if (tof_buffer_.updated.load()) {
        robot_custom_msgs::msg::TofData::SharedPtr tof_msg_copied;
        {
            std::lock_guard<std::mutex> lock(tof_buffer_.mtx);
            tof_msg_copied = std::move(tof_buffer_.latest_msg); // 참조카운트 변경 X, 소유권 이동
            tof_buffer_.updated.store(false);
        }
        if (!tof_msg_copied) {
            return;
        }

        // --- Calibration Interrupt ---
        if (mtof_calibrator_->getCalibrationState() != MTOF_CALIB_STATE::INACTIVE) {
            this->runMultizoneToFCalibration(tof_msg_copied);
            return;
        }
        // -----------------------------

        if (tof_buffer_.publishing_cnt_map["tof_mono"] >= pointcloud_publishing_rate_map_["tof_mono"]) {
            publishPointcloud("tof_mono", "tof/mono", tof_msg_copied);
            tof_buffer_.publishing_cnt_map["tof_mono"] = 0;
        }
        if (tof_buffer_.publishing_cnt_map["tof_multi"] >= pointcloud_publishing_rate_map_["tof_multi"]) {
            publishPointcloud("tof_multi_left", "tof/multi/left", tof_msg_copied);
            publishPointcloud("tof_multi_right", "tof/multi/right", tof_msg_copied);
            tof_buffer_.publishing_cnt_map["tof_multi"] = 0;
        }
    }

    camera_buffer_.publishing_cnt += 10;
    if (camera_buffer_.updated.load() && (camera_buffer_.publishing_cnt >= pointcloud_publishing_rate_map_["camera"])) {
        robot_custom_msgs::msg::CameraDataArray::SharedPtr camera_msg_copied;
        {
            std::lock_guard<std::mutex> lock(camera_buffer_.mtx);
            camera_msg_copied = std::move(camera_buffer_.latest_msg);
            camera_buffer_.updated.store(false);
        }
        if (!camera_msg_copied) {
            return;
        }
        publishPointcloud("camera", "camera_object", camera_msg_copied);
        camera_buffer_.publishing_cnt = 0;
    }

    bottom_ir_buffer_.publishing_cnt += 10;
    if (bottom_ir_buffer_.updated.load() && (bottom_ir_buffer_.publishing_cnt >= pointcloud_publishing_rate_map_["bottom_ir"])) {
        robot_custom_msgs::msg::BottomIrData::SharedPtr bottom_ir_msg_copied;
        {
            std::lock_guard<std::mutex> lock(bottom_ir_buffer_.mtx);
            bottom_ir_msg_copied = std::move(bottom_ir_buffer_.latest_msg);
            bottom_ir_buffer_.updated.store(false);
        }
        if (!bottom_ir_msg_copied) {
            return;
        }
        publishPointcloud("bottom_ir", "bottom_ir", bottom_ir_msg_copied);
        bottom_ir_buffer_.publishing_cnt = 0;
    }

    collision_buffer_.publishing_cnt_map["collision_front"] += 10;
    collision_buffer_.publishing_cnt_map["collision_rear"] += 10;
    if (collision_buffer_.updated.load()) {
        robot_custom_msgs::msg::AbnormalEventData::SharedPtr collision_msg_copied;
        {
            std::lock_guard<std::mutex> lock(collision_buffer_.mtx);
            collision_msg_copied = std::move(collision_buffer_.latest_msg);
            collision_buffer_.updated.store(false);
        }
        if (!collision_msg_copied) {
            return;
        }
        if (collision_buffer_.publishing_cnt_map["collision_front"] >= pointcloud_publishing_rate_map_["collision_front"]
            && collision_msg_copied->event_trigger == 1) {
            publishPointcloud("collision_front", "collision/front", collision_msg_copied);
            collision_buffer_.publishing_cnt_map["collision"] = 0;
        }
        if (collision_buffer_.publishing_cnt_map["collision_rear"] >= pointcloud_publishing_rate_map_["collision_rear"]
            && collision_msg_copied->event_trigger == -1) {
            publishPointcloud("collision_rear", "collision/rear", collision_msg_copied);
            collision_buffer_.publishing_cnt_map["collision_rear"] = 0;
        }
    }
}

/**
 * @brief 센서 데이터 변환 후 메시지 퍼블리싱하는 함수
 * @param converter_key: pointcloud 변환기 식별 기 (string)
 * @param topic_key: 발행할 토픽명 (string)
 * @param msg_copied: 변환할 센서 raw data
 */
void SensorManagerNode::publishPointcloud(const std::string& converter_key, const std::string& topic_key, const std::shared_ptr<void> msg_copied)
{
    auto it = converters_.find(converter_key);
    if (it == converters_.end() || !it->second) {
        RCLCPP_WARN(this->get_logger(),
            "No converter found for key '%s'. Skipping publish.",
            converter_key.c_str()
        );
        return;
    }

    auto clouds = it->second->pc_convert(static_cast<const void*>(msg_copied.get()));

    if (topic_key == "tof/multi/left" || topic_key == "tof/multi/right") { 
        this->publishMultiTofIdxPointcloud(clouds, topic_key);
        return;
    }

    auto pub_it = pointcloud_pubs_.find(topic_key);
    if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
        for (auto& cloud : clouds) {
            pub_it->second->publish(cloud);
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Publisher for topic key '%s' not found. Skipping publish.",
            topic_key.c_str()
        );
    }
}

void SensorManagerNode::publishEmptyMsg()
{
    auto it = converters_.find("empty");
    if (it == converters_.end() || !it->second) {
        RCLCPP_INFO(this->get_logger(), "No converter for empty msg");
        return;
    }

    auto empty_msg = it->second->pc_convert(nullptr);

    for (auto& [name, pub] : pointcloud_pubs_) {
        if (pub && pub->get_subscription_count() > 0) {
            pub->publish(empty_msg[0]);
        }
    }

    RCLCPP_INFO(this->get_logger(), "All Active Publisher publish empty_cloud msgs!");
}

void SensorManagerNode::publishMultiTofIdxPointcloud(const PointCloudMsgVector& clouds, const std::string& topic_key)
{
    std::vector<int> std_sub_cell_idx = (topic_key == "tof/multi/left") ? multi_tof_left_sub_cell_idx_array_ : multi_tof_right_sub_cell_idx_array_;

    size_t cloud_idx = 0;
    for (auto& idx : std_sub_cell_idx) {
        std::string pub_key = topic_key + "/idx_" + std::to_string(idx);

        auto pub_it = pointcloud_pubs_.find(pub_key);
        if (pub_it != pointcloud_pubs_.end() && pub_it->second) {
            if (cloud_idx < clouds.size()) {
                pub_it->second->publish(clouds[cloud_idx]);
            } else {
                RCLCPP_WARN(this->get_logger(), "No cloud[%zu] available for '%s'", cloud_idx, pub_key.c_str());
            }
        }
        cloud_idx++;
    }
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

        if ((side == TOF_SIDE::LEFT && result >= MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE) ||
            (side == TOF_SIDE::RIGHT && mtof_calibrator_->isCalibrationDone(TOF_SIDE::LEFT) && result >= MTOF_CALIB_RESULT::PASS)) {
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

        if (mtof_calibrator_->isCalibrationDone(TOF_SIDE::LEFT) && mtof_calibrator_->isCalibrationDone(TOF_SIDE::RIGHT)) {
            std_msgs::msg::Float32MultiArray msg_arr;
            msg_arr.data.assign(mtof_calibrator_->getResultArray().begin(), mtof_calibrator_->getResultArray().end());
            mtof_calibration_complete_pub_->publish(msg_arr);
            RCLCPP_INFO(this->get_logger(), "[Calibration Result: PASS] Publish m-ToF Calibration Data to A1_Perception");
            mtof_calibrator_->setCalibrationDone(TOF_SIDE::LEFT, false);
            mtof_calibrator_->setCalibrationDone(TOF_SIDE::RIGHT, false);
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

} // namespace sensor_manager