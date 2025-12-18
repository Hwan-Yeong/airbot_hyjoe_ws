#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_manager_node.hpp"

namespace sensor_manager {

SensorManagerNode::SensorManagerNode() : Node("sensor_manager_node")
{
    this->loadConfig();
    this->declare_parameter("target_frame", "map");
    this->get_parameter("target_frame", node_target_frame_);
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", node_target_frame_.c_str());

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
            // start_left_calibration = 0x01, start_right_calibration = 0x02, unknown = else,
            isActiveMToFCalibration = static_cast<MTOF_CALIB_STATE>(msg->data);
            if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT || isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT) {
                if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT) {
                    bLeftMToFCalibrationSet = false;
                    calib_session_.reset();
                } else { // MTOF_CALIB_STATE::ACTIVE_RIGHT
                    bRightMToFCalibrationSet = false;
                    calib_session_.reset();
                }
                RCLCPP_INFO(this->get_logger(), "multi-ToF Calibration Cmd : [%s]", enumToString(isActiveMToFCalibration).c_str());
            } else {
                isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
                RCLCPP_INFO(this->get_logger(), "multi-ToF Calibration Wrong Cmd : [%d], Set State => [%s]", msg->data, enumToString(isActiveMToFCalibration).c_str());
            }
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
    isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
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
    if ((!this->node_active_cmd_) && (isActiveMToFCalibration == MTOF_CALIB_STATE::INACTIVE)) {
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
        if (isActiveMToFCalibration != MTOF_CALIB_STATE::INACTIVE) {
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
    MTOF_CALIB_RESULT left_result = MTOF_CALIB_RESULT::INACTIVE;
    MTOF_CALIB_RESULT right_result = MTOF_CALIB_RESULT::INACTIVE;
    MTOF_CALIB_DATA result_data = MTOF_CALIB_DATA();


    if ((!bLeftMToFCalibrationSet || !bRightMToFCalibrationSet) &&
        (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT || isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT)) {

        if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT && !bLeftMToFCalibrationSet) {
            left_result = handleCalibrationSide(result_data, tof_msg, TOF_SIDE::LEFT, bLeftMToFCalibrationSet);
        }

        if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT && !bRightMToFCalibrationSet) {
            right_result = handleCalibrationSide(result_data, tof_msg, TOF_SIDE::RIGHT, bRightMToFCalibrationSet);
        }

        if ((left_result >= MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE) ||
        ((left_result == MTOF_CALIB_RESULT::PASS) && (right_result >= MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE)) ||
        ((left_result == MTOF_CALIB_RESULT::PASS) && (right_result == MTOF_CALIB_RESULT::PASS))) {
            std_msgs::msg::Float32MultiArray left_msg_arr;
            result_data.setResult(TOF_SIDE::LEFT, static_cast<float>(left_result));
            result_data.setPublishValue(TOF_SIDE::LEFT);
            left_msg_arr.data.assign(result_data.left.pub_data.begin(), result_data.left.pub_data.end());
            RCLCPP_INFO(this->get_logger(), "[Publish Data L] : "); /////////////////////////////
            mtof_calibration_data_pub_->publish(left_msg_arr);

            std_msgs::msg::Float32MultiArray right_msg_arr;
            result_data.setResult(TOF_SIDE::RIGHT, static_cast<float>(right_result));
            result_data.setPublishValue(TOF_SIDE::RIGHT);
            right_msg_arr.data.assign(result_data.right.pub_data.begin(), result_data.right.pub_data.end());
            RCLCPP_INFO(this->get_logger(), "[Publish Data R] : "); /////////////////////////////
            mtof_calibration_data_pub_->publish(right_msg_arr);
        }

        if (bLeftMToFCalibrationSet && bRightMToFCalibrationSet) {
            std_msgs::msg::Float32MultiArray msg_arr;
            msg_arr.data.assign(mtof_calib_result_array_.begin(), mtof_calib_result_array_.end());
            mtof_calibration_complete_pub_->publish(msg_arr);
            RCLCPP_INFO(this->get_logger(), "[Calibration Result: PASS] Publish m-ToF Calibration Data to A1_Perception");
            bLeftMToFCalibrationSet = false;
            bRightMToFCalibrationSet = false;
        }
    }
}

MTOF_CALIB_RESULT SensorManagerNode::handleCalibrationSide(MTOF_CALIB_DATA& calib_result, const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, bool &side_calib_set)
{
    MTOF_CALIB_RESULT ret = multiToFCalibration(calib_result, msg);

    if (ret != MTOF_CALIB_RESULT::RUNNING) {
        RCLCPP_INFO(this->get_logger(),
            "[Calibration: %s] SIDE: %s",
            enumToString(ret).c_str(),
            enumToString(isActiveMToFCalibration).c_str()
        );

        isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;

        if (ret == MTOF_CALIB_RESULT::PASS) {
            side_calib_set = true;
        }

        // save log after calibration done
        writeSelfTestCalibFile(side, ret);
    }

    std_msgs::msg::UInt8 calib_state_msg;
    calib_state_msg.data = make_mtof_state(side, ret);
    mtof_calibration_state_pub_->publish(calib_state_msg);

    return ret;
}

MTOF_CALIB_RESULT SensorManagerNode::multiToFCalibration(MTOF_CALIB_DATA& calib_result, const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    tTofCalibrationParam mtof_calib_cfg_ = this->load_mtof_calibration_params_();
    MTOF_CALIB_RESULT ret = MTOF_CALIB_RESULT::RUNNING;

    TOF_SIDE side;
    std::string calib_converter_key;

    if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT) {
        side = TOF_SIDE::LEFT;
        calib_converter_key = "tof_multi_left";
    } else if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT) {
        side = TOF_SIDE::RIGHT;
        calib_converter_key = "tof_multi_right";
    } else {
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    auto calib_converter = converters_.find(calib_converter_key);
    if (calib_converter == converters_.end() || !calib_converter->second) {
        RCLCPP_WARN(this->get_logger(), "[Calib] No converter found for: %s", calib_converter_key.c_str());
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    // 1. 임계값(Threshold) TF 변환 계산
    auto pnp_min_msg = std::make_shared<robot_custom_msgs::msg::TofData>();
    auto pnp_max_msg = std::make_shared<robot_custom_msgs::msg::TofData>();

    auto fill_tof_msg = [](auto& tof_msg, float val) {
        for (int i = 0; i < 16; ++i) {
            tof_msg->bot_left[i] = val;
            tof_msg->bot_right[i] = val;
        }
    };

    fill_tof_msg(pnp_min_msg, mtof_calib_cfg_.pass_min_value);
    fill_tof_msg(pnp_max_msg, mtof_calib_cfg_.pass_max_value);

    auto min_th_arr = calib_converter->second->calibration_convert(static_cast<const void*>(pnp_min_msg.get()));
    auto max_th_arr = calib_converter->second->calibration_convert(static_cast<const void*>(pnp_max_msg.get()));

    // 2. 입력 데이터 TF 변환
    auto current_data_arr = calib_converter->second->calibration_convert(static_cast<const void*>(msg.get()));

    if (current_data_arr.data.size() < 3 || min_th_arr.data.size() < 3) {
        RCLCPP_ERROR(this->get_logger(), "[Calib] Data size mismatch!");
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    // 3. 세션 초기화 및 데이터 갱신 체크
    if (calib_session_.sample_count == 0) {
        RCLCPP_INFO(this->get_logger(), "[Calib] Starting session for %s. Method: %s, Target Samples: %d", 
                    (side == TOF_SIDE::LEFT ? "LEFT" : "RIGHT"), mtof_calib_cfg_.method.c_str(), mtof_calib_cfg_.sampling_count);
        calib_session_.reset();
    }

    // 갱신 체크 및 데이터 축적 (for 루프로 통합)
    for (int i = 0; i < 3; ++i) {
        int target_idx = calib_session_.TARGET_INDICES[i]; // 13, 14, 15
        float raw_val = (side == TOF_SIDE::LEFT) ? msg->bot_left[target_idx] : msg->bot_right[target_idx];

        // 데이터 갱신 여부 확인
        if (!calib_session_.origins[i].empty() && std::abs(calib_session_.origins[i].back() - raw_val) < 1e-6f) {
            calib_session_.non_renewal_counts[i]++;
        } else {
            calib_session_.non_renewal_counts[i] = 0;
        }

        if (calib_session_.non_renewal_counts[i] > mtof_calib_cfg_.data_non_renewal_count) {
            RCLCPP_ERROR(this->get_logger(), "[Calib] FAIL: Data not renewing on idx %d", target_idx);
            return MTOF_CALIB_RESULT::FAIL_DATA_NON_RENEWAL;
        }

        // 데이터 push
        calib_session_.origins[i].push_back(raw_val);
        calib_session_.samples[i].push_back(current_data_arr.data[i]);
    }
    calib_session_.sample_count++;

    if (calib_session_.sample_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "[Calib] Progress: %d/%d...", calib_session_.sample_count, mtof_calib_cfg_.sampling_count);
    }

    // 4. 결과 판정
    if (calib_session_.sample_count >= mtof_calib_cfg_.sampling_count) {
        // 통계값 계산 (Max / Median)
        for (int i = 0; i < 3; ++i) {
            if (mtof_calib_cfg_.method == "Max") {
                calib_session_.stats[i] = *std::max_element(calib_session_.samples[i].begin(), calib_session_.samples[i].end());
            } else {
                auto& v = calib_session_.samples[i];
                std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                calib_session_.stats[i] = v[v.size() / 2];
            }
        }

        // 안정성 체크 (idx 14 기준)
        float min_val = *std::min_element(calib_session_.samples[1].begin(), calib_session_.samples[1].end());
        float max_val = *std::max_element(calib_session_.samples[1].begin(), calib_session_.samples[1].end());
        float diff = max_val - min_val;

        // 결과 로깅 (심플 스타일)
        RCLCPP_INFO(
            this->get_logger(),
            "[Calibration Result] Method: %s | Samples: %d\n"
            "  idx_13: %.3f\n"
            "  idx_14: %.3f\n"
            "  idx_15: %.3f",
            mtof_calib_cfg_.method.c_str(), calib_session_.sample_count,
            calib_session_.stats[0], calib_session_.stats[1], calib_session_.stats[2]
        );

        // 범위 검사
        bool out_of_range = false;
        for (int i = 0; i < 3; ++i) {
            if (calib_session_.stats[i] < min_th_arr.data[i] || calib_session_.stats[i] > max_th_arr.data[i]) {
                out_of_range = true;
                break;
            }
        }

        if (out_of_range) {
            RCLCPP_INFO(
                this->get_logger(),
                "[Calibration: FAIL_OUT_OF_RANGE]\n"
                "  idx_13: %.3f (min_th: %.3f, max_th: %.3f)\n"
                "  idx_14: %.3f (min_th: %.3f, max_th: %.3f)\n"
                "  idx_15: %.3f (min_th: %.3f, max_th: %.3f)",
                calib_session_.stats[0], min_th_arr.data[0], max_th_arr.data[0],
                calib_session_.stats[1], min_th_arr.data[1], max_th_arr.data[1],
                calib_session_.stats[2], min_th_arr.data[2], max_th_arr.data[2]
            );
            ret = MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE;
        }
        else if (diff > mtof_calib_cfg_.pass_diff_th) {
            RCLCPP_INFO(this->get_logger(), "[Calibration: FAIL_UNSTABLE_RANGE] Diff: %.4f (Th: %.4f)", diff, mtof_calib_cfg_.pass_diff_th);
            ret = MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "[Calibration: PASS] Side %s successfully calibrated.", (side == TOF_SIDE::LEFT ? "LEFT" : "RIGHT"));
            ret = MTOF_CALIB_RESULT::PASS;

            // 결과 데이터 저장
            int offset = (side == TOF_SIDE::LEFT) ? 0 : 3;
            for (int i = 0; i < 3; ++i) {
                mtof_calib_result_array_[offset + i] = calib_session_.stats[i];
            }
            calib_result.setCalibValue(side, calib_session_.stats[0], calib_session_.stats[1], calib_session_.stats[2]);
        }
        calib_session_.sample_count = 0;
    }

    return ret;
}

/**
 * @brief Left (LSB) / Right (MSB)
 *   Running:         0x01 (Left), 0x10 (Right)
 *   Complete:        0x02 (Left), 0x20 (Right)
 *   Out of Range:    0x03 (Left), 0x30 (Right)
 *   Unstable Range:  0x04 (Left), 0x40 (Right)
 *   Data non renewal:0x08 (Left), 0x80 (Right)
 */
uint8_t SensorManagerNode::make_mtof_state(TOF_SIDE side, MTOF_CALIB_RESULT state)
{
    uint8_t value = 0;

    switch (state) {
        case MTOF_CALIB_RESULT::RUNNING:                value = 0x01; break;
        case MTOF_CALIB_RESULT::PASS:                   value = 0x02; break;
        case MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE:      value = 0x03; break;
        case MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE:    value = 0x04; break;
        // case MTOF_CALIB_RESULT::FAIL_TIME_OUT:          value = 0x08; break;
        case MTOF_CALIB_RESULT::FAIL_DATA_NON_RENEWAL:  value = 0x08; break;
        default:                                        value = 0x00; break;
    }

    if (side == TOF_SIDE::RIGHT) {
        value = (value & 0x0F) << 4;  // Left 0x0?, Right 0x?0
    }

    return value;
}

tTofCalibrationParam SensorManagerNode::load_mtof_calibration_params_()
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

/**
  * @brief Save tof self test calibration data in json format
  *
  * @param side enum Class, Left or Right
  * @param resultCode uint8_t, tof calibration result
  */
void SensorManagerNode::writeSelfTestCalibFile(TOF_SIDE side, MTOF_CALIB_RESULT resultCode)
{
    std::deque<std::string> buffer;
    std::string tof_calib_file_path = "/home/airbot/app_rw/log/MultiCalibration.json";

    if (!checkFileExist(tof_calib_file_path, buffer)){
        return;
    }

    json j;
    createJsonData(j, side, resultCode);

    writeDataFile(tof_calib_file_path, buffer, j);
}

/**
 * @brief Check file exist, and save exist data in buffer
 *
 * @param path Json file path
 * @param buffer exist data in file
 * @return true, File open success
 * @return false, Fail to open file or fail to create file
 */
bool SensorManagerNode::checkFileExist(std::string path, std::deque<std::string> &buffer)
{
    std::ifstream read_file(path);
    int max_lines = 10;

    if (!read_file.good()){
        RCLCPP_WARN(this->get_logger(), "There is no file in path = %s", path.c_str());
        // 파일 없을 시 새로 생성
        std::ofstream make_new_file(path);
        if (!make_new_file){
            RCLCPP_ERROR(this->get_logger(), "Fail to make new file in path = %s", path.c_str());
            read_file.close();
            return false;
        }
        make_new_file.close();
    }

    RCLCPP_INFO(this->get_logger(), "File open success.");

    std::string line;
    while (std::getline(read_file, line)){
        buffer.push_back(line);
        // 만약 파일에 쓰여져있는 내용이 10줄 이상이라면 오래된 내용 삭제
        if ((int)buffer.size() >= max_lines){
            buffer.pop_front();
        }
    }
    read_file.close();

    return true;
}

/**
 * @brief Create json data.
 * Format : {"time": "YY-MM-DD HH:MM:SS", "side": "Left"/"Right", "result": "PASS"/"FAILE", "failCode": "0xXX", "data": [x.xxx, x.xxx, x.xxx]}
 *
 * @param j nlohmann::ordered_json
 */
void SensorManagerNode::createJsonData(json &j, TOF_SIDE side, MTOF_CALIB_RESULT resultCode)
{
    json tof_data;

    time_t now = time(0);
    tm* ltm = localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(ltm, "%Y-%m-%d %H:%M:%S");
    std::string time_str = oss.str();

    j["time"] = time_str;
    j["side"] = ((side == TOF_SIDE::LEFT)? "Left" : "Right");
    if (resultCode == MTOF_CALIB_RESULT::PASS){
        j["result"] = "PASS";
    }
    else{
        j["result"] = "FAIL";
        j["failCode"] = resultCode;
    }

    if (side == TOF_SIDE::LEFT){
        for (uint8_t i=0; i<3; i++){
            j["data"].push_back(truncate_to_n(mtof_calib_result_array_[i], 3));
        }
    }
    else if (side == TOF_SIDE::RIGHT){
        for (uint8_t i=3; i<(uint8_t)mtof_calib_result_array_.size(); i++){
            j["data"].push_back(truncate_to_n(mtof_calib_result_array_[i], 3));
        }
    }
}

/**
 * @brief Save json data to file
 *
 * @param path json file path
 * @param buffer exist data in json file
 * @param output_data new data to wirte file
 */
void SensorManagerNode::writeDataFile(const std::string& path, const std::deque<std::string>& buffer, const json& output_data)
{
    std::ofstream output_file(path);
    if (!output_file.is_open()){
        RCLCPP_ERROR(this->get_logger(), "Fail to open file for writing, path = %s", path.c_str());
        return;
    }

    for (const auto &line : buffer){
        output_file << line << std::endl;
    }
    output_file << output_data << std::endl;
    output_file.flush(); // kernel buffer
    output_file.close();

    int fd = ::open(path.c_str(), O_WRONLY | O_APPEND);
    if (fd != -1) {
        // kernel buffer to disk
        ::fsync(fd);
        ::close(fd);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Fail to open fsync");
    }

    RCLCPP_INFO(this->get_logger(), "File write success.");
}

/**
 * @brief 소수점 n자리 이하 버림
 *
 * @param value double, 원본 실수값
 * @param n int, n자리 이하 버림값
 * @return double, n자리 이하 버려진 실수값
 */
double SensorManagerNode::truncate_to_n(double value, int n)
{
    double scale = std::pow(10.0, n);
    return std::round(value * scale) / scale;
}

} // namespace sensor_manager