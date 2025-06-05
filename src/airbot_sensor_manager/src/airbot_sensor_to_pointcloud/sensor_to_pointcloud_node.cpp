#include "ament_index_cpp/get_package_share_directory.hpp"
#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud_node.hpp"

/*
    4x4 tof를 (기존)행별로 사용할지, 인덱스로 사용할지 결정

    false: row (각 4개) <- 기존 버전
    true : idx (각 16개)
*/
#define IS_4X4_INDEX false

using namespace std::chrono_literals;

// Robot, Sensor Geometric Specification
double tof_top_sensor_frame_x_translate = 0.0942;       //[meter]
double tof_top_sensor_frame_y_translate = 0.0;          //[meter]
double tof_top_sensor_frame_z_translate = 0.56513;      //[meter]
double tof_bot_sensor_frame_x_translate = 0.14316;      //[meter]
double tof_bot_sensor_frame_y_translate = 0.075446;     //[meter]
double tof_bot_sensor_frame_z_translate = 0.03;         //[meter]
double tof_bot_left_sensor_frame_yaw_ang = 15.0;        //[deg]
double tof_bot_rihgt_sensor_frame_yaw_ang = -15.0;      //[deg]
double tof_bot_fov_ang = 45;                            //[deg]
double camera_sensor_frame_x_translate = 0.15473;       //[meter]
double camera_sensor_frame_y_translate = 0.0;           //[meter]
double camera_sensor_frame_z_translate = 0.5331;        //[meter]
double cliff_sensor_distance_center_to_front_ir = 0.15; //[meter]
double cliff_sensor_angle_to_next_ir_sensor = 50;       //[deg]
double collision_forward_point_offset = 0.25;           //[meter]

SensorToPointcloud::SensorToPointcloud()
    : rclcpp::Node("airbot_sensor_to_pointcloud"),
    point_cloud_tof_(tof_top_sensor_frame_x_translate,
                     tof_top_sensor_frame_y_translate,
                     tof_top_sensor_frame_z_translate,
                     tof_bot_sensor_frame_x_translate,
                     tof_bot_sensor_frame_y_translate,
                     tof_bot_sensor_frame_z_translate,
                     tof_bot_left_sensor_frame_yaw_ang,
                     tof_bot_rihgt_sensor_frame_yaw_ang,
                     tof_bot_fov_ang),
    point_cloud_cliff_(cliff_sensor_distance_center_to_front_ir,
                       cliff_sensor_angle_to_next_ir_sensor),
    point_cloud_collosion_(collision_forward_point_offset),
    bounding_box_generator_(camera_sensor_frame_x_translate,
                            camera_sensor_frame_y_translate,
                            camera_sensor_frame_z_translate)
{
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("airbot_sensor_manager");
        std::string full_path = package_share_directory + "/config/sensor_to_pointcloud_param.yaml";
        this->config = YAML::LoadFile(full_path)["airbot_sensor_to_pointcloud"]["ros__parameters"]["output"];
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load YAML config: %s", e.what());
    }

    // Dynamic Parameter Handler
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    target_frame_callback_handle_ = param_handler_->add_parameter_callback(
        "target_frame",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                std::string before = target_frame_;
                target_frame_ = param.as_string();
                updateTargetFrame();
                std::string after;
                if (this->get_parameter("target_frame", after)) {
                    RCLCPP_INFO(this->get_logger(), "[=== Updating target_frame: %s -> %s ===]", before.c_str(), after.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "target_frame parameter not found!");
                }
            }
        }
    );

    mtof_left_subcell_callback_handle_ = param_handler_->add_parameter_callback(
        "tof.multi.left.sub_cell_idx_array",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
                auto new_array = param.as_integer_array();
                if (new_array.size() != 16) {
                    RCLCPP_WARN(this->get_logger(), "Invalid left sub_cell_idx_array size: %zu", new_array.size());
                    return;
                }
                std::vector<int> vec(new_array.begin(), new_array.end());
                point_cloud_tof_.updateLeftSubCellIndexArray(vec);
                RCLCPP_INFO(this->get_logger(), "Updated left sub_cell_idx_array");
            }
        }
    );

    mtof_right_subcell_callback_handle_ = param_handler_->add_parameter_callback(
        "tof.multi.right.sub_cell_idx_array",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
                auto new_array = param.as_integer_array();
                if (new_array.size() != 16) {
                    RCLCPP_WARN(this->get_logger(), "Invalid right sub_cell_idx_array size: %zu", new_array.size());
                    return;
                }
                std::vector<int> vec(new_array.begin(), new_array.end());
                point_cloud_tof_.updateRightSubCellIndexArray(vec);
                RCLCPP_INFO(this->get_logger(), "Updated right sub_cell_idx_array");
            }
        }
    );

    // Cmd Subscribers
    sensor_to_pointcloud_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "cmd_sensor_manager", 10, std::bind(&SensorToPointcloud::activeCmdCallback, this, std::placeholders::_1));

    // Msg Subscribers
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensorToPointcloud::tofMsgUpdate, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "camera_data", 10, std::bind(&SensorToPointcloud::cameraMsgUpdate, this, std::placeholders::_1));
    cliff_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_ir_data", 10, std::bind(&SensorToPointcloud::cliffMsgUpdate, this, std::placeholders::_1));
    collision_sub_ = this->create_subscription<robot_custom_msgs::msg::AbnormalEventData>(
        "collision_detected", 10, std::bind(&SensorToPointcloud::collisionMsgUpdate, this, std::placeholders::_1));

    // Monitor Timer
    poincloud_publish_timer_ = this->create_wall_timer(
        10ms, std::bind(&SensorToPointcloud::publisherMonitor, this));

    RCLCPP_INFO(this->get_logger(), "Node init finished!");
}

SensorToPointcloud::~SensorToPointcloud()
{
    param_handler_.reset();
}

void SensorToPointcloud::init()
{
    wasActiveSensorToPointcloud_tof = false;
    wasActiveSensorToPointcloud_camera = false;
    wasActiveSensorToPointcloud_cliff = false;
    wasActiveSensorToPointcloud_collision = false;
    camera_object_logger_.setNode(shared_from_this());
    initVariables();
    declareParams();
    setParams();
    initSensorConfig(this->config);
    botTofPitchAngle_.bot_left = sensor_config_.multi_tof_left.pitch_angle_deg;
    botTofPitchAngle_.bot_right = sensor_config_.multi_tof_right.pitch_angle_deg;
    initPublisher(this->config);
    initFilterParam(this->config["tof_multi"]["filter"]);
    updateAllParameters();
    printParams();
}

void SensorToPointcloud::initVariables()
{
    isTofUpdating = false;
    isCameraUpdating = false;
    isCliffUpdating = false;
    isCollisionUpdating = false;
    isActiveSensorToPointcloud = false;

    publish_cnt_1d_tof_ = 0;
    publish_cnt_multi_tof_ = 0;
    publish_cnt_row_tof_ = 0;
    publish_cnt_camera_ = 0;
    publish_cnt_cliff_ = 0;
    publish_cnt_collision_ = 0;
}

void SensorToPointcloud::initSensorConfig(const YAML::Node& config)
{
    for(YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string sensor_name = it->first.as<std::string>();
        YAML::Node sensor_config = it->second;

        if (sensor_name == "topic_prefix") continue;
        if (sensor_name == "tof_mono") sensor_config_.one_d_tof = getSensorCfg(sensor_config);
        else if (sensor_name == "tof_multi") sensor_config_.multi_tof = getSensorCfg(sensor_config);
        else if (sensor_name == "tof_multi_left") sensor_config_.multi_tof_left = getSensorCfg(sensor_config);
        else if (sensor_name == "tof_multi_right") sensor_config_.multi_tof_right = getSensorCfg(sensor_config);
        else if (sensor_name == "camera") sensor_config_.camera = getSensorCfg(sensor_config);
        else if (sensor_name == "cliff") sensor_config_.cliff = getSensorCfg(sensor_config);
        else if (sensor_name == "collision") sensor_config_.collision = getSensorCfg(sensor_config);
    }
}

void SensorToPointcloud::initPublisher(const YAML::Node& config)
{
    std::string topic_prefix;
    if (config["topic_prefix"] && config["topic_prefix"].IsScalar()) {
        topic_prefix = config["topic_prefix"].as<std::string>();
    }

    auto create_pc_pub = [this, topic_prefix](const std::string& topic_name) {
        return this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_prefix + topic_name, 10
        );
    };

    for(YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        std::string sensor_name = it->first.as<std::string>();
        YAML::Node sensor_config = it->second;

        bool is_publihser = (sensor_name == "topic_prefix" || sensor_name == "tof_multi_left" || sensor_name == "tof_multi_right");
        if (is_publihser) continue;

        if (sensor_config.IsMap()) {
            bool is_use = sensor_config["use"] ? sensor_config["use"].as<bool>() : false;

            if (is_use) {
                std::string topic_name = sensor_config["topic"].as<std::string>();
                pointcloud_pubs_[topic_name] = create_pc_pub(topic_name);
                if (sensor_name == "camera") {
                    bbox_array_camera_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>(
                        "sensor_to_pointcloud/camera/bbox", 10
                    );
                }
            }
        }
    }

    bool is_enable_8x8 = config["tof_multi"]["enable_8x8"] ? config["tof_multi"]["enable_8x8"].as<bool>() : false;
    if (is_enable_8x8) {
        if (sensor_config_.multi_tof_left.use) {
            for (auto index : sensor_config_.multi_tof_left.sub_cell_idx_array) {
                pointcloud_pubs_[sensor_config_.multi_tof_left.topic_idx + std::to_string(index)]
                = create_pc_pub(sensor_config_.multi_tof_left.topic_idx + std::to_string(index));
            }
        }
        if (sensor_config_.multi_tof_right.use){
            for (auto index : sensor_config_.multi_tof_right.sub_cell_idx_array) {
                pointcloud_pubs_[sensor_config_.multi_tof_right.topic_idx + std::to_string(index)]
                    = create_pc_pub(sensor_config_.multi_tof_right.topic_idx + std::to_string(index));
            }
        }
    } else {
        #if IS_4X4_INDEX
        for (int i = 0; i < 16; ++i) {
            if (sensor_config_.multi_tof_left.use) {
                pointcloud_pubs_[sensor_config_.multi_tof_left.topic_idx + std::to_string(i)]
                    = create_pc_pub(sensor_config_.multi_tof_left.topic_idx + std::to_string(i));
            }
            if (sensor_config_.multi_tof_right.use){
                pointcloud_pubs_[sensor_config_.multi_tof_right.topic_idx + std::to_string(i)]
                    = create_pc_pub(sensor_config_.multi_tof_right.topic_idx + std::to_string(i));
            }
        }
        #else
        for (int i = 0; i < 4; ++i) {
            if (sensor_config_.multi_tof_left.use) {
                pointcloud_pubs_[sensor_config_.multi_tof_left.topic_row + std::to_string(i+1)]
                    = create_pc_pub(sensor_config_.multi_tof_left.topic_row + std::to_string(i+1));
            }
            if (sensor_config_.multi_tof_right.use){
                pointcloud_pubs_[sensor_config_.multi_tof_right.topic_row + std::to_string(i+1)]
                    = create_pc_pub(sensor_config_.multi_tof_right.topic_row + std::to_string(i+1));
            }
        }
        #endif
    }

    //debug
    tof_debug_pub_ = this->create_publisher<robot_custom_msgs::msg::TofData>(
        "filtered_tof_data", 10
    );
    sensor_to_pointcloud_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "sensor_to_pointcloud_active", 10
    );

    RCLCPP_INFO(this->get_logger(), "Publisher init finished!");
}

void SensorToPointcloud::initFilterParam(const YAML::Node& node)
{
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        std::string filter_name = it->first.as<std::string>();
        YAML::Node filter_config = it->second;

        if (filter_name == "moving_average") {
            mtof_filter_.moving_average.use = filter_config["use"].as<bool>();
            if (filter_config["enabled_4x4_idx"] && filter_config["enabled_4x4_idx"].IsSequence()) {
                for (const auto& idx_node : filter_config["enabled_4x4_idx"]) {
                    mtof_filter_.moving_average.enabled_4x4_idx.push_back(idx_node.as<int>());
                }
            }
            mtof_filter_.moving_average.window_size = filter_config["window_size"].as<int>();
            mtof_filter_.moving_average.max_distance_th = filter_config["max_distance_th"].as<double>();
        } else if (filter_name == "low_pass") {
            mtof_filter_.low_pass.use = filter_config["use"].as<bool>();
            if (filter_config["enabled_4x4_idx"] && filter_config["enabled_4x4_idx"].IsSequence()) {
                for (const auto& idx_node : filter_config["enabled_4x4_idx"]) {
                    mtof_filter_.low_pass.enabled_4x4_idx.push_back(idx_node.as<int>());
                }
            }
            mtof_filter_.low_pass.alpha = filter_config["alpha"].as<double>();
        } else if (filter_name == "complementary") {
            mtof_filter_.complementary.use = filter_config["use"].as<bool>();
            if (filter_config["enabled_4x4_idx"] && filter_config["enabled_4x4_idx"].IsSequence()) {
                for (const auto& idx_node : filter_config["enabled_4x4_idx"]) {
                    mtof_filter_.complementary.enabled_4x4_idx.push_back(idx_node.as<int>());
                }
            }
            mtof_filter_.complementary.alpha = filter_config["alpha"].as<double>();
        }
    }
}

void SensorToPointcloud::updateAllParameters()
{
    updateTargetFrame();
    updateFilterParam();
    point_cloud_tof_.updateTofMode(use_tof_8x8_);
    point_cloud_tof_.updateLeftSubCellIndexArray(sensor_config_.multi_tof_left.sub_cell_idx_array);
    point_cloud_tof_.updateRightSubCellIndexArray(sensor_config_.multi_tof_right.sub_cell_idx_array);
    camera_object_logger_.updateParams(camera_logger_distance_margin_);
    for (const auto& item : sensor_config_.camera.class_id) {
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
}

void SensorToPointcloud::updateTargetFrame()
{
    point_cloud_tof_.updateTargetFrame(target_frame_);
    bounding_box_generator_.updateTargetFrame(target_frame_);
    point_cloud_cliff_.updateTargetFrame(target_frame_);
    point_cloud_collosion_.updateTargetFrame(target_frame_);
}

void SensorToPointcloud::updateFilterParam()
{
    tof_lp_filter_.updateParams(mtof_filter_.low_pass.alpha, mtof_filter_.low_pass.enabled_4x4_idx);
    tof_ma_filter_.updateParams(mtof_filter_.moving_average.window_size, mtof_filter_.moving_average.enabled_4x4_idx, mtof_filter_.moving_average.max_distance_th);
    tof_comp_filter_.updateParams(mtof_filter_.complementary.alpha, mtof_filter_.low_pass.alpha, mtof_filter_.moving_average.window_size, mtof_filter_.complementary.enabled_4x4_idx);
}

void SensorToPointcloud::declareParams()
{
    this->declare_parameter("target_frame","base_link");

    this->declare_parameter("output.tof_multi.enable_8x8", false);

    this->declare_parameter("output.camera.logger.use",false);
    this->declare_parameter("output.camera.logger.margin.distance_diff",1.0);
}

void SensorToPointcloud::setParams()
{
    this->get_parameter("target_frame", target_frame_);

    this->get_parameter("output.tof_multi.enable_8x8", use_tof_8x8_);

    this->get_parameter("output.camera.logger.use", use_camera_log_);
    this->get_parameter("output.camera.logger.margin.distance_diff", camera_logger_distance_margin_);
}

void SensorToPointcloud::printParams()
{
    RCLCPP_INFO(this->get_logger(), "================== SENSOR MANAGER PARAMETERS ==================");
    // General Settings
    RCLCPP_INFO(this->get_logger(), "[General]");
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", target_frame_.c_str());
    // TOF Settings
    RCLCPP_INFO(this->get_logger(), "[TOF Settings]");
    printSensorConfig("TOF 1D", sensor_config_.one_d_tof);
    printSensorConfig("TOF Multi", sensor_config_.multi_tof);
    RCLCPP_INFO(this->get_logger(), "    TOF Multi 8x8 Use: %s", use_tof_8x8_ ? "True" : "False");
    printSensorConfig("TOF Multi Left", sensor_config_.multi_tof_left);
    printSensorConfig("TOF Multi Right", sensor_config_.multi_tof_right);
    // Filters
    RCLCPP_INFO(this->get_logger(), "---------------------- FILTERS PARAMETERS ---------------------");
    printFilterConfig("Moving Average", mtof_filter_.moving_average);
    printFilterConfig("Low Pass", mtof_filter_.low_pass);
    printFilterConfig("Complementary", mtof_filter_.complementary);
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------");
    // Camera Settings
    RCLCPP_INFO(this->get_logger(), "[Camera Settings]");
    printSensorConfig("Camera Front", sensor_config_.camera);
    // Cliff Settings
    RCLCPP_INFO(this->get_logger(), "[Cliff Settings]");
    printSensorConfig("Cliff", sensor_config_.cliff);
    // Collision Settings
    RCLCPP_INFO(this->get_logger(), "[Collision Settings]");
    printSensorConfig("Collision", sensor_config_.collision);
    RCLCPP_INFO(this->get_logger(), "===============================================================");
    RCLCPP_INFO(this->get_logger(), "Parameter update finished!");
}

void SensorToPointcloud::printSensorConfig(const std::string& name, const tSensor& cfg)
{
    RCLCPP_INFO(this->get_logger(), "  [%s]", name.c_str());
    RCLCPP_INFO(this->get_logger(), "    Use: %s", cfg.use ? "True" : "False");
    #if IS_4X4_INDEX
    RCLCPP_INFO(this->get_logger(), "    Topic: %s", cfg.topic_idx.c_str());
    #else
    if (cfg.use) {
        RCLCPP_INFO(this->get_logger(), "    Topic: %s", cfg.topic_idx.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "    Topic: %s", cfg.topic_row.c_str());
    }
    #endif
    RCLCPP_INFO(this->get_logger(), "    Publish Rate: %d ms", cfg.publish_rate);
    if (cfg.pitch_angle_deg != 0.0 || !cfg.sub_cell_idx_array.empty()) { // Tof
        RCLCPP_INFO(this->get_logger(), "    Pitch Angle: %.2f deg", cfg.pitch_angle_deg);
        if (cfg.sub_cell_idx_array.size() == 16) {
            RCLCPP_INFO(this->get_logger(), "    Sub Cell Index Array:");
            for (int i = 0; i < 4; ++i) {
                std::stringstream row;
                row << "      [ ";
                for (int j = 0; j < 4; ++j) {
                    row << cfg.sub_cell_idx_array[i * 4 + j] << " ";
                }
                row << "]";
                RCLCPP_INFO(this->get_logger(), "%s", row.str().c_str());
            }
        }
    }
    if (cfg.pc_resolution > 0.0 || !cfg.class_id.empty()) { // Camera
        RCLCPP_INFO(this->get_logger(), "    Point Cloud Resolution: %.2f", cfg.pc_resolution);
        RCLCPP_INFO(this->get_logger(), "    Direction: %s", cfg.direction ? "Forward(CCW+)" : "Reverse(CW+)");
        RCLCPP_INFO(this->get_logger(), "    Object Max Distance: %.2f", cfg.object_max_dist);
        if (!cfg.class_id.empty()) {
            std::stringstream class_stream;
            class_stream << "    Target Objects (ID: Conf_th): [ ";
            for (const auto& id : cfg.class_id) {
                class_stream << "(" << id << "), ";
            }
            class_stream << "]";
            RCLCPP_INFO(this->get_logger(), "%s", class_stream.str().c_str());
        }
    }
}

void SensorToPointcloud::printFilterConfig(const std::string& name, const tFilter& filter)
{
    RCLCPP_INFO(this->get_logger(), "     %s Filter", name.c_str());
    RCLCPP_INFO(this->get_logger(), "        use: %s", filter.use ? "true" : "false");
    if (name == "Moving Average") {
        RCLCPP_INFO(this->get_logger(), "        window_size: %d", filter.window_size);
        RCLCPP_INFO(this->get_logger(), "        max_distance_th: %.2f", filter.max_distance_th);
    } else if (name == "Low Pass" || name == "Complementary") {
        RCLCPP_INFO(this->get_logger(), "        alpha: %.2f", filter.alpha);
    }
    std::stringstream ss;
    for (const auto& idx : filter.enabled_4x4_idx) {
        ss << idx << " ";
    }
    RCLCPP_INFO(this->get_logger(), "        enabled_4x4_idx: [%s]", ss.str().c_str());
}

tSensor SensorToPointcloud::getSensorCfg(const YAML::Node& node)
{
    tSensor cfg;
    if (node["use"]) cfg.use = node["use"].as<bool>();
    if (node["topic"]) cfg.topic = node["topic"].as<std::string>();
    if (node["topic_idx"]) cfg.topic_idx = node["topic_idx"].as<std::string>();
    if (node["topic_row"]) cfg.topic_row = node["topic_row"].as<std::string>();
    if (node["publish_rate_ms"]) cfg.publish_rate = node["publish_rate_ms"].as<int>();
    if (node["pitch_angle_deg"]) cfg.pitch_angle_deg = node["pitch_angle_deg"].as<double>();
    if (node["sub_cell_idx_array"]) {
        for (auto idx_node : node["sub_cell_idx_array"]) {
            cfg.sub_cell_idx_array.push_back(idx_node.as<int>());
        }
    }
    if (node["pointcloud_resolution"]) cfg.pc_resolution = node["pointcloud_resolution"].as<float>();
    if (node["object_direction"]) cfg.direction = node["object_direction"].as<bool>();
    if (node["object_max_distance_m"]) cfg.object_max_dist = node["object_max_distance_m"].as<double>();
    if (node["class_id_confidence_th"]) {
        for (auto conf_node : node["class_id_confidence_th"]) {
            cfg.class_id.push_back(conf_node.as<std::string>());
        }
    }

    return cfg;
}

void SensorToPointcloud::publisherMonitor()
{
    if (!isActiveSensorToPointcloud) {
        initVariables();
        return;
    }

    publish_cnt_1d_tof_ += 10;
    publish_cnt_multi_tof_ += 10;
    publish_cnt_row_tof_ += 10;
    publish_cnt_camera_ += 10;
    publish_cnt_cliff_ += 10;
    publish_cnt_collision_ += 10;

    // publish pointCloud Data
    if (isTofUpdating) { // ToF
        if (sensor_config_.one_d_tof.use) {
            if (publish_cnt_1d_tof_ >= sensor_config_.one_d_tof.publish_rate) {
                pointcloud_pubs_[sensor_config_.one_d_tof.topic]->publish(pc_tof_1d_msg);
                publish_cnt_1d_tof_ = 0;
            }
        }
        if (sensor_config_.multi_tof.use) {
            if (publish_cnt_multi_tof_ >= sensor_config_.multi_tof.publish_rate) {
                pointcloud_pubs_[sensor_config_.multi_tof.topic]->publish(pc_tof_multi_msg);
                publish_cnt_multi_tof_ = 0;
            }
        }
        if (publish_cnt_row_tof_ >= sensor_config_.multi_tof.publish_rate) {
            if (use_tof_8x8_) {
                if (sensor_config_.multi_tof_left.use) {
                    for (auto index : sensor_config_.multi_tof_left.sub_cell_idx_array) {
                        std::string topic_name = sensor_config_.multi_tof_left.topic_idx + std::to_string(index);
                        pointcloud_pubs_[topic_name]->publish(pc_8x8_tof_left_msg_map_[index]);
                    }
                }
                if (sensor_config_.multi_tof_right.use) {
                    for (auto index : sensor_config_.multi_tof_right.sub_cell_idx_array) {
                        std::string topic_name = sensor_config_.multi_tof_right.topic_idx+ std::to_string(index);
                        pointcloud_pubs_[topic_name]->publish(pc_8x8_tof_right_msg_map_[index]);
                    }
                }
            } else {
            #if IS_4X4_INDEX
                for (int i = 0; i < 16; ++i) {
                    if (sensor_config_.multi_tof_left.use) {
                        std::string topic_name = sensor_config_.multi_tof_left.topic_idx + std::to_string(i);
                        pointcloud_pubs_[topic_name]->publish(pc_4x4_tof_left_msg_map_[i]);
                    }
                    if (sensor_config_.multi_tof_right.use) {
                        std::string topic_name = sensor_config_.multi_tof_right.topic_idx + std::to_string(i);
                        pointcloud_pubs_[topic_name]->publish(pc_4x4_tof_right_msg_map_[i]);
                    }
                }
            #else
                for (int i = 0; i < 4; ++i) {
                    if (sensor_config_.multi_tof_left.use) {
                        std::string topic_name = sensor_config_.multi_tof_left.topic_row + std::to_string(i+1);
                        pointcloud_pubs_[topic_name]->publish(pc_4x4_tof_left_msg_map_[i]);
                    }
                    if (sensor_config_.multi_tof_right.use) {
                        std::string topic_name = sensor_config_.multi_tof_right.topic_row + std::to_string(i+1);
                        pointcloud_pubs_[topic_name]->publish(pc_4x4_tof_right_msg_map_[i]);
                    }
                }
            #endif
            }
            publish_cnt_row_tof_ = 0;
        }
        isTofUpdating = false;
    }
    if (sensor_config_.camera.use && isCameraUpdating) {
        if (publish_cnt_camera_ >= sensor_config_.camera.publish_rate) {
            pointcloud_pubs_[sensor_config_.camera.topic]->publish(pc_camera_msg);
            bbox_array_camera_pub_->publish(bbox_msg);
            isCameraUpdating = false;
            publish_cnt_camera_ = 0;
        }
    }
    if (sensor_config_.cliff.use && isCliffUpdating) {
        if (publish_cnt_cliff_ >= sensor_config_.cliff.publish_rate) {
            pointcloud_pubs_[sensor_config_.cliff.topic]->publish(pc_cliff_msg);
            isCliffUpdating = false;
            publish_cnt_cliff_ = 0;
        }
    }
    if (sensor_config_.collision.use && isCollisionUpdating) {
        if (publish_cnt_collision_ >= sensor_config_.collision.publish_rate) {
            pointcloud_pubs_[sensor_config_.collision.topic]->publish(pc_collision_msg);
            isCollisionUpdating = false;
            publish_cnt_collision_ = 0;
        }
    }

    if (publish_cnt_1d_tof_ > 10000)     publish_cnt_1d_tof_ = 0;
    if (publish_cnt_multi_tof_ > 10000)  publish_cnt_multi_tof_ = 0;
    if (publish_cnt_row_tof_ > 10000)    publish_cnt_row_tof_ = 0;
    if (publish_cnt_camera_ > 10000)     publish_cnt_camera_ = 0;
    if (publish_cnt_cliff_ > 10000)      publish_cnt_cliff_ = 0;
    if (publish_cnt_collision_ > 10000)  publish_cnt_collision_ = 0;
}

void SensorToPointcloud::publishEmptyMsg()
{
    sensor_msgs::msg::PointCloud2 empty_cloud = pointcloud_generator_.generatePointCloud2EmptyMessage(target_frame_);

    for (auto& [name, pub] : pointcloud_pubs_) {
        if (pub && pub->get_subscription_count() > 0) {
            pub->publish(empty_cloud);
        }
    }

    RCLCPP_INFO(this->get_logger(), "All Active Publisher publish empty_cloud msgs!");
}

void SensorToPointcloud::activeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "cmd_sensor_to_pointcloud topic is a nullptr message.");
        return;
    }

    isActiveSensorToPointcloud = msg->data;
    std_msgs::msg::Bool sensor_manager_state_msg;
    sensor_manager_state_msg.data = msg->data;

    if (isActiveSensorToPointcloud){
        RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : Active");
        for (int i=0; i<3; ++i) {
            sensor_to_pointcloud_state_pub_->publish(sensor_manager_state_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
    } else {
        publishEmptyMsg();
        RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : De-Active");
        for (int i=0; i<3; ++i) {
            sensor_to_pointcloud_state_pub_->publish(sensor_manager_state_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void SensorToPointcloud::tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if (wasActiveSensorToPointcloud_tof && !isActiveSensorToPointcloud) {
        if (sensor_config_.one_d_tof.use) pc_tof_1d_msg = sensor_msgs::msg::PointCloud2();
        if (sensor_config_.multi_tof.use) pc_tof_multi_msg = sensor_msgs::msg::PointCloud2();
        if (sensor_config_.multi_tof_left.use) {
            if (use_tof_8x8_) pc_8x8_tof_left_msg_map_.clear();
            else pc_4x4_tof_left_msg_map_.clear();
        }
        if (sensor_config_.multi_tof_right.use) {
            if (use_tof_8x8_) pc_8x8_tof_right_msg_map_.clear();
            else pc_4x4_tof_right_msg_map_.clear();
        }
        RCLCPP_INFO(this->get_logger(), "TOF PointCloud Msg Clear");
    }
    wasActiveSensorToPointcloud_tof = isActiveSensorToPointcloud;
    if (!isActiveSensorToPointcloud) return;

    if (target_frame_ == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_tof_.updateRobotPose(pose);
    }

    robot_custom_msgs::msg::TofData::SharedPtr filtered_msg = msg;
    if (mtof_filter_.moving_average.use) filtered_msg = tof_ma_filter_.update(filtered_msg);
    if (mtof_filter_.low_pass.use) filtered_msg = tof_lp_filter_.update(filtered_msg);
    if (mtof_filter_.complementary.use) filtered_msg = tof_comp_filter_.update(filtered_msg);
    tof_debug_pub_->publish(*filtered_msg);

    if (sensor_config_.one_d_tof.use) pc_tof_1d_msg = point_cloud_tof_.updateTopTofPointCloudMsg(msg, sensor_config_.one_d_tof.pitch_angle_deg);
    if (sensor_config_.multi_tof.use || sensor_config_.multi_tof_left.use || sensor_config_.multi_tof_right.use) {
        TOF_SIDE side = (sensor_config_.multi_tof_left.use && sensor_config_.multi_tof_right.use)
                        ? TOF_SIDE::BOTH : (sensor_config_.multi_tof_left.use ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT);
        auto pc_msgs = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, side, botTofPitchAngle_);
        if (side == TOF_SIDE::LEFT) {
            if (use_tof_8x8_) {
                int i = 0;
                for (auto index : sensor_config_.multi_tof_left.sub_cell_idx_array) {
                    pc_8x8_tof_left_msg_map_[index] = pc_msgs[i];
                    i++;
                }
            } else {
                #if IS_4X4_INDEX
                for (int i=0; i<16; i++) {
                    pc_4x4_tof_left_msg_map_[i] = pc_msgs[i];
                }
                #else
                for (int i=0; i<4; ++i) {
                    int start_idx = i * 4;
                    std::vector<sensor_msgs::msg::PointCloud2> slice(pc_msgs.begin() + start_idx, pc_msgs.begin() + start_idx + 4);
                    pc_4x4_tof_left_msg_map_[i] = pointcloud_generator_.mergePointCloud2Vector(slice, target_frame_);
                }
                #endif
            }
        } else if (side == TOF_SIDE::RIGHT) {
            if (use_tof_8x8_) {
                int i = 0;
                for (auto index : sensor_config_.multi_tof_right.sub_cell_idx_array) {
                    pc_8x8_tof_right_msg_map_[index] = pc_msgs[i];
                    i++;
                }
            } else {
                #if IS_4X4_INDEX
                for (int i=0; i<16; i++) {
                    pc_4x4_tof_right_msg_map_[i] = pc_msgs[i];
                }
                #else
                for (int i=0; i<4; ++i) {
                    int start_idx = i * 4;
                    std::vector<sensor_msgs::msg::PointCloud2> slice(pc_msgs.begin() + start_idx, pc_msgs.begin() + start_idx + 4);
                    pc_4x4_tof_right_msg_map_[i] = pointcloud_generator_.mergePointCloud2Vector(slice, target_frame_);
                }
                #endif
            }
        } else { // TOF_SIDE::BOTH
            if (use_tof_8x8_) {
                int i = 0;
                int j = 0;
                for (auto index : sensor_config_.multi_tof_left.sub_cell_idx_array) {
                    pc_8x8_tof_left_msg_map_[index] = pc_msgs[i]; // 앞쪽 0~15
                    i++;
                }

                for (auto index : sensor_config_.multi_tof_right.sub_cell_idx_array) {
                    pc_8x8_tof_right_msg_map_[index] = pc_msgs[16 + j]; // 뒤쪽 16~31
                    j++;
                }
                if (sensor_config_.multi_tof.use) pc_tof_multi_msg = pointcloud_generator_.mergePointCloud2Vector(pc_msgs, target_frame_);
            } else {
                #if IS_4X4_INDEX
                for (int i=0; i<16; i++) {
                    pc_4x4_tof_left_msg_map_[i] = pc_msgs[i];
                }
                for (int i=0; i<16; i++) {
                    pc_4x4_tof_right_msg_map_[i] = pc_msgs[16+i];
                }
                #else
                for (int i=0; i<4; ++i) {
                    int start_idx = i * 4;
                    std::vector<sensor_msgs::msg::PointCloud2> slice(pc_msgs.begin() + start_idx, pc_msgs.begin() + start_idx + 4);
                    pc_4x4_tof_left_msg_map_[i] = pointcloud_generator_.mergePointCloud2Vector(slice, target_frame_);
                }
                for (int i=0; i<4; ++i) {
                    int start_idx = 16 + i * 4;
                    std::vector<sensor_msgs::msg::PointCloud2> slice(pc_msgs.begin() + start_idx, pc_msgs.begin() + start_idx + 4);
                    pc_4x4_tof_right_msg_map_[i] = pointcloud_generator_.mergePointCloud2Vector(slice, target_frame_);
                }
                #endif
                if (sensor_config_.multi_tof.use) pc_tof_multi_msg = pointcloud_generator_.mergePointCloud2Vector(pc_msgs, target_frame_);
            }
        }
    }

    isTofUpdating = true;
}

void SensorToPointcloud::cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg)
{
    if (wasActiveSensorToPointcloud_camera && !isActiveSensorToPointcloud) {
        if (use_camera_log_) camera_object_logger_.logInfoClear();
        if (sensor_config_.camera.use) pc_camera_msg = sensor_msgs::msg::PointCloud2();
        RCLCPP_INFO(this->get_logger(), "Camera PointCloud Msg Clear");
    }
    wasActiveSensorToPointcloud_camera = isActiveSensorToPointcloud;
    if (!isActiveSensorToPointcloud) return;

    if (target_frame_ == "map" || sensor_config_.camera.use) {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        bounding_box_generator_.updateRobotPose(pose);
    }

    if (use_camera_log_) {
        camera_object_logger_.log(bounding_box_generator_.getObjectBoundingBoxInfo(msg, camera_class_id_confidence_th_, sensor_config_.camera.direction, sensor_config_.camera.object_max_dist));
    }
    if (sensor_config_.camera.use) {
        bbox_msg = bounding_box_generator_.generateBoundingBoxMessage(msg, camera_class_id_confidence_th_, sensor_config_.camera.direction, sensor_config_.camera.object_max_dist);
        pc_camera_msg = point_cloud_camera_.updateCameraPointCloudMsg(bbox_msg, sensor_config_.camera.pc_resolution);
    }

    isCameraUpdating = true;
}

void SensorToPointcloud::cliffMsgUpdate(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    if (wasActiveSensorToPointcloud_cliff && !isActiveSensorToPointcloud) {
        if (sensor_config_.cliff.use) pc_cliff_msg = sensor_msgs::msg::PointCloud2();
        RCLCPP_INFO(this->get_logger(), "Cliff PointCloud Msg Clear");
    }
    wasActiveSensorToPointcloud_cliff = isActiveSensorToPointcloud;
    if (!isActiveSensorToPointcloud) return;

    if (target_frame_ == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_cliff_.updateRobotPose(pose);
    }

    if (sensor_config_.cliff.use) pc_cliff_msg = point_cloud_cliff_.updateCliffPointCloudMsg(msg);

    isCliffUpdating = true;
}

void SensorToPointcloud::collisionMsgUpdate(const robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg)
{
    if (wasActiveSensorToPointcloud_collision && !isActiveSensorToPointcloud) {
        if (sensor_config_.collision.use) pc_collision_msg = sensor_msgs::msg::PointCloud2();
        RCLCPP_INFO(this->get_logger(), "Collision PointCloud Msg Clear");
    }
    wasActiveSensorToPointcloud_collision = isActiveSensorToPointcloud;
    if (!isActiveSensorToPointcloud) return;

    if (target_frame_ == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_collosion_.updateRobotPose(pose);
    }

    if (sensor_config_.collision.use && msg->event_trigger) pc_collision_msg = point_cloud_collosion_.updateCollisionPointCloudMsg(msg);

    isCollisionUpdating = true;
}
