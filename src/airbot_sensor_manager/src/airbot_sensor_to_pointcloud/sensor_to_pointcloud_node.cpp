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
double collision_forward_point_offset = 0.19;           //[meter]

SensorToPointcloud::SensorToPointcloud()
    : rclcpp::Node("airbot_sensor_to_pointcloud"), clock_(RCL_STEADY_TIME),
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
    mtof_calibration_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "start_tofcalib", 10, std::bind(&SensorToPointcloud::mToFCalibrationCmdCallback, this, std::placeholders::_1));

    // Msg Subscribers
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensorToPointcloud::tofMsgUpdate, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "camera_data", 10, std::bind(&SensorToPointcloud::cameraMsgUpdate, this, std::placeholders::_1));
    cliff_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_ir_data", 10, std::bind(&SensorToPointcloud::cliffMsgUpdate, this, std::placeholders::_1));
    collision_sub_ = this->create_subscription<robot_custom_msgs::msg::AbnormalEventData>(
        "collision_detected", 10, std::bind(&SensorToPointcloud::collisionMsgUpdate, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data", 10, std::bind(&SensorToPointcloud::imuCallback, this, std::placeholders::_1));

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.best_effort();
    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/init_pose",
            qos_profile,
            std::bind(&SensorToPointcloud::init_pose_callback, this, std::placeholders::_1));

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
    isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
    isCompleteMToFCalibration = false;
    mtof_calib_left_finish_time_ = clock_.now();
    mtof_calib_right_finish_time_ = clock_.now();
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

    ramp_cnt_ = 0;
    ramp_detected_ = false;
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
    // tof_debug_pub_ = this->create_publisher<robot_custom_msgs::msg::TofData>(
    //     "filtered_tof_data", 10
    // );
    sensor_to_pointcloud_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "sensor_to_pointcloud_active", 10
    );

    mtof_calibration_complete_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "perception/calibration/update", 10
    );

    mtof_calibration_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "tof_calib_state", 10
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
    std::ostringstream oss;

    oss << "\n================== SENSOR MANAGER PARAMETERS ==================\n";
    oss << "[General]\n";
    oss << "  Target Frame: '" << target_frame_ << "'\n";

    oss << "[TOF Settings]\n";
    appendSensorConfig(oss, "TOF 1D", sensor_config_.one_d_tof);
    appendSensorConfig(oss, "TOF Multi", sensor_config_.multi_tof);
    oss << "    TOF Multi 8x8 Use: " << (use_tof_8x8_ ? "True" : "False") << "\n";
    oss << "    Calibration: " << "\n";
    oss << "      Method: " << sensor_config_.multi_tof.calibration.method << "\n";
    oss << "      Sampling count: " << sensor_config_.multi_tof.calibration.sampling_count << "\n";
    oss << "      Pass Min Value: " << sensor_config_.multi_tof.calibration.pass_min_value << "\n";
    oss << "      Pass Max Value: " << sensor_config_.multi_tof.calibration.pass_max_value << "\n";
    oss << "      Pass Diff Threshold: " << sensor_config_.multi_tof.calibration.pass_diff_th << "\n";
    oss << "      Time Out Threshold: " << sensor_config_.multi_tof.calibration.timeout << "\n";
    appendSensorConfig(oss, "TOF Multi Left", sensor_config_.multi_tof_left);
    appendSensorConfig(oss, "TOF Multi Right", sensor_config_.multi_tof_right);

    oss << "---------------------- FILTERS PARAMETERS ---------------------\n";
    appendFilterConfig(oss, "Moving Average", mtof_filter_.moving_average);
    appendFilterConfig(oss, "Low Pass", mtof_filter_.low_pass);
    appendFilterConfig(oss, "Complementary", mtof_filter_.complementary);
    oss << "---------------------------------------------------------------\n";

    oss << "[Camera Settings]\n";
    appendSensorConfig(oss, "Camera Front", sensor_config_.camera);

    oss << "[Cliff Settings]\n";
    appendSensorConfig(oss, "Cliff", sensor_config_.cliff);

    oss << "[Collision Settings]\n";
    appendSensorConfig(oss, "Collision", sensor_config_.collision);

    oss << "===============================================================\n";
    oss << "Parameter update finished!";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void SensorToPointcloud::appendSensorConfig(std::ostringstream& oss, const std::string& name, const tSensor& cfg)
{
    oss << "  [" << name << "]\n";
    oss << "    Use: " << (cfg.use ? "True" : "False") << "\n";

#if IS_4X4_INDEX
    oss << "    Topic: " << cfg.topic_idx << "\n";
#else
    oss << "    Topic: " << (cfg.use ? cfg.topic_idx : cfg.topic_row) << "\n";
#endif

    oss << "    Publish Rate: " << cfg.publish_rate << " ms\n";

    if (cfg.pitch_angle_deg != 0.0 || !cfg.sub_cell_idx_array.empty()) {
        oss << "    Pitch Angle: " << std::fixed << std::setprecision(2) << cfg.pitch_angle_deg << " deg\n";
        if (cfg.sub_cell_idx_array.size() == 16) {
            oss << "    Sub Cell Index Array:\n";
            for (int i = 0; i < 4; ++i) {
                oss << "      [ ";
                for (int j = 0; j < 4; ++j) {
                    oss << cfg.sub_cell_idx_array[i * 4 + j] << " ";
                }
                oss << "]\n";
            }
        }
    }

    if (cfg.pc_resolution > 0.0 || !cfg.class_id.empty()) {
        oss << "    Point Cloud Resolution: " << cfg.pc_resolution << "\n";
        oss << "    Direction: " << (cfg.direction ? "Forward(CCW+)" : "Reverse(CW+)") << "\n";
        oss << "    Object Max Distance: " << cfg.object_max_dist << "\n";
        oss << "    Object Pitch Threshold: " << RAD2DEG(cfg.camera_object_ignore_pitch_th) << " deg\n";
        if (!cfg.class_id.empty()) {
            oss << "    Target Objects (ID): [ ";
            for (const auto& id : cfg.class_id) {
                oss << "(" << id << "), ";
            }
            oss << "]\n";
        }
    }
}

void SensorToPointcloud::appendFilterConfig(std::ostringstream& oss, const std::string& name, const tFilter& filter)
{
    oss << "     " << name << " Filter\n";
    oss << "        use: " << (filter.use ? "true" : "false") << "\n";

    if (name == "Moving Average") {
        oss << "        window_size: " << filter.window_size << "\n";
        oss << std::fixed << std::setprecision(2);
        oss << "        max_distance_th: " << filter.max_distance_th << "\n";
    } else if (name == "Low Pass" || name == "Complementary") {
        oss << std::fixed << std::setprecision(2);
        oss << "        alpha: " << filter.alpha << "\n";
    }

    oss << "        enabled_4x4_idx: [";
    for (size_t i = 0; i < filter.enabled_4x4_idx.size(); ++i) {
        oss << filter.enabled_4x4_idx[i];
        if (i != filter.enabled_4x4_idx.size() - 1)
            oss << " ";
    }
    oss << "]\n";
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
    if (node["object_ignore_pitch_th_deg"]) cfg.camera_object_ignore_pitch_th = DEG2RAD(node["object_ignore_pitch_th_deg"].as<double>());
    if (node["class_id_confidence_th"]) {
        for (auto conf_node : node["class_id_confidence_th"]) {
            cfg.class_id.push_back(conf_node.as<std::string>());
        }
    }
    YAML::Node calib = node["calibration"];
    if (calib && calib["method"]) cfg.calibration.method = calib["method"].as<std::string>();
    if (calib && calib["sampling_count"]) cfg.calibration.sampling_count = calib["sampling_count"].as<int>();
    if (calib && calib["pass_min_value"]) cfg.calibration.pass_min_value = calib["pass_min_value"].as<double>();
    if (calib && calib["pass_max_value"]) cfg.calibration.pass_max_value = calib["pass_max_value"].as<double>();
    if (calib && calib["pass_diff_th"]) cfg.calibration.pass_diff_th = calib["pass_diff_th"].as<double>();
    if (calib && calib["time_out_sec"]) cfg.calibration.timeout = calib["time_out_sec"].as<double>();

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

void SensorToPointcloud::mToFCalibrationCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    // start_left_calibration = 0x01, start_right_calibration = 0x02, unknown = else,
    isActiveMToFCalibration = static_cast<MTOF_CALIB_STATE>(msg->data);
    if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT || isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT) {
        if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT) {
            bLeftMToFCalibrationSet = false;
            mtof_calib_sample_count = 0;
        } else { // MTOF_CALIB_STATE::ACTIVE_RIGHT
            bRightMToFCalibrationSet = false;
            mtof_calib_sample_count = 0;
        }
        RCLCPP_INFO(this->get_logger(), "multi-ToF Calibration Cmd : [%s]", enumToString(isActiveMToFCalibration).c_str());
    } else {
        isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
        RCLCPP_INFO(this->get_logger(), "multi-ToF Calibration Wrong Cmd : [%d], Set State => [%s]", msg->data, enumToString(isActiveMToFCalibration).c_str());
    }
}

void SensorToPointcloud::tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if ((!bLeftMToFCalibrationSet || !bRightMToFCalibrationSet) &&
        (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT || isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT)) {

        if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT && !bLeftMToFCalibrationSet) {
            handleCalibrationSide(msg, TOF_SIDE::LEFT, bLeftMToFCalibrationSet, mtof_calib_left_finish_time_);
        }

        if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT && !bRightMToFCalibrationSet) {
            handleCalibrationSide(msg, TOF_SIDE::RIGHT, bRightMToFCalibrationSet, mtof_calib_right_finish_time_);
        }

        if (bLeftMToFCalibrationSet && bRightMToFCalibrationSet) {
            std_msgs::msg::Float32MultiArray msg_arr;
            msg_arr.data.assign(mtof_calib_result_array_.begin(), mtof_calib_result_array_.end());
            mtof_calibration_complete_pub_->publish(msg_arr);
            RCLCPP_INFO(this->get_logger(), "[Calibration Result: PASS] Publish m-ToF Calibration Data to A1_Perception");
            bLeftMToFCalibrationSet = false;
            bRightMToFCalibrationSet = false;
        }

        return;
    } else {
        auto now = clock_.now();
        auto time_diff_right = (now - mtof_calib_left_finish_time_).seconds();
        auto time_diff_left = (now - mtof_calib_right_finish_time_).seconds();

        if (bLeftMToFCalibrationSet && !bRightMToFCalibrationSet && time_diff_right > sensor_config_.multi_tof.calibration.timeout) {
            isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
            bLeftMToFCalibrationSet = false;
            calib_state_msg.data = make_mtof_state(TOF_SIDE::RIGHT, MTOF_CALIB_RESULT::FAIL_TIME_OUT);
            mtof_calibration_state_pub_->publish(calib_state_msg);
            RCLCPP_INFO(this->get_logger(),
                "[Calibration: TIMEOUT] Right calibration command not received within %.2f sec after Left finished. Calibration state set to [%s]",
                time_diff_right, enumToString(isActiveMToFCalibration).c_str()
            );
            mtof_calib_left_finish_time_ = now;
        }

        if (!bLeftMToFCalibrationSet && bRightMToFCalibrationSet && time_diff_left > sensor_config_.multi_tof.calibration.timeout) {
            isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
            bRightMToFCalibrationSet = false;
            calib_state_msg.data = make_mtof_state(TOF_SIDE::LEFT, MTOF_CALIB_RESULT::FAIL_TIME_OUT);
            mtof_calibration_state_pub_->publish(calib_state_msg);
            RCLCPP_INFO(this->get_logger(),
                "[Calibration: TIMEOUT] Left calibration command not received within %.2f sec after Right finished. Calibration state set to [%s]",
                time_diff_left, enumToString(isActiveMToFCalibration).c_str()
            );
            mtof_calib_right_finish_time_ = now;
        }
    }

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
    // tof_debug_pub_->publish(*filtered_msg);

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
    if (!isActiveSensorToPointcloud) {
        ramp_cnt_ = 0;
        ramp_detected_ = false;
        prev_camera_logged_ids.clear();
        return;
    }

    if (isDetectRamp()) {
        if (msg->num > 0) {
            std::unordered_set<int> current_ids;

            for (int i = 0; i < msg->num; i++) {
                current_ids.insert(msg->data_array[i].id);
            }

            if (current_ids != prev_camera_logged_ids) {
                std::stringstream ss;
                ss << "[Camera_Msg_Update] Slope Detected! (Roll: " << RAD2DEG(roll_) << " deg, Pitch: " << RAD2DEG(pitch_) << " deg)  Filtered Object ID: ";

                int count = 0;
                for (int id : current_ids) {
                    if (count++ > 0) ss << ", ";
                    ss << id;
                }

                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                prev_camera_logged_ids = current_ids;
            }
        }

        return;
    } else {
        prev_camera_logged_ids.clear();
    }

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
        cam_object_ids.clear();
        bbox_msg = bounding_box_generator_.generateBoundingBoxMessage(msg, camera_class_id_confidence_th_, sensor_config_.camera.direction, sensor_config_.camera.object_max_dist, cam_object_ids);
        pc_camera_msg = point_cloud_camera_.updateCameraPointCloudMsg(bbox_msg, sensor_config_.camera.pc_resolution, cam_object_ids, init_pose_msg);
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

void SensorToPointcloud::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, yaw_);
}

bool SensorToPointcloud::isDetectRamp()
{
    bool ret = false;

    if (abs(roll_) >= sensor_config_.camera.camera_object_ignore_pitch_th ||
        abs(pitch_) >= sensor_config_.camera.camera_object_ignore_pitch_th) {
        ramp_detected_ = true;
        ramp_cnt_ = 0;
        ret = true;
    } else {
        if (ramp_detected_) {
            ramp_cnt_++;
            if (ramp_cnt_ > 20) { // 경사로 감지 해제 후 1초 뒤 객체인식 재개 (camera_data 토픽 주기: 50ms)
                ramp_detected_ = false;
                ramp_cnt_ = 0;
                ret = false;
            } else {
                ret = true;
            }
        } else {
            ret = false;
        }
    }

    return ret;
}

/**
 * @brief [MSB: Right] / [LSB: Left]
 * Running:         1
 * Complete:        2
 * Out of Range:    3
 * Unstable Range:  4
 * Time Out:        8
 */
uint8_t SensorToPointcloud::make_mtof_state(TOF_SIDE side, MTOF_CALIB_RESULT state)
{
    uint8_t value = 0;

    switch (state) {
        case MTOF_CALIB_RESULT::RUNNING:                value = 0x01; break;
        case MTOF_CALIB_RESULT::PASS:                   value = 0x02; break;
        case MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE:      value = 0x03; break;
        case MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE:    value = 0x04; break;
        case MTOF_CALIB_RESULT::FAIL_TIME_OUT:          value = 0x08; break;
        default:                                        value = 0x00; break;
    }

    if (side == TOF_SIDE::RIGHT) {
        value = (value & 0x0F) << 4;  // Left 0x0?, Right 0x?0
    }

    return value;
}

void SensorToPointcloud::handleCalibrationSide(const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, bool &side_calib_set, rclcpp::Time &side_finish_time)
{
    MTOF_CALIB_RESULT result = multiToFCalibration(msg);

    if (result != MTOF_CALIB_RESULT::RUNNING) {
        RCLCPP_INFO(this->get_logger(),
            "[Calibration: %s] SIDE: %s",
            enumToString(result).c_str(),
            enumToString(isActiveMToFCalibration).c_str()
        );

        isActiveMToFCalibration = MTOF_CALIB_STATE::INACTIVE;
        side_finish_time = clock_.now();

        if (result == MTOF_CALIB_RESULT::PASS) {
            side_calib_set = true;
        }
    }

    std_msgs::msg::UInt8 calib_state_msg;
    calib_state_msg.data = make_mtof_state(side, result);
    mtof_calibration_state_pub_->publish(calib_state_msg);
}

MTOF_CALIB_RESULT SensorToPointcloud::multiToFCalibration(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    MTOF_CALIB_RESULT ret = MTOF_CALIB_RESULT::RUNNING;

    TOF_SIDE side;
    if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_LEFT) {
        side = TOF_SIDE::LEFT;
    } else if (isActiveMToFCalibration == MTOF_CALIB_STATE::ACTIVE_RIGHT) {
        side = TOF_SIDE::RIGHT;
    } else {
        RCLCPP_INFO(this->get_logger(), "Wrong Clib Command");
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    // Min,Max 기준값의 tf 변환
    auto pnp_th_msg = std::make_shared<robot_custom_msgs::msg::TofData>();
    std_msgs::msg::Float32MultiArray min_th;
    std_msgs::msg::Float32MultiArray max_th;
    auto set_pass_values = [&](double value) {
        for (int idx : {13, 14, 15}) {
            pnp_th_msg->bot_left[idx]  = value;
            pnp_th_msg->bot_right[idx] = value;
        }
    };
    if (pnp_th_msg->bot_left.size() == 16 && pnp_th_msg->bot_right.size() == 16) {
        set_pass_values(sensor_config_.multi_tof.calibration.pass_min_value);
        min_th = point_cloud_tof_.updateBotTofCalibrationData(pnp_th_msg, side, botTofPitchAngle_);
        set_pass_values(sensor_config_.multi_tof.calibration.pass_max_value);
        max_th = point_cloud_tof_.updateBotTofCalibrationData(pnp_th_msg, side, botTofPitchAngle_);
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Invalid pnp_th_msg size (left: %zu, right: %zu)",
            pnp_th_msg->bot_left.size(), pnp_th_msg->bot_right.size()
        );
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    // input 데이터의 tf 변환
    std_msgs::msg::Float32MultiArray result;
    if (msg->bot_left.size() == 16 && msg->bot_right.size() == 16) {
        result = point_cloud_tof_.updateBotTofCalibrationData(msg, side, botTofPitchAngle_);
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Invalid msg size (left: %zu, right: %zu)",
            msg->bot_left.size(), msg->bot_right.size()
        );
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }
    const auto& arr = result.data;

    if (arr.size() <= 2) {
        RCLCPP_WARN(this->get_logger(),
            "updateBotTofCalibrationData() returned size=%zu (<3). Skip sample.",
            arr.size()
        );
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    if (mtof_calib_sample_count == 0) {
        mtof_calib_stat13 = mtof_calib_stat14 = mtof_calib_stat15 = -1;
        if (mtof_calib_max_samples13.size() > 0) mtof_calib_max_samples13.clear();
        if (mtof_calib_max_samples14.size() > 0) mtof_calib_max_samples14.clear();
        if (mtof_calib_max_samples15.size() > 0) mtof_calib_max_samples15.clear();
        if (mtof_calib_median_samples13.size() > 0) mtof_calib_median_samples13.clear();
        if (mtof_calib_median_samples14.size() > 0) mtof_calib_median_samples14.clear();
        if (mtof_calib_median_samples15.size() > 0) mtof_calib_median_samples15.clear();
    }

    if (sensor_config_.multi_tof.calibration.method == "Max") {
        mtof_calib_max_samples13.push_back(arr[0]);
        mtof_calib_max_samples14.push_back(arr[1]);
        mtof_calib_max_samples15.push_back(arr[2]);
    } else if (sensor_config_.multi_tof.calibration.method == "Median") {
        mtof_calib_median_samples13.push_back(arr[0]);
        mtof_calib_median_samples14.push_back(arr[1]);
        mtof_calib_median_samples15.push_back(arr[2]);
    }

    ++mtof_calib_sample_count;

    // sample 수집 완료 후 판단
    if (mtof_calib_sample_count >= sensor_config_.multi_tof.calibration.sampling_count) {
        if (sensor_config_.multi_tof.calibration.method == "Max") {

            if (!mtof_calib_max_samples13.empty()) {
                mtof_calib_stat13 = *std::max_element(mtof_calib_max_samples13.begin(), mtof_calib_max_samples13.end());
            }
            if (!mtof_calib_max_samples14.empty()) {
                mtof_calib_stat14 = *std::max_element(mtof_calib_max_samples14.begin(), mtof_calib_max_samples14.end());
            }
            if (!mtof_calib_max_samples15.empty()) {
                mtof_calib_stat15 = *std::max_element(mtof_calib_max_samples15.begin(), mtof_calib_max_samples15.end());
            }
            std::vector<float> non_zero_samples;
            std::copy_if(
                mtof_calib_max_samples14.begin(),
                mtof_calib_max_samples14.end(),
                std::back_inserter(non_zero_samples),
                [](float v){ return v >= 0.20f; }
            );

            if (min_th.data.size() == 3 && max_th.data.size() == 3 && !non_zero_samples.empty()) {
                float max_val = *std::max_element(non_zero_samples.begin(), non_zero_samples.end());
                float min_val = *std::min_element(non_zero_samples.begin(), non_zero_samples.end());

                if (mtof_calib_stat13 < min_th.data[0] || mtof_calib_stat13 > max_th.data[0]
                || mtof_calib_stat14 < min_th.data[1] || mtof_calib_stat14 > max_th.data[1]
                || mtof_calib_stat15 < min_th.data[2] || mtof_calib_stat15 > max_th.data[2])
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[Calibration: FAIL]\n"
                        "  idx_13: %.3f (min_th: %.3f, max_th: %.3f)\n"
                        "  idx_14: %.3f (min_th: %.3f, max_th: %.3f)\n"
                        "  idx_15: %.3f (min_th: %.3f, max_th: %.3f)",
                        mtof_calib_stat13, min_th.data[0], max_th.data[0],
                        mtof_calib_stat14, min_th.data[1], max_th.data[1],
                        mtof_calib_stat15, min_th.data[2], max_th.data[2]
                    );
                    ret = MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE;
                }
                else if ((max_val - min_val) >= sensor_config_.multi_tof.calibration.pass_diff_th)
                {
                    ret = MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE;
                }
                else
                {
                    ret = MTOF_CALIB_RESULT::PASS;
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Invalid threshold data size (min_th: %zu, max_th: %zu)",
                    min_th.data.size(), max_th.data.size()
                );
                return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
            }
        } else if (sensor_config_.multi_tof.calibration.method == "Median") {

            auto calcMedian = [](std::vector<float> v) {
                std::sort(v.begin(), v.end());
                size_t n = v.size();
                if (n % 2 == 0) {
                    return (v[n / 2 - 1] + v[n / 2]) / 2.0f;
                } else {
                    return v[n / 2];
                }
            };
            mtof_calib_stat13 = calcMedian(mtof_calib_median_samples13);
            mtof_calib_stat14 = calcMedian(mtof_calib_median_samples14);
            mtof_calib_stat15 = calcMedian(mtof_calib_median_samples15);
            std::vector<float> non_zero_samples;
            std::copy_if(
                mtof_calib_median_samples14.begin(),
                mtof_calib_median_samples14.end(),
                std::back_inserter(non_zero_samples),
                [](float v){ return v >= 0.20f; }
            );

            if (min_th.data.size() == 3 && max_th.data.size() == 3 && !non_zero_samples.empty()) {
                float max_val = *std::max_element(non_zero_samples.begin(), non_zero_samples.end());
                float min_val = *std::min_element(non_zero_samples.begin(), non_zero_samples.end());

                if (mtof_calib_stat13 < min_th.data[0] || mtof_calib_stat13 > max_th.data[0]
                 || mtof_calib_stat14 < min_th.data[1] || mtof_calib_stat14 > max_th.data[1]
                 || mtof_calib_stat15 < min_th.data[2] || mtof_calib_stat15 > max_th.data[2])
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[Calibration: FAIL]\n"
                        "  idx_13: %.3f (min_th: %.3f, max_th: %.3f)\n"
                        "  idx_14: %.3f (min_th: %.3f, max_th: %.3f)\n"
                        "  idx_15: %.3f (min_th: %.3f, max_th: %.3f)",
                        mtof_calib_stat13, min_th.data[0], max_th.data[0],
                        mtof_calib_stat14, min_th.data[1], max_th.data[1],
                        mtof_calib_stat15, min_th.data[2], max_th.data[2]
                    );
                    ret = MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE;
                }
                else if ((max_val - min_val) >= sensor_config_.multi_tof.calibration.pass_diff_th)
                {
                    ret = MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE;
                }
                else
                {
                    ret = MTOF_CALIB_RESULT::PASS;
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Invalid threshold data size (min_th: %zu, max_th: %zu)",
                    min_th.data.size(), max_th.data.size()
                );
                return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
            }
        }

        // 결과 로깅
        RCLCPP_INFO(
            this->get_logger(),
            "[Calibration: PASS] Method: %s | Samples: %d\n"
            "  idx_13: %.3f\n"
            "  idx_14: %.3f\n"
            "  idx_15: %.3f",
            sensor_config_.multi_tof.calibration.method.c_str(),
            mtof_calib_sample_count,
            mtof_calib_stat13,
            mtof_calib_stat14,
            mtof_calib_stat15
        );

        // 외부 노드(A1_perception)로 보낼 Float64MultiArray에 데이터 저장
        if (side == TOF_SIDE::LEFT) {
            mtof_calib_result_array_[0] = mtof_calib_stat13;
            mtof_calib_result_array_[1] = mtof_calib_stat14;
            mtof_calib_result_array_[2] = mtof_calib_stat15;
        } else {
            mtof_calib_result_array_[3] = mtof_calib_stat13;
            mtof_calib_result_array_[4] = mtof_calib_stat14;
            mtof_calib_result_array_[5] = mtof_calib_stat15;
        }

        mtof_calib_sample_count = 0;
        mtof_calib_stat13 = mtof_calib_stat14 = mtof_calib_stat15 = -1.0;
    }

    return ret;
}

void SensorToPointcloud::init_pose_callback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    init_pose_msg.header.stamp = this->get_clock()->now();
    init_pose_msg.header.frame_id = msg->header.frame_id;
    init_pose_msg.pose.pose.position.x = msg->pose.pose.position.x;
    init_pose_msg.pose.pose.position.y = msg->pose.pose.position.y;
    init_pose_msg.pose.pose.position.z = msg->pose.pose.position.z;
    init_pose_msg.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    init_pose_msg.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    init_pose_msg.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    init_pose_msg.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    for (size_t i = 0; i < 36; ++i)
    {
        init_pose_msg.pose.covariance[i] = msg->pose.covariance[i];
    }
}