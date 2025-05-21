#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud_node.hpp"

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
    declareParams();
    setParams();
    initVariables();
    isActiveSensorToPointcloud = false;

    // Dynamic Parameter Handler
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    target_frame_callback_handle_ = param_handler_->add_parameter_callback(
        "target_frame",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                std::string before = target_frame_;
                target_frame_ = param.as_string();
                updateAllFrames();
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

    // Update Parameters
    updateAllParameters();
    printParams();
    RCLCPP_INFO(this->get_logger(), "All Parameters init finished!");

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

    // Msg Publishers
    initPublisher();

    // Monitor Timer
    poincloud_publish_timer_ = this->create_wall_timer(
        10ms, std::bind(&SensorToPointcloud::publisherMonitor, this));
}

SensorToPointcloud::~SensorToPointcloud()
{
    param_handler_.reset();
}

void SensorToPointcloud::init()
{
    camera_object_logger_.setNode(shared_from_this());
}

void SensorToPointcloud::updateAllParameters()
{
    updateAllFrames();
    tof_lp_filter_.updateParams(mtof_lp_filter_alpha_, mtof_lp_filter_enabled_4x4_idx_);
    tof_ma_filter_.updateParams(mtof_average_window_size_, mtof_ma_filter_enabled_4x4_idx_, mtof_ma_max_distance_th_);
    tof_comp_filter_.updateParams(mtof_complementary_alpha_, mtof_lp_filter_alpha_, mtof_average_window_size_, mtof_comp_filter_enabled_4x4_idx_);
    point_cloud_tof_.updateTofMode(use_tof_8x8_);
    point_cloud_tof_.updateLeftSubCellIndexArray(mtof_left_sub_cell_idx_array_);
    point_cloud_tof_.updateRightSubCellIndexArray(mtof_right_sub_cell_idx_array_);
    camera_object_logger_.updateParams(camera_logger_distance_margin_,camera_logger_width_margin_,camera_logger_height_margin_);
    for (const auto& item : camera_param_raw_vector_) {
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
}

void SensorToPointcloud::updateAllFrames()
{
    point_cloud_tof_.updateTargetFrame(target_frame_);
    bounding_box_generator_.updateTargetFrame(target_frame_);
    point_cloud_cliff_.updateTargetFrame(target_frame_);
    point_cloud_collosion_.updateTargetFrame(target_frame_);
}

void SensorToPointcloud::declareParams()
{
    this->declare_parameter("target_frame","base_link");

    this->declare_parameter("tof.all.use",false);
    this->declare_parameter("tof.1D.use",false);
    this->declare_parameter("tof.1D.publish_rate_ms",100);
    this->declare_parameter("tof.1D.tilting_angle_deg",0.0);
    this->declare_parameter("tof.multi.publish_rate_ms",100);
    this->declare_parameter("tof.multi.enable_8x8", false);
    this->declare_parameter("tof.multi.filter.moving_average.use",false);
    this->declare_parameter("tof.multi.filter.moving_average.enabled_4x4_idx", std::vector<int64_t>());
    this->declare_parameter("tof.multi.filter.moving_average.window_size",0);
    this->declare_parameter("tof.multi.filter.moving_average.max_distance_th",0.0);
    this->declare_parameter("tof.multi.filter.low_pass.use",false);
    this->declare_parameter("tof.multi.filter.low_pass.enabled_4x4_idx", std::vector<int64_t>());
    this->declare_parameter("tof.multi.filter.low_pass.alpha", 0.0);
    this->declare_parameter("tof.multi.filter.complementary.use",false);
    this->declare_parameter("tof.multi.filter.complementary.enabled_4x4_idx", std::vector<int64_t>{});
    this->declare_parameter("tof.multi.filter.complementary.alpha", 0.0);
    this->declare_parameter("tof.multi.left.use",false);
    this->declare_parameter("tof.multi.left.pitch_angle_deg",0.0);
    this->declare_parameter("tof.multi.left.sub_cell_idx_array",std::vector<int64_t>(16, 0));
    this->declare_parameter("tof.multi.right.use",false);
    this->declare_parameter("tof.multi.right.pitch_angle_deg",0.0);
    this->declare_parameter("tof.multi.right.sub_cell_idx_array",std::vector<int64_t>(16, 0));
    this->declare_parameter("tof.multi.row.use",false);
    this->declare_parameter("tof.multi.row.publish_rate_ms",100);

    this->declare_parameter("camera.use",false);
    this->declare_parameter("camera.publish_rate_ms",100);
    this->declare_parameter("camera.pointcloud_resolution",0.05);
    this->declare_parameter("camera.class_id_confidence_th",std::vector<std::string>());
    this->declare_parameter("camera.object_direction",false);
    this->declare_parameter("camera.object_max_distance_m",1.0);
    this->declare_parameter("camera.logger.use",false);
    this->declare_parameter("camera.logger.margin.distance_diff",1.0);
    this->declare_parameter("camera.logger.margin.width_diff",1.0);
    this->declare_parameter("camera.logger.margin.height_diff",1.0);

    this->declare_parameter("cliff.use",false);
    this->declare_parameter("cliff.publish_rate_ms",100);

    this->declare_parameter("collision.use",false);
    this->declare_parameter("collision.publish_rate_ms",100);
}

void SensorToPointcloud::setParams()
{
    this->get_parameter("target_frame", target_frame_);

    this->get_parameter("tof.all.use", use_tof_);
    this->get_parameter("tof.1D.use", use_tof_1D_);
    this->get_parameter("tof.1D.publish_rate_ms", publish_rate_1d_tof_);
    this->get_parameter("tof.1D.tilting_angle_deg", tilting_ang_1d_tof_);
    this->get_parameter("tof.multi.publish_rate_ms", publish_rate_multi_tof_);
    this->get_parameter("tof.multi.enable_8x8", use_tof_8x8_);
    this->get_parameter("tof.multi.filter.moving_average.use", use_mtof_ma_filter_);
    std::vector<int64_t> tmp64_ma;
    this->get_parameter("tof.multi.filter.moving_average.enabled_4x4_idx", tmp64_ma);
    mtof_ma_filter_enabled_4x4_idx_.assign(tmp64_ma.begin(), tmp64_ma.end());
    this->get_parameter("tof.multi.filter.moving_average.window_size", mtof_average_window_size_);
    this->get_parameter("tof.multi.filter.moving_average.max_distance_th", mtof_ma_max_distance_th_);
    this->get_parameter("tof.multi.filter.low_pass.use", use_mtof_lp_filter_);
    std::vector<int64_t> tmp64_lp;
    this->get_parameter("tof.multi.filter.low_pass.enabled_4x4_idx", tmp64_lp);
    mtof_lp_filter_enabled_4x4_idx_.assign(tmp64_lp.begin(), tmp64_lp.end());
    this->get_parameter("tof.multi.filter.low_pass.alpha", mtof_lp_filter_alpha_);
    this->get_parameter("tof.multi.filter.complementary.use", use_mtof_comp_filter_);
    std::vector<int64_t> tmp64_comp;
    this->get_parameter("tof.multi.filter.complementary.enabled_4x4_idx", tmp64_comp);
    mtof_comp_filter_enabled_4x4_idx_.assign(tmp64_comp.begin(), tmp64_comp.end());
    this->get_parameter("tof.multi.filter.complementary.alpha", mtof_complementary_alpha_);
    this->get_parameter("tof.multi.left.use", use_tof_left_);
    this->get_parameter("tof.multi.left.pitch_angle_deg", bot_left_pitch_angle_);
    std::vector<int64_t> tmp64_left;
    this->get_parameter("tof.multi.left.sub_cell_idx_array", tmp64_left);
    mtof_left_sub_cell_idx_array_.assign(tmp64_left.begin(), tmp64_left.end());
    this->get_parameter("tof.multi.right.use", use_tof_right_);
    this->get_parameter("tof.multi.right.pitch_angle_deg", bot_right_pitch_angle_);
    std::vector<int64_t> tmp64_right;
    this->get_parameter("tof.multi.right.sub_cell_idx_array", tmp64_right);
    mtof_right_sub_cell_idx_array_.assign(tmp64_right.begin(), tmp64_right.end());
    this->get_parameter("tof.multi.row.use", use_tof_row_);
    this->get_parameter("tof.multi.row.publish_rate_ms", publish_rate_row_tof_);

    this->get_parameter("camera.use", use_camera_);
    this->get_parameter("camera.publish_rate_ms", publish_rate_camera_);
    this->get_parameter("camera.pointcloud_resolution", camera_pointcloud_resolution_);
    this->get_parameter("camera.class_id_confidence_th", camera_param_raw_vector_);
    this->get_parameter("camera.object_direction", camera_object_direction_);
    this->get_parameter("camera.object_max_distance_m", object_max_distance_);
    this->get_parameter("camera.logger.use", use_camera_object_logger_);
    this->get_parameter("camera.logger.margin.distance_diff", camera_logger_distance_margin_);
    this->get_parameter("camera.logger.margin.width_diff", camera_logger_width_margin_);
    this->get_parameter("camera.logger.margin.height_diff", camera_logger_height_margin_);

    this->get_parameter("cliff.use", use_cliff_);
    this->get_parameter("cliff.publish_rate_ms", publish_rate_cliff_);

    this->get_parameter("collision.use", use_collision_);
    this->get_parameter("collision.publish_rate_ms", publish_rate_collision_);
}

void SensorToPointcloud::printParams()
{
    RCLCPP_INFO(this->get_logger(), "================== SENSOR MANAGER PARAMETERS ==================");

    // General Settings
    RCLCPP_INFO(this->get_logger(), "[General]");
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", target_frame_.c_str());

    // TOF Settings
    RCLCPP_INFO(this->get_logger(), "[TOF Settings]");
    RCLCPP_INFO(this->get_logger(), "  TOF All Use: %s", use_tof_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF 1D Use: %s", use_tof_1D_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF 1D Publish Rate: %d ms", publish_rate_1d_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF 1D Tilting Angle: %.2f deg", tilting_ang_1d_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Publish Rate: %d ms", publish_rate_multi_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi 8x8 Use: %s", use_tof_8x8_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Moving Average Use: %s", use_mtof_ma_filter_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Moving Average Enabled Row:");
    std::stringstream ma_row_stream;
    ma_row_stream << "    [ ";
    for (int i = 0; i < static_cast<int>(mtof_ma_filter_enabled_4x4_idx_.size()); ++i) {
        ma_row_stream << mtof_ma_filter_enabled_4x4_idx_[i] << " ";
    }
    ma_row_stream << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ma_row_stream.str().c_str());
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Moving Average Window Size: %d", mtof_average_window_size_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Moving Average Max Distance Threshold: %.2f", mtof_ma_max_distance_th_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Low Pass Use: %s", use_mtof_lp_filter_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Low Pass Enabled Row:");
    std::stringstream lp_row_stream;
    lp_row_stream << "    [ ";
    for (int i = 0; i < static_cast<int>(mtof_lp_filter_enabled_4x4_idx_.size()); ++i) {
        lp_row_stream << mtof_lp_filter_enabled_4x4_idx_[i] << " ";
    }
    lp_row_stream << "]";
    RCLCPP_INFO(this->get_logger(), "%s", lp_row_stream.str().c_str());
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Low Pass Alpha: %.2f", mtof_lp_filter_alpha_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Complementary Use: %s", use_mtof_comp_filter_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Complementary Enabled Row:");
    std::stringstream comp_row_stream;
    comp_row_stream << "    [ ";
    for (int i = 0; i < static_cast<int>(mtof_comp_filter_enabled_4x4_idx_.size()); ++i) {
        comp_row_stream << mtof_comp_filter_enabled_4x4_idx_[i] << " ";
    }
    comp_row_stream << "]";
    RCLCPP_INFO(this->get_logger(), "%s", comp_row_stream.str().c_str());
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Filter - Complementary Alpha: %.2f", mtof_complementary_alpha_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Left Use: %s", use_tof_left_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Left Pitch Angle: %.2f", bot_left_pitch_angle_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Left Sub Cell Index Array:");
    for (int i = 0; i < 4; ++i) {
        std::stringstream row_stream;
        row_stream << "    [ ";
        for (int j = 0; j < 4; ++j) {
            row_stream << mtof_left_sub_cell_idx_array_[i * 4 + j] << " ";
        }
        row_stream << "]";
        RCLCPP_INFO(this->get_logger(), "%s", row_stream.str().c_str());
    }
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Right Use: %s", use_tof_right_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Right Pitch Angle: %.2f", bot_right_pitch_angle_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Right Sub Cell Index Array:");
    for (int i = 0; i < 4; ++i) {
        std::stringstream row_stream;
        row_stream << "    [ ";
        for (int j = 0; j < 4; ++j) {
            row_stream << mtof_right_sub_cell_idx_array_[i * 4 + j] << " ";
        }
        row_stream << "]";
        RCLCPP_INFO(this->get_logger(), "%s", row_stream.str().c_str());
    }
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Row Use: %s", use_tof_row_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Row Publish Rate: %d ms", publish_rate_row_tof_);

    // Camera Settings
    RCLCPP_INFO(this->get_logger(), "[Camera Settings]");
    RCLCPP_INFO(this->get_logger(), "  Camera Use: %s", use_camera_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  Camera Publish Rate: %d ms", publish_rate_camera_);
    RCLCPP_INFO(this->get_logger(), "  Camera Pointcloud Resolution: %.2f", camera_pointcloud_resolution_);
    RCLCPP_INFO(this->get_logger(), "  Camera Object Direction: %s", camera_object_direction_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  Camera Object Max Distance: %.2f", object_max_distance_);
    RCLCPP_INFO(this->get_logger(), "  Camera Class ID Confidence Threshold:");
    for (const auto& conf : camera_class_id_confidence_th_) {
        RCLCPP_INFO(this->get_logger(), "    Class ID: %d, Confidence: %d", conf.first, conf.second);
    }
    RCLCPP_INFO(this->get_logger(), "  Camera Logger Use: %s", use_camera_object_logger_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  Camera Logger Margins: Distance Diff: %.2f, Width Diff: %.2f, Height Diff: %.2f",
                camera_logger_distance_margin_,
                camera_logger_width_margin_,
                camera_logger_height_margin_);

    // Cliff Settings
    RCLCPP_INFO(this->get_logger(), "[Cliff Settings]");
    RCLCPP_INFO(this->get_logger(), "  Cliff Use: %s", use_cliff_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  Cliff Publish Rate: %d ms", publish_rate_cliff_);

    // Collision Settings
    RCLCPP_INFO(this->get_logger(), "[Collision Settings]");
    RCLCPP_INFO(this->get_logger(), "  Collision Use: %s", use_collision_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  Collision Publish Rate: %d ms", publish_rate_collision_);
    RCLCPP_INFO(this->get_logger(), "===============================================================");

}

void SensorToPointcloud::initVariables()
{
    isTofUpdating = false;
    isCameraUpdating = false;
    isCliffUpdating = false;
    isCollisionUpdating = false;

    publish_cnt_1d_tof_ = 0;
    publish_cnt_multi_tof_ = 0;
    publish_cnt_row_tof_ = 0;
    publish_cnt_camera_ = 0;
    publish_cnt_cliff_ = 0;
    publish_cnt_collision_ = 0;

    botTofPitchAngle.bot_left = bot_left_pitch_angle_;
    botTofPitchAngle.bot_right = bot_right_pitch_angle_;
}

void SensorToPointcloud::initPublisher()
{
    auto create_pc_pub = [this](const std::string& topic_name) {
        return this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/" + topic_name, 10
        );
    };

    if (use_tof_ && use_tof_1D_) pointcloud_pubs_["tof/mono"] = create_pc_pub("tof/mono");
    if (use_tof_) pointcloud_pubs_["tof/multi"] = create_pc_pub("tof/multi");
    if (use_tof_ && use_tof_row_) {
        if (use_tof_8x8_) {
            for (auto index : mtof_left_sub_cell_idx_array_) {
                pointcloud_pubs_["tof/multi/left/idx_" + std::to_string(index)]
                    = create_pc_pub("tof/multi/left/idx_" + std::to_string(index));
            }
            for (auto index : mtof_right_sub_cell_idx_array_) {
                pointcloud_pubs_["tof/multi/right/idx_" + std::to_string(index)]
                    = create_pc_pub("tof/multi/right/idx_" + std::to_string(index));
            }
        } else {
            for (int i = 0; i < 16; ++i) {
                pointcloud_pubs_["tof/multi/left/idx_" + std::to_string(i)]
                    = create_pc_pub("tof/multi/left/idx_" + std::to_string(i));
                pointcloud_pubs_["tof/multi/right/idx_" + std::to_string(i)]
                    = create_pc_pub("tof/multi/right/idx_" + std::to_string(i));
            }
        }
    }
    if (use_cliff_) pointcloud_pubs_["cliff"] = create_pc_pub("cliff");
    if (use_collision_) pointcloud_pubs_["collision"] = create_pc_pub("collision");
    if (use_camera_) {
        pointcloud_pubs_["camera_object"] = create_pc_pub("camera_object");
        bbox_array_camera_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>(
            "sensor_to_pointcloud/camera/bbox", 10
        );
    }

    //debug
    tof_debug_pub_ = this->create_publisher<robot_custom_msgs::msg::TofData>(
        "filtered_tof_data", 10
    );

    RCLCPP_INFO(this->get_logger(), "Publisher init finished!");
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
    if (use_tof_ && isTofUpdating) { // ToF
        if (use_tof_1D_) {
            if (publish_cnt_1d_tof_ >= publish_rate_1d_tof_) {
                pointcloud_pubs_["tof/mono"]->publish(pc_tof_1d_msg);
                publish_cnt_1d_tof_ = 0;
            }
        }
        if (use_tof_left_ || use_tof_right_) {
            if (publish_cnt_multi_tof_ >= publish_rate_multi_tof_) {
                pointcloud_pubs_["tof/multi"]->publish(pc_tof_multi_msg);
                publish_cnt_multi_tof_ = 0;
            }
        }
        if (use_tof_row_) {
            if (publish_cnt_row_tof_ >= publish_rate_row_tof_) {
                if (use_tof_left_) {
                    if (use_tof_8x8_) {
                        for (auto index : mtof_left_sub_cell_idx_array_) {
                            std::string topic_name = "tof/multi/left/idx_" + std::to_string(index);
                            pointcloud_pubs_[topic_name]->publish(pc_8x8_tof_left_msg_map_[index]);
                        }
                    } else {
                        for (size_t i=0; i<16; i++) {
                            std::string topic_name = "tof/multi/left/idx_" + std::to_string(i);
                            pointcloud_pubs_[topic_name]->publish(pc_4x4_tof_left_msg_map_[i]);
                        }
                    }
                }
                if (use_tof_right_) {
                    if (use_tof_8x8_) {
                        for (auto index : mtof_right_sub_cell_idx_array_) {
                            std::string topic_name = "tof/multi/right/idx_" + std::to_string(index);
                            pointcloud_pubs_[topic_name]->publish(pc_8x8_tof_right_msg_map_[index]);
                        }
                    } else {
                        for (size_t i=0; i<16; i++) {
                            std::string topic_name = "tof/multi/right/idx_" + std::to_string(i);
                            pointcloud_pubs_[topic_name]->publish(pc_4x4_tof_right_msg_map_[i]);
                        }
                    }
                }
                publish_cnt_row_tof_ = 0;
            }
        }
        isTofUpdating = false;
    }
    if (use_camera_ && isCameraUpdating) { // Camera
        if (publish_cnt_camera_ >= publish_rate_camera_) {
            pointcloud_pubs_["camera_object"]->publish(pc_camera_msg);
            bbox_array_camera_pub_->publish(bbox_msg);
            isCameraUpdating = false;
            publish_cnt_camera_ = 0;
        }
    }
    if (use_cliff_ && isCliffUpdating) {
        if (publish_cnt_cliff_ >= publish_rate_cliff_) {
            pointcloud_pubs_["cliff"]->publish(pc_cliff_msg);
            isCliffUpdating = false;
            publish_cnt_cliff_ = 0;
        }
    }
    if (use_collision_ && isCollisionUpdating) {
        if (publish_cnt_collision_ >= publish_rate_collision_) {
            pointcloud_pubs_["collision"]->publish(pc_collision_msg);
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
    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header.stamp = this->now();
    empty_cloud.header.frame_id = target_frame_;
    empty_cloud.height = 1;
    empty_cloud.width = 0;
    empty_cloud.is_dense = false;
    empty_cloud.is_bigendian = false;
    empty_cloud.point_step = 12;  // x, y, z (float32 x 3)
    empty_cloud.row_step = 0;

    // x, y, z 필드 설정
    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    empty_cloud.fields = {field_x, field_y, field_z};
    empty_cloud.data.clear();

    for (auto& [name, pub] : pointcloud_pubs_) {
        if (pub && pub->get_subscription_count() > 0) {
            pub->publish(empty_cloud);
        }
    }

    RCLCPP_INFO(this->get_logger(), "All Active Publisher publish empty_cloud msgs!");
}

void SensorToPointcloud::activeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg) {
        isActiveSensorToPointcloud = msg->data;
        if(isActiveSensorToPointcloud){
            RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : Active");
        }else{
            publishEmptyMsg();
            RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : De-Active");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "cmd_sensor_to_pointcloud topic is a nullptr message.");
    }
}

void SensorToPointcloud::tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if (target_frame_ == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_tof_.updateRobotPose(pose);
    }

    robot_custom_msgs::msg::TofData::SharedPtr filtered_msg = msg;
    if (use_mtof_ma_filter_) filtered_msg = tof_ma_filter_.update(filtered_msg);
    if (use_mtof_lp_filter_) filtered_msg = tof_lp_filter_.update(filtered_msg);
    if (use_mtof_comp_filter_) filtered_msg = tof_comp_filter_.update(filtered_msg);
    tof_debug_pub_->publish(*filtered_msg);

    if (use_tof_) {
        if (use_tof_1D_) {
            pc_tof_1d_msg = point_cloud_tof_.updateTopTofPointCloudMsg(msg, tilting_ang_1d_tof_);
        }
        // if (use_tof_8x8_) {
        if (use_tof_left_ || use_tof_right_) {
            TOF_SIDE side = (use_tof_left_ && use_tof_right_)
                            ? TOF_SIDE::BOTH : (use_tof_left_ ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT);
            auto pc_msgs = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, side, botTofPitchAngle);
            if (side == TOF_SIDE::LEFT) {
                if (use_tof_8x8_) {
                    int i = 0;
                    for (auto index : mtof_left_sub_cell_idx_array_) {
                        pc_8x8_tof_left_msg_map_[index] = pc_msgs[i];
                        i++;
                    }
                } else {
                    for (int i=0; i<16; i++) {
                        pc_4x4_tof_left_msg_map_[i] = pc_msgs[i];
                    }
                }
            } else if (side == TOF_SIDE::RIGHT) {
                if (use_tof_8x8_) {
                    for (auto index : mtof_right_sub_cell_idx_array_) {
                        int i = 0;
                        pc_8x8_tof_right_msg_map_[index] = pc_msgs[i];
                        i++;
                    }
                } else {
                    for (int i=0; i<16; i++) {
                        pc_4x4_tof_right_msg_map_[i] = pc_msgs[i];
                    }
                }
            } else { // TOF_SIDE::BOTH
                if (use_tof_8x8_) {
                    int i = 0;
                    int j = 0;
                    for (auto index : mtof_left_sub_cell_idx_array_) {
                        pc_8x8_tof_left_msg_map_[index] = pc_msgs[i]; // 앞쪽 0~15
                        i++;
                    }

                    for (auto index : mtof_right_sub_cell_idx_array_) {
                        pc_8x8_tof_right_msg_map_[index] = pc_msgs[16 + j]; // 뒤쪽 16~31
                        j++;
                    }
                    pc_tof_multi_msg = pointcloud_generator_.mergePointCloud2Vector(pc_msgs, target_frame_);
                } else {
                    for (int i=0; i<16; i++) {
                        pc_4x4_tof_left_msg_map_[i] = pc_msgs[i];
                    }
                    for (int i=0; i<16; i++) {
                        pc_4x4_tof_right_msg_map_[i] = pc_msgs[16+i];
                    }
                    pc_tof_multi_msg = pointcloud_generator_.mergePointCloud2Vector(pc_msgs, target_frame_);
                }
            }
        }
    }

    isTofUpdating = true;
}

void SensorToPointcloud::cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg)
{
    if (target_frame_ == "map" || use_camera_object_logger_) {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        bounding_box_generator_.updateRobotPose(pose);
    }

    if (use_camera_) {
        if (use_camera_object_logger_) {
            camera_object_logger_.log(bounding_box_generator_.getObjectBoundingBoxInfo(msg, camera_class_id_confidence_th_, camera_object_direction_, object_max_distance_));
        }
        bbox_msg = bounding_box_generator_.generateBoundingBoxMessage(msg, camera_class_id_confidence_th_, camera_object_direction_, object_max_distance_);
        pc_camera_msg = point_cloud_camera_.updateCameraPointCloudMsg(bbox_msg, camera_pointcloud_resolution_);
    }

    isCameraUpdating = true;
}

void SensorToPointcloud::cliffMsgUpdate(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    if (target_frame_ == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_cliff_.updateRobotPose(pose);
    }

    if (use_cliff_) {
        pc_cliff_msg = point_cloud_cliff_.updateCliffPointCloudMsg(msg);
    }

    isCliffUpdating = true;
}

void SensorToPointcloud::collisionMsgUpdate(const robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg)
{
    if (target_frame_ == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_collosion_.updateRobotPose(pose);
    }

    if (use_collision_ && msg->event_trigger) {
        pc_collision_msg = point_cloud_collosion_.updateCollisionPointCloudMsg(msg);
    }

    isCollisionUpdating = true;
}
