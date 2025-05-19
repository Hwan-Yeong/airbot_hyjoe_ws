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
                point_cloud_tof_.updateTargetFrame(target_frame_);
                bounding_box_generator_.updateTargetFrame(target_frame_);
                point_cloud_cliff_.updateTargetFrame(target_frame_);
                point_cloud_collosion_.updateTargetFrame(target_frame_);
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
    tof_lpf.updateAlpha(mtof_lpf_alpha_);
    point_cloud_tof_.updateTofMode(use_tof_8x8_);
    point_cloud_tof_.updateTargetFrame(target_frame_);
    point_cloud_tof_.updateLeftSubCellIndexArray(mtof_left_sub_cell_idx_array_);
    point_cloud_tof_.updateRightSubCellIndexArray(mtof_right_sub_cell_idx_array_);
    bounding_box_generator_.updateTargetFrame(target_frame_);
    point_cloud_cliff_.updateTargetFrame(target_frame_);
    point_cloud_collosion_.updateTargetFrame(target_frame_);
    camera_object_logger_.updateParams(camera_logger_distance_margin_,camera_logger_width_margin_,camera_logger_height_margin_);
    for (const auto& item : camera_param_raw_vector_) {
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
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
    if (use_tof_) {
        pc_tof_1d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/tof/mono", 10);
        pc_tof_multi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/tof/multi", 10);
        RCLCPP_INFO(this->get_logger(), "1D/Multi TOF init finished!");
        if (use_tof_row_) {
            int tof_cnt = 0;
            if (use_tof_8x8_) {
                for (auto index : mtof_left_sub_cell_idx_array_) {
                    std::string topic_name = "sensor_to_pointcloud/tof/multi/left/idx_" + std::to_string(index);
                    pc_8x8_tof_left_pub_map_[tof_cnt++] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_name, 10);
                }
                tof_cnt = 0;
                for (auto index : mtof_right_sub_cell_idx_array_) {
                    std::string topic_name = "sensor_to_pointcloud/tof/multi/right/idx_" + std::to_string(index);
                    pc_8x8_tof_right_pub_map_[tof_cnt++] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        topic_name, 10);
                }
            } else {
                pc_tof_left_row1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/left/row_1", 10);
                pc_tof_left_row2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/left/row_2", 10);
                pc_tof_left_row3_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/left/row_3", 10);
                pc_tof_left_row4_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/left/row_4", 10);
                pc_tof_right_row1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/right/row_1", 10);
                pc_tof_right_row2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/right/row_2", 10);
                pc_tof_right_row3_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/right/row_3", 10);
                pc_tof_right_row4_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "sensor_to_pointcloud/tof/multi/right/row_4", 10);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Multi TOF Row init finished!");
    }
    if (use_camera_) {
        pc_camera_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/camera_object", 10);
        bbox_array_camera_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>(
            "sensor_to_pointcloud/camera/bbox", 10);
        RCLCPP_INFO(this->get_logger(), "Camera init finished!");
    }
    if (use_cliff_) {
        pc_cliff_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/cliff", 10);
        RCLCPP_INFO(this->get_logger(), "Cliff init finished!");
    }
    if (use_collision_) {
        pc_collision_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/collision", 10);
        RCLCPP_INFO(this->get_logger(), "Collision init finished!");
    }

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

void SensorToPointcloud::declareParams()
{
    this->declare_parameter("target_frame","base_link");

    this->declare_parameter("tof.all.use",false);
    this->declare_parameter("tof.1D.use",false);
    this->declare_parameter("tof.1D.publish_rate_ms",100);
    this->declare_parameter("tof.1D.tilting_angle_deg",0.0);
    this->declare_parameter("tof.multi.publish_rate_ms",100);
    this->declare_parameter("tof.multi.enable_8x8", false);
    this->declare_parameter("tof.multi.lpf_alpha", 0.0);
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
    this->get_parameter("tof.multi.lpf_alpha", mtof_lpf_alpha_);
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
    RCLCPP_INFO(this->get_logger(), "  TOF Multi LPf Alpha: %.2f", mtof_lpf_alpha_);
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

void SensorToPointcloud::publisherMonitor()
{
    // static int inactive_cnt = 0;
    if (!isActiveSensorToPointcloud) {
        // inactive_cnt++;
        // if (inactive_cnt >= 1000) {
        //     RCLCPP_INFO(this->get_logger(), "Sensor to Pointcloud is not active yet.");
        //     inactive_cnt = 0;
        // }
        initVariables();
        return;
    }
    // inactive_cnt = 0;

    publish_cnt_1d_tof_ += 10;
    publish_cnt_multi_tof_ += 10;
    publish_cnt_row_tof_ += 10;
    publish_cnt_camera_ += 10;
    publish_cnt_cliff_ += 10;
    publish_cnt_collision_ += 10;

    // msg Reset
    if (!isTofUpdating) {
        if (use_tof_1D_) {
            pc_tof_1d_msg = sensor_msgs::msg::PointCloud2(); //clear
        }
        if (use_tof_left_ || use_tof_right_) {
            pc_tof_multi_msg = sensor_msgs::msg::PointCloud2(); //clear
        }
        if (use_tof_row_) {
            if (use_tof_left_) {
                if (use_tof_8x8_) {
                    for (auto& [key, msg] : pc_8x8_tof_left_msg_map_) {
                        msg = sensor_msgs::msg::PointCloud2(); //clear
                    }
                } else {
                    pc_tof_left_row1_msg = sensor_msgs::msg::PointCloud2(); //clear
                    pc_tof_left_row2_msg = sensor_msgs::msg::PointCloud2(); //clear
                    pc_tof_left_row3_msg = sensor_msgs::msg::PointCloud2(); //clear
                    pc_tof_left_row4_msg = sensor_msgs::msg::PointCloud2(); //clear
                }
            }
            if (use_tof_right_) {
                if (use_tof_8x8_) {
                    for (auto& [key, msg] : pc_8x8_tof_right_msg_map_) {
                        msg = sensor_msgs::msg::PointCloud2(); //clear
                    }
                } else {
                    pc_tof_right_row1_msg = sensor_msgs::msg::PointCloud2(); //clear
                    pc_tof_right_row2_msg = sensor_msgs::msg::PointCloud2(); //clear
                    pc_tof_right_row3_msg = sensor_msgs::msg::PointCloud2(); //clear
                    pc_tof_right_row4_msg = sensor_msgs::msg::PointCloud2(); //clear
                }
            }
        }
    }
    if (!isCameraUpdating) {
        if (use_camera_) {
            pc_camera_msg = sensor_msgs::msg::PointCloud2(); //clear
            bbox_msg = vision_msgs::msg::BoundingBox2DArray(); //clear
            marker_msg = visualization_msgs::msg::MarkerArray(); //clear
        }
    }
    if (!isCliffUpdating) {
        if (use_cliff_) {
            pc_cliff_msg = sensor_msgs::msg::PointCloud2();
        }
    }
    if (!isCollisionUpdating) {
        if (use_collision_) {
            pc_collision_msg = sensor_msgs::msg::PointCloud2();
        }
    }

    // publish pointCloud Data
    if (use_tof_ && isTofUpdating) { // ToF
        if (use_tof_1D_) {
            if (publish_cnt_1d_tof_ >= publish_rate_1d_tof_) {
                pc_tof_1d_pub_->publish(pc_tof_1d_msg);
                publish_cnt_1d_tof_ = 0;
            }
        }
        if (use_tof_left_ || use_tof_right_) {
            if (publish_cnt_multi_tof_ >= publish_rate_multi_tof_) {
                pc_tof_multi_pub_->publish(pc_tof_multi_msg);
                publish_cnt_multi_tof_ = 0;
            }
        }
        if (use_tof_row_) {
            if (publish_cnt_row_tof_ >= publish_rate_row_tof_) {
                if (use_tof_left_) {
                    if (use_tof_8x8_) {
                        for (size_t i=0; i<mtof_left_sub_cell_idx_array_.size(); i++) {
                            int index = i;
                            pc_8x8_tof_left_pub_map_[index]->publish(pc_8x8_tof_left_msg_map_[index]);
                        }
                    } else {
                        pc_tof_left_row1_pub_->publish(pc_tof_left_row1_msg);
                        pc_tof_left_row2_pub_->publish(pc_tof_left_row2_msg);
                        pc_tof_left_row3_pub_->publish(pc_tof_left_row3_msg);
                        pc_tof_left_row4_pub_->publish(pc_tof_left_row4_msg);
                    }
                }
                if (use_tof_right_) {
                    if (use_tof_8x8_) {
                        for (size_t i=0; i<mtof_right_sub_cell_idx_array_.size(); i++) {
                            int index = i;
                            pc_8x8_tof_right_pub_map_[index]->publish(pc_8x8_tof_right_msg_map_[index]);
                        }
                    } else {
                        pc_tof_right_row1_pub_->publish(pc_tof_right_row1_msg);
                        pc_tof_right_row2_pub_->publish(pc_tof_right_row2_msg);
                        pc_tof_right_row3_pub_->publish(pc_tof_right_row3_msg);
                        pc_tof_right_row4_pub_->publish(pc_tof_right_row4_msg);
                    }
                }
                publish_cnt_row_tof_ = 0;
            }
        }
        isTofUpdating = false;
    }
    if (use_camera_ && isCameraUpdating) { // Camera
        if (publish_cnt_camera_ >= publish_rate_camera_) {
            pc_camera_pub_->publish(pc_camera_msg);
            bbox_array_camera_pub_->publish(bbox_msg);
            isCameraUpdating = false;
            publish_cnt_camera_ = 0;
        }
    }
    if (use_cliff_ && isCliffUpdating) {
        if (publish_cnt_cliff_ >= publish_rate_cliff_) {
            pc_cliff_pub_->publish(pc_cliff_msg);
            isCliffUpdating = false;
            publish_cnt_cliff_ = 0;
        }
    }
    if (use_collision_ && isCollisionUpdating) {
        if (publish_cnt_collision_ >= publish_rate_collision_) {
            pc_collision_pub_->publish(pc_collision_msg);
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

void SensorToPointcloud::activeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg) {
        isActiveSensorToPointcloud = msg->data;
        if(isActiveSensorToPointcloud){
            RCLCPP_INFO(this->get_logger(), "[sensor to pointcloud] activeCmdCallback : Active");
        }else{
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

    robot_custom_msgs::msg::TofData::SharedPtr filtered_msg = tof_lpf.update(msg);

    if (use_tof_) {
        if (use_tof_1D_) {
            pc_tof_1d_msg = point_cloud_tof_.updateTopTofPointCloudMsg(msg, tilting_ang_1d_tof_);
        }
        if (use_tof_8x8_) {
            if (use_tof_left_ || use_tof_right_) {
                TOF_SIDE side = (use_tof_left_ && use_tof_right_)
                                ? TOF_SIDE::BOTH : (use_tof_left_ ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT);
                auto pc_msgs = point_cloud_tof_.generateAllBotTofPointCloudMsgs(filtered_msg, side, botTofPitchAngle);
                if (side == TOF_SIDE::LEFT) {
                    for (size_t i=0; i<mtof_left_sub_cell_idx_array_.size(); i++) {
                        int index = i;
                        pc_8x8_tof_left_msg_map_[index] = pc_msgs[index];
                    }
                } else if (side == TOF_SIDE::RIGHT) {
                    for (size_t i=0; i<mtof_right_sub_cell_idx_array_.size(); i++) {
                        int index = i;
                        pc_8x8_tof_right_msg_map_[index] = pc_msgs[index];
                    }
                } else { // TOF_SIDE::BOTH
                    size_t left_size = mtof_left_sub_cell_idx_array_.size();
                    size_t right_size = mtof_right_sub_cell_idx_array_.size();

                    for (size_t i = 0; i < left_size; i++) {
                        pc_8x8_tof_left_msg_map_[i] = pc_msgs[i]; // 앞쪽 0~15
                    }

                    for (size_t i = 0; i < right_size; i++) {
                        pc_8x8_tof_right_msg_map_[i] = pc_msgs[left_size + i]; // 뒤쪽 16~31
                    }
                    pc_tof_multi_msg = pointcloud_generator_.mergePointCloud2Vector(pc_msgs, target_frame_);
                }
            }
        } else {
            if (use_tof_left_ && use_tof_right_) {
                pc_tof_multi_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::BOTH, botTofPitchAngle, false);
            }
            if (use_tof_left_) {
                pc_tof_left_row1_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::LEFT, botTofPitchAngle, true, ROW_NUMBER::FIRST);
                pc_tof_left_row2_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::LEFT, botTofPitchAngle, true, ROW_NUMBER::SECOND);
                pc_tof_left_row3_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::LEFT, botTofPitchAngle, true, ROW_NUMBER::THIRD);
                pc_tof_left_row4_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::LEFT, botTofPitchAngle, true, ROW_NUMBER::FOURTH);
            }
            if (use_tof_right_) {
                pc_tof_right_row1_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::RIGHT, botTofPitchAngle, true, ROW_NUMBER::FIRST);
                pc_tof_right_row2_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::RIGHT, botTofPitchAngle, true, ROW_NUMBER::SECOND);
                pc_tof_right_row3_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::RIGHT, botTofPitchAngle, true, ROW_NUMBER::THIRD);
                pc_tof_right_row4_msg = point_cloud_tof_.updateBotTofPointCloudMsg(filtered_msg, TOF_SIDE::RIGHT, botTofPitchAngle, true, ROW_NUMBER::FOURTH);
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

    if (use_camera_object_logger_) {
        camera_object_logger_.log(bounding_box_generator_.getObjectBoundingBoxInfo(msg, camera_class_id_confidence_th_, camera_object_direction_, object_max_distance_));
    }
    if (use_camera_) {
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
