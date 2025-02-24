#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud/sensor_to_pointcloud.hpp"

using namespace std::chrono_literals;

// Robot, Sensor Geometric Specification
double tof_top_sensor_frame_x_translate = 0.0942;       //[meter]
double tof_top_sensor_frame_y_translate = 0.0;          //[meter]
double tof_top_sensor_frame_z_translate = 0.56513;      //[meter]
double tof_bot_sensor_frame_x_translate = 0.14316;      //[meter]
double tof_bot_sensor_frame_y_translate = 0.075446;     //[meter]
double tof_bot_sensor_frame_z_translate = 0.03;         //[meter]
double tof_bot_left_sensor_frame_pitch_ang = -2.0;      //[deg]
double tof_bot_right_sensor_frame_pitch_ang = -2.0;     //[deg]
double tof_bot_left_sensor_frame_yaw_ang = 13.0;        //[deg]
double tof_bot_rihgt_sensor_frame_yaw_ang = -15.0;      //[deg]
double tof_bot_fov_ang = 45;                            //[deg]
double camera_sensor_frame_x_translate = 0.15473;       //[meter]
double camera_sensor_frame_y_translate = 0.0;           //[meter]
double camera_sensor_frame_z_translate = 0.5331;        //[meter]
double cliff_sensor_distance_center_to_front_ir = 0.15; //[meter]
double cliff_sensor_angle_to_next_ir_sensor = 50;       //[deg]

SensorToPointcloud::SensorToPointcloud()
    : rclcpp::Node("airbot_sensor_to_pointcloud"),
    point_cloud_tof_(tof_top_sensor_frame_x_translate,
                     tof_top_sensor_frame_y_translate,
                     tof_top_sensor_frame_z_translate,
                     tof_bot_sensor_frame_x_translate,
                     tof_bot_sensor_frame_y_translate,
                     tof_bot_sensor_frame_z_translate,
                     tof_bot_left_sensor_frame_pitch_ang,
                     tof_bot_right_sensor_frame_pitch_ang,
                     tof_bot_left_sensor_frame_yaw_ang,
                     tof_bot_rihgt_sensor_frame_yaw_ang,
                     tof_bot_fov_ang),
    point_cloud_cliff_(cliff_sensor_distance_center_to_front_ir,
                       cliff_sensor_angle_to_next_ir_sensor),
    bounding_box_generator_(camera_sensor_frame_x_translate,
                            camera_sensor_frame_y_translate,
                            camera_sensor_frame_z_translate)
{
    declareParams();
    setParams();
    initVariables();

    // Dynamic Parameter Handler
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_callback_handle_ = param_handler_->add_parameter_callback(
        "target_frame",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                std::string before = target_frame_;
                target_frame_ = param.as_string();
                point_cloud_tof_.updateTargetFrame(target_frame_);
                bounding_box_generator_.updateTargetFrame(target_frame_);
                point_cloud_cliff_.updateTargetFrame(target_frame_);
                std::string after;
                if (this->get_parameter("target_frame", after)) {
                    RCLCPP_INFO(this->get_logger(), "[=== Updating target_frame: %s -> %s ===]", before.c_str(), after.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "target_frame parameter not found!");
                }
            }
        }
    );

    // Update Parameters
    point_cloud_tof_.updateTargetFrame(target_frame_);
    bounding_box_generator_.updateTargetFrame(target_frame_);
    point_cloud_cliff_.updateTargetFrame(target_frame_);
    camera_object_logger_.updateParams(camera_logger_distance_margin_,camera_logger_width_margin_,camera_logger_height_margin_);
    point_cloud_lidar_.updateParams(front_lidar_params_, back_lidar_params_);
    for (const auto& item : camera_param_raw_vector_) {
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
    printParams();
    RCLCPP_INFO(this->get_logger(), "All Parameters init finished!");

    // Msg Subscribers
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensorToPointcloud::tofMsgUpdate, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "camera_data", 10, std::bind(&SensorToPointcloud::cameraMsgUpdate, this, std::placeholders::_1));
    cliff_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "bottom_status", 10, std::bind(&SensorToPointcloud::cliffMsgUpdate, this, std::placeholders::_1));
    lidar_front_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        this, "scan_front");
    lidar_back_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        this, "scan_back");
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *lidar_front_sub_, *lidar_back_sub_);
    sync_->registerCallback(
        std::bind(&SensorToPointcloud::lidarMsgUpdate, this, std::placeholders::_1, std::placeholders::_2));

    // Msg Publishers
    if (use_tof_) {
        pc_tof_1d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/tof/mono", 10);
        pc_tof_multi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/tof/multi", 10);
        RCLCPP_INFO(this->get_logger(), "1D/Multi TOF init finished!");
        if (use_tof_row_) {
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
    if (use_lidar_) {
        pc_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/lidar", rclcpp::SensorDataQoS().best_effort());
        RCLCPP_INFO(this->get_logger(), "Lidar init finished!");
    }

    // Monitor Timer
    poincloud_publish_timer_ = this->create_wall_timer(
        10ms, std::bind(&SensorToPointcloud::publisherMonitor, this));
}

SensorToPointcloud::~SensorToPointcloud()
{
    param_handler_.reset();
}

void SensorToPointcloud::declareParams()
{
    this->declare_parameter("target_frame","base_link");

    this->declare_parameter("tof.all.use",false);
    this->declare_parameter("tof.1D.use",false);
    this->declare_parameter("tof.1D.publish_rate_ms",100);
    this->declare_parameter("tof.1D.tilting_angle_deg",0.0);
    this->declare_parameter("tof.multi.publish_rate_ms",100);
    this->declare_parameter("tof.multi.left.use",false);
    this->declare_parameter("tof.multi.right.use",false);
    this->declare_parameter("tof.multi.row.use",false);
    this->declare_parameter("tof.multi.row.publish_rate_ms",100);

    this->declare_parameter("camera.use",false);
    this->declare_parameter("camera.publish_rate_ms",100);
    this->declare_parameter("camera.pointcloud_resolution",0.05);
    this->declare_parameter("camera.class_id_confidence_th",std::vector<std::string>());
    this->declare_parameter("camera.object_direction",false);
    this->declare_parameter("camera.logger.use",false);
    this->declare_parameter("camera.logger.margin.distance_diff",1.0);
    this->declare_parameter("camera.logger.margin.width_diff",1.0);
    this->declare_parameter("camera.logger.margin.height_diff",1.0);

    this->declare_parameter("cliff.use",false);
    this->declare_parameter("cliff.publish_rate_ms",100);

    this->declare_parameter("lidar.use", false);
    this->declare_parameter("lidar.publish_rate_ms", 100);
    this->declare_parameter("lidar.front.range.angle_max", 0.0);
    this->declare_parameter("lidar.front.range.angle_min", 0.0);
    this->declare_parameter("lidar.front.geometry.alpha", 0.0);
    this->declare_parameter("lidar.front.geometry.offset.x", 0.0);
    this->declare_parameter("lidar.front.geometry.offset.y", 0.0);
    this->declare_parameter("lidar.front.geometry.offset.z", 0.0);
    this->declare_parameter("lidar.back.range.angle_max", 0.0);
    this->declare_parameter("lidar.back.range.angle_min", 0.0);
    this->declare_parameter("lidar.back.geometry.alpha", 0.0);
    this->declare_parameter("lidar.back.geometry.offset.x", 0.0);
    this->declare_parameter("lidar.back.geometry.offset.y", 0.0);
    this->declare_parameter("lidar.back.geometry.offset.z", 0.0);
}

void SensorToPointcloud::setParams()
{
    this->get_parameter("target_frame", target_frame_);

    this->get_parameter("tof.all.use", use_tof_);
    this->get_parameter("tof.1D.use", use_tof_1D_);
    this->get_parameter("tof.1D.publish_rate_ms", publish_rate_1d_tof_);
    this->get_parameter("tof.1D.tilting_angle_deg", tilting_ang_1d_tof_);
    this->get_parameter("tof.multi.publish_rate_ms", publish_rate_multi_tof_);
    this->get_parameter("tof.multi.left.use", use_tof_left_);
    this->get_parameter("tof.multi.right.use", use_tof_right_);
    this->get_parameter("tof.multi.row.use", use_tof_row_);
    this->get_parameter("tof.multi.row.publish_rate_ms", publish_rate_row_tof_);

    this->get_parameter("camera.use", use_camera_);
    this->get_parameter("camera.publish_rate_ms", publish_rate_camera_);
    this->get_parameter("camera.pointcloud_resolution", camera_pointcloud_resolution_);
    this->get_parameter("camera.class_id_confidence_th", camera_param_raw_vector_);
    this->get_parameter("camera.object_direction", camera_object_direction_);
    this->get_parameter("camera.logger.use", use_camera_object_logger_);
    this->get_parameter("camera.logger.margin.distance_diff", camera_logger_distance_margin_);
    this->get_parameter("camera.logger.margin.width_diff", camera_logger_width_margin_);
    this->get_parameter("camera.logger.margin.height_diff", camera_logger_height_margin_);

    this->get_parameter("cliff.use", use_cliff_);
    this->get_parameter("cliff.publish_rate_ms", publish_rate_cliff_);

    this->get_parameter("lidar.use", use_lidar_);
    this->get_parameter("lidar.publish_rate_ms", publish_rate_lidar_);
    this->get_parameter("lidar.front.range.angle_max", front_lidar_params_.angle_max);
    this->get_parameter("lidar.front.range.angle_min", front_lidar_params_.angle_min);
    this->get_parameter("lidar.front.geometry.alpha", front_lidar_params_.alpha);
    this->get_parameter("lidar.front.geometry.offset.x", front_lidar_params_.offset.x);
    this->get_parameter("lidar.front.geometry.offset.y", front_lidar_params_.offset.y);
    this->get_parameter("lidar.front.geometry.offset.z", front_lidar_params_.offset.z);
    this->get_parameter("lidar.back.range.angle_max", back_lidar_params_.angle_max);
    this->get_parameter("lidar.back.range.angle_min", back_lidar_params_.angle_min);
    this->get_parameter("lidar.back.geometry.alpha", back_lidar_params_.alpha);
    this->get_parameter("lidar.back.geometry.offset.x", back_lidar_params_.offset.x);
    this->get_parameter("lidar.back.geometry.offset.y", back_lidar_params_.offset.y);
    this->get_parameter("lidar.back.geometry.offset.z", back_lidar_params_.offset.z);
}

void SensorToPointcloud::printParams()
{
    RCLCPP_INFO(this->get_logger(), "================== SENSOR MANAGER PARAMETERS ==================");

    // General Settings
    RCLCPP_INFO(this->get_logger(), "[General]");
    RCLCPP_INFO(this->get_logger(), "  Target Frame: '%s'", target_frame_.c_str());

    // TOF Settings
    RCLCPP_INFO(this->get_logger(), "[TOF Settings]");
    RCLCPP_INFO(this->get_logger(), "  TOF All Use: %d", use_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF 1D Use: %d", use_tof_1D_);
    RCLCPP_INFO(this->get_logger(), "  TOF 1D Publish Rate: %d ms", publish_rate_1d_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF 1D Tilting Angle: %.2f deg", tilting_ang_1d_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Publish Rate: %d ms", publish_rate_multi_tof_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Left Use: %d", use_tof_left_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Right Use: %d", use_tof_right_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Row Use: %d", use_tof_row_);
    RCLCPP_INFO(this->get_logger(), "  TOF Multi Row Publish Rate: %d ms", publish_rate_row_tof_);

    // Camera Settings
    RCLCPP_INFO(this->get_logger(), "[Camera Settings]");
    RCLCPP_INFO(this->get_logger(), "  Camera Use: %d", use_camera_);
    RCLCPP_INFO(this->get_logger(), "  Camera Publish Rate: %d ms", publish_rate_camera_);
    RCLCPP_INFO(this->get_logger(), "  Camera Pointcloud Resolution: %.2f", camera_pointcloud_resolution_);
    RCLCPP_INFO(this->get_logger(), "  Camera Object Direction: %s", camera_object_direction_ ? "True" : "False");
    RCLCPP_INFO(this->get_logger(), "  Camera Class ID Confidence Threshold:");
    for (const auto& conf : camera_class_id_confidence_th_) {
        RCLCPP_INFO(this->get_logger(), "    Class ID: %d, Confidence: %d", conf.first, conf.second);
    }
    RCLCPP_INFO(this->get_logger(), "  Camera Logger Use: %d", use_camera_object_logger_);
    RCLCPP_INFO(this->get_logger(), "  Camera Logger Margins: Distance Diff: %.2f, Width Diff: %.2f, Height Diff: %.2f", 
                camera_logger_distance_margin_, 
                camera_logger_width_margin_, 
                camera_logger_height_margin_);

    // Cliff Settings
    RCLCPP_INFO(this->get_logger(), "[Cliff Settings]");
    RCLCPP_INFO(this->get_logger(), "  Cliff Use: %d", use_cliff_);
    RCLCPP_INFO(this->get_logger(), "  Cliff Publish Rate: %d ms", publish_rate_cliff_);

    // Lidar Settings
    RCLCPP_INFO(this->get_logger(), "[Lidar Settings]");
    RCLCPP_INFO(this->get_logger(), "  Lidar Use: %d", use_lidar_);
    RCLCPP_INFO(this->get_logger(), "  Lidar Publish Rate: %d ms", publish_rate_lidar_);
    RCLCPP_INFO(this->get_logger(), "  Lidar Front: Angle Min: %.2f, Angle Max: %.2f, Alpha: %.2f", 
                front_lidar_params_.angle_min, 
                front_lidar_params_.angle_max, 
                front_lidar_params_.alpha);
    RCLCPP_INFO(this->get_logger(), "  Lidar Front Offset: (X: %.2f, Y: %.2f, Z: %.2f)", 
                front_lidar_params_.offset.x, 
                front_lidar_params_.offset.y, 
                front_lidar_params_.offset.z);
    RCLCPP_INFO(this->get_logger(), "  Lidar Back: Angle Min: %.2f, Angle Max: %.2f, Alpha: %.2f", 
                back_lidar_params_.angle_min, 
                back_lidar_params_.angle_max, 
                back_lidar_params_.alpha);
    RCLCPP_INFO(this->get_logger(), "  Lidar Back Offset: (X: %.2f, Y: %.2f, Z: %.2f)", 
                back_lidar_params_.offset.x, 
                back_lidar_params_.offset.y, 
                back_lidar_params_.offset.z);

    RCLCPP_INFO(this->get_logger(), "===============================================================");

}

void SensorToPointcloud::initVariables()
{
    isTofUpdating = false;
    isCameraUpdating = false;
    isCliffUpdating = false;
    isLidarUpdating = false;

    publish_cnt_1d_tof_ = 0;
    publish_cnt_multi_tof_ = 0;
    publish_cnt_row_tof_ = 0;
    publish_cnt_camera_ = 0;
    publish_cnt_cliff_ = 0;
    publish_cnt_lidar_ = 0;
}

void SensorToPointcloud::publisherMonitor()
{
    publish_cnt_1d_tof_ += 10;
    publish_cnt_multi_tof_ += 10;
    publish_cnt_row_tof_ += 10;
    publish_cnt_camera_ += 10;
    publish_cnt_cliff_ += 10;
    publish_cnt_lidar_ += 10;

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
                pc_tof_left_row1_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_left_row2_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_left_row3_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_left_row4_msg = sensor_msgs::msg::PointCloud2(); //clear
            }
            if (use_tof_right_) {
                pc_tof_right_row1_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_right_row2_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_right_row3_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_right_row4_msg = sensor_msgs::msg::PointCloud2(); //clear
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
    if (!isLidarUpdating) {
        if (use_lidar_) {
            pc_lidar_msg = sensor_msgs::msg::PointCloud2();
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
                    pc_tof_left_row1_pub_->publish(pc_tof_left_row1_msg);
                    pc_tof_left_row2_pub_->publish(pc_tof_left_row2_msg);
                    pc_tof_left_row3_pub_->publish(pc_tof_left_row3_msg);
                    pc_tof_left_row4_pub_->publish(pc_tof_left_row4_msg);
                }
                if (use_tof_right_) {
                    pc_tof_right_row1_pub_->publish(pc_tof_right_row1_msg);
                    pc_tof_right_row2_pub_->publish(pc_tof_right_row2_msg);
                    pc_tof_right_row3_pub_->publish(pc_tof_right_row3_msg);
                    pc_tof_right_row4_pub_->publish(pc_tof_right_row4_msg);
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
    if (use_lidar_ && isLidarUpdating) {
        if (publish_cnt_lidar_ >= publish_rate_lidar_) {
            pc_lidar_pub_->publish(pc_lidar_msg);
            isLidarUpdating = false;
            publish_cnt_lidar_ = 0;
        }
    }

    if (publish_cnt_1d_tof_ > 10000)     publish_cnt_1d_tof_ = 0;
    if (publish_cnt_multi_tof_ > 10000)  publish_cnt_multi_tof_ = 0;
    if (publish_cnt_row_tof_ > 10000)    publish_cnt_row_tof_ = 0;
    if (publish_cnt_camera_ > 10000)     publish_cnt_camera_ = 0;
    if (publish_cnt_cliff_ > 10000)      publish_cnt_cliff_ = 0;
    if (publish_cnt_lidar_ > 10000)      publish_cnt_lidar_ = 0;
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

    if (use_tof_) {
        if (use_tof_1D_) {
            pc_tof_1d_msg = point_cloud_tof_.updateTopTofPointCloudMsg(msg, tilting_ang_1d_tof_);
        }
        if (use_tof_left_ || use_tof_right_) {
            TOF_SIDE side = (use_tof_left_ && use_tof_right_) ? TOF_SIDE::BOTH : 
                            (use_tof_left_ ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT);
            pc_tof_multi_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, side, false);
        }
        if (use_tof_row_) {
            if (use_tof_left_) {
                pc_tof_left_row1_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::FIRST);
                pc_tof_left_row2_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::SECOND);
                pc_tof_left_row3_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::THIRD);
                pc_tof_left_row4_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::FOURTH);
            }
            if (use_tof_right_) {
                pc_tof_right_row1_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::FIRST);
                pc_tof_right_row2_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::SECOND);
                pc_tof_right_row3_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::THIRD);
                pc_tof_right_row4_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::FOURTH);
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
        camera_object_logger_.log(bounding_box_generator_.getObjectBoundingBoxInfo(msg, camera_class_id_confidence_th_, camera_object_direction_));
    }
    if (use_camera_) {
        bbox_msg = bounding_box_generator_.generateBoundingBoxMessage(msg, camera_class_id_confidence_th_, camera_object_direction_);
        pc_camera_msg = point_cloud_camera_.updateCameraPointCloudMsg(bbox_msg, camera_pointcloud_resolution_);
    }

    isCameraUpdating = true;
}

void SensorToPointcloud::cliffMsgUpdate(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (target_frame_ == "map") {
        // tPose pose;
        // pose.position.x = msg->robot_x;
        // pose.position.y = msg->robot_y;
        // pose.orientation.yaw = msg->robot_angle;
        // point_cloud_cliff_.updateRobotPose(pose);
    }

    if (use_cliff_ && target_frame_ == "base_link") {
        pc_cliff_msg = point_cloud_cliff_.updateCliffPointCloudMsg(msg);
    }

    isCliffUpdating = true;
}

void SensorToPointcloud::lidarMsgUpdate(const sensor_msgs::msg::LaserScan::ConstSharedPtr &lidar_front_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr &lidar_back_msg)
{
    // Always publish at "base_scan" frame
    sensor_msgs::msg::LaserScan front_msg = *lidar_front_msg;
    sensor_msgs::msg::LaserScan back_msg = *lidar_back_msg;
    pc_lidar_msg = point_cloud_lidar_.updateLidarPointCloudMsg(front_msg, back_msg);

    isLidarUpdating = true;
}
