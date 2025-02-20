#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud.hpp"

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
    // Dynamic Parameter Handler
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    param_callback_handle_ = param_handler_->add_parameter_callback(
        "target_frame",
        [this](const rclcpp::Parameter & param) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                target_frame_ = param.as_string();
                point_cloud_tof_.updateTargetFrame(target_frame_);
                bounding_box_generator_.updateTargetFrame(target_frame_);
                point_cloud_cliff_.updateTargetFrame(target_frame_);
                RCLCPP_INFO(this->get_logger(),
                            "Success to Change [target_frame : %s]",
                            target_frame_.c_str());
            }
        }
    );

    initialize_lidar_params();
    refresh_lidar_params();
    print_lidar_params();

    // Declare Parameters
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

    // Set Parameters
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
    this->get_parameter("cliff.use", ues_cliff_);
    this->get_parameter("cliff.publish_rate_ms", publish_rate_cliff_);

    // Update Parameters
    point_cloud_tof_.updateTargetFrame(target_frame_);
    bounding_box_generator_.updateTargetFrame(target_frame_);
    point_cloud_cliff_.updateTargetFrame(target_frame_);
    camera_object_logger_.updateParams(camera_logger_distance_margin_,camera_logger_width_margin_,camera_logger_height_margin_);
    for (const auto& item : camera_param_raw_vector_) {
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            camera_class_id_confidence_th_[std::stoi(key)] = std::stoi(value);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Parameters init finished!");

    // Msg Update Flags
    isTofUpdating = false;
    isCameraUpdating = false;
    isCliffUpdating = false;

    // Msg Subscribers
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensorToPointcloud::tofMsgUpdate, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::CameraDataArray>(
        "camera_data", 10, std::bind(&SensorToPointcloud::cameraMsgUpdate, this, std::placeholders::_1));
    cliff_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "bottom_status", 10, std::bind(&SensorToPointcloud::cliffMsgUpdate, this, std::placeholders::_1));

    laser1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic1_);
    laser2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic2_);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *laser1_sub_, *laser2_sub_);
    sync_->registerCallback(std::bind(&SensorToPointcloud::synchronizedCallback, this, std::placeholders::_1, std::placeholders::_2));

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
    if (ues_cliff_) {
        pc_cliff_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/cliff", 10);
        RCLCPP_INFO(this->get_logger(), "Cliff init finished!");
    }

    pc_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloudTopic_, rclcpp::SensorDataQoS().best_effort());

    publish_cnt_1d_tof_ = 0;
    publish_cnt_multi_tof_ = 0;
    publish_cnt_row_tof_ = 0;
    publish_cnt_camera_ = 0;
    publish_cnt_cliff_ = 0;

    // Monitor Timer
    poincloud_publish_timer_ = this->create_wall_timer(
        100ms, std::bind(&SensorToPointcloud::publisherMonitor, this));
}

SensorToPointcloud::~SensorToPointcloud()
{
}

void SensorToPointcloud::initialize_lidar_params()
{
    this->declare_parameter("pointCloudTopic", "sensor_to_pointcloud/lidar");
    this->declare_parameter("pointCloutFrameId", "base_scan");
    this->declare_parameter("scanTopic1", "/scan_front");
    this->declare_parameter("laser1XOff", 0.15);
    this->declare_parameter("laser1YOff", 0.0);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 180.0);
    this->declare_parameter("laser1AngleMin", 90.0);
    this->declare_parameter("laser1AngleMax", 270.0);
    this->declare_parameter("laser1R", 255);
    this->declare_parameter("laser1G", 0);
    this->declare_parameter("laser1B", 0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", false);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "/scan_back");
    this->declare_parameter("laser2XOff", -0.15);
    this->declare_parameter("laser2YOff", 0.0);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 0.0);
    this->declare_parameter("laser2AngleMin", 90.0);
    this->declare_parameter("laser2AngleMax", 270.0);
    this->declare_parameter("laser2R", 0);
    this->declare_parameter("laser2G", 0);
    this->declare_parameter("laser2B", 255);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", false);
    this->declare_parameter("inverse2", false);
}

void SensorToPointcloud::refresh_lidar_params()
{
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "sensor_to_pointcloud/lidar");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "base_scan");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "/scan_front");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.15);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 180.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, 90.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 270.0);
    this->get_parameter_or<uint8_t>("laser1R", laser1R_, 255);
    this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
    this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, false);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "/scan_back");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, -0.15);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, 90.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 270.0);
    this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
    this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
    this->get_parameter_or<uint8_t>("laser2B", laser2B_, 255);
    this->get_parameter_or<bool>("show2", show2_, true);
    this->get_parameter_or<bool>("flip2", flip2_, false);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);
}

void SensorToPointcloud::print_lidar_params()
{
    RCLCPP_INFO(this->get_logger(), "PointCloudTopic: %s", cloudTopic_.c_str());
    RCLCPP_INFO(this->get_logger(), "PointCloudFrameId: %s", cloudFrameId_.c_str());

    RCLCPP_INFO(this->get_logger(), "LIDAR 1:");
    RCLCPP_INFO(this->get_logger(), "  Scan Topic: %s", topic1_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Offset: X=%.2f, Y=%.2f, Z=%.2f", laser1XOff_, laser1YOff_, laser1ZOff_);
    RCLCPP_INFO(this->get_logger(), "  Alpha: %.2f", laser1Alpha_);
    RCLCPP_INFO(this->get_logger(), "  Angle Min: %.2f, Angle Max: %.2f", laser1AngleMin_, laser1AngleMax_);
    RCLCPP_INFO(this->get_logger(), "  RGB: (%d, %d, %d)", laser1R_, laser1G_, laser1B_);
    RCLCPP_INFO(this->get_logger(), "  Show: %s, Flip: %s, Inverse: %s", 
        show1_ ? "true" : "false",
        flip1_ ? "true" : "false",
        inverse1_ ? "true" : "false");

    RCLCPP_INFO(this->get_logger(), "LIDAR 2:");
    RCLCPP_INFO(this->get_logger(), "  Scan Topic: %s", topic2_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Offset: X=%.2f, Y=%.2f, Z=%.2f", laser2XOff_, laser2YOff_, laser2ZOff_);
    RCLCPP_INFO(this->get_logger(), "  Alpha: %.2f", laser2Alpha_);
    RCLCPP_INFO(this->get_logger(), "  Angle Min: %.2f, Angle Max: %.2f", laser2AngleMin_, laser2AngleMax_);
    RCLCPP_INFO(this->get_logger(), "  RGB: (%d, %d, %d)", laser2R_, laser2G_, laser2B_);
    RCLCPP_INFO(this->get_logger(), "  Show: %s, Flip: %s, Inverse: %s", 
        show2_ ? "true" : "false",
        flip2_ ? "true" : "false",
        inverse2_ ? "true" : "false");
}

void SensorToPointcloud::publisherMonitor()
{
    publish_cnt_1d_tof_ += 10;
    publish_cnt_multi_tof_ += 10;
    publish_cnt_row_tof_ += 10;
    publish_cnt_camera_ += 10;
    publish_cnt_cliff_ += 10;

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
        if (ues_cliff_) {
            pc_cliff_msg = sensor_msgs::msg::PointCloud2();
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
    if (ues_cliff_ && isCliffUpdating) {
        if (publish_cnt_cliff_ >= publish_rate_cliff_) {
            pc_cliff_pub_->publish(pc_cliff_msg);
            isCliffUpdating = false;
            publish_cnt_cliff_ = 0;
        }
    }

    pc_lidar_pub_->publish(pc_lidar_msg);

    if (publish_cnt_1d_tof_ > 10000)     publish_cnt_1d_tof_ = 0;
    if (publish_cnt_multi_tof_ > 10000)  publish_cnt_multi_tof_ = 0;
    if (publish_cnt_row_tof_ > 10000)    publish_cnt_row_tof_ = 0;
    if (publish_cnt_camera_ > 10000)     publish_cnt_camera_ = 0;
    if (publish_cnt_cliff_ > 10000)      publish_cnt_cliff_ = 0;
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

    if (ues_cliff_ && target_frame_ == "base_link") {
        pc_cliff_msg = point_cloud_cliff_.updateCliffPointCloudMsg(msg);
    }

    isCliffUpdating = true;
}

void SensorToPointcloud::synchronizedCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg)
{
    laser1_ = *laser1_msg;
    laser2_ = *laser2_msg;
    update_point_cloud_rgb();
}

void SensorToPointcloud::update_point_cloud_rgb()
{
    // refresh_params();
    pcl::PointCloud<pcl::PointXYZ> cloud_; 
    std::vector<std::array<float, 2>> scan_data;
    float min_theta = std::numeric_limits<float>::max();
    float max_theta = std::numeric_limits<float>::lowest();

    auto process_laser = [&](const sensor_msgs::msg::LaserScan &laser, 
                            float x_off, float y_off, float z_off, float alpha, 
                            float angle_min, float angle_max, 
                            bool flip, bool inverse, bool show) {
        if (!show || laser.ranges.empty()) return;

        float temp_min = std::min(laser.angle_min, laser.angle_max);
        float temp_max = std::max(laser.angle_min, laser.angle_max);
        float alpha_rad = alpha * M_PI / 180.0;
        float cos_alpha = std::cos(alpha_rad);
        float sin_alpha = std::sin(alpha_rad);
        float angle_min_rad = angle_min * M_PI / 180.0;
        float angle_max_rad = angle_max * M_PI / 180.0;

        for (size_t i = 0; i < laser.ranges.size(); ++i) {
            float angle = temp_min + i * laser.angle_increment;
            if (angle > temp_max) break;

            size_t idx = flip ? laser.ranges.size() - 1 - i : i;
            float range = laser.ranges[idx];

            if (std::isnan(range) || range < laser.range_min || range > laser.range_max) continue;

            bool is_in_range = (angle >= angle_min_rad && angle <= angle_max_rad);
            if (inverse == is_in_range) continue;

            pcl::PointXYZ pt; 
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            pt.x = x * cos_alpha - y * sin_alpha + x_off;
            pt.y = x * sin_alpha + y * cos_alpha + y_off;
            pt.z = z_off;

            cloud_.points.push_back(pt);

            float r_ = std::hypot(pt.x, pt.y);
            float theta_ = std::atan2(pt.y, pt.x);
            scan_data.push_back({theta_, r_});

            min_theta = std::min(min_theta, theta_);
            max_theta = std::max(max_theta, theta_);
        }
    };

    // Process both lasers
    process_laser(laser1_, laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, 
                laser1AngleMin_, laser1AngleMax_, flip1_, inverse1_, show1_);

    process_laser(laser2_, laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, 
                laser2AngleMin_, laser2AngleMax_, flip2_, inverse2_, show2_);

    // Create and publish PointCloud2 message
    removePointsWithinRadius(cloud_, 0.2); //radius 0.19
    pcl::toROSMsg(cloud_, pc_lidar_msg);
    pc_lidar_msg.header.frame_id = cloudFrameId_;

    rclcpp::Time time1(laser1_.header.stamp);
    rclcpp::Time time2(laser2_.header.stamp);
    pc_lidar_msg.header.stamp = (time1 < time2) ? laser2_.header.stamp : laser1_.header.stamp;
    pc_lidar_msg.is_dense = false;
}

void SensorToPointcloud::removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius, float center_x, float center_y)
{
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;

    for (const auto& point : cloud.points) {
        float distance = std::sqrt((point.x - center_x) * (point.x - center_x) + 
                                    (point.y - center_y) * (point.y - center_y));
        if (distance >= radius) {
            filteredCloud.points.push_back(point);
        }
    }

    cloud.points.swap(filteredCloud.points);
}
