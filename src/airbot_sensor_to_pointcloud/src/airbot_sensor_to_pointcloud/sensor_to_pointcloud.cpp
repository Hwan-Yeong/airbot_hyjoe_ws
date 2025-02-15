#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud.hpp"

using namespace std::chrono_literals;

// Robot, Sensor Geometric Specification
double tof_top_sensor_frame_x_translate = 0.0942;       //[meter]
double tof_top_sensor_frame_y_translate = 0.0;          //[meter]
double tof_top_sensor_frame_z_translate = 0.56513;      //[meter]
double tof_top_sensor_frame_pitch_ang = 45;             //[deg]
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
                     tof_top_sensor_frame_pitch_ang,
                     tof_bot_sensor_frame_x_translate,
                     tof_bot_sensor_frame_y_translate,
                     tof_bot_sensor_frame_z_translate,
                     tof_bot_left_sensor_frame_pitch_ang,
                     tof_bot_right_sensor_frame_pitch_ang,
                     tof_bot_left_sensor_frame_yaw_ang,
                     tof_bot_rihgt_sensor_frame_yaw_ang,
                     tof_bot_fov_ang),
    point_cloud_camera_(camera_sensor_frame_x_translate,
                        camera_sensor_frame_y_translate,
                        camera_sensor_frame_z_translate),
    point_cloud_cliff_(cliff_sensor_distance_center_to_front_ir,
                       cliff_sensor_angle_to_next_ir_sensor)
{
    // Declare Parameters
    this->declare_parameter("target_frame","base_link");
    this->declare_parameter("use.tof",false);
    this->declare_parameter("use.tof.1D",false);
    this->declare_parameter("use.tof.left",false);
    this->declare_parameter("use.tof.right",false);
    this->declare_parameter("use.tof.row",false);
    this->declare_parameter("use.camera",false);
    this->declare_parameter("use.cliff",false);
    this->declare_parameter("camera.pointcloud_resolution",0.05);
    this->declare_parameter("camera.class_id_confidence_th",std::vector<std::string>());
    this->declare_parameter("camera.object_direction",false);
    this->declare_parameter("publish.rate_ms.tof_1d",100);
    this->declare_parameter("publish.rate_ms.tof_multi",100);
    this->declare_parameter("publish.rate_ms.tof_row",100);
    this->declare_parameter("publish.rate_ms.camera",100);
    this->declare_parameter("publish.rate_ms.cliff",100);

    // Set Parameters
    this->get_parameter("target_frame", target_frame);
    this->get_parameter("use.tof", use_tof_pointcloud);
    this->get_parameter("use.tof.1D", use_tof_1D);
    this->get_parameter("use.tof.left", use_tof_left);
    this->get_parameter("use.tof.right", use_tof_right);
    this->get_parameter("use.tof.row", tof_debug_mode);
    this->get_parameter("use.camera", use_camera_pointcloud);
    this->get_parameter("use.cliff", use_cliff_pointcloud);
    this->get_parameter("camera.pointcloud_resolution", camera_pointcloud_resolution_m);
    this->get_parameter("camera.class_id_confidence_th", camera_param_raw_vector);
    this->get_parameter("camera.object_direction", camera_object_direction);
    this->get_parameter("publish.rate_ms.tof_1d", publish_rate_1d_tof);
    this->get_parameter("publish.rate_ms.tof_multi", publish_rate_multi_tof);
    this->get_parameter("publish.rate_ms.tof_row", publish_rate_row_tof);
    this->get_parameter("publish.rate_ms.camera", publish_rate_camera);
    this->get_parameter("publish.rate_ms.cliff", publish_rate_cliff);

    // Update Parameters
    point_cloud_tof_.updateTargetFrame(target_frame);
    point_cloud_camera_.updateTargetFrame(target_frame);
    point_cloud_cliff_.updateTargetFrame(target_frame);
    for (const auto& item : camera_param_raw_vector) {
        std::istringstream ss(item);
        std::string key, value;
        if (std::getline(ss, key, ':') && std::getline(ss, value)) {
            camera_class_id_confidence_th[std::stoi(key)] = std::stoi(value);
        }
    }

    // Msg Update Flags
    isTofUpdating = false;
    isCameraUpdating = false;
    isCliffUpdating = false;

    // Msg Subscribers
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensorToPointcloud::tofMsgUpdate, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::AIDataArray>(
        "camera_data", 10, std::bind(&SensorToPointcloud::cameraMsgUpdate, this, std::placeholders::_1));
    cliff_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "bottom_status", 10, std::bind(&SensorToPointcloud::cliffMsgUpdate, this, std::placeholders::_1));
    
    // Msg Publishers
    if (use_tof_pointcloud) {
        pc_tof_1d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/tof/mono", 10);
        pc_tof_multi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/tof/multi", 10);
        if (tof_debug_mode) {
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
    if (use_camera_pointcloud) {
        pc_camera_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/camera_object", 10);
        bbox_array_camera_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>(
            "sensor_to_pointcloud/camera/bbox", 10);
    }
    if (use_cliff_pointcloud) {
        pc_cliff_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sensor_to_pointcloud/cliff", 10);
    }

    publish_cnt_1d_tof = 0;
    publish_cnt_multi_tof = 0;
    publish_cnt_row_tof = 0;
    publish_cnt_camera = 0;
    publish_cnt_cliff = 0;

    // Monitor Timer
    poincloud_publish_timer_ = this->create_wall_timer(
        10ms, std::bind(&SensorToPointcloud::publisherMonitor, this));
}

SensorToPointcloud::~SensorToPointcloud()
{
}

void SensorToPointcloud::publisherMonitor()
{
    publish_cnt_1d_tof += 10;
    publish_cnt_multi_tof += 10;
    publish_cnt_row_tof += 10;
    publish_cnt_camera += 10;
    publish_cnt_cliff += 10;

    // msg Reset
    if (!isTofUpdating) {
        if (use_tof_1D) {
            pc_tof_1d_msg = sensor_msgs::msg::PointCloud2(); //clear
        }
        if (use_tof_left || use_tof_right) {
            pc_tof_multi_msg = sensor_msgs::msg::PointCloud2(); //clear
        }
        if (tof_debug_mode) {
            if (use_tof_left) {
                pc_tof_left_row1_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_left_row2_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_left_row3_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_left_row4_msg = sensor_msgs::msg::PointCloud2(); //clear
            }
            if (use_tof_right) {
                pc_tof_right_row1_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_right_row2_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_right_row3_msg = sensor_msgs::msg::PointCloud2(); //clear
                pc_tof_right_row4_msg = sensor_msgs::msg::PointCloud2(); //clear
            }
        }
    }
    if (!isCameraUpdating) {
        if (use_camera_pointcloud) {
            pc_camera_msg = sensor_msgs::msg::PointCloud2(); //clear
            bbox_msg = vision_msgs::msg::BoundingBox2DArray(); //clear
            marker_msg = visualization_msgs::msg::MarkerArray(); //clear
        }
    }
    if (!isCliffUpdating) {
        if (use_cliff_pointcloud) {
            pc_cliff_msg = sensor_msgs::msg::PointCloud2();
        }
    }

    // publish pointCloud Data
    if (use_tof_pointcloud && isTofUpdating) { // ToF
        if (use_tof_1D) {
            if (publish_cnt_1d_tof >= publish_rate_1d_tof) {
                pc_tof_1d_pub_->publish(pc_tof_1d_msg);
                publish_cnt_1d_tof = 0;
            }
        }
        if (use_tof_left || use_tof_right) {
            if (publish_cnt_multi_tof >= publish_rate_multi_tof) {
                pc_tof_multi_pub_->publish(pc_tof_multi_msg);
                publish_cnt_multi_tof = 0;
            }
        }
        if (tof_debug_mode) {
            if (publish_cnt_row_tof >= publish_rate_row_tof) {
                if (use_tof_left) {
                    pc_tof_left_row1_pub_->publish(pc_tof_left_row1_msg);
                    pc_tof_left_row2_pub_->publish(pc_tof_left_row2_msg);
                    pc_tof_left_row3_pub_->publish(pc_tof_left_row3_msg);
                    pc_tof_left_row4_pub_->publish(pc_tof_left_row4_msg);
                }
                if (use_tof_right) {
                    pc_tof_right_row1_pub_->publish(pc_tof_right_row1_msg);
                    pc_tof_right_row2_pub_->publish(pc_tof_right_row2_msg);
                    pc_tof_right_row3_pub_->publish(pc_tof_right_row3_msg);
                    pc_tof_right_row4_pub_->publish(pc_tof_right_row4_msg);
                }
                publish_cnt_row_tof = 0;
            }
        }
        isTofUpdating = false;
    }
    if (use_camera_pointcloud && isCameraUpdating) { // Camera
        if (publish_cnt_camera >= publish_rate_camera) {
            pc_camera_pub_->publish(pc_camera_msg);
            bbox_array_camera_pub_->publish(bbox_msg);
            isCameraUpdating = false;
            publish_cnt_camera = 0;
        }
    }
    if (use_cliff_pointcloud && isCliffUpdating) {
        if (publish_cnt_cliff >= publish_rate_cliff) {
            pc_cliff_pub_->publish(pc_cliff_msg);
            isCliffUpdating = false;
            publish_cnt_cliff = 0;
        }
    }

    if (publish_cnt_1d_tof > 10000)     publish_cnt_1d_tof = 0;
    if (publish_cnt_multi_tof > 10000)  publish_cnt_multi_tof = 0;
    if (publish_cnt_row_tof > 10000)    publish_cnt_row_tof = 0;
    if (publish_cnt_camera > 10000)     publish_cnt_camera = 0;
    if (publish_cnt_cliff > 10000)      publish_cnt_cliff = 0;
}

void SensorToPointcloud::tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if (target_frame == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_tof_.updateRobotPose(pose);
    }

    if (use_tof_pointcloud) {
        if (use_tof_1D) {
            pc_tof_1d_msg = point_cloud_tof_.updateTopTofPointCloudMsg(msg);
        }
        if (use_tof_left || use_tof_right) {
            TOF_SIDE side = (use_tof_left && use_tof_right) ? TOF_SIDE::BOTH : 
                            (use_tof_left ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT);
            pc_tof_multi_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, side, false);
        }
        if (tof_debug_mode) {
            if (use_tof_left) {
                pc_tof_left_row1_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::FIRST);
                pc_tof_left_row2_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::SECOND);
                pc_tof_left_row3_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::THIRD);
                pc_tof_left_row4_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::FOURTH);
            }
            if (use_tof_right) {
                pc_tof_right_row1_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::FIRST);
                pc_tof_right_row2_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::SECOND);
                pc_tof_right_row3_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::THIRD);
                pc_tof_right_row4_msg = point_cloud_tof_.updateBotTofPointCloudMsg(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::FOURTH);
            }
        }
    }

    isTofUpdating = true;
}

void SensorToPointcloud::cameraMsgUpdate(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg)
{
    if (target_frame == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_camera_.updateRobotPose(pose);
    }

    if (use_camera_pointcloud) {
        bbox_msg = point_cloud_camera_.updateCameraBoundingBoxMsg(msg, camera_class_id_confidence_th, camera_object_direction);
        pc_camera_msg = point_cloud_camera_.updateCameraPointCloudMsg(bbox_msg, camera_pointcloud_resolution_m);
    }

    isCameraUpdating = true;
}

void SensorToPointcloud::cliffMsgUpdate(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (target_frame == "map") {
        // tPose pose;
        // pose.position.x = msg->robot_x;
        // pose.position.y = msg->robot_y;
        // pose.orientation.yaw = msg->robot_angle;
        // point_cloud_camera_.updateRobotPose(pose);
    }

    if (use_cliff_pointcloud) {
        pc_cliff_msg = point_cloud_cliff_.updateCliffPointCloudMsg(msg);
    }

    isCliffUpdating = true;
}
