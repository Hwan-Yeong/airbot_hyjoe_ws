#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud.hpp"

using namespace std::chrono_literals;

// Robot, Sensor Geometric Specification
double tof_top_sensor_frame_x_translate = 0.0942;    //[meter]
double tof_top_sensor_frame_y_translate = 0.0;       //[meter]
double tof_tof_sensor_frame_z_translate = 0.56513;   //[meter]
double tof_top_sensor_frame_pitch_ang = 50; //30     //[deg]
double tof_bot_sensor_frame_x_translate = 0.14316;   //[meter]
double tof_bot_sensor_frame_y_translate = 0.075446;  //[meter]
double tof_bot_sensor_frame_z_translate = 0.03;      //[meter]
double tof_bot_left_sensor_frame_yaw_ang = 13.0;     //[deg]
double tof_bot_rihgt_sensor_frame_yaw_ang = -15.0;   //[deg]
double tof_bot_fov_ang = 45;                         //[deg]
double camera_sensor_frame_x_translate = 0.15473;    //[meter]
double camera_sensor_frame_y_translate = 0.0;        //[meter]
double camera_sensor_frame_z_translate = 0.5331;     //[meter]

SensoeToPointcloud::SensoeToPointcloud()
    : rclcpp::Node("airbot_sensor_to_pointcloud"),
    point_cloud_tof_(tof_top_sensor_frame_x_translate,
                     tof_top_sensor_frame_y_translate,
                     tof_tof_sensor_frame_z_translate,
                     tof_top_sensor_frame_pitch_ang,
                     tof_bot_sensor_frame_x_translate,
                     tof_bot_sensor_frame_y_translate,
                     tof_bot_sensor_frame_z_translate,
                     tof_bot_left_sensor_frame_yaw_ang,
                     tof_bot_rihgt_sensor_frame_yaw_ang,
                     tof_bot_fov_ang),
    point_cloud_camera_(camera_sensor_frame_x_translate,
                        camera_sensor_frame_y_translate,
                        camera_sensor_frame_z_translate)
{
    // Declare Parameters
    this->declare_parameter("target_frame","base_link");
    this->declare_parameter("use_tof_map_pointcloud",false);
    this->declare_parameter("use_tof_1D",false);
    this->declare_parameter("use_tof_left",false);
    this->declare_parameter("use_tof_right",false);
    this->declare_parameter("tof_debug_mode",false);
    this->declare_parameter("use_camera_map_pointcloud",false);
    this->declare_parameter("camera_pointcloud_resolution_m",0.0);
    this->declare_parameter("camera_target_class_id_list",std::vector<long int>());
    this->declare_parameter("camera_confidence_threshold",0);
    this->declare_parameter("camera_object_direction",false);
    this->declare_parameter("pointcloud_publish_rate_ms",0);

    // Set Parameters
    this->get_parameter("target_frame", target_frame);
    this->get_parameter("use_tof_map_pointcloud", use_tof_map_pointcloud);
    this->get_parameter("use_tof_1D", use_tof_1D);
    this->get_parameter("use_tof_left", use_tof_left);
    this->get_parameter("use_tof_right", use_tof_right);
    this->get_parameter("tof_debug_mode", tof_debug_mode);
    this->get_parameter("use_camera_map_pointcloud", use_camera_map_pointcloud);
    this->get_parameter("camera_pointcloud_resolution_m", camera_pointcloud_resolution_m);
    this->get_parameter("camera_target_class_id_list", camera_target_class_id_list);
    this->get_parameter("camera_confidence_threshold", camera_confidence_threshold);
    this->get_parameter("camera_object_direction", camera_object_direction);
    this->get_parameter("pointcloud_publish_rate_ms", pointcloud_publish_rate_ms);

    // Update Parameters
    point_cloud_tof_.updateTargetFrame(target_frame);
    point_cloud_camera_.updateTargetFrame(target_frame);

    // Msg Update Flags
    isTofUpdating = false;
    isCameraUpdating = false;

    // Msg Subscribers
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensoeToPointcloud::tofMsgUpdate, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::AIDataArray>(
        "camera_data", 10, std::bind(&SensoeToPointcloud::cameraMsgUpdate, this, std::placeholders::_1));
    
    // Msg Publishers
    if (use_tof_map_pointcloud) {
        pc_tof_1d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/mono", 10);
        pc_tof_multi_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi", 10);
        if (tof_debug_mode) {
            pc_tof_left_row1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/left/row_1", 10);
            pc_tof_left_row2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/left/row_2", 10);
            pc_tof_left_row3_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/left/row_3", 10);
            pc_tof_left_row4_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/left/row_4", 10);
            pc_tof_right_row1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/right/row_1", 10);
            pc_tof_right_row2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/right/row_2", 10);
            pc_tof_right_row3_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/right/row_3", 10);
            pc_tof_right_row4_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/tof/multi/right/row_4", 10);
        }
    }
    if (use_camera_map_pointcloud) {
        pc_camera_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/camera_object", 10);
        bbox_array_camera_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2DArray>("sensor_to_pointcloud/camera/bbox", 10);
    }

    // Monitor Timer
    auto publish_rate = std::max(pointcloud_publish_rate_ms, 1) * 1ms; // pointcloud_publish_rate_ms = 0 예외처리
    poincloud_publish_timer_ = this->create_wall_timer(publish_rate,std::bind(&SensoeToPointcloud::publisherMonitor, this));
}

SensoeToPointcloud::~SensoeToPointcloud()
{
}

void SensoeToPointcloud::publisherMonitor()
{
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
        if (use_camera_map_pointcloud) {
            pc_camera_msg = sensor_msgs::msg::PointCloud2(); //clear
            bbox_msg = vision_msgs::msg::BoundingBox2DArray(); //clear
            marker_msg = visualization_msgs::msg::MarkerArray(); //clear
        }
    }

    // publish pointCloud Data
    if (use_tof_map_pointcloud && isTofUpdating) { // ToF
        if (use_tof_1D) {
            pc_tof_1d_pub_->publish(pc_tof_1d_msg);
        }
        if (use_tof_left || use_tof_right) {
            pc_tof_multi_pub_->publish(pc_tof_multi_msg);
        }
        if (tof_debug_mode) {
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
        }
        isTofUpdating = false;
    }

    if (use_camera_map_pointcloud && isCameraUpdating) { // Camera
        pc_camera_pub_->publish(pc_camera_msg);
        bbox_array_camera_pub_->publish(bbox_msg);
        isCameraUpdating = false;
    }
}

void SensoeToPointcloud::tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if (target_frame == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_tof_.updateRobotPose(pose);
    }

    if (use_tof_map_pointcloud) {
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

void SensoeToPointcloud::cameraMsgUpdate(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg)
{
    if (target_frame == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        point_cloud_camera_.updateRobotPose(pose);
    }

    if (use_camera_map_pointcloud) {
        bbox_msg = point_cloud_camera_.updateCameraBoundingBoxMsg(msg, camera_target_class_id_list, camera_confidence_threshold, camera_object_direction);
        pc_camera_msg = point_cloud_camera_.updateCameraPointCloudMsg(bbox_msg, camera_pointcloud_resolution_m);
    }

    isCameraUpdating = true;
}
