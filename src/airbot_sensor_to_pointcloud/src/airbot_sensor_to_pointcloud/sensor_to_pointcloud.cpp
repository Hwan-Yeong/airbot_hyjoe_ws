#include "airbot_sensor_to_pointcloud/sensor_to_pointcloud.hpp"

using namespace std::chrono_literals;

float robot_radius = 0.3;                           //[meter]
float tof_top_sensor_frame_x_translate = 0.0942;    //[meter]
float tof_top_sensor_frame_y_translate = 0.0;       //[meter]
float tof_tof_sensor_frame_z_translate = 0.56513;   //[meter]
float tof_top_sensor_frame_pitch_ang = 50; //30     //[deg]
float tof_bot_sensor_frame_x_translate = 0.14316;   //[meter]
float tof_bot_sensor_frame_y_translate = 0.075446;  //[meter]
float tof_bot_sensor_frame_z_translate = 0.03;      //[meter]
float tof_bot_left_sensor_frame_yaw_ang = 13.0;     //[deg]
float tof_bot_rihgt_sensor_frame_yaw_ang = -15.0;   //[deg]
float tof_bot_fov_ang = 45;                         //[deg]
float camera_sensor_frame_x_translate = 0.15473;    //[meter]
float camera_sensor_frame_y_translate = 0.0;        //[meter]
float camera_sensor_frame_z_translate = 0.5331;     //[meter]

SensoeToPointcloud::SensoeToPointcloud()
    : rclcpp::Node("airbot_sensor_to_pointcloud"),
    pointCloud(robot_radius,
               tof_top_sensor_frame_x_translate,
               tof_top_sensor_frame_y_translate,
               tof_tof_sensor_frame_z_translate,
               tof_top_sensor_frame_pitch_ang,
               tof_bot_sensor_frame_x_translate,
               tof_bot_sensor_frame_y_translate,
               tof_bot_sensor_frame_z_translate,
               tof_bot_left_sensor_frame_yaw_ang,
               tof_bot_rihgt_sensor_frame_yaw_ang,
               tof_bot_fov_ang,
               camera_sensor_frame_x_translate,
               camera_sensor_frame_y_translate,
               camera_sensor_frame_z_translate)
{
    this->declare_parameter("target_frame","base_link");
    this->declare_parameter("use_tof_map_pointcloud",false);
    this->declare_parameter("use_tof_1D",false);
    this->declare_parameter("use_tof_left",false);
    this->declare_parameter("use_tof_right",false);
    this->declare_parameter("tof_debug_mode",false);
    this->declare_parameter("use_camera_map_pointcloud",false);
    this->declare_parameter("use_line_laser_map_pointcloud",false);
    this->declare_parameter("camera_pointcloud_resolution_m",0.0);
    this->declare_parameter("camera_number_of_object",0);
    this->declare_parameter("pointcloud_publish_rate_ms",0);

    this->get_parameter("target_frame", target_frame);
    this->get_parameter("use_tof_map_pointcloud", use_tof_map_pointcloud);
    this->get_parameter("use_tof_1D", use_tof_1D);
    this->get_parameter("use_tof_left", use_tof_left);
    this->get_parameter("use_tof_right", use_tof_right);
    this->get_parameter("tof_debug_mode", tof_debug_mode);
    this->get_parameter("use_camera_map_pointcloud", use_camera_map_pointcloud);
    this->get_parameter("use_line_laser_map_pointcloud", use_line_laser_map_pointcloud);
    this->get_parameter("camera_pointcloud_resolution_m", camera_pointcloud_resolution_m);
    this->get_parameter("camera_number_of_object", camera_number_of_object);
    this->get_parameter("pointcloud_publish_rate_ms", pointcloud_publish_rate_ms);

    isTofUpdating = false;
    isCameraUpdating = false;
    isLineLaserUpdating = false;

    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "tof_data", 10, std::bind(&SensoeToPointcloud::tofCallback, this, std::placeholders::_1));

    camera_sub_ = this->create_subscription<robot_custom_msgs::msg::AIDataArray>(
        "camera_data", 10, std::bind(&SensoeToPointcloud::cameraCallback, this, std::placeholders::_1));
    
    line_laser_sub_ = this->create_subscription<robot_custom_msgs::msg::LineLaserData>(
        "line_laser_data", 10, std::bind(&SensoeToPointcloud::lineLaserCallback, this, std::placeholders::_1));
    
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
        marker_array_camera_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sensor_to_pointcloud/camera/marker", 10);
    }

    if (use_line_laser_map_pointcloud) {
        pc_line_laser_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_to_pointcloud/line_laser", 10);
    }

    auto publish_rate = std::max(pointcloud_publish_rate_ms, 1) * 1ms; // pointcloud_publish_rate_ms = 0 예외처리
    poincloud_publish_timer_ = this->create_wall_timer(publish_rate,std::bind(&SensoeToPointcloud::publisherMonitor, this));

    pointCloud.updateTargetFrame(target_frame);
}

SensoeToPointcloud::~SensoeToPointcloud()
{
}

void SensoeToPointcloud::publisherMonitor()
{
#if 0 // 동작 확인용 로그
    RCLCPP_INFO(this->get_logger(), "[PARAM] ToF : %d, Camera: %d, 2LL: %d", use_tof_map_pointcloud, use_camera_map_pointcloud, use_line_laser_map_pointcloud);
    RCLCPP_INFO(this->get_logger(), "[PARAM-DEBUG] 1D : %d, left: %d, right: %d", use_tof_1D, use_tof_left, use_tof_right);
    RCLCPP_WARN(this->get_logger(), "[FLAG] ToF : %d, Camera: %d, 2LL: %d", isTofUpdating, isCameraUpdating, isLineLaserUpdating);
#endif
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
        }
    }
    if (!isLineLaserUpdating) {
        if (use_line_laser_map_pointcloud) {
            pc_line_laser_msg = sensor_msgs::msg::PointCloud2(); //clear
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
        isCameraUpdating = false;
    }

    if (use_line_laser_map_pointcloud && isLineLaserUpdating) { // Line Laser
        pc_line_laser_pub_->publish(pc_line_laser_msg);
        isLineLaserUpdating = false;
    }
}

void SensoeToPointcloud::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    if (use_tof_map_pointcloud) {
        if (use_tof_1D) {
            pc_tof_1d_msg = pointCloud.getConvertedTofTopToPointCloud(msg);
        }
        if (use_tof_left || use_tof_right) {
            TOF_SIDE side = (use_tof_left && use_tof_right) ? TOF_SIDE::BOTH : 
                            (use_tof_left ? TOF_SIDE::LEFT : TOF_SIDE::RIGHT);
            pc_tof_multi_msg = pointCloud.getConvertedTofBotToPointCloud(msg, side, false);
        }
        if (tof_debug_mode) {
            if (use_tof_left) {
                pc_tof_left_row1_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::FIRST);
                pc_tof_left_row2_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::SECOND);
                pc_tof_left_row3_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::THIRD);
                pc_tof_left_row4_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::LEFT, true, ROW_NUMBER::FOURTH);
            }
            if (use_tof_right) {
                pc_tof_right_row1_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::FIRST);
                pc_tof_right_row2_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::SECOND);
                pc_tof_right_row3_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::THIRD);
                pc_tof_right_row4_msg = pointCloud.getConvertedTofBotToPointCloud(msg, TOF_SIDE::RIGHT, true, ROW_NUMBER::FOURTH);
            }
        }
    }

    if (target_frame == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        pointCloud.updateRobotPose(pose);
    }
    isTofUpdating = true;
}

void SensoeToPointcloud::cameraCallback(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg)
{
    if (use_camera_map_pointcloud) {
        pc_camera_msg = pointCloud.getConvertedCameraToPointCloud(msg, camera_pointcloud_resolution_m, camera_number_of_object);
    }

    vision_msgs::msg::BoundingBox2DArray bbox_msg = pointCloud.getCameraBoundingBoxMessage();
    visualization_msgs::msg::MarkerArray marker_msg = bboxArrayToMarkerArray(bbox_msg);
    bbox_array_camera_pub_->publish(bbox_msg);
    marker_array_camera_pub_->publish(marker_msg);

    if (target_frame == "map") {
        tPose pose;
        pose.position.x = msg->robot_x;
        pose.position.y = msg->robot_y;
        pose.orientation.yaw = msg->robot_angle;
        pointCloud.updateRobotPose(pose);
    }
    isCameraUpdating = true;
}

void SensoeToPointcloud::lineLaserCallback(const robot_custom_msgs::msg::LineLaserData::SharedPtr msg)
{
    if (use_line_laser_map_pointcloud) {
        pc_line_laser_msg = pointCloud.getConvertedLineLaserToPointCloud(msg);
    }

    // Line Laser에 amcl pose 추가되면 필요
    // if (target_frame == "map") {
    //     tPose pose;
    //     pose.position.x = msg->robot_x;
    //     pose.position.y = msg->robot_y;
    //     pose.orientation.yaw = msg->robot_angle;
    //     pointCloud.updateRobotPose(pose);
    // }
    isLineLaserUpdating = true;
}

// 검증 안됨
visualization_msgs::msg::MarkerArray SensoeToPointcloud::bboxArrayToMarkerArray(const vision_msgs::msg::BoundingBox2DArray msg)
{
    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    for (const auto& bbox : msg.boxes) {
        visualization_msgs::msg::Marker marker;
        marker.header = msg.header;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5; // Line thickness
        marker.color.r = 1.0; // Red
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Opaque

        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.center.position.x - bbox.size_x/2; p1.y = bbox.center.position.y - bbox.size_y/2; p1.z = 0.0;
        p2.x = bbox.center.position.x + bbox.size_x/2; p2.y = bbox.center.position.y - bbox.size_y/2; p2.z = 0.0;
        p3.x = bbox.center.position.x + bbox.size_x/2; p3.y = bbox.center.position.y + bbox.size_y/2; p3.z = 0.0;
        p4.x = bbox.center.position.x - bbox.size_x/2; p4.y = bbox.center.position.y + bbox.size_y/2; p4.z = 0.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p4);
        marker.points.push_back(p1); // Close the loop

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}
