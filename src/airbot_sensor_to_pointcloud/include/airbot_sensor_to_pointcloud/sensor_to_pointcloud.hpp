#ifndef __SENSOR_TO_POINTCLOUD__
#define __SENSOR_TO_POINTCLOUD__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "airbot_sensor_to_pointcloud/pointcloud.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/ai_data.hpp"
#include "robot_custom_msgs/msg/ai_data_array.hpp"
#include "robot_custom_msgs/msg/line_laser_data.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <builtin_interfaces/msg/time.hpp>


class SensoeToPointcloud : public rclcpp::Node
{
public:
    SensoeToPointcloud();
    ~SensoeToPointcloud();

private:
    PointCloud pointCloud;

    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AIDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::LineLaserData>::SharedPtr line_laser_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_1d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_multi_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_left_row1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_left_row2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_left_row3_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_left_row4_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_right_row1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_right_row2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_right_row3_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_right_row4_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_camera_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_line_laser_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_array_camera_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_camera_pub_;

    rclcpp::TimerBase::SharedPtr poincloud_publish_timer_;

    std::string target_frame;
    bool use_tof_map_pointcloud;
    bool use_tof_1D;
    bool use_tof_left;
    bool use_tof_right;
    bool tof_debug_mode;
    bool use_camera_map_pointcloud;
    bool use_line_laser_map_pointcloud;
    float camera_pointcloud_resolution_m;
    int camera_number_of_object;
    int pointcloud_publish_rate_ms;

    sensor_msgs::msg::PointCloud2 pc_tof_1d_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_multi_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_left_row1_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_left_row2_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_left_row3_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_left_row4_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_right_row1_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_right_row2_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_right_row3_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_right_row4_msg;
    sensor_msgs::msg::PointCloud2 pc_camera_msg;
    sensor_msgs::msg::PointCloud2 pc_line_laser_msg;

    bool isTofUpdating;
    bool isCameraUpdating;
    bool isLineLaserUpdating;

    void publisherMonitor();
    void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraCallback(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg);
    void lineLaserCallback(const robot_custom_msgs::msg::LineLaserData::SharedPtr msg);
    visualization_msgs::msg::MarkerArray bboxArrayToMarkerArray(const vision_msgs::msg::BoundingBox2DArray msg);
};

#endif // SENSOR_TO_POINTCLOUD