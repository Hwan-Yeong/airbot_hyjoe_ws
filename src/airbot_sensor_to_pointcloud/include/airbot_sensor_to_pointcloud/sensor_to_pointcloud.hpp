#ifndef __SENSOR_TO_POINTCLOUD__
#define __SENSOR_TO_POINTCLOUD__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
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
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "airbot_sensor_to_pointcloud/tof/pointcloud_tof.hpp"
#include "airbot_sensor_to_pointcloud/camera/pointcloud_camera.hpp"
#include "airbot_sensor_to_pointcloud/cliff/pointcloud_cliff.hpp"


class SensorToPointcloud : public rclcpp::Node
{
public:
    SensorToPointcloud();
    ~SensorToPointcloud();

private:
    PointCloudTof point_cloud_tof_;
    PointCloudCamera point_cloud_camera_;
    PointCloudCliff point_cloud_cliff_;

    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AIDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cliff_sub_;

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_cliff_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_array_camera_pub_;

    rclcpp::TimerBase::SharedPtr poincloud_publish_timer_;

    std::string target_frame;
    bool use_tof;
    bool use_tof_1D;
    bool use_tof_left;
    bool use_tof_right;
    bool use_tof_row;
    bool use_camera;
    bool ues_cliff;
    float camera_pointcloud_resolution;
    std::vector<std::string> camera_param_raw_vector;
    std::map<int, int> camera_class_id_confidence_th;
    bool camera_object_direction;
    int publish_rate_1d_tof;
    int publish_rate_multi_tof;
    int publish_rate_row_tof;
    int publish_rate_camera;
    int publish_rate_cliff;
    int publish_cnt_1d_tof;
    int publish_cnt_multi_tof;
    int publish_cnt_row_tof;
    int publish_cnt_camera;
    int publish_cnt_cliff;

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
    sensor_msgs::msg::PointCloud2 pc_cliff_msg;
    vision_msgs::msg::BoundingBox2DArray bbox_msg;
    visualization_msgs::msg::MarkerArray marker_msg;

    bool isTofUpdating;
    bool isCameraUpdating;
    bool isCliffUpdating;

    void publisherMonitor();
    void tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraMsgUpdate(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg);
    void cliffMsgUpdate(const std_msgs::msg::UInt8::SharedPtr msg);
};

#endif // SENSOR_TO_POINTCLOUD