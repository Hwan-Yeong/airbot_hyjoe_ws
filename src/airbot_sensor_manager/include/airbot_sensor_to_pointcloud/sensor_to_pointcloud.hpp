#ifndef __SENSOR_TO_POINTCLOUD__
#define __SENSOR_TO_POINTCLOUD__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "airbot_sensor_to_pointcloud/tof/pointcloud_tof.hpp"
#include "airbot_sensor_to_pointcloud/camera/pointcloud_camera.hpp"
#include "airbot_sensor_to_pointcloud/cliff/pointcloud_cliff.hpp"
#include "logger/camera_object_logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace message_filters;
using SyncPolicy = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;


class SensorToPointcloud : public rclcpp::Node
{
public:
    SensorToPointcloud();
    ~SensorToPointcloud();

private:
    PointCloudTof point_cloud_tof_;
    PointCloudCamera point_cloud_camera_;
    PointCloudCliff point_cloud_cliff_;
    BoundingBoxGenerator bounding_box_generator_;
    CameraObjectLogger camera_object_logger_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;

    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cliff_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser1_sub_, laser2_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_lidar_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_array_camera_pub_;

    rclcpp::TimerBase::SharedPtr poincloud_publish_timer_;

    std::string target_frame_;
    bool use_tof_;
    bool use_tof_1D_;
    bool use_tof_left_;
    bool use_tof_right_;
    bool use_tof_row_;
    bool use_camera_;
    bool ues_cliff_;
    bool use_camera_object_logger_;
    float camera_pointcloud_resolution_;
    double camera_logger_distance_margin_;
    double camera_logger_width_margin_;
    double camera_logger_height_margin_;
    std::vector<std::string> camera_param_raw_vector_;
    std::map<int, int> camera_class_id_confidence_th_;
    bool camera_object_direction_;
    int publish_rate_1d_tof_;
    int publish_rate_multi_tof_;
    int publish_rate_row_tof_;
    int publish_rate_camera_;
    int publish_rate_cliff_;
    int publish_cnt_1d_tof_;
    int publish_cnt_multi_tof_;
    int publish_cnt_row_tof_;
    int publish_cnt_camera_;
    int publish_cnt_cliff_;
    double tilting_ang_1d_tof_;

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
    sensor_msgs::msg::PointCloud2 pc_lidar_msg;
    vision_msgs::msg::BoundingBox2DArray bbox_msg;
    visualization_msgs::msg::MarkerArray marker_msg;

    bool isTofUpdating;
    bool isCameraUpdating;
    bool isCliffUpdating;

    std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
    bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;
    float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
    uint8_t laser1R_, laser1G_, laser1B_;

    float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
    uint8_t laser2R_, laser2G_, laser2B_;

    sensor_msgs::msg::LaserScan laser1_;
    sensor_msgs::msg::LaserScan laser2_;

    void initialize_lidar_params();
    void refresh_lidar_params();
    void print_lidar_params();
    void publisherMonitor();
    void tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void cliffMsgUpdate(const std_msgs::msg::UInt8::SharedPtr msg);
    void synchronizedCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg);
    void update_point_cloud_rgb();
    void removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ>& cloud, float radius, float center_x = 0.0f, float center_y = 0.0f);
};

#endif // SENSOR_TO_POINTCLOUD