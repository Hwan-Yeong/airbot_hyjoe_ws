#ifndef __SENSOR_INTERFACE_NODE_HPP__
#define __SENSOR_INTERFACE_NODE_HPP__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
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
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "sensor_interface/tof/pointcloud_tof.hpp"
#include "sensor_interface/camera/pointcloud_camera.hpp"
#include "sensor_interface/cliff/pointcloud_cliff.hpp"
#include "sensor_interface/collision/pointcloud_collision.hpp"
#include "logger/camera_object_logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SensorInterfaceNode : public rclcpp::Node
{
public:
    SensorInterfaceNode();
    ~SensorInterfaceNode();

private:
    PointCloudTof point_cloud_tof_;
    PointCloudCamera point_cloud_camera_;
    PointCloudCliff point_cloud_cliff_;
    PointCloudCollision point_cloud_collosion_;
    BoundingBoxGenerator bounding_box_generator_;
    CameraObjectLogger camera_object_logger_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_to_pointcloud_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr cliff_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_sub_;

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_collision_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_array_camera_pub_;

    rclcpp::TimerBase::SharedPtr poincloud_publish_timer_;

    bool isActiveSensorToPointcloud;
    std::string target_frame_;
    bool use_tof_, use_tof_1D_, use_tof_left_, use_tof_right_, use_tof_row_,
        use_camera_, use_cliff_, use_collision_, use_camera_object_logger_;
    float camera_pointcloud_resolution_;
    double camera_logger_distance_margin_, camera_logger_width_margin_, camera_logger_height_margin_;
    std::vector<std::string> camera_param_raw_vector_;
    std::map<int, int> camera_class_id_confidence_th_;
    bool camera_object_direction_;
    int publish_rate_1d_tof_, publish_rate_multi_tof_, publish_rate_row_tof_,
        publish_rate_camera_, publish_rate_cliff_, publish_rate_collision_;
    int publish_cnt_1d_tof_, publish_cnt_multi_tof_, publish_cnt_row_tof_,
        publish_cnt_camera_, publish_cnt_cliff_, publish_cnt_collision_;
    double tilting_ang_1d_tof_;

    sensor_msgs::msg::PointCloud2 pc_tof_1d_msg, pc_tof_multi_msg,
        pc_tof_left_row1_msg, pc_tof_left_row2_msg, pc_tof_left_row3_msg, pc_tof_left_row4_msg,
        pc_tof_right_row1_msg, pc_tof_right_row2_msg, pc_tof_right_row3_msg, pc_tof_right_row4_msg,
        pc_camera_msg, pc_cliff_msg, pc_collision_msg;

    vision_msgs::msg::BoundingBox2DArray bbox_msg;
    visualization_msgs::msg::MarkerArray marker_msg;

    bool isTofUpdating, isCameraUpdating, isCliffUpdating, isCollisionUpdating;

    void publisherMonitor();
    void declareParams();
    void setParams();
    void printParams();
    void initVariables();
    void activeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void cliffMsgUpdate(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void collisionMsgUpdate(const robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg);
    void updateTargetFrames(std::string target_frame);
    void pc_msgReset();
    void pubTofPointcloudMsg();
    void pubCameraPointcloudMsg();
    void pubCliffPointcloudMsg();
    void pubCollisionPointcloudMsg();
    void countPublishHz();
    void checkPublishCnt();
};

#endif // __SENSOR_INTERFACE_NODE_HPP__