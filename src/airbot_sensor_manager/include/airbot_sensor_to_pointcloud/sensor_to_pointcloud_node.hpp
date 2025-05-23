#ifndef __SENSOR_TO_POINTCLOUD__
#define __SENSOR_TO_POINTCLOUD__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
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
#include "airbot_sensor_to_pointcloud/modules/tof/pointcloud_tof.hpp"
#include "airbot_sensor_to_pointcloud/modules/camera/pointcloud_camera.hpp"
#include "airbot_sensor_to_pointcloud/modules/cliff/pointcloud_cliff.hpp"
#include "airbot_sensor_to_pointcloud/modules/collision/pointcloud_collision.hpp"
#include "airbot_sensor_to_pointcloud/modules/camera/logging/camera_object_logger.hpp"
#include "airbot_sensor_to_pointcloud/filters/tof_low_pass_filter.hpp"
#include "airbot_sensor_to_pointcloud/filters/tof_moving_average_filter.hpp"
#include "airbot_sensor_to_pointcloud/filters/tof_complementary_filte.hpp"

using PC2PublisherPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

class SensorToPointcloud : public rclcpp::Node
{
public:
    SensorToPointcloud();
    ~SensorToPointcloud();

    void init();

private:
    PointCloudTof point_cloud_tof_;
    PointCloudCamera point_cloud_camera_;
    PointCloudCliff point_cloud_cliff_;
    PointCloudCollision point_cloud_collosion_;
    BoundingBoxGenerator bounding_box_generator_;
    CameraObjectLogger camera_object_logger_;
    PointCloudGenerator pointcloud_generator_;
    TofLowPassFilter tof_lp_filter_;
    TofMovingAverageFilter tof_ma_filter_;
    TofComplementaryFilter tof_comp_filter_;

    YAML::Node config{};

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> target_frame_callback_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> mtof_left_subcell_callback_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> mtof_right_subcell_callback_handle_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_to_pointcloud_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr cliff_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_sub_;

    std::unordered_map<std::string, PC2PublisherPtr> pointcloud_pubs_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_array_camera_pub_;

    rclcpp::TimerBase::SharedPtr poincloud_publish_timer_;

    //debug
    rclcpp::Publisher<robot_custom_msgs::msg::TofData>::SharedPtr tof_debug_pub_;

    std::string target_frame_;

    bool use_tof_8x8_;
    bool use_camera_log_;
    bool camera_object_direction_;
    int publish_cnt_1d_tof_;
    int publish_cnt_row_tof_;
    int publish_cnt_multi_tof_;
    int publish_cnt_camera_;
    int publish_cnt_cliff_;
    int publish_cnt_collision_;
    float camera_pointcloud_resolution_;
    double camera_logger_distance_margin_;
    double camera_logger_width_margin_;
    double camera_logger_height_margin_;
    double object_max_distance_;
    std::vector<std::string> camera_param_raw_vector_;
    std::map<int, int> camera_class_id_confidence_th_;

    tFilterConfig mtof_filter_;
    tSensorConfig sensor_config_;
    tTofPitchAngle botTofPitchAngle_;

    sensor_msgs::msg::PointCloud2 pc_tof_1d_msg;
    sensor_msgs::msg::PointCloud2 pc_tof_multi_msg;
    sensor_msgs::msg::PointCloud2 pc_camera_msg;
    sensor_msgs::msg::PointCloud2 pc_cliff_msg;
    sensor_msgs::msg::PointCloud2 pc_collision_msg;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_8x8_tof_left_msg_map_;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_8x8_tof_right_msg_map_;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_4x4_tof_left_msg_map_;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_4x4_tof_right_msg_map_;
    vision_msgs::msg::BoundingBox2DArray bbox_msg;

    bool isActiveSensorToPointcloud;
    bool isTofUpdating;
    bool isCameraUpdating;
    bool isCliffUpdating;
    bool isCollisionUpdating;

    void declareParams();
    void setParams();
    void printParams();
    void initVariables();
    void initSensorConfig(const YAML::Node& config);
    void initPublisher(const YAML::Node& config);
    void initFilterParam(const YAML::Node & node);
    tSensor getSensorCfg(const YAML::Node& node);

    void updateAllParameters();
    void updateAllFrames();
    void updateAllFilters();
    void publishEmptyMsg();

    void publisherMonitor();

    void activeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void cliffMsgUpdate(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void collisionMsgUpdate(const robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg);
};

#endif // SENSOR_TO_POINTCLOUD