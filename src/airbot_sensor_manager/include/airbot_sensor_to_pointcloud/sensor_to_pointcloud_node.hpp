#ifndef __SENSOR_TO_POINTCLOUD__
#define __SENSOR_TO_POINTCLOUD__

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
#include "airbot_sensor_to_pointcloud/modules/tof/pointcloud_tof.hpp"
#include "airbot_sensor_to_pointcloud/modules/camera/pointcloud_camera.hpp"
#include "airbot_sensor_to_pointcloud/modules/cliff/pointcloud_cliff.hpp"
#include "airbot_sensor_to_pointcloud/modules/collision/pointcloud_collision.hpp"
#include "airbot_sensor_to_pointcloud/modules/camera/logging/camera_object_logger.hpp"
#include "airbot_sensor_to_pointcloud/filters/tof_low_pass_filter.hpp"
#include "airbot_sensor_to_pointcloud/filters/tof_moving_average_filter.hpp"

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
    TofLowPassFilter tof_lpf_;
    TofMovingAverageFilter tof_window_filter_;
    PointCloudGenerator pointcloud_generator_;


    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> target_frame_callback_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> mtof_left_subcell_callback_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> mtof_right_subcell_callback_handle_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_to_pointcloud_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr cliff_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_1d_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_tof_multi_pub_;
    std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_4x4_tof_left_pub_map_;
    std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_4x4_tof_right_pub_map_;
    std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_8x8_tof_left_pub_map_;
    std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_8x8_tof_right_pub_map_;

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
    bool camera_object_direction_, use_tof_8x8_;
    int publish_rate_1d_tof_, publish_rate_multi_tof_, publish_rate_row_tof_,
    publish_rate_camera_, publish_rate_cliff_, publish_rate_collision_;
    int publish_cnt_1d_tof_, publish_cnt_multi_tof_, publish_cnt_row_tof_,
    publish_cnt_camera_, publish_cnt_cliff_, publish_cnt_collision_;
    double tilting_ang_1d_tof_, bot_left_pitch_angle_, bot_right_pitch_angle_;
    double object_max_distance_;
    double mtof_lpf_alpha_;
    int mtof_average_window_size_;
    tTofPitchAngle botTofPitchAngle;
    std::vector<int> mtof_left_sub_cell_idx_array_;
    std::vector<int> mtof_right_sub_cell_idx_array_;

    sensor_msgs::msg::PointCloud2 pc_tof_1d_msg, pc_tof_multi_msg,
        pc_tof_left_row1_msg, pc_tof_left_row2_msg, pc_tof_left_row3_msg, pc_tof_left_row4_msg,
        pc_tof_right_row1_msg, pc_tof_right_row2_msg, pc_tof_right_row3_msg, pc_tof_right_row4_msg,
        pc_camera_msg, pc_cliff_msg, pc_collision_msg;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_8x8_tof_left_msg_map_;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_8x8_tof_right_msg_map_;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_4x4_tof_left_msg_map_;
    std::unordered_map<int, sensor_msgs::msg::PointCloud2> pc_4x4_tof_right_msg_map_;

    vision_msgs::msg::BoundingBox2DArray bbox_msg;
    visualization_msgs::msg::MarkerArray marker_msg;

    bool isTofUpdating, isCameraUpdating, isCliffUpdating, isCollisionUpdating;

    void declareParams();
    void setParams();
    void printParams();
    void initVariables();
    void updateAllParameters();
    void updateAllFrames();

    void publisherMonitor();

    void activeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void cliffMsgUpdate(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void collisionMsgUpdate(const robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg);
};

#endif // SENSOR_TO_POINTCLOUD