#ifndef __SENSOR_INTERFACE_NODE_HPP__
#define __SENSOR_INTERFACE_NODE_HPP__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
#include <builtin_interfaces/msg/time.hpp>
#include "sensor_interface/modules/tof/pointcloud_tof.hpp"
#include "sensor_interface/modules/tof/tof_row_34_processor.hpp"
#include "sensor_interface/modules/camera/pointcloud_camera.hpp"
#include "sensor_interface/modules/cliff/pointcloud_cliff.hpp"
#include "sensor_interface/modules/collision/pointcloud_collision.hpp"
#include "sensor_interface/modules/camera/logging/camera_object_logger.hpp"

/**
 * @brief: sensor_interface는 각 하드웨어 인터페이스에서 발생한 센서 데이터를 구독하고, 주행(navigation)에 필요한 데이터로 가공하여 발행하는 노드입니다.
 * sensor_interface_param.yaml 파일에서 센서 사용 여부, 발행주기 등을 설정할 수 있으며
 * 런타임에 특정 파라미터를 동적으로 변경할 수 있도록 별도의 param_manager_node와 파라미터 서버를 통해 간접적으로 연결되어있습니다.
 */
class SensorInterfaceNode : public rclcpp::Node
{
public:
    SensorInterfaceNode();
    ~SensorInterfaceNode();

private:
    /**
     * @brief SensorInterfaceNode가 발행하는 데이터의 주기를 관리하는 timer_입니다.
     */
    void publisherMonitor();
    /**
     * @brief SensorInterfaceNode에서 사용하는 파라미터를 파라미터 서버에 등록하는 함수입니다.
     */
    void declareParams();

    /**
     * @brief 파라미터 서버에 등록된 파라미터를 함수의 멤버변수에 저장하는 함수입니다.
     */
    void setParams();

    /**
     * @brief 파라미터 값이 적용된 멤버변수를 출력하는 함수입니다.
     */
    void printParams();

    /**
     * @brief 메시지 업데이트 플래그, 데이터 발행주기 관리 변수 등을 초기화하는 함수입니다.
     */
    void initVariables();

    /**
     * @brief SensorInterfaceNode 토픽 발행 on/off 명령 콜백함수입니다.
     */
    void cmdSensorInterfaceCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @name 센서 데이터 업데이트 함수
     * @brief 센서 데이터를 업데이트하는 함수들입니다.
     * @{
     */
    void tofMsgUpdate(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    void cameraMsgUpdate(const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg);
    void cliffMsgUpdate(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg);
    void collisionMsgUpdate(const robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg);
    /** @} */

    /**
     * @brief 센서 데이터 업데이트에 필요한 기준 좌표계를 업데이트하는 함수입니다.
     */
    void updateTargetFrames(std::string target_frame);

    /**
     * @brief 센서 데이터 업데이트가 정상적으로 이루어지지 않았을 때, 메시지를 초기화 하는 함수입니다.
     */
    void msgReset();

    /**
     * @name 센서 데이터 발행 함수
     * @brief 업데이트 된 각 센서 데이터를 발행하는 함수들입니다.
     * @{
     */
    void pubTofMsg();
    void pubCameraMsg();
    void pubCliffMsg();
    void pubCollisionMsg();
    /** @} */

    /**
     * @brief 각 센서 데이터의 발행 주기 카운터를 증가시키는 함수입니다.
     */
    void countPublishCnt();

    /**
     * @brief 비정상 동작 및 비활성 센서가 있는 경우, int 변수 오버플로우(약 21억(2^31 - 1, INT_MAX) 초과) 방지 함수입니다.
     */
    void checkPublishCnt();


    PointCloudTof point_cloud_tof_;
    PointCloudCamera point_cloud_camera_;
    PointCloudCliff point_cloud_cliff_;
    PointCloudCollision point_cloud_collosion_;
    BoundingBoxGenerator bounding_box_generator_;
    CameraObjectLogger camera_object_logger_;
    TofRow34Processor tof_row_34_processor_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_callback_handle_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_interface_cmd_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr cliff_sub_;
    rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr collision_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pc_tof_1d_pub_, pc_tof_multi_pub_,
        pc_tof_left_row1_pub_, pc_tof_left_row2_pub_,
        pc_tof_left_row3_pub_, pc_tof_left_row4_pub_,
        pc_tof_right_row1_pub_, pc_tof_right_row2_pub_,
        pc_tof_right_row3_pub_, pc_tof_right_row4_pub_,
        pc_camera_pub_, pc_cliff_pub_, pc_collision_pub_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr bbox_array_camera_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr float_array_tof_row_34_pub_;

    rclcpp::TimerBase::SharedPtr poincloud_publish_timer_;

    bool isActiveSensorInterface;
    bool isTofUpdating, isCameraUpdating, isCliffUpdating, isCollisionUpdating;
    bool use_tof_, use_tof_1D_,
         use_tof_left_, use_tof_right_, use_tof_row_,
         use_camera_, use_cliff_, use_collision_, use_camera_object_logger_;

    bool camera_object_direction_;
    float camera_pointcloud_resolution_;
    double camera_logger_distance_margin_, camera_logger_width_margin_, camera_logger_height_margin_;
    double object_max_distance_;
    std::vector<std::string> camera_param_raw_vector_;
    std::map<int, int> camera_class_id_confidence_th_;

    double tilting_ang_1d_tof_, bot_left_pitch_angle_, bot_right_pitch_angle_;
    tTofPitchAngle botTofPitchAngle;

    int publish_rate_1d_tof_, publish_rate_multi_tof_, publish_rate_row_tof_,
        publish_rate_camera_, publish_rate_cliff_, publish_rate_collision_,
        publish_cnt_1d_tof_, publish_cnt_multi_tof_, publish_cnt_row_tof_,
        publish_cnt_camera_, publish_cnt_cliff_, publish_cnt_collision_;

    std::string target_frame_;

    sensor_msgs::msg::PointCloud2
        pc_tof_1d_msg, pc_tof_multi_msg,
        pc_tof_left_row1_msg, pc_tof_left_row2_msg,
        pc_tof_left_row3_msg, pc_tof_left_row4_msg,
        pc_tof_right_row1_msg, pc_tof_right_row2_msg,
        pc_tof_right_row3_msg, pc_tof_right_row4_msg,
        pc_camera_msg, pc_cliff_msg, pc_collision_msg;
    std_msgs::msg::Float64MultiArray float_array_tof_row_34_msg;
    vision_msgs::msg::BoundingBox2DArray bbox_camera_msg;
};

#endif // __SENSOR_INTERFACE_NODE_HPP__