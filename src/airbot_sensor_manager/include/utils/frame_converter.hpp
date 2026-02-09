#ifndef __FRAME_CONVERTER__
#define __FRAME_CONVERTER__

#include <cmath>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"

#include "utils/common_struct.hpp"

namespace sensor_manager {

/**
 * @brief Camera 객체 정보와 ID를 묶어서 관리하기 위한 구조체
 */
struct CameraObject {
    uint32_t id;
    vision_msgs::msg::BoundingBox2D bbox;
};

class FrameConverter
{
public:
    FrameConverter();
    ~FrameConverter();

    /**
     * @brief Mono ToF 센서 좌표 데이터를 Robot 좌표계로 변환
     * 
     * @param[in] input_dist 센서 좌표계 기준 1D ToF 거리 데이터
     * @param[in] mono_tof_sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    tPoint tfMonoTofSensor2RobotFrame(const double input_dist, tPose mono_tof_sensor_frame_pose);

    /**
     * @brief Multi ToF 센서 거리 데이터를 Robot 좌표계로 변환
     * 
     * @param[in] tof_dists 센서 거리 데이터
     * @param[in] multi_tof_sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    std::vector<tPoint> tfMultiTofSensor2RobotFrame(
        const std::vector<double>& tof_dists,
        const std::vector<double>& y_tan,
        const std::vector<double>& z_tan,
        const tPose& multi_tof_sensor_frame_pose);

    /**
     * @brief Multi ToF 센서 거리 데이터를 센서 좌표계 위치로 변환
     * 
     * @param[in] tof_dists 센서 거리 데이터
     * @return 센서 좌표계 기준 위치 데이터
     */
    std::vector<tPoint> tfMultiTofDistance2SensorFrame(
        const std::vector<double>& tof_dists,
        const std::vector<double>& y_tan,
        const std::vector<double>& z_tan);

    /**
     * @brief Camera 센서 데이터를 센서 좌표계(Sensor Frame) 기준 객체 리스트로 변환
     */
    std::vector<CameraObject> tfCameraSensor2SensorFrame(
        const robot_custom_msgs::msg::CameraDataArray* camera_msg,
        bool direction,
        double object_max_distance);

    /**
     * @brief 센서 좌표계 기준 객체 리스트를 로봇 좌표계(base_link)로 변환
     */
    std::vector<CameraObject> tfCameraObjects2RobotFrame(
        const std::vector<CameraObject>& objects_sensor,
        const tPose& sensor_frame_pose);

    /**
     * @brief 로봇 좌표계 기준 객체 리스트를 글로벌 좌표계(map)로 변환
     */
    std::vector<CameraObject> tfCameraObjects2GlobalFrame(
        const std::vector<CameraObject>& objects_robot,
        const tPose& robot_pose);

    /**
     * @brief CameraObject 리스트를 BoundingBox2DArray 메시지로 변환
     */
    vision_msgs::msg::BoundingBox2DArray toBBoxArray(
        const std::vector<CameraObject>& objects);

    /**
     * @brief 센서 좌표계 위치를 로봇 좌표계 기준 위치로 변환 (단일 포인트)
     * 
     * @param[in] pt_sensor 센서 좌표계 기준 위치 데이터
     * @param[in] sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇 좌표계 기준 위치 데이터
     */
    tPoint tfSensorFrame2RobotFrame(
        const tPoint& pt_sensor,
        const tPose& sensor_frame_pose);

    /**
     * @brief 센서 좌표계 위치를 로봇 좌표계 기준 위치로 변환 (포인트 리스트)
     * 
     * @param[in] pts_sensor 센서 좌표계 기준 위치 데이터
     * @param[in] sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇 좌표계 기준 위치 데이터
     */
    std::vector<tPoint> tfSensorFrame2RobotFrame(
        const std::vector<tPoint>& pts_sensor,
        const tPose& sensor_frame_pose);

    /**
     * @brief Camera 센서 데이터를 센서 좌표계(Sensor Frame) 기준 Bounding Box Array로 변환
     * 
     * @param[in] camera_msg camera 센서 데이터 (CameraDataArray)
     * @param[in] direction 객체 인식 방향 (CCW+, 현재는 true 고정)
     * @param[in] object_max_distance 객체 인식 최대 거리 제한 [m]
     * @param[in] child_frame 센서 좌표계 이름 (예: "camera_link")
     * @return 센서 좌표계(Sensor Frame) 기준 Bounding Box Array 데이터
     */
    vision_msgs::msg::BoundingBox2DArray tfCameraSensor2SensorFrameBBoxArray(
        const robot_custom_msgs::msg::CameraDataArray* camera_msg,
        bool direction,
        double object_max_distance,
        std::string child_frame);

    /**
     * @brief 센서 좌표계 기준 Bounding Box Array를 로봇/타겟 좌표계로 변환 (Camera 전용)
     * 
     * @param[in] bbox_array_sensor 센서 좌표계 기준 BBox Array
     * @param[in] robot_pose 로봇의 현재 위치 (Target Frame이 map일 경우 필요)
     * @param[in] target_frame 변환할 대상 좌표계 ("base_link" 또는 "map")
     * @param[in] camera_sensor_frame_pose 로봇(base_link) 기준 카메라 센서의 위치 정보
     * @param[in] class_id_confidence_th 객체 id 및 confidence threshold 매칭 데이터
     * @param[in] use_object_logger 객체 인식 로깅 사용 여부
     * @param[in] logger_dist_margin 로깅 거리 임계값
     * @param[in] logger rclcpp 로거 객체
     * @return 변환된 타겟 좌표계(Target Frame) 기준 Bounding Box Array 데이터
     */
    vision_msgs::msg::BoundingBox2DArray tfCameraSensorFrameBBoxArray2TargetFrame(
        const vision_msgs::msg::BoundingBox2DArray& bbox_array_sensor,
        tPose &robot_pose,
        std::string target_frame,
        tPose camera_sensor_frame_pose,
        std::map<int, int> class_id_confidence_th = {},
        bool use_object_logger = false,
        double logger_dist_margin = 0.0,
        rclcpp::Logger logger = rclcpp::get_logger("FrameConverter"));

    /**
     * @brief Bottom IR 센서의 활성화 위치를 센서 좌표계(Sensor Frame) 기준으로 반환
     * 
     * @param[in] cliff_msg IR 센서 데이터 (BottomIrData)
     * @param[in] distance_center_to_front_ir_sensor 로봇 중심에서 정방 IR 센서까지의 거리 [m]
     * @param[in] angle_to_next_ir_sensor 센서 간 간격 각도 [deg]
     * @return 센서 좌표계(Sensor Frame) 기준 활성화된 센서 위치 리스트
     */
    std::vector<tPoint> tfBottomIrSensor2SensorFrame(
        const robot_custom_msgs::msg::BottomIrData* cliff_msg,
        double distance_center_to_front_ir_sensor,
        double angle_to_next_ir_sensor);

    /**
     * @brief Bottom IR 센서의 활성화 위치를 로봇 좌표계(Robot Frame) 기준으로 반환
     */
    std::vector<tPoint> tfBottomIrSensor2RobotFrame(
        const robot_custom_msgs::msg::BottomIrData* cliff_msg,
        double distance_center_to_front_ir_sensor,
        double angle_to_next_ir_sensor);

    /**
     * @brief Collision 이벤트를 센서 좌표계(Sensor Frame) 기준 좌표로 변환
     * 
     * @param[in] collision_msg 충돌 이벤트 데이터
     * @param[in] offset_m 센서 기준 전방 장애물 생성 오프셋 [m]
     * @return 센서 좌표계(Sensor Frame) 기준 충돌 감지 포인트
     */
    tPoint tfCollisionData2SensorFrame(
        const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
        double offset_m);

    /**
     * @brief Collision 이벤트를 로봇 좌표계(Robot Frame) 기준 좌표로 변환
     */
    tPoint tfCollisionData2RobotFrame(
        const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
        double offset_m);

    /**
     * @brief 로봇좌표계 -> 글로벌좌표계 변환 함수
     * 
     * @param[in] input_points_on_robot_frame 로봇좌표계 기준 좌표 데이터
     * @param[in] robot_pose 로봇의 현재 위치 데이터
     * @return 변환된 글로벌(map) 기준 좌표 데이터
     */
    std::vector<tPoint> tfRobot2GlobalFrame(const std::vector<tPoint> &input_points_on_robot_frame, tPose robot_pose);
    std::vector<tPoint> tfRobot2GlobalFrame(const tPoint &input_point_on_robot_frame, tPose robot_pose);

    /**
     * @brief 카메라 객체 logging 정보 이력 초기화 함수
     */
    void loggedObjectInfoClear() { logged_objects_.clear(); };
private:
    // tof mono
    bool tof_mono_extrinsics_updated = false;
    double tof_mono_sensor_frame_pitch_cosine = 0.0;
    double tof_mono_sensor_frame_pitch_sine = 0.0;

    // tof multi
    bool tof_multi_extrinsics_updated = false;
    double multi_tof_sensor_frame_yaw_cosine = 0.0;
    double multi_tof_sensor_frame_yaw_sine = 0.0;
    double multi_tof_sensor_frame_pitch_cosine = 0.0;
    double multi_tof_sensor_frame_pitch_sine = 0.0;

    // camera
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> logged_objects_;

    // bottom ir
    bool bottom_ir_extrinsics_updated = false;
    std::vector<tPoint> bottom_ir_sensor_positions;
};

} // namespace sensor_manager

#endif // FRAME_CONVERTER