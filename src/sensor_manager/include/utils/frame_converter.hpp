#ifndef __FRAME_CONVERTER__
#define __FRAME_CONVERTER__

#include <cmath>
#include <vector>
#include <map>
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"

#include "utils/common_struct.hpp"


class FrameConverter
{
public:
    FrameConverter();
    ~FrameConverter();

    /**
     * @brief Mono ToF 센서 좌표 데이터를 Robot 좌표계로 변환
     * @param[in] input_dist 센서 좌표계 기준 1D ToF 거리 데이터
     * @param[in] mono_tof_sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    tPoint tfMonoTofSensor2RobotFrame(const double input_dist, tPose mono_tof_sensor_frame_pose);

    /**
     * @brief Multi ToF 센서 좌표 데이터를 Robot 좌표계로 변환
     * @param[in] input_points 센서 좌표계 기준 좌표데이터
     * @param[in] isLeft Left Tof 인지 Right Tof 인지 구분 (Left: true / Right: false)
     * @param[in] multi_tof_sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    std::vector<tPoint> tfMultiTofSensor2RobotFrame(
        const std::vector<double>& tof_dists,
        const std::vector<double>& y_tan,
        const std::vector<double>& z_tan,
        bool is_left,
        const tPose& multi_tof_sensor_frame_pose);

    /**
     * @brief Camera 센서 좌표 데이터를 Robot 좌표계 기준 bbox msg로 변환
     * @param[in] camera_msg camera 센서 데이터
     * @param[in] robot_pose 현재 robot 위치 좌표
     * @param[in] class_id_confidence_th 객체 id 및 confidence threshold 매칭 데이터 <map type> (객체별 장애물 처리 차별화를 위해)
     * @param[in] direction [legacy code] 좌표계 CCW/CW(+) 정책 변경에 대비하여 (개발기간 발생한 소통 오류로 인해 추가된 기능. 현재는 true로 고정)
     * @param[in] object_max_distance 장애물 최대 거리 제한 (filtering)
     * @return 변환된 로봇좌표계(base_link) 기준 bounding box array 데이터
     */
    vision_msgs::msg::BoundingBox2DArray tfCameraSensor2RobotFrameBBoxArray(
        const robot_custom_msgs::msg::CameraDataArray* camera_msg,
        tPose &robot_pose,
        std::map<int, int> class_id_confidence_th,
        bool direction,
        double object_max_distance,
        std::string camera_target_frame,
        tPose camera_sensor_frame_pose);

    /**
     * @brief Cliff 센서 좌표 데이터를 Robot 좌표계로 변환
     * @param[in] cliff_msg IR 센서 데이터 정보 (On 된 센서에 대해서만 좌표 변환 실행하기 위함)
     * @param[in] sensor_positions base_link 기준으로 미리 변환된 데이터를 전달 (센서 위치는 변하지 않으므로 실행될 때 마다 동일한 연산을 수행하지 않게 하기 위함)
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    std::vector<tPoint> tfBottomIrSensor2RobotFrame(const robot_custom_msgs::msg::BottomIrData* cliff_msg, double distance_center_to_front_ir_sensor, double angle_to_next_ir_sensor);

    /**
     * @brief Collision 이벤트 발생 시 장애물을 생성할 Robot좌표계 기준 좌표 데이터 변환
     * @param[in] collision_msg Collision 이벤트 데이터 정보
     * @param[in] offset_m 로봇 base_link 기준 해당 거리 전방에 장애물로 쓰일 point 1개 생성 [m]
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    tPoint tfCollisionData2RobotFrame(const robot_custom_msgs::msg::AbnormalEventData* collision_msg, double offset_m);

    /**
     * @brief 로봇좌표계 -> 글로벌좌표계 변환 함수
     * @param[in] input_points_on_robot_frame 로봇좌표계 기준 좌표 데이터
     * @param[in] robot_pose 로봇의 현재 위치 데이터
     * @return 변환된 글로벌(map) 기준 좌표 데이터
     */
    std::vector<tPoint> tfRobot2GlobalFrame(const std::vector<tPoint> &input_points_on_robot_frame, tPose robot_pose);
    std::vector<tPoint> tfRobot2GlobalFrame(const tPoint &input_point_on_robot_frame, tPose robot_pose);
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

    // bottom ir
    bool bottom_ir_extrinsics_updated = false;
    std::vector<tPoint> bottom_ir_sensor_positions;
};

#endif // FRAME_CONVERTER