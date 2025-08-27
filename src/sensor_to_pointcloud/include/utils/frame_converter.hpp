#ifndef __FRAME_CONVERTER__
#define __FRAME_CONVERTER__

#include <cmath>
#include <vector>
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "utils/common_struct.hpp"


class FrameConverter
{
public:
    FrameConverter();
    ~FrameConverter();

    /**
     * @brief ToF 센서 좌표 데이터를 Robot 좌표계로 변환
     * @param[in] input_points 센서 좌표계 기준 좌표데이터
     * @param[in] isLeft Left Tof 인지 Right Tof 인지 구분 (Left: true / Right: false)
     * @param[in] sensor_frame_pose base_link 기준 센서 frame 위치 정보
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    std::vector<tPoint> tfTofSensor2RobotFrame(const std::vector<tPoint> &input_points, bool isLeft, tPose sensor_frame_pose);

    /**
     * @brief Cliff 센서 좌표 데이터를 Robot 좌표계로 변환
     * @param[in] msg IR 센서 데이터 정보 (On 된 센서에 대해서만 좌표 변환 실행하기 위함)
     * @param[in] sensor_positions base_link 기준으로 미리 변환된 데이터를 전달 (센서 위치는 변하지 않으므로 실행될 때 마다 동일한 연산을 수행하지 않게 하기 위함)
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    std::vector<tPoint> tfCliffSensor2RobotFrame(robot_custom_msgs::msg::BottomIrData::SharedPtr msg, std::vector<tPoint> &sensor_positions);

    /**
     * @brief Collision 이벤트 발생 시 장애물을 생성할 Robot좌표계 기준 좌표 데이터 변환
     * @param[in] msg Collision 이벤트 데이터 정보
     * @param[in] offset_m 로봇 base_link 기준 해당 거리 전방에 장애물로 쓰일 point 1개 생성 [m]
     * @return 변환된 로봇좌표계(base_link) 기준 좌표 데이터
     */
    std::vector<tPoint> tfCollisionData2RobotFrame(robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg, double offset_m);

    /**
     * @brief 로봇좌표계 -> 글로벌좌표계 변환 함수
     * @param[in] input_points 로봇좌표계 기준 좌표 데이터
     * @param[in] robot_pose 로봇의 현재 위치 데이터
     * @return 변환된 글로벌(map) 기준 좌표 데이터
     */
    std::vector<tPoint> tfRobot2GlobalFrame(const std::vector<tPoint> &input_points, tPose robot_pose);
private:
};

#endif // FRAME_CONVERTER