#ifndef __POINTCLOUD_TOF_HPP__
#define __POINTCLOUD_TOF_HPP__

#include <cmath>
#include <string>
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "utils/frame_converter.hpp"
#include "utils/pointcloud_generator.hpp"

struct tTofPitchAngle {
    double bot_left;
    double bot_right;
};

/**
 * @brief: 1D/Multi ToF 데이터를 pointcloud로 변환시키는 클래스입니다.
 */
class PointCloudTof
{
public:
    PointCloudTof(double tof_top_sensor_frame_x_translate,
                  double tof_top_sensor_frame_y_translate,
                  double tof_top_sensor_frame_z_translate,
                  double tof_bot_sensor_frame_x_translate,
                  double tof_bot_sensor_frame_y_translate,
                  double tof_bot_sensor_frame_z_translate,
                  double tof_bot_left_sensor_frame_yaw_ang,
                  double tof_bot_rihgt_sensor_frame_yaw_ang,
                  double tof_bot_fov_ang);
    ~PointCloudTof();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    sensor_msgs::msg::PointCloud2 updateTopTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, double tilting_angle);
    sensor_msgs::msg::PointCloud2 updateBotTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, tTofPitchAngle pitchAngle, bool isShowRow = false, ROW_NUMBER row = ROW_NUMBER::FIRST);

private:
    FrameConverter frame_converter_;
    PointCloudGenerator pointcloud_generator_;

    tPose robot_pose_;
    std::string target_frame_;
    tPoint tof_top_translation_;
    tPoint tof_bot_translation_;
    double tof_bot_left_sensor_frame_pitch_ang_;
    double tof_bot_right_sensor_frame_pitch_ang_;
    double tof_bot_left_sensor_frame_yaw_ang_;
    double tof_bot_right_sensor_frame_yaw_ang_;
    double tof_bot_fov_ang_;

    double tof_bot_row_1_z_tan_;
    double tof_bot_row_2_z_tan_;
    double tof_bot_row_3_z_tan_;
    double tof_bot_row_4_z_tan_;
    double tof_bot_col_1_xy_tan_;
    double tof_bot_col_2_xy_tan_;
    double tof_bot_col_3_xy_tan_;
    double tof_bot_col_4_xy_tan_;

    std::vector<bool> zero_dist_index = std::vector<bool>(false);

    std::vector<tPoint> transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide);
    std::vector<tPoint> filterPoints(const std::vector<tPoint> &input_points);
};

#endif // __POINTCLOUD_TOF_HPP__