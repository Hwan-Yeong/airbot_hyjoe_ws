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

    void updateTofMode(bool use_8x8);
    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    void updateLeftSubCellIndexArray(std::vector<int> &left_sub_cell_idx_array);
    void updateRightSubCellIndexArray(std::vector<int> &right_sub_cell_idx_array);
    sensor_msgs::msg::PointCloud2 updateTopTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, double tilting_angle);
    sensor_msgs::msg::PointCloud2 updateBotTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, tTofPitchAngle pitchAngle, bool isShowRow = false, ROW_NUMBER row = ROW_NUMBER::FIRST, int input_index = -1);

private:
    FrameConverter frame_converter_;
    PointCloudGenerator pointcloud_generator_;

    bool use_tof_8x8_;
    tPose robot_pose_;
    std::string target_frame_;
    tPoint tof_top_translation_;
    tPoint tof_bot_translation_;
    double tof_bot_left_sensor_frame_pitch_ang_;
    double tof_bot_right_sensor_frame_pitch_ang_;
    double tof_bot_left_sensor_frame_yaw_ang_;
    double tof_bot_right_sensor_frame_yaw_ang_;
    double tof_bot_fov_ang_;

    std::vector<double> left_y_tan_;
    std::vector<double> left_z_tan_;
    std::vector<double> right_y_tan_;
    std::vector<double> right_z_tan_;

    std::vector<bool> zero_dist_index = std::vector<bool>(false);

    void updateSubCellIndexArray(const std::vector<int>& sub_cell_idx_array, std::vector<double>& y_tan_out, std::vector<double>& z_tan_out);
    std::vector<tPoint> transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide, TOF_SIDE side);
    std::vector<tPoint> filterPoints(const std::vector<tPoint> &input_points);
};

#endif //POINTCLOUD_TOF_HPP