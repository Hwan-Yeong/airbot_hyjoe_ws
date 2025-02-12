#ifndef __POINTCLOUD_HPP__
#define __POINTCLOUD_HPP__

#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/ai_data.hpp"
#include "robot_custom_msgs/msg/ai_data_array.hpp"
#include "robot_custom_msgs/msg/line_laser_data.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "utils/frame_converter.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/boundingbox_generator.hpp"


class PointCloud
{
public:
    PointCloud(double robot_radius,
               double tof_top_sensor_frame_x_translate,
               double tof_top_sensor_frame_y_translate,
               double tof_tof_sensor_frame_z_translate,
               double tof_top_sensor_frame_pitch_ang,
               double tof_bot_sensor_frame_x_translate,
               double tof_bot_sensor_frame_y_translate,
               double tof_bot_sensor_frame_z_translate,
               double tof_bot_left_sensor_frame_yaw_ang,
               double tof_bot_rihgt_sensor_frame_yaw_ang,
               double tof_bot_fov_ang,
               double camera_sensor_frame_x_translate,
               double camera_sensor_frame_y_translate,
               double camera_sensor_frame_z_translate);
    ~PointCloud();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    sensor_msgs::msg::PointCloud2 updateTopTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    sensor_msgs::msg::PointCloud2 updateBotTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, bool isShowRow = false, ROW_NUMBER row = ROW_NUMBER::FIRST);
    vision_msgs::msg::BoundingBox2DArray updateCameraBoundingBoxMsg(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg, std::vector<long int> class_id_list, int th_confidence, bool direction);
    sensor_msgs::msg::PointCloud2 updateCameraPointCloudMsg(vision_msgs::msg::BoundingBox2DArray msg, float pc_resolution);
    sensor_msgs::msg::PointCloud2 updateLineLaserPointCloudMsg(const robot_custom_msgs::msg::LineLaserData::SharedPtr msg);

private:
    std::shared_ptr<FrameConverter> frame_converter_;
    std::shared_ptr<PointCloudGenerator> pointcloud_generator_;
    std::shared_ptr<BoundingBoxGenerator> boundingbox_generator_;

    double robot_radius_;
    tPoint tof_top_translation_;
    double tof_top_sensor_frame_pitch_ang_;
    tPoint tof_bot_translation_;
    double tof_bot_left_sensor_frame_yaw_ang_;
    double tof_bot_right_sensor_frame_yaw_ang_;
    double tof_bot_fov_ang_;
    tPoint camera_translation_;

    double tof_top_sensor_frame_pitch_cosine_;
    double tof_top_sensor_frame_pitch_sine_;
    double tof_bot_row_1_z_tan_;
    double tof_bot_row_2_z_tan_;
    double tof_bot_row_3_z_tan_;
    double tof_bot_row_4_z_tan_;
    double tof_bot_col_1_xy_tan_;
    double tof_bot_col_2_xy_tan_;
    double tof_bot_col_3_xy_tan_;
    double tof_bot_col_4_xy_tan_;

    tPose robot_pose_;
    std::string target_frame;
    vision_msgs::msg::BoundingBox2DArray camera_bbox_array;
    std::vector<bool> zero_dist_index = std::vector<bool>(false);

    std::vector<tPoint> transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide);
    std::vector<tPoint> filterPoints(const std::vector<tPoint> &input_points);
};

#endif  // POINTCLOUD_HPP