#ifndef __POINTCLOUD_HPP__
#define __POINTCLOUD_HPP__

#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "robot_custom_msgs/msg/ai_data.hpp"
#include "robot_custom_msgs/msg/ai_data_array.hpp"
#include "robot_custom_msgs/msg/line_laser_data.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

enum class TOF_SIDE
{
    LEFT,
    RIGHT,
    BOTH
};

enum class ROW_NUMBER
{
    FIRST,
    SECOND,
    THIRD,
    FOURTH
};

struct tPoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    tPoint() = default;
};

struct tOrientation
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tOrientation() = default;
};

struct tPose
{
    tPoint position;
    tOrientation orientation;
    tPose() = default;
};


class PointCloud
{
public:
    PointCloud(float robot_radius,
               float tof_top_sensor_frame_x_translate,
               float tof_top_sensor_frame_y_translate,
               float tof_tof_sensor_frame_z_translate,
               float tof_top_sensor_frame_pitch_ang,
               float tof_bot_sensor_frame_x_translate,
               float tof_bot_sensor_frame_y_translate,
               float tof_bot_sensor_frame_z_translate,
               float tof_bot_left_sensor_frame_yaw_ang,
               float tof_bot_rihgt_sensor_frame_yaw_ang,
               float tof_bot_fov_ang,
               float camera_sensor_frame_x_translate,
               float camera_sensor_frame_y_translate,
               float camera_sensor_frame_z_translate);
    ~PointCloud();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    sensor_msgs::msg::PointCloud2 getConvertedTofTopToPointCloud(const robot_custom_msgs::msg::TofData::SharedPtr msg);
    sensor_msgs::msg::PointCloud2 getConvertedTofBotToPointCloud(const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, bool debug = false, ROW_NUMBER row = ROW_NUMBER::FIRST);
    sensor_msgs::msg::PointCloud2 getConvertedCameraToPointCloud(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg, float pc_resolution, int number_of_object);
    sensor_msgs::msg::PointCloud2 getConvertedLineLaserToPointCloud(const robot_custom_msgs::msg::LineLaserData::SharedPtr msg);
    vision_msgs::msg::BoundingBox2DArray getCameraBoundingBoxMessage();

private:
    tPose robotPose;
    std::string target_frame;

    float robot_radius_;
    float tof_top_sensor_frame_x_translate_;
    float tof_top_sensor_frame_y_translate_;
    float tof_tof_sensor_frame_z_translate_;
    float tof_top_sensor_frame_pitch_ang_;
    float tof_bot_sensor_frame_x_translate_;
    float tof_bot_sensor_frame_y_translate_;
    float tof_bot_sensor_frame_z_translate_;
    float tof_bot_left_sensor_frame_yaw_ang_;
    float tof_bot_right_sensor_frame_yaw_ang_;
    float tof_bot_fov_ang_;
    float camera_sensor_frame_x_translate_;
    float camera_sensor_frame_y_translate_;
    float camera_sensor_frame_z_translate_;

    double tof_top_sensor_frame_pitch_cosine_;
    double tof_top_sensor_frame_pitch_sine_;
    double tof_left_sensor_frame_yaw_cosine_;
    double tof_left_sensor_frame_yaw_sine_;
    double tof_right_sensor_frame_yaw_cosine_;
    double tof_right_sensor_frame_yaw_sine_;
    double tof_bot_row_1_z_tan_;
    double tof_bot_row_2_z_tan_;
    double tof_bot_row_3_z_tan_;
    double tof_bot_row_4_z_tan_;
    double tof_bot_col_1_xy_tan_;
    double tof_bot_col_2_xy_tan_;
    double tof_bot_col_3_xy_tan_;
    double tof_bot_col_4_xy_tan_;

    std::vector<bool> zero_dist_index = std::vector<bool>(false);
    vision_msgs::msg::BoundingBox2DArray camera_bbox_array;

    std::vector<tPoint> transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide);
    std::vector<tPoint> transformCameraMsg2PointsOnSensorFrame(std::vector<robot_custom_msgs::msg::AIData> input_camera_objs);
    std::vector<tPoint> transformTofSensor2RobotFrame(const std::vector<tPoint> &input_points, bool left);
    std::vector<tPoint> transformCameraSensor2RobotFrame(const std::vector<tPoint> &input_points);
    std::vector<tPoint> transformRobot2GlobalFrame(const std::vector<tPoint> &input_points);
    std::vector<tPoint> transformRobot2GlobalFrame(const tPoint &input_point);
    std::vector<tPoint> filterPoints(const std::vector<tPoint> &input_points);
    sensor_msgs::msg::PointCloud2 createTofTopPointCloud2Message(const tPoint &point);
    sensor_msgs::msg::PointCloud2 createTofBotPointCloud2Message(const std::vector<tPoint> &points);
    sensor_msgs::msg::PointCloud2 createCameraPointCloud2Message(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution);
    vision_msgs::msg::BoundingBox2DArray createBoundingBoxMessage(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg);
    void setCameraBoundingBoxMessage(vision_msgs::msg::BoundingBox2DArray bbox_array);
};

#endif  // POINTCLOUD_HPP