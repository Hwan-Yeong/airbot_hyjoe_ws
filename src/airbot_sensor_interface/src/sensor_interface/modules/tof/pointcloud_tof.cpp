#include "sensor_interface/modules/tof/pointcloud_tof.hpp"

PointCloudTof::PointCloudTof(double tof_top_sensor_frame_x_translate = 0.0942,
                             double tof_top_sensor_frame_y_translate = 0.0,
                             double tof_top_sensor_frame_z_translate = 0.56513,
                             double tof_bot_sensor_frame_x_translate = 0.145,
                             double tof_bot_sensor_frame_y_translate = 0.076,
                             double tof_bot_sensor_frame_z_translate = 0.03,
                             double tof_bot_left_sensor_frame_yaw_ang = 15.0,
                             double tof_bot_right_sensor_frame_yaw_ang = -15.0,
                             double tof_bot_fov_ang = 45)
    : tof_top_translation_(tof_top_sensor_frame_x_translate,
                           tof_top_sensor_frame_y_translate,
                           tof_top_sensor_frame_z_translate),
      tof_bot_translation_(tof_bot_sensor_frame_x_translate,
                           tof_bot_sensor_frame_y_translate,
                           tof_bot_sensor_frame_z_translate),
      tof_bot_left_sensor_frame_yaw_ang_(tof_bot_left_sensor_frame_yaw_ang),
      tof_bot_right_sensor_frame_yaw_ang_(tof_bot_right_sensor_frame_yaw_ang),
      tof_bot_fov_ang_(tof_bot_fov_ang)
{
    tof_bot_row_1_z_tan_ = std::tan(tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);
    tof_bot_row_2_z_tan_ = std::tan(tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_row_3_z_tan_ = std::tan(-tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_row_4_z_tan_ = std::tan(-tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);

    tof_bot_col_1_xy_tan_ = std::tan(tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);
    tof_bot_col_2_xy_tan_ = std::tan(tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_col_3_xy_tan_ = std::tan(-tof_bot_fov_ang_*(1.0/8.0)*M_PI/180);
    tof_bot_col_4_xy_tan_ = std::tan(-tof_bot_fov_ang_*(3.0/8.0)*M_PI/180);
}

PointCloudTof::~PointCloudTof()
{
}

void PointCloudTof::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

void PointCloudTof::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

sensor_msgs::msg::PointCloud2 PointCloudTof::updateTopTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, double tilting_angle)
{
    tPoint point_on_robot_frame, point_on_map_frame;
    double tof_top_sensor_frame_pitch_cosine = std::cos(tilting_angle*M_PI/180);
    double tof_top_sensor_frame_pitch_sine = std::sin(tilting_angle*M_PI/180);
    point_on_robot_frame.x = tof_top_translation_.x + msg->top * tof_top_sensor_frame_pitch_cosine;
    point_on_robot_frame.y = tof_top_translation_.y;
    point_on_robot_frame.z = tof_top_translation_.z + msg->top * tof_top_sensor_frame_pitch_sine;
    std::vector<tPoint> points_on_robot_frame = {point_on_robot_frame};

    if (target_frame_ == "map") {
        std::vector<tPoint> points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(points_on_robot_frame,
                                                                                               robot_pose_);
        return pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, target_frame_);
    } else if (target_frame_ == "base_link") {
        return pointcloud_generator_.generatePointCloud2Message(points_on_robot_frame, target_frame_);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame_.c_str());
        return sensor_msgs::msg::PointCloud2();
    }
}

sensor_msgs::msg::PointCloud2 PointCloudTof::updateBotTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side, tTofPitchAngle pitchAngle, bool isShowRow, ROW_NUMBER row)
{
    tof_bot_left_sensor_frame_pitch_ang_ = pitchAngle.bot_left;
    tof_bot_right_sensor_frame_pitch_ang_ = pitchAngle.bot_right;
    std::vector<tPoint> multi_tof_points_on_sensor_frame;
    std::vector<tPoint> multi_left_tof_points_on_sensor_frame;
    std::vector<tPoint> multi_right_tof_points_on_sensor_frame;
    std::vector<tPoint> multi_tof_points_on_robot_frame;
    std::vector<tPoint> multi_left_tof_points_on_robot_frame;
    std::vector<tPoint> multi_right_tof_points_on_robot_frame;
    std::vector<tPoint> multi_tof_points_on_map_frame;
    std::vector<double> multi_tof_bot_left(msg->bot_left.begin(), msg->bot_left.end());
    std::vector<double> multi_tof_bot_right(msg->bot_right.begin(), msg->bot_right.end());
    switch(side)
    {
    case TOF_SIDE::LEFT:
        zero_dist_index = std::vector<bool>(16,false);
        multi_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_left, false);
        multi_tof_points_on_robot_frame = frame_converter_.transformTofSensor2RobotFrame(multi_tof_points_on_sensor_frame,
                                                                                          true,
                                                                                          tof_bot_left_sensor_frame_yaw_ang_,
                                                                                          tof_bot_left_sensor_frame_pitch_ang_,
                                                                                          tof_bot_translation_);
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame
                = frame_converter_.transformRobot2GlobalFrame(multi_tof_points_on_robot_frame,
                                                               robot_pose_);
        }
        break;
    case TOF_SIDE::RIGHT:
        zero_dist_index = std::vector<bool>(16,false);
        multi_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_right, false);
        multi_tof_points_on_robot_frame = frame_converter_.transformTofSensor2RobotFrame(multi_tof_points_on_sensor_frame,
                                                                                          false,
                                                                                          tof_bot_right_sensor_frame_yaw_ang_,
                                                                                          tof_bot_right_sensor_frame_pitch_ang_,
                                                                                          tof_bot_translation_);
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(multi_tof_points_on_robot_frame,
                                                                                         robot_pose_);
        }
        break;
    case TOF_SIDE::BOTH:
        zero_dist_index = std::vector<bool>(32,false);
        multi_left_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_left, false);
        multi_right_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_right, true);
        multi_left_tof_points_on_robot_frame = frame_converter_.transformTofSensor2RobotFrame(multi_left_tof_points_on_sensor_frame,
                                                                                               true,
                                                                                               tof_bot_left_sensor_frame_yaw_ang_,
                                                                                               tof_bot_left_sensor_frame_pitch_ang_,
                                                                                               tof_bot_translation_);
        multi_right_tof_points_on_robot_frame = frame_converter_.transformTofSensor2RobotFrame(multi_right_tof_points_on_sensor_frame,
                                                                                                false,
                                                                                                tof_bot_right_sensor_frame_yaw_ang_,
                                                                                                tof_bot_right_sensor_frame_pitch_ang_,
                                                                                                tof_bot_translation_);
        multi_tof_points_on_robot_frame.insert(multi_tof_points_on_robot_frame.end(),
                                               multi_left_tof_points_on_robot_frame.begin(),
                                               multi_left_tof_points_on_robot_frame.end());
        multi_tof_points_on_robot_frame.insert(multi_tof_points_on_robot_frame.end(),
                                               multi_right_tof_points_on_robot_frame.begin(),
                                               multi_right_tof_points_on_robot_frame.end());
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(multi_tof_points_on_robot_frame,
                                                                                         robot_pose_);
        }
        break;
    default:
        break;
    }

    if (isShowRow) {
        int start_index = static_cast<int>(row) * 4;
        int end_index = start_index + 4;
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame.erase(multi_tof_points_on_map_frame.begin(),
                                                multi_tof_points_on_map_frame.begin() + start_index);
            zero_dist_index.erase(zero_dist_index.begin(),
                                  zero_dist_index.begin() + start_index);
            if ((end_index - start_index) < static_cast<int>(multi_tof_points_on_map_frame.size())) {
                multi_tof_points_on_map_frame.erase(multi_tof_points_on_map_frame.begin() + (end_index - start_index),
                                                    multi_tof_points_on_map_frame.end());
                zero_dist_index.erase(zero_dist_index.begin() + (end_index - start_index),
                                      zero_dist_index.end());
            }
        } else if (target_frame_ == "base_link") {
            multi_tof_points_on_robot_frame.erase(multi_tof_points_on_robot_frame.begin(),
                                                  multi_tof_points_on_robot_frame.begin() + start_index);
            zero_dist_index.erase(zero_dist_index.begin(),
                                  zero_dist_index.begin() + start_index);
            if ((end_index - start_index) < static_cast<int>(multi_tof_points_on_robot_frame.size())) {
                multi_tof_points_on_robot_frame.erase(multi_tof_points_on_robot_frame.begin() + (end_index - start_index),
                                                      multi_tof_points_on_robot_frame.end());
                zero_dist_index.erase(zero_dist_index.begin() + (end_index - start_index),
                                      zero_dist_index.end());
            }
        }
    }

    if (target_frame_ == "map") {
        std::vector<tPoint> filtered_tof_points = filterPoints(multi_tof_points_on_map_frame);
        return pointcloud_generator_.generatePointCloud2Message(filtered_tof_points, target_frame_);
    } else if (target_frame_ == "base_link") {
        std::vector<tPoint> filtered_tof_points = filterPoints(multi_tof_points_on_robot_frame);
        return pointcloud_generator_.generatePointCloud2Message(filtered_tof_points, target_frame_);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame_.c_str());
        return sensor_msgs::msg::PointCloud2();
    }
}

std::vector<tPoint> PointCloudTof::transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide)
{
    std::vector<tPoint> points;

    constexpr int ROWS = 4;
    constexpr int COLS = 4;

    const double xy_sine[COLS] = {tof_bot_col_4_xy_tan_, tof_bot_col_3_xy_tan_,
                                  tof_bot_col_2_xy_tan_, tof_bot_col_1_xy_tan_};
    const double z_sine[ROWS] = {tof_bot_row_1_z_tan_, tof_bot_row_2_z_tan_,
                                 tof_bot_row_3_z_tan_, tof_bot_row_4_z_tan_};

    for (int row = 0; row < ROWS; ++row) {
        for (int col = 0; col < COLS; ++col) {
            int index = row * COLS + col;

            if (!isBothSide) {
                if (input_tof_dist[index] > 0.001) { // 0값 필터링을 위한 인덱스 저장
                    zero_dist_index[index] = false;
                } else {
                    zero_dist_index[index] = true;
                }
            } else {
                if (input_tof_dist[index] > 0.001) { // 0값 필터링을 위한 인덱스 저장
                    zero_dist_index[index+16] = false;
                } else {
                    zero_dist_index[index+16] = true;
                }
            }

            tPoint p;
            p.x = input_tof_dist[index];
            p.y = input_tof_dist[index] * xy_sine[col];
            p.z = input_tof_dist[index] * z_sine[row];
            points.push_back(p);
        }
    }

    return points;
}

std::vector<tPoint> PointCloudTof::filterPoints(const std::vector<tPoint> &input_points)
{
    std::vector<tPoint> filtered_points;

    for (size_t i=0; i<zero_dist_index.size(); ++i) {
        if (!zero_dist_index[i]) {
            filtered_points.push_back(input_points[i]);
        }
    }
    return filtered_points;
}
