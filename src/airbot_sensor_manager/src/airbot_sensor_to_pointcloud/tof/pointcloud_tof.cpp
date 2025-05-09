#include "airbot_sensor_to_pointcloud/tof/pointcloud_tof.hpp"

#define ENABLE_TOF_8x8 false

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
}

PointCloudTof::~PointCloudTof()
{
}

/**
 * @brief ### 노드 초기화 시점에, 파라미터로 받은 frame_id로 업데이트
 */
void PointCloudTof::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

/**
 * @brief ### Callback 받은 시점에, topic에 찍힌 robot_pose로 업데이트
 */
void PointCloudTof::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

void PointCloudTof::updateLeftSubCellIndexArray(std::vector<int> &left_sub_cell_idx_array)
{
    updateSubCellIndexArray(left_sub_cell_idx_array, left_y_tan_, left_z_tan_);
}

void PointCloudTof::updateRightSubCellIndexArray(std::vector<int> &right_sub_cell_idx_array)
{
    updateSubCellIndexArray(right_sub_cell_idx_array, right_y_tan_, right_z_tan_);
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
        std::vector<tPoint> points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(points_on_robot_frame, robot_pose_);
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
        multi_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_left, false, TOF_SIDE::LEFT);
        multi_tof_points_on_robot_frame = frame_converter_.transformTofSensor2RobotFrame(multi_tof_points_on_sensor_frame,
                                                                                         true,
                                                                                         tof_bot_left_sensor_frame_yaw_ang_,
                                                                                         tof_bot_left_sensor_frame_pitch_ang_,
                                                                                         tof_bot_translation_);
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame
                = frame_converter_.transformRobot2GlobalFrame(multi_tof_points_on_robot_frame, robot_pose_);
        }
        break;
    case TOF_SIDE::RIGHT:
        zero_dist_index = std::vector<bool>(16,false);
        multi_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_right, false, TOF_SIDE::RIGHT);
        multi_tof_points_on_robot_frame = frame_converter_.transformTofSensor2RobotFrame(multi_tof_points_on_sensor_frame,
                                                                                         false,
                                                                                         tof_bot_right_sensor_frame_yaw_ang_,
                                                                                         tof_bot_right_sensor_frame_pitch_ang_,
                                                                                         tof_bot_translation_);
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(multi_tof_points_on_robot_frame, robot_pose_);
        }
        break;
    case TOF_SIDE::BOTH:
        zero_dist_index = std::vector<bool>(32,false);
        multi_left_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_left, false, TOF_SIDE::LEFT);
        multi_right_tof_points_on_sensor_frame = transformTofMsg2PointsOnSensorFrame(multi_tof_bot_right, true, TOF_SIDE::RIGHT);
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
        multi_tof_points_on_robot_frame.insert(multi_tof_points_on_robot_frame.end(), multi_left_tof_points_on_robot_frame.begin(), multi_left_tof_points_on_robot_frame.end());
        multi_tof_points_on_robot_frame.insert(multi_tof_points_on_robot_frame.end(), multi_right_tof_points_on_robot_frame.begin(), multi_right_tof_points_on_robot_frame.end());
        if (target_frame_ == "map") {
            multi_tof_points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(multi_tof_points_on_robot_frame, robot_pose_);
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
            zero_dist_index.erase(zero_dist_index.begin(), zero_dist_index.begin() + start_index);
            if ((end_index - start_index) < static_cast<int>(multi_tof_points_on_map_frame.size())) {
                multi_tof_points_on_map_frame.erase(multi_tof_points_on_map_frame.begin() + (end_index - start_index), multi_tof_points_on_map_frame.end());
                zero_dist_index.erase(zero_dist_index.begin() + (end_index - start_index), zero_dist_index.end());
            }
        } else if (target_frame_ == "base_link") {
            multi_tof_points_on_robot_frame.erase(multi_tof_points_on_robot_frame.begin(), multi_tof_points_on_robot_frame.begin() + start_index);
            zero_dist_index.erase(zero_dist_index.begin(), zero_dist_index.begin() + start_index);
            if ((end_index - start_index) < static_cast<int>(multi_tof_points_on_robot_frame.size())) {
                multi_tof_points_on_robot_frame.erase(multi_tof_points_on_robot_frame.begin() + (end_index - start_index), multi_tof_points_on_robot_frame.end());
                zero_dist_index.erase(zero_dist_index.begin() + (end_index - start_index), zero_dist_index.end());
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

void PointCloudTof::updateSubCellIndexArray(const std::vector<int> &sub_cell_idx_array, std::vector<double> &y_tan_out, std::vector<double> &z_tan_out)
{
#if ENABLE_TOF_8x8
    // =========== 사용자 입력 기반으로 사용할 8x8 마스킹 Mat 만들기 ===========
    bool masked_mat[8][8] = {false};

    //// Input 디버깅 출력
    RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "==== Input sub_cell_idx_array ====");
    for (int r = 0; r < 4; ++r) {
        std::stringstream ss;
        for (int c = 0; c < 4; ++c) {
            ss << sub_cell_idx_array[r * 4 + c] << " ";
        }
        RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "%s", ss.str().c_str());
    }
    ////

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            int val = sub_cell_idx_array[r * 4 + c];
            int base_row = r * 2;
            int base_col = c * 2;

            switch (val)
            {
                case 0:
                    masked_mat[base_row][base_col] = true;
                    break;
                case 1:
                    masked_mat[base_row][base_col +1] = true;
                    break;
                case 2:
                    masked_mat[base_row + 1][base_col] = true;
                    break;
                case 3:
                    masked_mat[base_row + 1][base_col + 1] = true;
                    break;
                default:
                    break;
            }
        }
    }

    //// Masked 행렬 디버깅 출력
    RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "==== Masked 8x8 Matrix ====");
    for (int i = 0; i < 8; ++i) {
        std::stringstream ss;
        for (int j = 0; j < 8; ++j) {
            ss << (masked_mat[i][j] ? "1 " : "0 ");
        }
        RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "%s", ss.str().c_str());
    }
    ////

    // =========== true인 거 기반으로 y_tan[4][4], z_tan[4][4] 만들기 ===========
    std::vector<std::pair<int, int>> true_indices;
    y_tan_out.clear();
    z_tan_out.clear();

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            if (masked_mat[i][j]) {
                true_indices.emplace_back(i, j);
            }
        }
    }

    if (true_indices.size() != 16) {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"),
            "Expected 16 true values in mask, but got %zu",
            true_indices.size()
        );
    } else {
        for (const auto& [i, j] : true_indices) {
            double y = std::tan(tof_bot_fov_ang_*((7 - 2*j)/16.0)*M_PI/180);
            double z = std::tan(tof_bot_fov_ang_*((7 - 2*i)/16.0)*M_PI/180);
            y_tan_out.emplace_back(y);
            z_tan_out.emplace_back(z);
        }
    }
#else
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double y = std::tan(tof_bot_fov_ang_*((3 - 2*j)/8.0)*M_PI/180);
            double z = std::tan(tof_bot_fov_ang_*((3 - 2*i)/8.0)*M_PI/180);
            y_tan_out.emplace_back(y);
            z_tan_out.emplace_back(z);
        }
    }
#endif
}

std::vector<tPoint> PointCloudTof::transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide, TOF_SIDE side)
{
    std::vector<tPoint> points;

    constexpr int ROWS = 4;
    constexpr int COLS = 4;

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
            if (side == TOF_SIDE::LEFT) {
                p.y = input_tof_dist[index] * left_y_tan_[(row * ROWS) + col];
                p.z = input_tof_dist[index] * left_z_tan_[(row * ROWS) + col];
            } else {
                p.y = input_tof_dist[index] * right_y_tan_[(row * ROWS) + col];
                p.z = input_tof_dist[index] * right_z_tan_[(row * ROWS) + col];
            }
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
