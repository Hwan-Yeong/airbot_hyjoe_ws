#include "airbot_sensor_to_pointcloud/modules/tof/pointcloud_tof.hpp"

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

void PointCloudTof::updateTofMode(bool use_8x8)
{
    use_tof_8x8_ = use_8x8;
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
    point_on_robot_frame.z = tof_top_translation_.z - msg->top * tof_top_sensor_frame_pitch_sine;
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

std::vector<sensor_msgs::msg::PointCloud2> PointCloudTof::updateBotTofPointCloudMsg(const robot_custom_msgs::msg::TofData::SharedPtr &msg, TOF_SIDE side, const tTofPitchAngle &pitchAngle)
{
    tof_bot_left_sensor_frame_pitch_ang_ = pitchAngle.bot_left;
    tof_bot_right_sensor_frame_pitch_ang_ = pitchAngle.bot_right;

    std::vector<sensor_msgs::msg::PointCloud2> result_msgs;

    auto process_side = [&](TOF_SIDE current_side, const std::vector<double>& tof_dists, const std::vector<double>& y_tan, const std::vector<double>& z_tan, double yaw_angle, double pitch_angle) {
        std::vector<bool> zero_dist_index(tof_dists.size(), false);
        std::vector<tPoint> sensor_pts;

        constexpr int INDEX_SIZE = 16;
        for (int i = 0; i < INDEX_SIZE; ++i) {
            double dist = tof_dists[i];
            bool is_zero = dist <= 0.001;
            zero_dist_index[i] = is_zero;

            tPoint p;
            p.x = dist;
            p.y = dist * y_tan[i];
            p.z = dist * z_tan[i];
            sensor_pts.push_back(p);
        }

        std::vector<tPoint> robot_pts = frame_converter_.transformTofSensor2RobotFrame(
            sensor_pts,
            current_side == TOF_SIDE::LEFT,
            yaw_angle,
            pitch_angle,
            tof_bot_translation_);

        std::vector<tPoint> global_pts = (target_frame_ == "map")
            ? frame_converter_.transformRobot2GlobalFrame(robot_pts, robot_pose_)
            : robot_pts;

        // const size_t step = use_tof_8x8_ ? 1 : 4;
        const size_t step = 1;
        const size_t count = global_pts.size();

        for (size_t i = 0; i < count; i += step) {
            auto sliced_points = std::vector<tPoint>(global_pts.begin() + i, global_pts.begin() + std::min(i + step, count));
            std::vector<bool> sliced_zero_mask(zero_dist_index.begin() + i, zero_dist_index.begin() + std::min(i + step, count));

            std::vector<tPoint> filtered;
            for (size_t j = 0; j < sliced_zero_mask.size(); ++j) {
                if (!sliced_zero_mask[j]) {
                    filtered.push_back(sliced_points[j]);
                    result_msgs.push_back(pointcloud_generator_.generatePointCloud2Message(filtered, target_frame_));
                } else {
                    result_msgs.push_back(pointcloud_generator_.generatePointCloud2EmptyMessage(target_frame_));
                }
            }
        }
    };

    if (side == TOF_SIDE::LEFT || side == TOF_SIDE::BOTH) {
        std::vector<double> left_dists(msg->bot_left.begin(), msg->bot_left.end());
        process_side(TOF_SIDE::LEFT, left_dists, left_y_tan_, left_z_tan_, tof_bot_left_sensor_frame_yaw_ang_, tof_bot_left_sensor_frame_pitch_ang_);
    }

    if (side == TOF_SIDE::RIGHT || side == TOF_SIDE::BOTH) {
        std::vector<double> right_dists(msg->bot_right.begin(), msg->bot_right.end());
        process_side(TOF_SIDE::RIGHT, right_dists, right_y_tan_, right_z_tan_, tof_bot_right_sensor_frame_yaw_ang_, tof_bot_right_sensor_frame_pitch_ang_);
    }

    return result_msgs;
}

void PointCloudTof::updateSubCellIndexArray(const std::vector<int> &sub_cell_idx_array, std::vector<double> &y_tan_out, std::vector<double> &z_tan_out)
{
    if (use_tof_8x8_) {
        // =========== 사용자 입력 기반으로 사용할 8x8 마스킹 Mat 만들기 ===========
        bool masked_mat[8][8] = {false};

        //// Input sub cell 인덱스 로깅
        RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "==== Input sub_cell_idx_array ====");
        for (int r = 0; r < 4; ++r) {
            std::stringstream ss;
            for (int c = 0; c < 4; ++c) {
                ss << sub_cell_idx_array[r * 4 + c] << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "%s", ss.str().c_str());
        }
        ////

        for (int idx : sub_cell_idx_array) {
            if (idx >= 0 && idx <64) {
                int row = idx / 8;
                int col = idx % 8;
                masked_mat[row][col] = true;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("PointCloudTof"),
                    "Each value in sub_cell_idx_array must be between 0 and 63. Invalid input idx: %d",
                    idx
                );
            }
        }

        //// Masked 행렬 로깅
        RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "==== Masked 8x8 Matrix ====");
        for (int i = 0; i < 8; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 8; ++j) {
                ss << (masked_mat[i][j] ? "1 " : "0 ");
            }
            RCLCPP_INFO(rclcpp::get_logger("PointCloudTof"), "%s", ss.str().c_str());
        }
        ////

        // =========== true인 거 기반으로 y_tan, z_tan 만들기 ===========
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
    } else {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double y = std::tan(tof_bot_fov_ang_*((3 - 2*j)/8.0)*M_PI/180);
                double z = std::tan(tof_bot_fov_ang_*((3 - 2*i)/8.0)*M_PI/180);
                y_tan_out.emplace_back(y);
                z_tan_out.emplace_back(z);
            }
        }
    }
}

std::vector<tPoint> PointCloudTof::transformTofMsg2PointsOnSensorFrame(std::vector<double> input_tof_dist, bool isBothSide, TOF_SIDE side)
{
    std::vector<tPoint> points;

    constexpr int INDEX_SIZE = 16;

    for (int index = 0; index < INDEX_SIZE; ++index) {
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
            p.y = p.x * left_y_tan_[index];
            p.z = p.x * left_z_tan_[index];
        } else {
            p.y = p.x * right_y_tan_[index];
            p.z = p.x * right_z_tan_[index];
        }
        points.push_back(p);
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
