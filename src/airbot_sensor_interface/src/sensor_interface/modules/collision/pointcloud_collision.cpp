#include "sensor_interface/modules/collision/pointcloud_collision.hpp"

PointCloudCollision::PointCloudCollision(double collision_forward_point_offset_m = 0.25)
{
    collosion_point_offset_m_ = collision_forward_point_offset_m;
}

PointCloudCollision::~PointCloudCollision()
{
}

/**
 * @brief ### 노드 초기화 시점에, 파라미터로 받은 frame_id로 업데이트
 */
void PointCloudCollision::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

/**
 * @brief ### Callback 받은 시점에, topic에 찍힌 robot_pose로 업데이트
 */
void PointCloudCollision::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

sensor_msgs::msg::PointCloud2 PointCloudCollision::updateCollisionPointCloudMsg(robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg)
{
    std::vector<tPoint> points_on_robot_frame = frame_converter_.transformCollisionData2RobotFrame(msg, collosion_point_offset_m_);
    std::vector<tPoint> points_on_map_frame = frame_converter_.transformRobot2GlobalFrame(points_on_robot_frame, robot_pose_);

    if (target_frame_ == "map") {
        return pointcloud_generator_.generatePointCloud2Message(points_on_map_frame, target_frame_);
    } else if (target_frame_ == "base_link") {
        return pointcloud_generator_.generatePointCloud2Message(points_on_robot_frame, target_frame_);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame_.c_str());
        return sensor_msgs::msg::PointCloud2();
    }
}
