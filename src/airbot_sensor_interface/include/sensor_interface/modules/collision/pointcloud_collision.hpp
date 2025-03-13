#ifndef __POINTCLOUD_COLLISION_HPP__
#define __POINTCLOUD_COLLISION_HPP__

#include <cmath>
#include <string>
#include "robot_custom_msgs/msg/abnormal_event_data.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/frame_converter.hpp"

class PointCloudCollision
{
public:
    PointCloudCollision(double collision_forward_point_offset_m);
    ~PointCloudCollision();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    sensor_msgs::msg::PointCloud2 updateCollisionPointCloudMsg(robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg);

private:
    PointCloudGenerator pointcloud_generator_;
    FrameConverter frame_converter_;

    tPose robot_pose_;
    std::string target_frame_;
    double collosion_point_offset_m_;
};

#endif // __POINTCLOUD_COLLISION_HPP__