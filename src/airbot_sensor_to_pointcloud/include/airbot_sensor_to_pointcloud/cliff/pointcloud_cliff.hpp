#ifndef __POINTCLOUD_CLIFF_HPP__
#define __POINTCLOUD_CLIFF_HPP__

#include <cmath>
#include <string>
#include "std_msgs/msg/u_int8.hpp"
#include "utils/pointcloud_generator.hpp"

class PointCloudCliff
{
public:
    PointCloudCliff(double distance_center_to_front_ir_sensor,
                    double angle_to_next_ir_sensor);
    ~PointCloudCliff();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    sensor_msgs::msg::PointCloud2 updateCliffPointCloudMsg(std_msgs::msg::UInt8::SharedPtr msg);


private:
    std::shared_ptr<PointCloudGenerator> pointcloud_generator_;

    tPose robot_pose_;
    std::string target_frame_;
    tPoint ir_1_position_, ir_2_position_, ir_3_position_, ir_4_position_, ir_5_position_, ir_6_position_;
    std::vector<tPoint> ir_sensor_points_;

    std::vector<tPoint> transformCliffMsg2PointsOnRobotFrame(std_msgs::msg::UInt8::SharedPtr msg);
};

#endif //POINTCLOUD_CLIFF_HPP