#ifndef __POINTCLOUD_CLIFF_HPP__
#define __POINTCLOUD_CLIFF_HPP__

#include <cmath>
#include <string>
#include "robot_custom_msgs/msg/bottom_ir_data.hpp"
#include "utils/pointcloud_generator.hpp"
#include "utils/frame_converter.hpp"

/**
 * @brief: 낙하 IR 데이터를 pointcloud로 변환시키는 클래스입니다.
 */
class PointCloudCliff
{
public:
    PointCloudCliff(double distance_center_to_front_ir_sensor,
                    double angle_to_next_ir_sensor);
    ~PointCloudCliff();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);
    sensor_msgs::msg::PointCloud2 updateCliffPointCloudMsg(robot_custom_msgs::msg::BottomIrData::SharedPtr msg);

private:
    PointCloudGenerator pointcloud_generator_;
    FrameConverter frame_converter_;

    tPose robot_pose_;
    std::string target_frame_;
    tPoint ir_1_position_, ir_2_position_, ir_3_position_, ir_4_position_, ir_5_position_, ir_6_position_;
    std::vector<tPoint> ir_sensor_points_;
};

#endif // __POINTCLOUD_CLIFF_HPP__