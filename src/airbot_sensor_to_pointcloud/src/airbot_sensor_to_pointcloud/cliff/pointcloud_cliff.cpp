#include "airbot_sensor_to_pointcloud/cliff/pointcloud_cliff.hpp"

/*
#######################
#### IR Sensor Num ####
#######################
###                 ###
###  < Top View >   ###
###                 ###
###        1        ###
###    2       6    ###
###                 ###
###    3       5    ###
###        4        ###
###                 ###
#######################
*/

PointCloudCliff::PointCloudCliff(double distance_center_to_front_ir_sensor = 0.15,
                                 double angle_to_next_ir_sensor = 50)
{
    double d = distance_center_to_front_ir_sensor;
    double deg_1 = 0;
    double deg_2 = angle_to_next_ir_sensor;
    double deg_3 = 180 - angle_to_next_ir_sensor;
    double deg_4 = 180;
    double deg_5 = 180 + angle_to_next_ir_sensor;
    double deg_6 = 360 - angle_to_next_ir_sensor;

    ir_1_position_ = tPoint(d*std::cos(deg_1 * M_PI/180), d*std::sin(deg_1 * M_PI/180), 0.0);
    ir_2_position_ = tPoint(d*std::cos(deg_2 * M_PI/180), d*std::sin(deg_2 * M_PI/180), 0.0);
    ir_3_position_ = tPoint(d*std::cos(deg_3 * M_PI/180), d*std::sin(deg_3 * M_PI/180), 0.0);
    ir_4_position_ = tPoint(d*std::cos(deg_4 * M_PI/180), d*std::sin(deg_4 * M_PI/180), 0.0);
    ir_5_position_ = tPoint(d*std::cos(deg_5 * M_PI/180), d*std::sin(deg_5 * M_PI/180), 0.0);
    ir_6_position_ = tPoint(d*std::cos(deg_6 * M_PI/180), d*std::sin(deg_6 * M_PI/180), 0.0);

    ir_sensor_points_ = {ir_1_position_, ir_2_position_, ir_3_position_,
                        ir_4_position_, ir_5_position_, ir_6_position_};
}

PointCloudCliff::~PointCloudCliff()
{
}

/**
 * @brief ### 노드 초기화 시점에, 파라미터로 받은 frame_id로 업데이트
 */
void PointCloudCliff::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

/**
 * @brief ### Callback 받은 시점에, topic에 찍힌 robot_pose로 업데이트
 */
void PointCloudCliff::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

sensor_msgs::msg::PointCloud2 PointCloudCliff::updateCliffPointCloudMsg(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::vector<tPoint> cliff_points_on_robot_frame = transformCliffMsg2PointsOnRobotFrame(msg);

    return pointcloud_generator_->generatePointCloud2Message(cliff_points_on_robot_frame, target_frame_);
}

std::vector<tPoint> PointCloudCliff::transformCliffMsg2PointsOnRobotFrame(std_msgs::msg::UInt8::SharedPtr msg)
{
    std::vector<tPoint> active_sensor_points;
    for (size_t i=0; i<ir_sensor_points_.size(); ++i) {
        if(msg->data & (1 << i)) {
            active_sensor_points.push_back(ir_sensor_points_[i]);
        }
    }

    return active_sensor_points;
}