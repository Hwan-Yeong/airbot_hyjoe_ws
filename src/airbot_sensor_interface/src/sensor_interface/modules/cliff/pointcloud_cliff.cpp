#include "sensor_interface/modules/cliff/pointcloud_cliff.hpp"

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

void PointCloudCliff::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

void PointCloudCliff::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

sensor_msgs::msg::PointCloud2 PointCloudCliff::updateCliffPointCloudMsg(robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    std::vector<tPoint> points_on_robot_frame = frame_converter_.transformCliffSensor2RobotFrame(msg, ir_sensor_points_);
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
