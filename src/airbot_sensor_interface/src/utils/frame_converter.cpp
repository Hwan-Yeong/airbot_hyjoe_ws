#include "utils/frame_converter.hpp"


FrameConverter::FrameConverter()
{
}

FrameConverter::~FrameConverter()
{
}

std::vector<tPoint> FrameConverter::transformTofSensor2RobotFrame(const std::vector<tPoint> &input_points,
                                                                  bool isLeft,
                                                                  double rotation_yaw,
                                                                  double rotation_pitch,
                                                                  tPoint translation)
{
    std::vector<tPoint> points;
    tPoint p;

    const double cosine_yaw = std::cos(rotation_yaw*M_PI/180);
    const double sine_yaw = std::sin(rotation_yaw*M_PI/180);
    const double cosine_pitch = std::cos(rotation_pitch*M_PI/180);
    const double sine_pitch = std::sin(rotation_pitch*M_PI/180);

    for (const auto& point : input_points) {
        double x_yaw = point.x * cosine_yaw - point.y * sine_yaw;
        double y_yaw = point.x * sine_yaw + point.y * cosine_yaw;
        double z_yaw = point.z;
        if (isLeft) {
            p.x = x_yaw * cosine_pitch + z_yaw * sine_pitch + translation.x;
            p.y = y_yaw + translation.y;
            p.z = -x_yaw * sine_pitch + z_yaw * cosine_pitch + translation.z;
        } else {
            p.x = x_yaw * cosine_pitch + z_yaw * sine_pitch + translation.x;
            p.y = y_yaw - translation.y;
            p.z = -x_yaw * sine_pitch + z_yaw * cosine_pitch + translation.z;
        }
        points.push_back(p);
    }

    return points;
}

std::vector<tPoint> FrameConverter::transformCameraSensor2RobotFrame(const std::vector<tPoint> &input_points,
                                                                     tPoint translation)
{
    std::vector<tPoint> points;
    tPoint p;

    for (const auto& point : input_points) {
        p.x = point.x + translation.x;
        p.y = point.y + translation.y;
        p.z = point.z + translation.z;
        points.push_back(p);
    }

    return points;
}

std::vector<tPoint> FrameConverter::transformCliffSensor2RobotFrame(robot_custom_msgs::msg::BottomIrData::SharedPtr msg,
                                                                    std::vector<tPoint> &sensor_positions)
{
    std::vector<tPoint> active_sensor_points;

    if (msg->ff) active_sensor_points.push_back(sensor_positions[0]);
    if (msg->fl) active_sensor_points.push_back(sensor_positions[1]);
    if (msg->bl) active_sensor_points.push_back(sensor_positions[2]);
    if (msg->bb) active_sensor_points.push_back(sensor_positions[3]);
    if (msg->br) active_sensor_points.push_back(sensor_positions[4]);
    if (msg->fr) active_sensor_points.push_back(sensor_positions[5]);

    return active_sensor_points;
}

std::vector<tPoint> FrameConverter::transformCollisionData2RobotFrame(robot_custom_msgs::msg::AbnormalEventData::SharedPtr msg,
                                                                      double offset_m)
{
    std::vector<tPoint> points;
    if (msg->event_trigger) {
        tPoint collision_point;

        collision_point.x = offset_m;
        collision_point.y = 0.0;
        collision_point.z = 0.0;

        points.push_back(collision_point);
    }
    return points;
}

std::vector<tPoint> FrameConverter::transformRobot2GlobalFrame(const std::vector<tPoint> &input_points,
                                                               tPose robot_pose)
{
    std::vector<tPoint> global_points;
    tPoint global_point;

    const double robot_cosine = std::cos(robot_pose.orientation.yaw);
    const double robot_sine = std::sin(robot_pose.orientation.yaw);

    for (const auto& local_point : input_points) {
        global_point.x = local_point.x*robot_cosine - local_point.y*robot_sine + robot_pose.position.x;
        global_point.y = local_point.x*robot_sine + local_point.y*robot_cosine + robot_pose.position.y;
        global_point.z = local_point.z + robot_pose.position.z;
        global_points.push_back(global_point);
    }

    return global_points;
}