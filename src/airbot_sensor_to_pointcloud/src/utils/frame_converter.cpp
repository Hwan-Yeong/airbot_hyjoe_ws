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
                                                                  tPoint translation)
{
    std::vector<tPoint> points;
    tPoint p;

    const double cosine = std::cos(rotation_yaw*M_PI/180);
    const double sine = std::sin(rotation_yaw*M_PI/180);

    for (const auto& point : input_points) {
        if (isLeft) {
            p.x = point.x * cosine - point.y * sine + translation.x;
            p.y = point.x * sine + point.y * cosine + translation.y;
            p.z = point.z + translation.z;
        } else {
            p.x = point.x * cosine - point.y * sine + translation.x;
            p.y = point.x * sine + point.y * cosine - translation.y;
            p.z = point.z + translation.z;
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