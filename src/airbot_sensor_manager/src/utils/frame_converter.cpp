#include "utils/frame_converter.hpp"

namespace sensor_manager {

FrameConverter::FrameConverter()
{
}

FrameConverter::~FrameConverter()
{
}

tPoint FrameConverter::tfMonoTofSensor2RobotFrame(const double input_dist, tPose mono_tof_sensor_frame_pose)
{
    tPoint point;

    if (!tof_mono_extrinsics_updated) {
        tof_mono_sensor_frame_pitch_cosine = std::cos(mono_tof_sensor_frame_pose.orientation.pitch);
        tof_mono_sensor_frame_pitch_sine = std::sin(mono_tof_sensor_frame_pose.orientation.pitch);
        tof_mono_extrinsics_updated = true;
    }

    point.x = mono_tof_sensor_frame_pose.position.x + input_dist * tof_mono_sensor_frame_pitch_cosine;
    point.y = mono_tof_sensor_frame_pose.position.y;
    point.z = mono_tof_sensor_frame_pose.position.z - input_dist * tof_mono_sensor_frame_pitch_sine;

    return point;
}

std::vector<tPoint> FrameConverter::tfMultiTofSensor2RobotFrame(
    const std::vector<double>& tof_dists,
    const std::vector<double>& y_tan,
    const std::vector<double>& z_tan,
    const tPose& multi_tof_sensor_frame_pose)
{
    std::vector<tPoint> points;

    if (tof_dists.size() != y_tan.size() || tof_dists.size() != z_tan.size()) return points;

    if (!tof_multi_extrinsics_updated) {
        multi_tof_sensor_frame_yaw_cosine   = std::cos(multi_tof_sensor_frame_pose.orientation.yaw);
        multi_tof_sensor_frame_yaw_sine     = std::sin(multi_tof_sensor_frame_pose.orientation.yaw);
        multi_tof_sensor_frame_pitch_cosine = std::cos(multi_tof_sensor_frame_pose.orientation.pitch);
        multi_tof_sensor_frame_pitch_sine   = std::sin(multi_tof_sensor_frame_pose.orientation.pitch);
        tof_multi_extrinsics_updated = true;
    }

    for (size_t i = 0; i < tof_dists.size(); ++i) {
        double dist = tof_dists[i];

        if (dist <= 1e-6) { // tof 거리값 0인 수들 NaN으로 처리 (추후 변환시 필터링)
            tPoint zero_p;
            zero_p.x = std::numeric_limits<double>::quiet_NaN();
            zero_p.y = std::numeric_limits<double>::quiet_NaN();
            zero_p.z = std::numeric_limits<double>::quiet_NaN();
            points.push_back(zero_p);
            continue;
        }

        tPoint p_sensor;
        p_sensor.x = dist;
        p_sensor.y = dist * y_tan[i];
        p_sensor.z = dist * z_tan[i];

        double x_yaw = p_sensor.x * multi_tof_sensor_frame_yaw_cosine - p_sensor.y * multi_tof_sensor_frame_yaw_sine;
        double y_yaw = p_sensor.x * multi_tof_sensor_frame_yaw_sine + p_sensor.y * multi_tof_sensor_frame_yaw_cosine;
        double z_yaw = p_sensor.z;

        tPoint p;
        p.x = x_yaw * multi_tof_sensor_frame_pitch_cosine + z_yaw * multi_tof_sensor_frame_pitch_sine + multi_tof_sensor_frame_pose.position.x;
        p.y = y_yaw + multi_tof_sensor_frame_pose.position.y;
        p.z = -x_yaw * multi_tof_sensor_frame_pitch_sine + z_yaw * multi_tof_sensor_frame_pitch_cosine + multi_tof_sensor_frame_pose.position.z;

        points.push_back(p);
    }

    return points;
}

std::vector<tPoint> FrameConverter::tfMultiTofDistance2SensorFrame(
    const std::vector<double>& tof_dists,
    const std::vector<double>& y_tan,
    const std::vector<double>& z_tan)
{
    std::vector<tPoint> points;
    if (tof_dists.size() != y_tan.size() || tof_dists.size() != z_tan.size()) return points;

    for (size_t i = 0; i < tof_dists.size(); ++i) {
        double dist = tof_dists[i];
        tPoint p_sensor;

        if (dist <= 1e-6) {
            p_sensor.x = std::numeric_limits<double>::quiet_NaN();
            p_sensor.y = std::numeric_limits<double>::quiet_NaN();
            p_sensor.z = std::numeric_limits<double>::quiet_NaN();
        } else {
            // 센서 고유의 좌표축 기준 (일반적으로 앞이 x, 왼쪽이 y, 위가 z)
            p_sensor.x = dist;
            p_sensor.y = dist * y_tan[i];
            p_sensor.z = dist * z_tan[i];
        }
        points.push_back(p_sensor);
    }
    return points;
}

tPoint FrameConverter::tfSensorFrame2RobotFrame(
    const tPoint& p_s,
    const tPose& sensor_frame_pose)
{
    if (std::isnan(p_s.x)) return p_s;

    double cos_yaw = std::cos(sensor_frame_pose.orientation.yaw);
    double sin_yaw = std::sin(sensor_frame_pose.orientation.yaw);
    double cos_pitch = std::cos(sensor_frame_pose.orientation.pitch);
    double sin_pitch = std::sin(sensor_frame_pose.orientation.pitch);

    double x_yaw = p_s.x * cos_yaw - p_s.y * sin_yaw;
    double y_yaw = p_s.x * sin_yaw + p_s.y * cos_yaw;
    double z_yaw = p_s.z;

    tPoint p_r;
    p_r.x = x_yaw * cos_pitch + z_yaw * sin_pitch + sensor_frame_pose.position.x;
    p_r.y = y_yaw + sensor_frame_pose.position.y;
    p_r.z = -x_yaw * sin_pitch + z_yaw * cos_pitch + sensor_frame_pose.position.z;

    return p_r;
}

std::vector<tPoint> FrameConverter::tfSensorFrame2RobotFrame(
    const std::vector<tPoint>& pts_sensor,
    const tPose& sensor_frame_pose)
{
    std::vector<tPoint> pts_robot;
    
    double cos_yaw = std::cos(sensor_frame_pose.orientation.yaw);
    double sin_yaw = std::sin(sensor_frame_pose.orientation.yaw);
    double cos_pitch = std::cos(sensor_frame_pose.orientation.pitch);
    double sin_pitch = std::sin(sensor_frame_pose.orientation.pitch);

    for (const auto& p_s : pts_sensor) {
        if (std::isnan(p_s.x)) {
            pts_robot.push_back(p_s); continue;
        }

        double x_yaw = p_s.x * cos_yaw - p_s.y * sin_yaw;
        double y_yaw = p_s.x * sin_yaw + p_s.y * cos_yaw;
        double z_yaw = p_s.z;

        tPoint p_r;
        p_r.x = x_yaw * cos_pitch + z_yaw * sin_pitch + sensor_frame_pose.position.x;
        p_r.y = y_yaw + sensor_frame_pose.position.y;
        p_r.z = -x_yaw * sin_pitch + z_yaw * cos_pitch + sensor_frame_pose.position.z;

        pts_robot.push_back(p_r);
    }
    return pts_robot;
}

std::vector<CameraObject> FrameConverter::tfCameraSensor2SensorFrame(
    const robot_custom_msgs::msg::CameraDataArray* camera_msg,
    bool direction,
    double object_max_distance)
{
    std::vector<CameraObject> objects;
    if (camera_msg->data_array.empty() || camera_msg->num == 0) return objects;

    for (const auto &obj : camera_msg->data_array)
    {
        if (obj.distance > object_max_distance) continue;
        if (obj.height < 0.0 || obj.width < 0.0) continue;

        CameraObject cam_obj;
        cam_obj.id = obj.id;
        
        double calculated_height = std::min(static_cast<double>(obj.width), 0.3);
        double theta_val = direction ? obj.theta : -obj.theta;
        
        cam_obj.bbox.center.position.x = obj.distance * std::cos(theta_val) + calculated_height/2.0;
        cam_obj.bbox.center.position.y = obj.distance * std::sin(theta_val);
        cam_obj.bbox.center.theta = 0.0;
        cam_obj.bbox.size_x = calculated_height;
        if (obj.id == 17) cam_obj.bbox.size_y = std::min(static_cast<double>(obj.width), 0.55);
        else cam_obj.bbox.size_y = obj.width;

        objects.push_back(cam_obj);
    }
    return objects;
}

std::vector<CameraObject> FrameConverter::tfCameraObjects2RobotFrame(
    const std::vector<CameraObject>& objects_sensor,
    const tPose& sensor_frame_pose)
{
    std::vector<CameraObject> objects_robot;
    for (auto obj_r : objects_sensor) {
        tPoint p_s; 
        p_s.x = obj_r.bbox.center.position.x; 
        p_s.y = obj_r.bbox.center.position.y; 
        p_s.z = 0.0;

        tPoint p_r = tfSensorFrame2RobotFrame(p_s, sensor_frame_pose);
        obj_r.bbox.center.position.x = p_r.x;
        obj_r.bbox.center.position.y = p_r.y;
        
        objects_robot.push_back(obj_r);
    }
    return objects_robot;
}

std::vector<CameraObject> FrameConverter::tfCameraObjects2GlobalFrame(
    const std::vector<CameraObject>& objects_robot,
    const tPose& robot_pose)
{
    std::vector<CameraObject> objects_global;
    double robot_cos = std::cos(robot_pose.orientation.yaw);
    double robot_sin = std::sin(robot_pose.orientation.yaw);

    for (auto obj_g : objects_robot) {
        double x_r = obj_g.bbox.center.position.x;
        double y_r = obj_g.bbox.center.position.y;

        obj_g.bbox.center.position.x = x_r * robot_cos - y_r * robot_sin + robot_pose.position.x;
        obj_g.bbox.center.position.y = x_r * robot_sin + y_r * robot_cos + robot_pose.position.y;
        
        objects_global.push_back(obj_g);
    }
    return objects_global;
}

vision_msgs::msg::BoundingBox2DArray FrameConverter::toBBoxArray(
    const std::vector<CameraObject>& objects)
{
    vision_msgs::msg::BoundingBox2DArray bbox_array;

    for (const auto& obj : objects) {
        bbox_array.boxes.push_back(obj.bbox);
    }
    return bbox_array;
}

std::vector<tPoint> FrameConverter::tfBottomIrSensor2SensorFrame(
    const robot_custom_msgs::msg::BottomIrData* cliff_msg,
    double distance_center_to_front_ir_sensor,
    double angle_to_next_ir_sensor)
{
    std::vector<tPoint> active_sensor_points;
    if (!bottom_ir_extrinsics_updated) {
        double d = distance_center_to_front_ir_sensor;
        double deg_1 = 0;
        double deg_2 = angle_to_next_ir_sensor;
        double deg_3 = 180 - angle_to_next_ir_sensor;
        double deg_4 = 180;
        double deg_5 = 180 + angle_to_next_ir_sensor;
        double deg_6 = 360 - angle_to_next_ir_sensor;

        bottom_ir_sensor_positions = {
            tPoint(d*std::cos(deg_1 * M_PI/180), d*std::sin(deg_1 * M_PI/180), 0.0),
            tPoint(d*std::cos(deg_2 * M_PI/180), d*std::sin(deg_2 * M_PI/180), 0.0),
            tPoint(d*std::cos(deg_3 * M_PI/180), d*std::sin(deg_3 * M_PI/180), 0.0),
            tPoint(d*std::cos(deg_4 * M_PI/180), d*std::sin(deg_4 * M_PI/180), 0.0),
            tPoint(d*std::cos(deg_5 * M_PI/180), d*std::sin(deg_5 * M_PI/180), 0.0),
            tPoint(d*std::cos(deg_6 * M_PI/180), d*std::sin(deg_6 * M_PI/180), 0.0)
        };
        bottom_ir_extrinsics_updated = true;
    }

    if (cliff_msg->ff) active_sensor_points.push_back(bottom_ir_sensor_positions[0]);
    if (cliff_msg->fl) active_sensor_points.push_back(bottom_ir_sensor_positions[1]);
    if (cliff_msg->bl) active_sensor_points.push_back(bottom_ir_sensor_positions[2]);
    if (cliff_msg->bb) active_sensor_points.push_back(bottom_ir_sensor_positions[3]);
    if (cliff_msg->br) active_sensor_points.push_back(bottom_ir_sensor_positions[4]);
    if (cliff_msg->fr) active_sensor_points.push_back(bottom_ir_sensor_positions[5]);

    return active_sensor_points;
}

tPoint FrameConverter::tfCollisionData2SensorFrame(
    const robot_custom_msgs::msg::AbnormalEventData* collision_msg,
    double offset_m)
{
    tPoint point;
    // 1: 전방 충돌, -1: 후방 충돌
    if (collision_msg->event_trigger == 1 || collision_msg->event_trigger == -1) {
        point.x = offset_m * collision_msg->event_trigger;
        point.y = 0.0;
        point.z = 0.0;
    }
    return point;
}

std::vector<tPoint> FrameConverter::tfRobot2GlobalFrame(const std::vector<tPoint> &input_points_on_robot_frame, tPose robot_pose)
{
    std::vector<tPoint> global_points;
    tPoint global_point;

    const double robot_cosine = std::cos(robot_pose.orientation.yaw);
    const double robot_sine = std::sin(robot_pose.orientation.yaw);

    for (const auto& local_point : input_points_on_robot_frame) {
        global_point.x = local_point.x*robot_cosine - local_point.y*robot_sine + robot_pose.position.x;
        global_point.y = local_point.x*robot_sine + local_point.y*robot_cosine + robot_pose.position.y;
        global_point.z = local_point.z + robot_pose.position.z;
        global_points.push_back(global_point);
    }

    return global_points;
}

std::vector<tPoint> FrameConverter::tfRobot2GlobalFrame(const tPoint &input_point_on_robot_frame, tPose robot_pose)
{
    return tfRobot2GlobalFrame(std::vector<tPoint>{input_point_on_robot_frame}, robot_pose);
}

} // namespace sensor_manager
