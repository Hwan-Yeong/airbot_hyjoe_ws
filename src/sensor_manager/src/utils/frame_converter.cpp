#include "utils/frame_converter.hpp"


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

std::vector<tPoint> FrameConverter::tfMultiTofSensor2RobotFrame(const std::vector<tPoint> &input_points, bool isLeft, tPose multi_tof_sensor_frame_pose)
{
    std::vector<tPoint> points;
    tPoint p;

    const double cosine_yaw = std::cos(multi_tof_sensor_frame_pose.orientation.yaw*M_PI/180);
    const double sine_yaw = std::sin(multi_tof_sensor_frame_pose.orientation.yaw*M_PI/180);
    const double cosine_pitch = std::cos(multi_tof_sensor_frame_pose.orientation.pitch*M_PI/180);
    const double sine_pitch = std::sin(multi_tof_sensor_frame_pose.orientation.pitch*M_PI/180);

    for (const auto& point : input_points) {
        double x_yaw = point.x * cosine_yaw - point.y * sine_yaw;
        double y_yaw = point.x * sine_yaw + point.y * cosine_yaw;
        double z_yaw = point.z;
        if (isLeft) {
            p.x = x_yaw * cosine_pitch + z_yaw * sine_pitch + multi_tof_sensor_frame_pose.position.x;
            p.y = y_yaw + multi_tof_sensor_frame_pose.position.y;
            p.z = -x_yaw * sine_pitch + z_yaw * cosine_pitch + multi_tof_sensor_frame_pose.position.z;
        } else {
            p.x = x_yaw * cosine_pitch + z_yaw * sine_pitch + multi_tof_sensor_frame_pose.position.x;
            p.y = y_yaw - multi_tof_sensor_frame_pose.position.y;
            p.z = -x_yaw * sine_pitch + z_yaw * cosine_pitch + multi_tof_sensor_frame_pose.position.z;
        }
        points.push_back(p);
    }

    return points;
}

vision_msgs::msg::BoundingBox2DArray FrameConverter::tfCameraSensor2RobotFrameBBoxArray(
    const robot_custom_msgs::msg::CameraDataArray* camera_msg, tPose &robot_pose, std::map<int, int> class_id_confidence_th, bool direction, double object_max_distance, std::string camera_target_frame, tPose camera_sensor_frame_pose)
{
    auto bbox_array = vision_msgs::msg::BoundingBox2DArray();

    if (camera_msg->data_array.empty() || camera_msg->num == 0) {
        return bbox_array;
    }

    bbox_array.header.frame_id = camera_target_frame;
    // bbox_array.header.stamp = rclcpp::Clock().now(); // 함수 호출부에서 찍도록 변경

    std::vector<robot_custom_msgs::msg::CameraData> objects(camera_msg->data_array.begin(), camera_msg->data_array.end());

    for (const auto &obj : objects)
    {
        if (obj.distance > object_max_distance) continue; // 객체인식 장애물 최대 거리 제한
        auto it = class_id_confidence_th.find(obj.id);
        if (it != class_id_confidence_th.end() && static_cast<int>(obj.score) >= it->second) { // data filtering with "class id", "confidencd score"
            if (obj.height >= 0.0 && obj.width >= 0.0) {
                auto bbox = vision_msgs::msg::BoundingBox2D();
                tPoint point_on_sensor_frame, point_on_robot_frame;

                /*
                    객체의 너비(가로폭)가 30cm 이하인 경우, 높이를 너비와 동일한 -> 정사각형 객체로 가공
                    객체의 너비(가로폭)가 30cm 이상인 경우, 높이를 30cm로 고정
                */
                double height = std::min(static_cast<double>(obj.width), 0.3);

                if (direction) {
                    point_on_sensor_frame.x = obj.distance * std::cos(obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(obj.theta);
                } else {
                    point_on_sensor_frame.x = obj.distance * std::cos(-obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(-obj.theta);
                }

                point_on_robot_frame.x = point_on_sensor_frame.x + camera_sensor_frame_pose.position.x;
                point_on_robot_frame.y = point_on_sensor_frame.y + camera_sensor_frame_pose.position.y;

                if (camera_target_frame == "map") {
                    double robot_cos = std::cos(robot_pose.orientation.yaw);
                    double robot_sin = std::sin(robot_pose.orientation.yaw);
                    bbox.center.position.x = point_on_robot_frame.x*robot_cos - point_on_robot_frame.y*robot_sin + robot_pose.position.x;
                    bbox.center.position.y = point_on_robot_frame.x*robot_sin + point_on_robot_frame.y*robot_cos + robot_pose.position.y;
                } else if (camera_target_frame == "base_link") {
                    bbox.center.position.x = point_on_robot_frame.x;
                    bbox.center.position.y = point_on_robot_frame.y;
                } else {
                    // RCLCPP_INFO(this->node_ptr->get_logger(), "Select Wrong Target Frame: %s", target_frame_.c_str());
                }

                bbox.center.theta = 0.0;
                bbox.size_x = height;

                /*
                    pole (의자 다리) 객체의 경우,
                    객체의 너비(가로폭)가 50cm 이하인 경우, 너비를 50cm로 고정
                    객체의 너비(가로폭)가 50cm 이상인 경우, 인식된 너비 그대로 사용
                */
                // if (obj.id == 12) { // pole
                //     bbox.size_y = obj.width < 0.5 ? 0.5 : obj.width;
                // } else {
                //     bbox.size_y = obj.width;
                // }

                /*
                    bed (침대) 객체의 경우,
                    객체의 최대 너비(가로폭)를 55cm로 제한
                */
                if (obj.id == 17) { // bed
                    bbox.size_y = std::min(obj.width, 0.55);
                } else {
                    bbox.size_y = obj.width;
                }

                bbox_array.boxes.push_back(bbox);
            }
        } else {
            // RCLCPP_INFO(this->node_ptr->get_logger(),
            //     "[Camera Filtered Data] ID: %d, SCORE: %d",
            //     static_cast<int>(obj.id), static_cast<int>(obj.score)
            // );
        }
    }

    return bbox_array;
}

std::vector<tPoint> FrameConverter::tfBottomIrSensor2RobotFrame(const robot_custom_msgs::msg::BottomIrData* cliff_msg, double distance_center_to_front_ir_sensor, double angle_to_next_ir_sensor)
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

        tPoint ir_1_position_ = tPoint(d*std::cos(deg_1 * M_PI/180), d*std::sin(deg_1 * M_PI/180), 0.0);
        tPoint ir_2_position_ = tPoint(d*std::cos(deg_2 * M_PI/180), d*std::sin(deg_2 * M_PI/180), 0.0);
        tPoint ir_3_position_ = tPoint(d*std::cos(deg_3 * M_PI/180), d*std::sin(deg_3 * M_PI/180), 0.0);
        tPoint ir_4_position_ = tPoint(d*std::cos(deg_4 * M_PI/180), d*std::sin(deg_4 * M_PI/180), 0.0);
        tPoint ir_5_position_ = tPoint(d*std::cos(deg_5 * M_PI/180), d*std::sin(deg_5 * M_PI/180), 0.0);
        tPoint ir_6_position_ = tPoint(d*std::cos(deg_6 * M_PI/180), d*std::sin(deg_6 * M_PI/180), 0.0);

        bottom_ir_sensor_positions = {ir_1_position_, ir_2_position_, ir_3_position_, ir_4_position_, ir_5_position_, ir_6_position_};

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

tPoint FrameConverter::tfCollisionData2RobotFrame(const robot_custom_msgs::msg::AbnormalEventData* collision_msg, double offset_m)
{
    tPoint points;

    // 1: 전방 충돌, -1: 후방 충돌
    if (collision_msg->event_trigger == 1 || collision_msg->event_trigger == -1) {
        points.x = offset_m * collision_msg->event_trigger;
        points.y = 0.0;
        points.z = 0.0;
    }

    return points;
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