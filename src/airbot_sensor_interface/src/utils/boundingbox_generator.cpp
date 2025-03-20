#include "utils/boundingbox_generator.hpp"

BoundingBoxGenerator::BoundingBoxGenerator(double sensor_frame_x_translate,
                                           double sensor_frame_y_translate,
                                           double sensor_frame_z_translate)
    : sensor_frame_translation_(sensor_frame_x_translate,
                                sensor_frame_y_translate,
                                sensor_frame_z_translate)
{
}

BoundingBoxGenerator::~BoundingBoxGenerator()
{
}

void BoundingBoxGenerator::updateTargetFrame(std::string &updated_frame)
{
    target_frame_ = updated_frame;
}

void BoundingBoxGenerator::updateRobotPose(tPose &pose)
{
    robot_pose_ = pose;
}

vision_msgs::msg::BoundingBox2DArray BoundingBoxGenerator::generateBoundingBoxMessage(
    const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg,
    std::map<int, int> class_id_confidence_th,
    bool direction, double object_max_distance)
{
    auto bbox_array = vision_msgs::msg::BoundingBox2DArray();

    if (msg->data_array.empty() || msg->num == 0) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return bbox_array;
    }

    bbox_array.header.stamp = rclcpp::Clock().now();
    bbox_array.header.frame_id = target_frame_;

    std::vector<robot_custom_msgs::msg::CameraData> objects(msg->data_array.begin(), msg->data_array.end());
    const double robot_cos = std::cos(robot_pose_.orientation.yaw);
    const double robot_sin = std::sin(robot_pose_.orientation.yaw);
    for (const auto &obj : objects)
    {
        if (obj.distance > object_max_distance) continue; // 객체인식 장애물 최대 거리 제한
        auto it = class_id_confidence_th.find(obj.id);
        if (it != class_id_confidence_th.end() && static_cast<int>(obj.score) >= it->second) { // data filtering with "class id", "confidencd score"
            if (obj.height >= 0.0 && obj.width >= 0.0) {
                auto bbox = vision_msgs::msg::BoundingBox2D();

                tPoint point_on_sensor_frame, point_on_robot_frame;
                double height = std::min(static_cast<double>(obj.width), 0.3);

                if (direction) {
                    point_on_sensor_frame.x = obj.distance * std::cos(obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(obj.theta);
                } else {
                    point_on_sensor_frame.x = obj.distance * std::cos(-obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(-obj.theta);
                }
                // point_on_sensor_frame.z = -sensor_frame_translation_.z;

                point_on_robot_frame.x = point_on_sensor_frame.x + sensor_frame_translation_.x;
                point_on_robot_frame.y = point_on_sensor_frame.y + sensor_frame_translation_.y;
                // point_on_robot_frame.z = point_on_sensor_frame.z + sensor_frame_translation_.z;
                if (target_frame_ == "map") {
                    bbox.center.position.x = point_on_robot_frame.x*robot_cos - point_on_robot_frame.y*robot_sin + robot_pose_.position.x;
                    bbox.center.position.y = point_on_robot_frame.x*robot_sin + point_on_robot_frame.y*robot_cos + robot_pose_.position.y;
                } else if (target_frame_ == "base_link") {
                    bbox.center.position.x = point_on_robot_frame.x;
                    bbox.center.position.y = point_on_robot_frame.y;
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", target_frame_.c_str());
                }
                bbox.center.theta = 0.0;
                bbox.size_x = height;
                bbox.size_y = obj.width;
                bbox_array.boxes.push_back(bbox);
            }
        } else {
            // RCLCPP_INFO(rclcpp::get_logger("BoundingBoxGenerator"),
            //             "[Camera Filtered Data] ID: %d, SCORE: %d", static_cast<int>(obj.id), static_cast<int>(obj.score));
        }
    }

    return bbox_array;
}

std::pair<robot_custom_msgs::msg::CameraDataArray, vision_msgs::msg::BoundingBox2DArray> BoundingBoxGenerator::getObjectBoundingBoxInfo(
    const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg,
    std::map<int, int> class_id_confidence_th,
    bool direction, double object_max_distance)
{
    auto filtered_objects = robot_custom_msgs::msg::CameraDataArray();
    auto bbox_array = vision_msgs::msg::BoundingBox2DArray();

    if (msg->data_array.empty() || msg->num == 0) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return std::make_pair(filtered_objects, bbox_array);
    }

    filtered_objects.timestamp = msg->timestamp;
    filtered_objects.num = msg->num;
    filtered_objects.robot_angle = msg->robot_angle;
    filtered_objects.robot_x = msg->robot_x;
    filtered_objects.robot_y = msg->robot_y;

    bbox_array.header.stamp = rclcpp::Clock().now();
    bbox_array.header.frame_id = target_frame_;

    std::vector<robot_custom_msgs::msg::CameraData> objects(msg->data_array.begin(), msg->data_array.end());
    const double robot_cos = std::cos(robot_pose_.orientation.yaw);
    const double robot_sin = std::sin(robot_pose_.orientation.yaw);
    for (const auto &obj : objects)
    {
        if (obj.distance > object_max_distance) continue; // 객체인식 장애물 최대 거리 제한
        auto it = class_id_confidence_th.find(obj.id);
        if (it != class_id_confidence_th.end() && static_cast<int>(obj.score) >= it->second) { // data filtering with "class id", "confidencd score"
            if (obj.height >= 0.0 && obj.width >= 0.0) {
                filtered_objects.data_array.push_back(obj);
                auto bbox = vision_msgs::msg::BoundingBox2D();

                tPoint point_on_sensor_frame, point_on_robot_frame;
                double height = std::min(static_cast<double>(obj.width), 0.3);

                if (direction) {
                    point_on_sensor_frame.x = obj.distance * std::cos(obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(obj.theta);
                } else {
                    point_on_sensor_frame.x = obj.distance * std::cos(-obj.theta) + height/2;
                    point_on_sensor_frame.y = obj.distance * std::sin(-obj.theta);
                }
                // point_on_sensor_frame.z = -sensor_frame_translation_.z;

                point_on_robot_frame.x = point_on_sensor_frame.x + sensor_frame_translation_.x;
                point_on_robot_frame.y = point_on_sensor_frame.y + sensor_frame_translation_.y;
                // point_on_robot_frame.z = point_on_sensor_frame.z + sensor_frame_translation_.z;

                bbox.center.position.x = point_on_robot_frame.x*robot_cos - point_on_robot_frame.y*robot_sin + robot_pose_.position.x;
                bbox.center.position.y = point_on_robot_frame.x*robot_sin + point_on_robot_frame.y*robot_cos + robot_pose_.position.y;
                bbox.center.theta = 0.0;
                bbox.size_x = height;
                bbox.size_y = obj.width;
                bbox_array.boxes.push_back(bbox);
            }
        } else {
            // RCLCPP_INFO(rclcpp::get_logger("BoundingBoxGenerator"),
            //             "[Camera Filtered Data] ID: %d, SCORE: %d", static_cast<int>(obj.id), static_cast<int>(obj.score));
        }
    }

    return std::make_pair(filtered_objects, bbox_array);
}