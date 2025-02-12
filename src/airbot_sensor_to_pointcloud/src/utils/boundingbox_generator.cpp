#include "utils/boundingbox_generator.hpp"


BoundingBoxGenerator::BoundingBoxGenerator()
{
}

BoundingBoxGenerator::~BoundingBoxGenerator()
{
}

vision_msgs::msg::BoundingBox2DArray BoundingBoxGenerator::generateBoundingBoxMessage(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg,
                                                                                      std::string frame,
                                                                                      tPose robot_pose,
                                                                                      tPoint translation)
{
    auto bbox_array = vision_msgs::msg::BoundingBox2DArray();

    if (msg->data_array.empty() || msg->num == 0) {
        // RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Input data is empty!");
        return bbox_array;
    }

    bbox_array.header.stamp = rclcpp::Clock().now();
    bbox_array.header.frame_id = frame;

    std::vector<robot_custom_msgs::msg::AIData> objects(msg->data_array.begin(), msg->data_array.end());
    const double robot_cos = std::cos(robot_pose.orientation.yaw);
    const double robot_sin = std::sin(robot_pose.orientation.yaw);
    for (const auto &obj : objects)
    {
        if (obj.height >= 0.0 && obj.width >= 0.0) {
            auto bbox = vision_msgs::msg::BoundingBox2D();

            tPoint point_on_sensor_frame, point_on_robot_frame;

            point_on_sensor_frame.x = obj.distance * std::cos(obj.theta) + obj.height/2;
            point_on_sensor_frame.y = obj.distance * std::sin(obj.theta);
            // point_on_sensor_frame.z = -translation.z;

            point_on_robot_frame.x = point_on_sensor_frame.x + translation.x;
            point_on_robot_frame.y = point_on_sensor_frame.y + translation.y;
            // point_on_robot_frame.z = point_on_sensor_frame.z + translation.z;
            if (frame == "map") {    
                bbox.center.position.x = point_on_robot_frame.x*robot_cos - point_on_robot_frame.y*robot_sin + robot_pose.position.x;
                bbox.center.position.y = point_on_robot_frame.x*robot_sin + point_on_robot_frame.y*robot_cos + robot_pose.position.y;
            } else if (frame == "base_link") {
                bbox.center.position.x = point_on_robot_frame.x;
                bbox.center.position.y = point_on_robot_frame.y;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("PointCloud"), "Select Wrong Target Frame: %s", frame.c_str());
            }
            bbox.center.theta = 0.0;
            bbox.size_x = obj.height;
            bbox.size_y = obj.width;
            bbox_array.boxes.push_back(bbox);
        }
    }

    return bbox_array;
}