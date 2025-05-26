#ifndef __CAMERA_OBJECT_LOGGER__
#define __CAMERA_OBJECT_LOGGER__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

class CameraObjectLogger
{
public:
    CameraObjectLogger();
    ~CameraObjectLogger();

    void setNode(const rclcpp::Node::SharedPtr& node);
    void updateParams(double dist_margin);
    void log(std::pair<robot_custom_msgs::msg::CameraDataArray, vision_msgs::msg::BoundingBox2DArray> object_info);
    void logInfoClear();

private:
    rclcpp::Node::SharedPtr node_ptr_;
    double dist_margin_;
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> objects_;

    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> updateObjects(
        robot_custom_msgs::msg::CameraDataArray object_array,
        vision_msgs::msg::BoundingBox2DArray object_bbox_array);
};





#endif // CAMERA_OBJECT_LOGGER