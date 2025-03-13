#include "sensor_interface/modules/camera/logging/camera_object_logger.hpp"

CameraObjectLogger::CameraObjectLogger()
{
}

CameraObjectLogger::~CameraObjectLogger()
{
}

void CameraObjectLogger::updateParams(double dist_margin, double width_margin, double height_margin)
{
    dist_margin_ = dist_margin;
    width_margin_ = width_margin;
    height_margin_ = height_margin;
}

void CameraObjectLogger::log(std::pair<robot_custom_msgs::msg::CameraDataArray, vision_msgs::msg::BoundingBox2DArray> object_info)
{
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> new_objects = updateObjects(object_info.first, object_info.second);

    if (new_objects != objects_) {
        objects_ = new_objects;
        auto logger = rclcpp::get_logger("CameraObjectLogger");
        RCLCPP_INFO(logger, "================ [UPDATE] ================");
        for (const auto& [id, object_list] : objects_) {
            for (const auto& object : object_list) {
                RCLCPP_INFO(logger, "[ID]: %u, [Position (X, Y): (%.3f, %.3f)], [Size (W, H): (%.3f, %.3f)]",
                            id, object.center.position.x, object.center.position.y, object.size_x, object.size_y);
            }
        }
    }
}

void CameraObjectLogger::logInfoClear()
{
    objects_.clear();
    RCLCPP_INFO(rclcpp::get_logger("CameraObjectLogger"), "Camera Objects Log Clear");
}

std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> CameraObjectLogger::updateObjects(
    robot_custom_msgs::msg::CameraDataArray object_array,
    vision_msgs::msg::BoundingBox2DArray object_bbox_array)
{
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> ret = objects_;

    for (size_t i = 0; i < object_bbox_array.boxes.size(); ++i) {
        const auto& object = object_bbox_array.boxes[i];
        int id = object_array.data_array[i].id;

        bool is_new_object = true;

        if (ret.find(id) != ret.end()) {
            for (const auto& old_object : ret[id]) {
                double distance = std::sqrt(std::pow(object.center.position.x - old_object.center.position.x, 2) +
                                            std::pow(object.center.position.y - old_object.center.position.y, 2));

                double height_diff = std::abs(object.size_x - old_object.size_x);
                double width_diff = std::abs(object.size_y - old_object.size_y);

                if (distance <= dist_margin_ && width_diff <= width_margin_ && height_diff <= height_margin_) {
                    is_new_object = false;
                    break;
                }
            }
        }

        if (is_new_object) {
            ret[id].push_back(object);
        }
    }
    return ret;
}