#include "airbot_sensor_to_pointcloud/modules/camera/logging/camera_object_logger.hpp"

CameraObjectLogger::CameraObjectLogger()
{
}

CameraObjectLogger::~CameraObjectLogger()
{
}

void CameraObjectLogger::setNode(const rclcpp::Node::SharedPtr& node)
{
    node_ptr_ = node;
}

void CameraObjectLogger::updateParams(double dist_margin)
{
    dist_margin_ = dist_margin;
}

void CameraObjectLogger::log(std::pair<robot_custom_msgs::msg::CameraDataArray, vision_msgs::msg::BoundingBox2DArray> object_info)
{
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> new_objects = updateObjects(object_info.first, object_info.second);

    if (new_objects != objects_) {
        objects_ = new_objects;
        RCLCPP_INFO(node_ptr_->get_logger(), "================ [UPDATE] ================");
        for (const auto& [id, object_list] : objects_) {
            for (const auto& object : object_list) {
                RCLCPP_INFO(node_ptr_->get_logger(), "[ID]: %u, [Position (X, Y): (%.3f, %.3f)], [Size (W, H): (%.3f, %.3f)]",
                            id, object.center.position.x, object.center.position.y, object.size_x, object.size_y);
            }
        }
    }
}

void CameraObjectLogger::logInfoClear()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "[CameraObjectLogger] Before Camera Objects Log Clear, objects.size: %zu", objects_.size());
    objects_.clear();
    RCLCPP_INFO(node_ptr_->get_logger(), "[CameraObjectLogger] After Camera Objects Log Clear, objects.size: %zu", objects_.size());
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
                if (distance <= dist_margin_) {
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