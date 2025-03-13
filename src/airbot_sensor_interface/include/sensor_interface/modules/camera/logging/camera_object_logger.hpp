#ifndef __CAMERA_OBJECT_LOGGER__
#define __CAMERA_OBJECT_LOGGER__

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

/**
 * @brief: Camera 객체인식 데이터를 RCLCPP_INFO 로그로 남기기 위한 클래스 입니다.
 * (정확히는 boundingbox 타입의 메시지로 처리된 객체인식 데이터가 input으로 들어옵니다)
 */
class CameraObjectLogger
{
public:
    CameraObjectLogger();
    ~CameraObjectLogger();

    void updateParams(double dist_margin, double width_margin, double height_margin);
    void log(std::pair<robot_custom_msgs::msg::CameraDataArray, vision_msgs::msg::BoundingBox2DArray> object_info);
    void logInfoClear();

private:
    double dist_margin_;
    double width_margin_;
    double height_margin_;
    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> objects_;

    std::map<int, std::vector<vision_msgs::msg::BoundingBox2D>> updateObjects(
        robot_custom_msgs::msg::CameraDataArray object_array,
        vision_msgs::msg::BoundingBox2DArray object_bbox_array);
};





#endif // __CAMERA_OBJECT_LOGGER__