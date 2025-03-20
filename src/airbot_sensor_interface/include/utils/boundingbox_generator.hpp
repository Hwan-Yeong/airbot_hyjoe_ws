#ifndef __BOUNDINGBOX_GENERATOR__
#define __BOUNDINGBOX_GENERATOR__

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "utils/common_struct.hpp"

/**
 * @brief: 카메라 객체인식 object를 boundingbox 데이터로 변환시키는 클래스입니다.
 */
class BoundingBoxGenerator
{
public:
    BoundingBoxGenerator(double sensor_frame_x_translate,
                         double sensor_frame_y_translate,
                         double sensor_frame_z_translate);
    ~BoundingBoxGenerator();

    void updateTargetFrame(std::string &updated_frame);
    void updateRobotPose(tPose &pose);

    vision_msgs::msg::BoundingBox2DArray generateBoundingBoxMessage(
        const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg,
        std::map<int, int> class_id_confidence_th,
        bool direction, double object_max_distance);

    std::pair<robot_custom_msgs::msg::CameraDataArray, vision_msgs::msg::BoundingBox2DArray> getObjectBoundingBoxInfo(
        const robot_custom_msgs::msg::CameraDataArray::SharedPtr msg,
        std::map<int, int> class_id_confidence_th,
        bool direction, double object_max_distance);

private:
    std::string target_frame_;
    tPoint sensor_frame_translation_;
    tPose robot_pose_;
};

#endif // BOUNDINGBOX_GENERATOR