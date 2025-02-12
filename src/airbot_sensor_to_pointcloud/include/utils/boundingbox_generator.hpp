#ifndef __BOUNDINGBOX_GENERATOR__
#define __BOUNDINGBOX_GENERATOR__

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/ai_data.hpp"
#include "robot_custom_msgs/msg/ai_data_array.hpp"
#include "utils/common_struct.hpp"

class BoundingBoxGenerator
{
public:
    BoundingBoxGenerator();
    ~BoundingBoxGenerator();

    vision_msgs::msg::BoundingBox2DArray generateBoundingBoxMessage(const robot_custom_msgs::msg::AIDataArray::SharedPtr msg,
                                                                    std::string frame,
                                                                    tPose robot_pose,
                                                                    tPoint translation);

private:
};

#endif // BOUNDINGBOX_GENERATOR