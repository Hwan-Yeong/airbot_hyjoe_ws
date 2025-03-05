#ifndef ROBOT_POSE_PUBLISHER
#define ROBOT_POSE_PUBLISHER

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "robot_pose_publisher/df_robot_pose_publisher.hpp"


using namespace std::chrono_literals;

class RobotPosePublisher : public rclcpp::Node{
    public:
        RobotPosePublisher();
    private:
        using PoseStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<PoseStamped>::SharedPtr publisher_robot_pose_;
        bool is_stamped_;
        std::string base_frame_;
        std::string map_frame_;
        const std::string tp_robot_pose_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        void FnTimerCallback();
};
#endif
