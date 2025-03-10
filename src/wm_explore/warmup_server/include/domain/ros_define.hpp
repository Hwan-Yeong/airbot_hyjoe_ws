//
// Created by changun on 25. 2. 13.
//

#ifndef WARMUP_SERVER_ROS_DEFINE_HPP
#define WARMUP_SERVER_ROS_DEFINE_HPP

#define NODE_NAME "warmup_server_node"
#include <string>

constexpr char kActionNameWarup[] = "/explore/warm_up";
constexpr char kTopicNamePubCmdVel[] = "/cmd_vel_nav";
constexpr char kTopicNameScan[] = "/scan";
constexpr char kActionNameNavToPose[] ="/navigate_to_pose";
constexpr char kTopicNameRobotPose[] = "/robot_pose";

class RosConstants{
public :
    const std::string action_name_warmup_ = kActionNameWarup;
    const std::string topic_name_cmdvel_= kTopicNamePubCmdVel;
    const std::string topic_name_scan_ = kTopicNameScan;
    const std::string action_name_nav_to_pose_ = kActionNameNavToPose;
    const std::string topic_name_robot_pose = kTopicNameRobotPose;
};

namespace explore {
    enum class CodeWarmupStatus : int {
        kCancel = 0,
        kSuccess = 1,
        kSensorFail = 2
    };
    enum class CodeWarmupResult : bool {
        kSuccess = true,
        kFail = false
    };
}

namespace navigation {
    enum class ResultCode : int {
        kUnknown = 0,
        kSuccess = 1,
        kAborted = 2,
        kCancelled = 3,
        kFailed = 4
    };
}
#endif //WARMUP_SERVER_ROS_DEFINE_HPP
