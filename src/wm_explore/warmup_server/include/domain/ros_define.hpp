//
// Created by changun on 25. 2. 13.
//

#ifndef WARMUP_SERVER_ROS_DEFINE_HPP
#define WARMUP_SERVER_ROS_DEFINE_HPP

#define NODE_NAME "warmup_server_node"
#include <string>

constexpr char kActionNameWarup[] = "/explore/warm_up";
constexpr char kTopicNamePubCmdVel[] = "/cmd_vel_nav";

class RosConstants{
public :
    const std::string action_name_warmup_ = kActionNameWarup;
    const std::string topic_name_cmdvel_= kTopicNamePubCmdVel;
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
#endif //WARMUP_SERVER_ROS_DEFINE_HPP
