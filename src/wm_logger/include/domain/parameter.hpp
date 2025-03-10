#ifndef DOMAIN_PARAMETER_HPP
#define DOMAIN_PARAMETER_HPP

#include <rclcpp/rclcpp.hpp>

#define PARAM_WARM_UP_STATUS_TOPIC "topic.warm_up_status"
#define PARAM_FRONTIER_STATUS_TOPIC "topic.frontier_status"
#define PARAM_NAVIGATE_TO_POSE_STATUS_TOPIC "topic.navigate_to_pose"
#define PARAM_CMD_VEL_TOPIC "topic.cmd_vel"
#define PARAM_CMD_VEL_NAV_TOPIC "topic.cmd_vel_nav"
#define PARAM_NODE_COMMAND "node.process_command"
#define PARAM_NODE_WAVEM_LIST "node.wavem.list"
#define PARAM_NODE_COMMON_LIST "node.common.list"
#define PARAM_TIMER_FREQUENCY_LOGGING "timer.frequency.logging"

class Parameter final
{
public:
    explicit Parameter() = default;
    virtual ~Parameter() = default;

    std::string warm_up_status_topic_;
    std::string frontier_status_topic_;
    std::string navigate_to_pose_status_topic_;
    std::string cmd_vel_topic_;
    std::string cmd_vel_nav_topic_;
    std::string process_command_;
    std::vector<std::string> wavem_node_list_;
    std::vector<std::string> common_node_list_;
    int timer_frequency_logging_;

public:
    using SharedPtr = std::shared_ptr<Parameter>;

};

#endif