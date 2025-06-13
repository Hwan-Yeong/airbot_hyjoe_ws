#ifndef ERROR_MANAGER_HPP
#define ERROR_MANAGER_HPP

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/qos.hpp"
#include "yaml-cpp/yaml.h"
#include "std_msgs/msg/bool.hpp"
#include "robot_custom_msgs/msg/error_list.hpp"
#include "robot_custom_msgs/msg/error_list_array.hpp"
#include "robot_custom_msgs/msg/robot_state.hpp"


/**
 * @brief 모든 노드에서 발행한 에러(bool)메시지를 구독하고, 업데이트하여
 * udp_interface 노드에게 /error_lists 토픽을 발행하는 노드입니다.
 * amr 기능사양서 기준으로 에러 코드를 부여하고, 추가/삭제되는 에러들을 여기서 관리합니다.
 */
class ErrorManagerNode : public rclcpp::Node
{
public:
    ErrorManagerNode();
    ~ErrorManagerNode();

    void init();

private:
    struct tErrorList {
        robot_custom_msgs::msg::ErrorList error;
        bool should_publish = false;        // 현재 퍼블리싱이 필요한지 여부
        bool has_occurred_before = false;   // 에러가 한 번이라도 발생했었는지 여부
    };

    void initSubscribers(const YAML::Node& config);
    void errorCallback(const std::string& error_code, std_msgs::msg::Bool::SharedPtr msg);
    void robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg);
    void publishErrorList();
    void updateErrorLists(std::string code);
    void addError(const std::string &error_code);
    void releaseErrorLists(std::string code);
    void allErrorReleased();
    void printErrorList();
    void LidarErrorRelease();

    YAML::Node config{};
    std::vector<tErrorList> error_list_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscribers_;
    std::unordered_map<std::string, std::string> error_code_descriptions_;
    std::unordered_set<std::string> erase_after_pub_error_codes_;
    rclcpp::Publisher<robot_custom_msgs::msg::ErrorListArray>::SharedPtr error_list_pub_;
    rclcpp::Subscription<robot_custom_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    int pub_cnt, error_list_size;
    int robot_state_, cur_robot_state, prev_robot_state;
};

#endif // ERROR_MANAGER_HPP
