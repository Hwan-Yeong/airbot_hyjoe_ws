#ifndef DOMAIN_DATA_HPP
#define DOMAIN_DATA_HPP

#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <actionlib_msgs/msg/goal_status_array.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>

class Data final
{
private:
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_nav_;
    std_msgs::msg::Bool::SharedPtr warm_up_status_;
    actionlib_msgs::msg::GoalStatusArray::SharedPtr navigate_to_pose_status_;
    std::vector<std::string> current_node_list_;

public:
    explicit Data();
    virtual ~Data();

    [[nodiscard]] geometry_msgs::msg::Twist::SharedPtr get__cmd_vel() const;
    void set__cmd_vel(geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    [[nodiscard]] geometry_msgs::msg::Twist::SharedPtr get__cmd_vel_nav() const;
    void set__cmd_vel_nav(geometry_msgs::msg::Twist::SharedPtr cmd_vel_nav);

    [[nodiscard]] std_msgs::msg::Bool::SharedPtr get__warm_up_status() const;
    void set__warm_up_status(std_msgs::msg::Bool::SharedPtr warm_up_status);

    [[nodiscard]] actionlib_msgs::msg::GoalStatusArray::SharedPtr get__navigate_to_pose_status() const;
    void set__navigate_to_pose_status(actionlib_msgs::msg::GoalStatusArray::SharedPtr navigate_to_pose_status);

    [[nodiscard]] std::vector<std::string> get__current_node_list() const;
    void set__current_node_list(const std::vector<std::string> &current_node_list);

public:
    using SharedPtr = std::shared_ptr<Data>;

};

#endif