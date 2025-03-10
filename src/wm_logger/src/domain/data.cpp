#include "domain/data.hpp"

Data::Data()
    : cmd_vel_(nullptr)
    , cmd_vel_nav_(nullptr)
    , warm_up_status_(nullptr)
    , navigate_to_pose_status_(nullptr)
{
}

Data::~Data() = default;

geometry_msgs::msg::Twist::SharedPtr
Data::get__cmd_vel() const
{
    return this->cmd_vel_;
}

void
Data::set__cmd_vel(geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    this->cmd_vel_ = cmd_vel;
}

geometry_msgs::msg::Twist::SharedPtr
Data::get__cmd_vel_nav() const
{
    return this->cmd_vel_nav_;
}

void
Data::set__cmd_vel_nav(geometry_msgs::msg::Twist::SharedPtr cmd_vel_nav)
{
    this->cmd_vel_nav_ = cmd_vel_nav;
}

std_msgs::msg::Bool::SharedPtr
Data::get__warm_up_status() const
{
    return this->warm_up_status_;
}

void
Data::set__warm_up_status(std_msgs::msg::Bool::SharedPtr warm_up_status)
{
    this->warm_up_status_ = warm_up_status;
}

actionlib_msgs::msg::GoalStatusArray::SharedPtr
Data::get__navigate_to_pose_status() const
{
    return this->navigate_to_pose_status_;
}

void
Data::set__navigate_to_pose_status(actionlib_msgs::msg::GoalStatusArray::SharedPtr navigate_to_pose_status)
{
    this->navigate_to_pose_status_ = navigate_to_pose_status;
}

std::vector<std::string>
Data::get__current_node_list() const
{
    return this->current_node_list_;
}

void
Data::set__current_node_list(const std::vector<std::string> &current_node_list)
{
    this->current_node_list_ = current_node_list;
}