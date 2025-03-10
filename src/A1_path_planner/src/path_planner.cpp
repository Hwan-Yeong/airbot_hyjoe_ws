// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "A1_path_planner/path_planner.hpp"
#include <memory>
#include <string>
#include <vector>
#include "A1_path_planner/theta_star.hpp"

namespace A1_path_planner
{
void PathPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    planner_ = std::make_unique<theta_star::ThetaStar>();
    parent_node_ = parent;
    auto node = parent_node_.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    name_ = name;
    tf_ = tf;
    planner_->costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    RCLCPP_INFO(node->get_logger(), "%s():%d: A1_path_planner start !", __FUNCTION__, __LINE__);

    nav2_util::declare_parameter_if_not_declared(node, name_ + ".how_many_corners", rclcpp::ParameterValue(8));

    node->get_parameter(name_ + ".how_many_corners", planner_->how_many_corners_);

    if (planner_->how_many_corners_ != 8 && planner_->how_many_corners_ != 4)
    {
        planner_->how_many_corners_ = 8;
        RCLCPP_WARN(
            logger_,
            "Your value for - .how_many_corners  was overridden, "
            "and is now set to 8");
    }

    nav2_util::declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".allow_unknown", planner_->allow_unknown_);

    nav2_util::declare_parameter_if_not_declared(node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
    node->get_parameter(name_ + ".w_euc_cost", planner_->w_euc_cost_);

    nav2_util::declare_parameter_if_not_declared(node, name_ + ".w_traversal_cost", rclcpp::ParameterValue(2.0));
    node->get_parameter(name_ + ".w_traversal_cost", planner_->w_traversal_cost_);

    planner_->w_heuristic_cost_ = planner_->w_euc_cost_ < 1.0 ? planner_->w_euc_cost_ : 1.0;

    nav2_util::declare_parameter_if_not_declared(
        node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

    dest_publisher_ = node->create_publisher<std_msgs::msg::Int8>("/path_planning/destination", 1);
}

void PathPlanner::cleanup()
{
    // RCLCPP_INFO(logger_, "CleaningUp plugin %s of type A1_path_planner", name_.c_str());
    planner_.reset();
}

void PathPlanner::activate()
{
    // RCLCPP_INFO(logger_, "Activating plugin %s of type A1_path_planner", name_.c_str());
    // Add callback for dynamic parameters
    auto node = parent_node_.lock();
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&PathPlanner::dynamicParametersCallback, this, std::placeholders::_1));
}

void PathPlanner::deactivate()
{
    // RCLCPP_INFO(logger_, "Deactivating plugin %s of type A1_path_planner", name_.c_str());
}

nav_msgs::msg::Path PathPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    nav_msgs::msg::Path global_path;
    auto start_time = std::chrono::steady_clock::now();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));

    unsigned int mx_start, my_start, mx_goal, my_goal;

    // 시작 위치 유효성 확인
    if (!planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start))
    {
        // RCLCPP_WARN(logger_, "Start Coordinates were outside map bounds");
        return global_path;
    }

    // 목표 위치 유효성 확인
    if (!planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
    {
        // RCLCPP_WARN(logger_, "Goal Coordinates were outside map bounds");
        return global_path;
    }

    // 목표 위치가 장애물 위에 있는 경우 처리
    unsigned char goal_cost = planner_->costmap_->getCost(mx_goal, my_goal);
    // cost value 255->250
    if (goal_cost >= 250)
    {
        // RCLCPP_WARN(logger_, "Cost at goal (%u, %u):%u", mx_goal, my_goal, goal_cost);
        // RCLCPP_WARN(logger_, "Goal pose is on an obstacle, adjusting goal position.");

        // -1m 이동한 목표 좌표 계산
        geometry_msgs::msg::PoseStamped adjusted_goal = goal;
        bool isFindPath = false;
        for (long unsigned int i = 0; i < offset_arr.size(); i++)
        {
            auto newCoordinate = offsetDistance(goal.pose.position.x, goal.pose.position.y, i);

            planner_->costmap_->worldToMap(newCoordinate.first, newCoordinate.second, mx_goal, my_goal);
            goal_cost = planner_->costmap_->getCost(mx_goal, my_goal);
            // RCLCPP_WARN(logger_, "new goal[%ld] (%u, %u):%u", i,  mx_goal, my_goal, goal_cost);

            // 재조정된 목표가 맵 범위 내인지 확인
            if (!planner_->costmap_->worldToMap(newCoordinate.first, newCoordinate.second, mx_goal, my_goal) ||
                planner_->costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE)
            {
                // RCLCPP_ERROR(logger_, "Adjusted goal position is still invalid or outside map bounds!");
                continue;
            }
            else
            {
                // RCLCPP_INFO(logger_,"isFindPath = true");
                adjusted_goal.pose.position.x = newCoordinate.first;
                adjusted_goal.pose.position.y = newCoordinate.second;
                planner_->setStartAndGoal(start, adjusted_goal);
                getPlan(global_path);
                if (global_path.poses.size() > 0)
                {
                    isFindPath = true;
                    break;
                }
            }
        }
        // 조정된 목표를 사용하도록 설정
        if (isFindPath)
        {
            planner_->setStartAndGoal(start, adjusted_goal);
            getPlan(global_path);
            std_msgs::msg::Int8 msg;
            msg.data = 1;
            dest_publisher_->publish(msg);
            RCLCPP_INFO(logger_, "%s():%d: Create path to alternative destination", __FUNCTION__, __LINE__);
        }
        else
        {
            return global_path;
        }
    }
    else
    {
        planner_->setStartAndGoal(start, goal);
        getPlan(global_path);
        std_msgs::msg::Int8 msg;
        msg.data = 0;
        dest_publisher_->publish(msg);
        RCLCPP_INFO(logger_, "%s():%d: Create path to original destination", __FUNCTION__, __LINE__);
    }

    size_t plan_size = global_path.poses.size();
    if (plan_size > 0)
    {
        global_path.poses.back().pose.orientation = goal.pose.orientation;
    }

    if (use_final_approach_orientation_)
    {
        if (plan_size == 1)
        {
            global_path.poses.back().pose.orientation = start.pose.orientation;
        }
        else if (plan_size > 1)
        {
            double dx, dy, theta;
            auto last_pose = global_path.poses.back().pose.position;
            auto approach_pose = global_path.poses[plan_size - 2].pose.position;
            dx = last_pose.x - approach_pose.x;
            dy = last_pose.y - approach_pose.y;
            theta = atan2(dy, dx);
            global_path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
        }
    }

    auto stop_time = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
    RCLCPP_DEBUG(logger_, "the time taken is : %i", static_cast<int>(dur.count()));
    RCLCPP_DEBUG(logger_, "the nodes_opened are:  %i", planner_->nodes_opened);
    return global_path;
}

std::pair<double, double> PathPlanner::offsetDistance(double x, double y, int offset_num)
{
    double newx = x;
    double newy = y;
    return {newx + offset_arr[offset_num].first, newy + offset_arr[offset_num].second};
}

void PathPlanner::getPlan(nav_msgs::msg::Path& global_path)
{
    std::vector<coordsW> path;
    if (planner_->isUnsafeToPlan())
    {
        RCLCPP_ERROR(logger_, "Either of the start or goal pose are an obstacle! ");
        global_path.poses.clear();
    }
    else if (planner_->generatePath(path))
    {
        global_path = linearInterpolation(path, planner_->costmap_->getResolution());
    }
    else
    {
        RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
        global_path.poses.clear();
    }
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
}

nav_msgs::msg::Path PathPlanner::linearInterpolation(const std::vector<coordsW>& raw_path, const double& dist_bw_points)
{
    nav_msgs::msg::Path pa;

    geometry_msgs::msg::PoseStamped p1;
    for (unsigned int j = 0; j < raw_path.size() - 1; j++)
    {
        coordsW pt1 = raw_path[j];
        p1.pose.position.x = pt1.x;
        p1.pose.position.y = pt1.y;
        pa.poses.push_back(p1);

        coordsW pt2 = raw_path[j + 1];
        double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
        int loops = static_cast<int>(distance / dist_bw_points);
        double sin_alpha = (pt2.y - pt1.y) / distance;
        double cos_alpha = (pt2.x - pt1.x) / distance;
        for (int k = 1; k < loops; k++)
        {
            p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
            p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
            pa.poses.push_back(p1);
        }
    }

    return pa;
}

rcl_interfaces::msg::SetParametersResult PathPlanner::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    for (auto parameter : parameters)
    {
        const auto& type = parameter.get_type();
        const auto& name = parameter.get_name();

        if (type == ParameterType::PARAMETER_INTEGER)
        {
            if (name == name_ + ".how_many_corners")
            {
                planner_->how_many_corners_ = parameter.as_int();
            }
        }
        else if (type == ParameterType::PARAMETER_DOUBLE)
        {
            if (name == name_ + ".w_euc_cost")
            {
                planner_->w_euc_cost_ = parameter.as_double();
            }
            else if (name == name_ + ".w_traversal_cost")
            {
                planner_->w_traversal_cost_ = parameter.as_double();
            }
        }
        else if (type == ParameterType::PARAMETER_BOOL)
        {
            if (name == name_ + ".use_final_approach_orientation")
            {
                use_final_approach_orientation_ = parameter.as_bool();
            }
            else if (name == name_ + ".allow_unknown")
            {
                planner_->allow_unknown_ = parameter.as_bool();
            }
        }
    }

    result.successful = true;
    return result;
}

}  // namespace A1_path_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(A1_path_planner::PathPlanner, nav2_core::GlobalPlanner)
