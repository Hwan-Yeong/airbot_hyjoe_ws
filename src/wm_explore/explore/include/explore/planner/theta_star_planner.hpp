//
// Created by wavem on 25. 3. 12.
//

#pragma once

#include <visualization_msgs/msg/marker_array.hpp>

#include "explore/planner/theta_star.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace explore {
    class ThetaStarPlanner {
    public:
        ThetaStarPlanner(nav2_costmap_2d::Costmap2D *costmap, int how_many_corners, double w_euc_cost, double w_traversal_cost);

        nav_msgs::msg::Path create_plan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal
        );

    protected:
        void get_plan(nav_msgs::msg::Path &global_path);

        static nav_msgs::msg::Path linear_interpolation(
            const std::vector<coordsW> &raw_path,
            const double &dist_bw_points
        );

        std::unique_ptr<ThetaStar> planner_;
        nav2_costmap_2d::Costmap2D *costmap_;
    };
} // explore
