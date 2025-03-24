//
// Created by wavem on 25. 3. 12.
//

#include "explore/planner/theta_star_planner.hpp"

namespace explore {
    ThetaStarPlanner::ThetaStarPlanner(
        nav2_costmap_2d::Costmap2D *costmap, int how_many_corners,
        double w_euc_cost, double w_traversal_cost
    ) : costmap_(costmap) {
        planner_ = std::make_unique<ThetaStar>();
        planner_->allow_unknown_ = true;
        planner_->costmap_ = costmap;

        planner_->how_many_corners_ = how_many_corners;
        planner_->w_euc_cost_ = w_euc_cost;
        planner_->w_traversal_cost_ = w_traversal_cost;
    }

    nav_msgs::msg::Path ThetaStarPlanner::create_plan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal
    ) {
        nav_msgs::msg::Path global_path;

        unsigned int mx_start, my_start, mx_goal, my_goal;
        if (!planner_->costmap_->worldToMap(
            start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
            return global_path;
        }

        if (!planner_->costmap_->worldToMap(
            goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
            return global_path;
        }

        if (mx_start == mx_goal && my_start == my_goal) {
            if (planner_->costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
                return global_path;
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.header = global_path.header;
            pose.pose.position.z = 0.0;

            pose.pose = start.pose;
            // if we have a different start and goal orientation, set the unique path pose to the goal
            // orientation, unless use_final_approach_orientation=true where we need it to be the start
            // orientation to avoid movement from the local planner
            if (start.pose.orientation != goal.pose.orientation) {
                pose.pose.orientation = goal.pose.orientation;
            }
            global_path.poses.push_back(pose);
            return global_path;
        }

        planner_->set_start_and_goal(start, goal);
        get_plan(global_path);
        // check if a plan is generated
        size_t plan_size = global_path.poses.size();
        if (plan_size > 0) {
            global_path.poses.back().pose.orientation = goal.pose.orientation;
        }

        return global_path;
    }

    void ThetaStarPlanner::get_plan(nav_msgs::msg::Path &global_path) {
        std::vector<coordsW> path;
        if (planner_->is_unsafe_to_plan()) {
            global_path.poses.clear();
        } else if (planner_->generate_path(path)) {
            global_path = linear_interpolation(path, planner_->costmap_->getResolution());
        } else {
            global_path.poses.clear();
        }
    }

    nav_msgs::msg::Path ThetaStarPlanner::linear_interpolation(
        const std::vector<coordsW> &raw_path,
        const double &dist_bw_points
    ) {
        nav_msgs::msg::Path pa;

        geometry_msgs::msg::PoseStamped p1;
        for (unsigned int j = 0; j < raw_path.size() - 1; j++) {
            coordsW pt1 = raw_path[j];
            p1.pose.position.x = pt1.x;
            p1.pose.position.y = pt1.y;
            pa.poses.push_back(p1);

            coordsW pt2 = raw_path[j + 1];
            double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
            int loops = static_cast<int>(distance / dist_bw_points);
            double sin_alpha = (pt2.y - pt1.y) / distance;
            double cos_alpha = (pt2.x - pt1.x) / distance;
            for (int k = 1; k < loops; k++) {
                p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
                p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
                pa.poses.push_back(p1);
            }
        }

        return pa;
    }
} // explore
