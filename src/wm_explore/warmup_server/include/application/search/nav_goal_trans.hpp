//
// Created by changun on 2/25/25.
//

#ifndef WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_NAV_GOAL_TRANS_HPP_
#define WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_NAV_GOAL_TRANS_HPP_
#include "domain/entity/pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
namespace search {
class NavGoalTrans {
 private :
  entity::Pose map_origin_pose_;
    double normalize_angle(double theta);


 public :
    explicit NavGoalTrans();
    geometry_msgs::msg::Pose get_goal_pose(geometry_msgs::msg::Pose origin_robot_pose);
    void set_origin_map_pose(float x, float y);
};

}  // namespace search

#endif  //WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_NAV_GOAL_TRANS_HPP_
