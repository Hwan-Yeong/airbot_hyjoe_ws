//
// Created by changun on 2/25/25.
//

#include <cmath>
#include <rclcpp/logger.hpp>

#include "application/search/nav_goal_trans.hpp"
#include "application/trans/TfTrans.hpp"
#include "domain/df_logger.hpp"
#include "domain/ros_define.hpp"

#define TWO_PI (2*M_PI)
#define MAP_THETA 0//90
//1.5708
//#define MAP_THETA 4.71239 //270 (-90)

namespace search {
    search::NavGoalTrans::NavGoalTrans() : map_origin_pose_(0, 0, MAP_THETA)  {

    }
    double NavGoalTrans::normalize_angle(double theta) {
      theta = fmod(theta, TWO_PI); // 2π(360도) 기준 모듈로 연산
      if (theta < 0) theta += TWO_PI; // 음수일 경우 보정
      return theta;
    }

    geometry_msgs::msg::Pose NavGoalTrans::get_goal_pose(geometry_msgs::msg::Pose origin_robot_pose) {
      double dx = origin_robot_pose.position.x;
      double dy = origin_robot_pose.position.y;
      TfTrans tf_trans;
      double dtheta = tf_trans.quaternion_to_radian(origin_robot_pose.orientation.x, origin_robot_pose.orientation.y, origin_robot_pose.orientation.z, origin_robot_pose.orientation.w);
      RCUTILS_LOG_INFO_NAMED(NODE_NAME,"%s:%d: origin_pose : %lf, %lf, %lf, %lf, %lf",__func__,__LINE__,origin_robot_pose.position.x, origin_robot_pose.position.y, origin_robot_pose.orientation.z, origin_robot_pose.orientation.w,dtheta);
      geometry_msgs::msg::Pose goal_pose;

      double new_theta = normalize_angle(map_origin_pose_.GetTheta() + dtheta);
        double temp_theta = normalize_angle(map_origin_pose_.GetTheta() + dtheta);
      goal_pose.position.x = map_origin_pose_.GetX() + dx * cos(new_theta) - dy * sin(new_theta);
      goal_pose.position.y = map_origin_pose_.GetY() + dx * sin(new_theta) + dy * cos(new_theta);

      float test_x = map_origin_pose_.GetX() + dx * cos(temp_theta) - dy * sin(temp_theta);
      float test_y = map_origin_pose_.GetY() + dx * sin(temp_theta) + dy * cos(temp_theta);

      goal_pose.orientation.w = origin_robot_pose.orientation.w;
      goal_pose.orientation.z = origin_robot_pose.orientation.z;
      RCUTILS_LOG_INFO_NAMED(NODE_NAME,"%s:%d: goal_pose : %lf, %lf, %lf, %lf, %lf",__func__,__LINE__,goal_pose.position.x, goal_pose.position.y, goal_pose.orientation.z, goal_pose.orientation.w,dtheta);
      RCUTILS_LOG_INFO_NAMED(NODE_NAME,"%s:%d: temp_pose : %lf, %lf,temp_theta %lf, new_theta %lf",__func__,__LINE__,test_x,test_y,temp_theta,new_theta);
      return goal_pose;
    }
    void NavGoalTrans::set_origin_map_pose(float x, float y) {
        map_origin_pose_.SetX(x);
        map_origin_pose_.SetY(y);
    }

    }  // namespace search