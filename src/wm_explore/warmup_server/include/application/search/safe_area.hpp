//
// Created by changun on 2/23/25.
//

#ifndef WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_SAFE_AREA_HPP_
#define WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_SAFE_AREA_HPP_

#include <vector>
#include "domain/entity/vertex.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <map>
#include "domain/search_define.hpp"
namespace search {
    class SafeArea {
        private:
            std::vector<entity::Vertex> obs_area_vector_;

            std::map<int,bool> area_check_map_;
            search::Priority priority_;

            bool area_check(double point_x, double point_y,int *area_number,std::vector<entity::Vertex> obs_area_vector);
            std::vector<entity::Vertex> setting_area(int area_num);
            search::Priority get_highest_priority(const std::map<int, bool>& area_check_map);
            entity::Vertex get_area_center(entity::Vertex dot_one, entity::Vertex dot_two , entity::Vertex dot_three, entity::Vertex dot_four);
           public :
            explicit SafeArea();
            entity::Vertex search_goal(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan);
            std::vector<entity::Vertex> get_obs_area_vector();
            entity::Vertex select_pose();
            void area_check_map_false_update();
    };
}

#endif  //WARMUP_SERVER_INCLUDE_APPLICATION_SEARCH_SAFE_AREA_HPP_
