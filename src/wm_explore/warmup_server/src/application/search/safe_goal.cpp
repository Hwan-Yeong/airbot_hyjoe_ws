//
// Created by changun on 25. 3. 7.
//

#include "application/search/safe_goal.hpp"
#include <math.h>
namespace search {

    entity::Vertex SafeGoal::calculate_center(entity::Vertex dot_one, entity::Vertex dot_two) {
       return entity::Vertex (((dot_one.GetX() + dot_two.GetX()) / 2.0),(dot_one.GetY() + dot_two.GetY()) / 2.0);
    }

    entity::Vertex SafeGoal::adjust_center_to_origin(entity::Vertex dot_one, entity::Vertex dot_two,
        double target_closer_distance) {
        auto center_dot = calculate_center(dot_one, dot_two);
        double cx = center_dot.GetX();
        double cy = center_dot.GetY();

        // 중심점과 원점 사이의 거리 계산
        double distance = sqrt(cx * cx + cy * cy);

        // 원점과의 거리가 target_closer_distance만큼 더 가까워지도록 비율을 계산
        if (distance != 0 && distance > target_closer_distance) {  // 0으로 나누는 것을 방지하고, 현재 거리가 목표 거리보다 클 경우에만 처리
            double ratio = (distance - target_closer_distance) / distance;
            cx *= ratio;
            cy *= ratio;
        }
        return entity::Vertex(cx, cy);
    }
} // search