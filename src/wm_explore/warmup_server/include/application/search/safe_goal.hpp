//
// Created by changun on 25. 3. 7.
//

#ifndef SAFE_GOAL_HPP
#define SAFE_GOAL_HPP
#include <utility>
#include "domain/entity/vertex.hpp"
namespace search {

class SafeGoal {
public :
    entity::Vertex adjust_center_to_origin(entity::Vertex dot_one,
                                      entity::Vertex dot_two,
                                      double target_closer_distance);
private :
    entity::Vertex calculate_center(entity::Vertex dot_one,
                                    entity::Vertex dot_two);
};

} // search

#endif //SAFE_GOAL_HPP
