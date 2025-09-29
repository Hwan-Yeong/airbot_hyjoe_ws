#ifndef __ESCAPE_CANDIDATE_HPP__
#define __ESCAPE_CANDIDATE_HPP__

#include <cmath>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utility>  // for std::pair
#include <vector>

#include "position.hpp"

namespace A1::maneuver
{

class EscapeCandidate
{
   public:
    EscapeCandidate() = default;
    EscapeCandidate(double mx, double my, double distance, double angle_diff, int cost);
    ~EscapeCandidate() = default;
    double mx;        // 월드 좌표 x
    double my;        // 월드 좌표 y
    double distance;  // 로봇과의 거리 (m)
    double angle_diff;
    int cost;
};

constexpr double rad2deg(double deg)
{
    return deg * 180.0 / M_PI;
}

static double angleDiff(double a, double b) __attribute__((unused));
static double angleDiff(double a, double b)
{
    double d = a - b;
    while (d > M_PI)
        d -= 2 * M_PI;
    while (d < -M_PI)
        d += 2 * M_PI;
    return d;
}
std::vector<EscapeCandidate> findBestCandidate(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
    const nav_msgs::msg::OccupancyGrid::SharedPtr static_map,
    Position position,
    int cost_threshold,
    double min_radius,
    double max_radius);

}  // namespace A1::maneuver

#endif  //__ESCAPE_CANDIDATE_HPP__
