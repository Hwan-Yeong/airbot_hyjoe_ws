#include "escape_candidate.hpp"
#include <algorithm>
#include <iostream>

namespace A1::maneuver
{

EscapeCandidate::EscapeCandidate(double mx_, double my_, double distance_, double angle_diff_, int cost_)
    : mx(mx_), my(my_), distance(distance_), angle_diff(angle_diff_), cost(cost_)
{
}

std::vector<EscapeCandidate> findBestCandidate(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
    const nav_msgs::msg::OccupancyGrid::SharedPtr static_map,
    Position position,
    int cost_threshold,
    double min_radius,
    double max_radius)
{
    std::vector<EscapeCandidate> candidates;

    if (!costmap)
        return candidates;

    const int width = costmap->info.width;
    const int height = costmap->info.height;
    const double resolution = costmap->info.resolution;
    const double origin_x = costmap->info.origin.position.x;
    const double origin_y = costmap->info.origin.position.y;

    // 로봇 위치 맵 인덱스 변환
    const int mx_robot = static_cast<int>((position.x - origin_x) / resolution);
    const int my_robot = static_cast<int>((position.y - origin_y) / resolution);

    // 탐색 반경 계산
    int max_radius_cells = static_cast<int>(max_radius / resolution);

    for (int dx = -max_radius_cells; dx <= max_radius_cells; ++dx)
    {
        for (int dy = -max_radius_cells; dy <= max_radius_cells; ++dy)
        {
            int nx = mx_robot + dx;
            int ny = my_robot + dy;

            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;

            double dist = std::hypot(dx, dy) * resolution;
            if (dist < min_radius || dist > max_radius)
                continue;

            int index = ny * width + nx;
            int8_t cost = costmap->data[index];

            if (cost >= cost_threshold)
                continue;

            if (static_map)
            {
                int8_t static_cost = static_map->data[index];
                if (static_cost == -1)
                    continue;
            }

            double cell_angle = std::atan2(ny - my_robot, nx - mx_robot);
            double diff = angleDiff(cell_angle, position.yaw);

            // double wx = origin_x + nx * resolution;
            // double wy = origin_y + ny * resolution;
            // candidates.emplace_back(wx, wy, dist, diff);
            double rel_x = (nx - mx_robot) * resolution;
            double rel_y = (ny - my_robot) * resolution;
            candidates.emplace_back(rel_x, rel_y, dist, rad2deg(diff), cost);
        }
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const auto& a, const auto& b)
        {
            // 후방 우선순위
            // double a_cost = a.cost + (a.distance * 100) + (1 / std::fabs(a.angle_diff));
            // double b_cost = b.cost + (b.distance * 100) + (1 / std::fabs(b.angle_diff));

            // 전방 우선순위
            // double a_cost = a.cost + (a.distance * 100) + (std::fabs(a.angle_diff) / 2);
            // double b_cost = b.cost + (b.distance * 100) + (std::fabs(b.angle_diff) / 2);

            // 전 후방 코스트 우선
            auto a_diff_abs = std::fabs(a.angle_diff);
            auto b_diff_abs = std::fabs(b.angle_diff);
            if (a_diff_abs >= 90)
            {
                a_diff_abs = std::fabs(a_diff_abs - 180);
            }
            if (b_diff_abs >= 90)
            {
                b_diff_abs = std::fabs(b_diff_abs - 180);
            }
            double a_cost = a.cost + (a.distance * 100) + (std::fabs(a_diff_abs) / 2);
            double b_cost = b.cost + (b.distance * 100) + (std::fabs(b_diff_abs) / 2);
            return a_cost < b_cost;
        });

    return candidates;
}

}  // namespace A1::maneuver