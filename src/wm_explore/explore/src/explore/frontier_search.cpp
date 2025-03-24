#include "explore/frontier_search.hpp"
#include "explore/costmap_tools.hpp"
#include "explore/logger.hpp"

#include "nav2_costmap_2d/cost_values.hpp"

namespace explore {
    using nav2_costmap_2d::FREE_SPACE;
    using nav2_costmap_2d::LETHAL_OBSTACLE;
    using nav2_costmap_2d::NO_INFORMATION;

    FrontierSearch::FrontierSearch() : logger_(rclcpp::get_logger("explore_node")) {
    }

    FrontierSearch::FrontierSearch(
        nav2_costmap_2d::Costmap2D *costmap,
        double potential_scale, double gain_scale,
        double min_frontier_size
    ) : costmap_(costmap), potential_scale_(potential_scale),
        gain_scale_(gain_scale), min_frontier_size_(min_frontier_size),
        logger_(rclcpp::get_logger("explore")) {
    }


    std::vector<Frontier> FrontierSearch::SearchFrom(geometry_msgs::msg::Point position) {
        std::vector<Frontier> frontier_list;

        // Sanity check that robot is inside costmap bounds before searching
        unsigned int mx, my;
        if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
            RCL_LOG_ERROR(logger_, "Robot out of costmap bounds, cannot search for frontiers...");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

        map_ = costmap_->getCharMap();
        size_x_ = costmap_->getSizeInCellsX();
        size_y_ = costmap_->getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontier_flag(size_x_ * size_y_, false);
        std::vector<bool> visited_flag(size_x_ * size_y_, false);

        // initialize breadth first search
        std::queue<unsigned int> bfs;

        // find closest clear cell to start search
        unsigned int clear, pos = costmap_->getIndex(mx, my);
        if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
            bfs.push(clear);
        } else {
            bfs.push(pos);
        }
        visited_flag[bfs.front()] = true;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (unsigned nbr: nhood4(idx, *costmap_)) {
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell
                if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                } else if (IsNewFrontierCell(nbr, frontier_flag)) {
                    frontier_flag[nbr] = true;
                    Frontier new_frontier = BuildNewFrontier(nbr, pos, frontier_flag);
                    if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
                        frontier_list.push_back(new_frontier);
                    }
                }
            }
        }

        // remove frontiers that are too close to obstacles
        frontier_list.erase(
            std::remove_if(frontier_list.begin(), frontier_list.end(), [this](const Frontier &frontier) {
                unsigned int centroid_mx, centroid_my;
                costmap_->worldToMap(frontier.centroid.x, frontier.centroid.y, centroid_mx, centroid_my);
                unsigned int centroid_idx = costmap_->getIndex(centroid_mx, centroid_my);
                return map_[centroid_idx] == 254 || map_[centroid_idx] == 253;
            }),
            frontier_list.end()
        );

        // set costs of frontiers
        for (auto &frontier: frontier_list) {
            frontier.cost = FrontierCost(frontier);
        }
        std::sort(frontier_list.begin(), frontier_list.end(),
                  [](const Frontier &f1, const Frontier &f2) { return f1.cost < f2.cost; });

        return frontier_list;
    }

    Frontier FrontierSearch::BuildNewFrontier(
        unsigned int initial_cell,
        unsigned int reference,
        std::vector<bool> &frontier_flag
    ) {
        // initialize frontier structure
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;
        output.size = 1;
        output.min_distance = std::numeric_limits<double>::infinity();

        // record initial contact point for frontier
        unsigned int ix, iy;
        costmap_->indexToCells(initial_cell, ix, iy);
        costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

        // push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        // cache reference position in world coords
        unsigned int rx, ry;
        double reference_x, reference_y;
        costmap_->indexToCells(reference, rx, ry);
        costmap_->mapToWorld(rx, ry, reference_x, reference_y);

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr: nhood8(idx, *costmap_)) {
                // check if neighbour is a potential frontier cell
                if (IsNewFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_->indexToCells(nbr, mx, my);
                    costmap_->mapToWorld(mx, my, wx, wy);

                    geometry_msgs::msg::Point point;
                    point.x = wx;
                    point.y = wy;
                    output.points.push_back(point);

                    // update frontier size
                    output.size++;

                    // update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    // determine frontier's distance from robot, going by closest gridcell
                    // to robot
                    double distance = sqrt(pow(reference_x - wx, 2.0) + pow(reference_y - wy, 2.0));
                    if (distance < output.min_distance) {
                        output.min_distance = distance;
                        output.middle.x = wx;
                        output.middle.y = wy;
                    }

                    // add to queue for breadth first search
                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.size;
        output.centroid.y /= output.size;
        // output.centroid_distance = sqrt(pow(reference_x - output.centroid.x, 2.0) +
        //                                 pow(reference_y - output.centroid.y, 2.0));

        return output;
    }

    bool FrontierSearch::IsNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag) {
        // check that cell is unknown and not already marked as frontier
        if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
            return false;
        }

        // frontier cells should have at least one cell in 4-connected neighbourhood
        // that is free
        for (unsigned int nbr: nhood4(idx, *costmap_)) {
            if (map_[nbr] == FREE_SPACE) {
                return true;
            }
        }

        return false;
    }

    double FrontierSearch::FrontierCost(const Frontier &frontier) {
        return (potential_scale_ * frontier.min_distance * costmap_->getResolution()) - (
                   gain_scale_ * frontier.size * costmap_->getResolution());
    }
} // namespace explore
