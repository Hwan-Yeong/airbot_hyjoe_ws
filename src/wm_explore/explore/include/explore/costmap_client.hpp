//
// Created by wavem on 25. 2. 24.
//

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace explore {
    class Costmap2DClient {
    public:
        /**
         * @brief Contructs client and start listening
         * @details Constructor will block until first map update is received and
         * map is ready to use, also will block before trasformation
         * robot_base_frame <-> global_frame is available.
         *
         * @param node node handle to retrieve parameters from
         * @param tf_listener Will be used for transformation of robot pose.
         */
        Costmap2DClient(const rclcpp::Node::SharedPtr &node, const tf2_ros::Buffer *tf_listener);

        /**
         * @brief Get the pose of the robot in the global frame of the costmap
         * @return pose of the robot in the global frame of the costmap
         */
        geometry_msgs::msg::Pose GetRobotPose() const;

        /**
         * @brief Get the pose of the robot in the global frame of the costmap
         * @return pose of the robot in the global frame of the costmap
         */
        geometry_msgs::msg::Pose GetTFRobotPose() const;

        /**
         * @brief Return a pointer to the "master" costmap which receives updates from
         * all the layers.
         *
         * This pointer will stay the same for the lifetime of Costmap2DClient object.
         */
        nav2_costmap_2d::Costmap2D *GetMap() {
            return &map_;
        }

        /**
         * @brief Return a pointer to the "master" costmap which receives updates from
         * all the layers.
         *
         * This pointer will stay the same for the lifetime of Costmap2DClient object.
         */
        const nav2_costmap_2d::Costmap2D *GetMap() const {
            return &map_;
        }

        /**
         * @brief Return a pointer to the "master" costmap which receives updates from
         * all the layers.
         *
         * This pointer will stay the same for the lifetime of Costmap2DClient object.
         */
        nav2_costmap_2d::Costmap2D *GetCostmap() {
            return &costmap_;
        }

        /**
         * @brief Return a pointer to the "master" costmap which receives updates from
         * all the layers.
         *
         * This pointer will stay the same for the lifetime of Costmap2DClient object.
         */
        const nav2_costmap_2d::Costmap2D *GetCostmap() const {
            return &costmap_;
        }

        /**
         * @brief  Returns the global frame of the costmap
         * @return The global frame of the costmap
         */
        const std::string &GetGlobalFrameID() const {
            return global_frame_;
        }

        /**
         * @brief  Returns the local frame of the costmap
         * @return The local frame of the costmap
         */
        const std::string &GetBaseFrameID() const {
            return robot_base_frame_;
        }

        bool IsCostmapInitialized() const;

    protected:
        void UpdateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void UpdateFullCostMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        nav2_costmap_2d::Costmap2D map_;
        nav2_costmap_2d::Costmap2D costmap_;
        bool costmap_received_ = false;

        const tf2_ros::Buffer *const tf_; ///< @brief Used for transforming
        rclcpp::Node::SharedPtr node_;
        std::string global_frame_; ///< @brief The global frame for the costmap
        std::string robot_base_frame_; ///< @brief The frame_id of the robot base
        double transform_tolerance_; ///< timeout before transform errors

    private:
        // will be unsubscribed at destruction
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        // rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
        rclcpp::CallbackGroup::SharedPtr cbg_costmap_;
    };
} // namespace explore
