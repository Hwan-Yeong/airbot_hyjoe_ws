//
// Created by wavem on 25. 2. 24.
//

#include <explore/costmap_client.hpp>
#include <unistd.h>

#include <functional>
#include <mutex>
#include <string>
#include <explore/logger.hpp>

namespace explore {
    // static translation table to speed things up
    std::array<unsigned char, 256> init_translation_table();

    static const std::array<unsigned char, 256> cost_translation_table__ =
            init_translation_table();

    Costmap2DClient::Costmap2DClient(const rclcpp::Node::SharedPtr &node, const tf2_ros::Buffer *tf_listener)
        : tf_(tf_listener), node_(node) {
        std::string costmap_topic;
        std::string costmap_updates_topic;

        node_->declare_parameter<std::string>("costmap_topic", std::string("map"));
        node_->declare_parameter<std::string>("costmap_updates_topic",
                                             std::string("map_updates"));
        node_->declare_parameter<std::string>("robot_base_frame", std::string("base_link"));
        // transform tolerance is used for all tf transforms here
        node_->declare_parameter<double>("transform_tolerance", 0.3);

        node_->get_parameter("costmap_topic", costmap_topic);
        node_->get_parameter("costmap_updates_topic", costmap_updates_topic);
        node_->get_parameter("robot_base_frame", robot_base_frame_);
        node_->get_parameter("transform_tolerance", transform_tolerance_);

        /* initialize costmap */
        cbg_costmap_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sco_costmap;
        sco_costmap.callback_group = cbg_costmap_;

        costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic,
            1000,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                costmap_received_ = true;
                UpdateFullMap(msg);
            },
            sco_costmap
        );

        /* subscribe to map updates */
        costmap_updates_sub_ = node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            costmap_updates_topic,
            1000,
            [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
                UpdatePartialMap(msg);
            }
        );
    }

    void Costmap2DClient::UpdateFullMap(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg
    ) {
        global_frame_ = msg->header.frame_id;

        unsigned int size_in_cells_x = msg->info.width;
        unsigned int size_in_cells_y = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        RCL_LOG_DEBUG(node_->get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x, size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x, origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCL_LOG_DEBUG(node_->get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < msg->data.size(); ++i) {
            unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
            costmap_data[i] = cost_translation_table__[cell_cost];
        }
        RCL_LOG_DEBUG(node_->get_logger(), "full map update, written %lu values", costmap_size);
    }

    void Costmap2DClient::UpdatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
        RCL_LOG_DEBUG(node_->get_logger(), "received partial map update");
        global_frame_ = msg->header.frame_id;

        if (msg->x < 0 || msg->y < 0) {
            RCL_LOG_DEBUG(node_->get_logger(), "negative coordinates, invalid update. x: %d, y: %d", msg->x, msg->y);
            return;
        }

        size_t x0 = static_cast<size_t>(msg->x);
        size_t y0 = static_cast<size_t>(msg->y);
        size_t xn = msg->width + x0;
        size_t yn = msg->height + y0;

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*mutex);

        size_t costmap_xn = costmap_.getSizeInCellsX();
        size_t costmap_yn = costmap_.getSizeInCellsY();

        if (xn > costmap_xn || x0 > costmap_xn || yn > costmap_yn ||
            y0 > costmap_yn) {
            RCL_LOG_WARN(
                node_->get_logger(),
                "received update doesn't fully fit into existing map, "
                "nly part will be copied. received: [%lu, %lu], [%lu, %lu] "
                "map is: [0, %lu], [0, %lu]",
                x0, xn, y0, yn, costmap_xn, costmap_yn
            );
        }

        // update map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t i = 0;
        for (size_t y = y0; y < yn && y < costmap_yn; ++y) {
            for (size_t x = x0; x < xn && x < costmap_xn; ++x) {
                size_t idx = costmap_.getIndex(x, y);
                unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
                costmap_data[idx] = cost_translation_table__[cell_cost];
                ++i;
            }
        }
    }

    geometry_msgs::msg::Pose Costmap2DClient::GetRobotPose() const {
        geometry_msgs::msg::PoseStamped robot_pose;
        geometry_msgs::msg::Pose empty_pose;
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = node_->now();

        auto &clk = *node_->get_clock();

        // get the global pose of the robot
        try {
            robot_pose = tf_->transform(robot_pose, global_frame_, tf2::durationFromSec(transform_tolerance_));
        } catch (std::exception &e) {
            RCL_LOG_ERROR(node_->get_logger(), "Failed to transform robot pose: %s", e.what());
        }

        return robot_pose.pose;
    }

    geometry_msgs::msg::Pose Costmap2DClient::GetTFRobotPose() const {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCL_LOG_ERROR(node_->get_logger(), "Failed to transform robot pose: %s", ex.what());
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;

        return pose;
    }


    bool Costmap2DClient::IsCostmapInitialized() const {
        return costmap_received_;
    }


    std::array<unsigned char, 256> init_translation_table() {
        std::array<unsigned char, 256> cost_translation_table;

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = 0; // NO obstacle
        cost_translation_table[99] = 253; // INSCRIBED obstacle
        cost_translation_table[100] = 254; // LETHAL obstacle
        cost_translation_table[static_cast<unsigned char>(-1)] = 255; // UNKNOWN

        return cost_translation_table;
    }
} // namespace explore
