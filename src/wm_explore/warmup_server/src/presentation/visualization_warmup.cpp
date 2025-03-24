//
// Created by changun on 25. 3. 8.
//

#include "presentation/visualization_warmup.hpp"

#include <utility>
#include "rclcpp/duration.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

explore::VisualizationWarmup::VisualizationWarmup() : blue_ (setting(0.0,0.0,1.0,1.0)),
red_(setting(1.0, 0.0, 0.0, 1.0)),
green_(setting(0.0, 1.0, 0.0, 1.0)),
yellow_(setting(1.0, 1.0, 0.0, 1.0)){
}

std_msgs::msg::ColorRGBA explore::VisualizationWarmup::setting(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}



visualization_msgs::msg::Marker explore::VisualizationWarmup::visualize_goal(float x, float y ,std::string name_space, explore::Color color, int id) const {
    auto marker = visualization_msgs::msg::Marker();
    rclcpp::Clock ros_clock;
    rclcpp::Time current_time = ros_clock.now();

    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = std::move(name_space);
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime= rclcpp::Duration::from_seconds(10);
    marker.frame_locked = true;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.14;

    if (color == Color::kBlue) {
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.3;
        marker.color = blue_;
    }
    else if (color == Color::kRed) {
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = red_;
    }
    else if (color == Color::kGreen) {
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 1.0;
        marker.color = green_;
    }
    else {
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 1.0;
        marker.color = yellow_;
    }
    return marker;

}

visualization_msgs::msg::MarkerArray explore::VisualizationWarmup::visualize_list(
     std::vector<entity::Vertex> pose, std::string namespce,
    explore::Color color) const {
    visualization_msgs::msg::MarkerArray marker_list;
    for (int i = 0; i < pose.size(); i++) {
        marker_list.markers.push_back(visualize_goal(pose[i].GetX(), pose[i].GetY(), namespce, color,i));
    }
    return marker_list;
}
