//
// Created by changun on 25. 3. 8.
//

#ifndef VISUALIZATION_WARMUP_HPP
#define VISUALIZATION_WARMUP_HPP
#include "domain/entity/vertex.hpp"
#include "std_msgs/msg/color_rgba.h"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
namespace explore {
    enum class Color {
        kBlue,
        kRed,
        kGreen,
        kYellow,
    };

    class VisualizationWarmup {
        private :
            const std_msgs::msg::ColorRGBA blue_;
            const std_msgs::msg::ColorRGBA red_;
            const std_msgs::msg::ColorRGBA green_;
            const std_msgs::msg::ColorRGBA yellow_;
            std_msgs::msg::ColorRGBA setting(float r, float g, float b, float a);
        public:
            explicit VisualizationWarmup();
        visualization_msgs::msg::Marker visualize_goal(float x, float y, std::string name_space, explore::Color color,int id) const;
        visualization_msgs::msg::MarkerArray visualize_list( std::vector<entity::Vertex> pose, std::string namespce, explore::Color color) const;


    };
}


#endif //VISUALIZATION_WARMUP_HPP
