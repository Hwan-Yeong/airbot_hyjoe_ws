#ifndef __LAYER_HPP__
#define __LAYER_HPP__

#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"

namespace A1::perception
{
class Layer
{
   public:
    rclcpp::Time sensor_timestamp{};  // sensor_interface에서 받은 timestamp
    rclcpp::Time timestamp{};         // Subscribe 시점의 timestamp
    pcl::PointCloud<pcl::PointXYZ> cloud{};
};

using LayerVector = std::vector<Layer>;
}  // namespace A1::perception

#endif  // __LAYER_HPP__