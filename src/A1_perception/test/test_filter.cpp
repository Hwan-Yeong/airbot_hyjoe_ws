#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "filter/filter.hpp"
#include "filter/filter_factory.hpp"

// Test Density filter
TEST(FilterTest, DensityFilter)
{
    // auto node = std::make_shared<rclcpp::Node>("test_node");
    // YAML::Node config = YAML::Load("density:\n  max_count: 3\n  radius: 1.0\n");
    // auto filter = A1::perception::FilterFactory::create(node, std::string("density"), config);
    // auto layer = A1::perception::Layer();
    // layer.cloud.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
    // layer.cloud.push_back(pcl::PointXYZ(0.1, 0.1, 0.1));
    // layer.cloud.push_back(pcl::PointXYZ(0.2, 0.2, 0.2));
    // layer.cloud.push_back(pcl::PointXYZ(0.3, 0.3, 0.3));
    // layer.cloud.push_back(pcl::PointXYZ(0.4, 0.4, 0.4));
    // layer.cloud.push_back(pcl::PointXYZ(0.5, 0.5, 0.5));
    // layer.cloud.push_back(pcl::PointXYZ(0.6, 0.6, 0.6));
    // layer.cloud.push_back(pcl::PointXYZ(0.7, 0.7, 0.7));
    // layer.cloud.push_back(pcl::PointXYZ(0.8, 0.8, 0.8));
    // layer.cloud.push_back(pcl::PointXYZ(0.9, 0.9, 0.9));
    // auto result = filter->update({layer});
    // EXPECT_EQ(result[0].cloud.size(), 3);
}