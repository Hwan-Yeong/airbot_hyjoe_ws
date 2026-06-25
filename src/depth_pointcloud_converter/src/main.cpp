#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "depth_pointcloud_converter/depth_to_pointcloud_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<depth_pointcloud_converter::DepthToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
