#include "utils/pointcloud_generator.hpp"

namespace sensor_manager {

PointCloudGenerator::PointCloudGenerator() {}

PointCloudGenerator::~PointCloudGenerator() {}

sensor_msgs::msg::PointCloud2 PointCloudGenerator::GeneratePointCloud2Message(
    const std::vector<Point>& points, std::string frame) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());

  for (const auto& pt : points) {
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
    cloud.push_back(pcl::PointXYZ(static_cast<float>(pt.x),
                                  static_cast<float>(pt.y),
                                  static_cast<float>(pt.z)));
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = frame;

  return msg;
}

sensor_msgs::msg::PointCloud2 PointCloudGenerator::GeneratePointCloud2Message(
    const Point& point, std::string frame) {
  return GeneratePointCloud2Message(std::vector<Point>{point}, frame);
}

sensor_msgs::msg::PointCloud2
PointCloudGenerator::GenerateEmptyPointCloud2Message(const std::string& frame) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = frame;
  return msg;
}

sensor_msgs::msg::PointCloud2
PointCloudGenerator::GenerateCameraPointCloud2Message(
    const vision_msgs::msg::BoundingBox2DArray input_bbox_array,
    float resolution, std::string frame, Pose extrinsic_pose) {
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (input_bbox_array.boxes.empty() || resolution <= 0) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = frame;
    return msg;
  }

  for (const auto& box : input_bbox_array.boxes) {
    if (box.size_x <= 0 || box.size_y <= 0) continue;

    const double center_x = box.center.position.x;
    const double center_y = box.center.position.y;
    const double size_x = box.size_x;
    const double size_y = box.size_y;

    int point_size_x = static_cast<int>(size_x / resolution) + 1;
    int point_size_y = static_cast<int>(size_y / resolution) + 1;

    for (int i = 0; i < point_size_x; ++i) {
      for (int j = 0; j < point_size_y; ++j) {
        if (i == 0 || i == point_size_x - 1 || j == 0 || j == point_size_y - 1) {
          float x = static_cast<float>((center_x - size_x / 2.0) + i * resolution);
          float y = static_cast<float>((center_y - size_y / 2.0) + j * resolution);
          float z = 0.0f;
          if (!(frame == "map" || frame == "base_link")) {
            z = -extrinsic_pose.position.z;
          }
          cloud.push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = frame;

  return msg;
}

sensor_msgs::msg::PointCloud2 PointCloudGenerator::MergePointCloud2Vector(
    const std::vector<sensor_msgs::msg::PointCloud2>& pc_msgs,
    std::string frame) {
  pcl::PointCloud<pcl::PointXYZ> merged_cloud;

  for (const auto& pc : pc_msgs) {
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    if (!pc.fields.empty()) {
      pcl::fromROSMsg(pc, tmp_cloud);
    }
    merged_cloud += tmp_cloud;
  }

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(merged_cloud, output_msg);
  output_msg.header.frame_id = frame;
  output_msg.header.stamp = rclcpp::Clock().now();

  return output_msg;
}

}  // namespace sensor_manager
