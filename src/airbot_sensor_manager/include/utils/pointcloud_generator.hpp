#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "utils/common_struct.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"

namespace sensor_manager {

class PointCloudGenerator {
 public:
  PointCloudGenerator();
  ~PointCloudGenerator();

  /**
   * @brief Generates a single merged message from multiple PointCloud2 data (vector<PC2>).
   *
   * @param[in] pc_msgs Vector of PointCloud2 messages.
   * @param[in] frame target_frame <string>.
   * @return sensor_msgs::msg::PointCloud2
   */
  sensor_msgs::msg::PointCloud2 MergePointCloud2Vector(
      const std::vector<sensor_msgs::msg::PointCloud2>& pc_msgs,
      std::string frame);

  /**
   * @brief Generates PointCloud2 data from "General Sensor" position data.
   *
   * @param[in] points Vector of position data to convert to PointCloud2.
   * @param[in] frame target_frame <string>.
   * @return sensor_msgs::msg::PointCloud2
   */
  sensor_msgs::msg::PointCloud2 GeneratePointCloud2Message(
      const std::vector<Point>& points, std::string frame);
  sensor_msgs::msg::PointCloud2 GeneratePointCloud2Message(const Point& point,
                                                           std::string frame);

  /**
   * @brief Generates PointCloud2 data from "Camera Sensor" position data.
   *
   * @param[in] input_bbox_array Set of bounding box arrays to convert to PointCloud2.
   * @param[in] resolution Resolution of the PointCloud2 data.
   * @param[in] frame target_frame <string>.
   * @return sensor_msgs::msg::PointCloud2
   */
  sensor_msgs::msg::PointCloud2 GenerateCameraPointCloud2Message(
      const vision_msgs::msg::BoundingBox2DArray input_bbox_array,
      float resolution, std::string frame, Pose extrinsic_pose);

  /**
   * @brief Generates "Empty" PointCloud2 data (for initialization and clear purposes).
   *
   * @param[in] frame target_frame <string>.
   * @return sensor_msgs::msg::PointCloud2
   */
  sensor_msgs::msg::PointCloud2 GenerateEmptyPointCloud2Message(
      const std::string& frame);

 private:
};

}  // namespace sensor_manager


