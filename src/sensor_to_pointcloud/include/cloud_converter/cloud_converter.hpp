#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "yaml-cpp/yaml.h"

#include "utils/common_struct.hpp"


namespace sensor_to_pointcloud {

class SensorToPointcloudNode;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;

/**
 * sensor -> PointCloud2  Strategy interface
 * Customize conversion method by sensor
 */
class CloudConverterStrategy
{
  public:
    CloudConverterStrategy(std::shared_ptr<SensorToPointcloudNode> node_ptr_);

    virtual ~CloudConverterStrategy() = default;

    virtual PointCloudMsg pc_convert(const void* sensor_msg) = 0;

    /**
     * @brief 필터가 참조하는 SensorToPointcloudNode 스마트 포인터를 반환합니다.
     *
     * @return std::shared_ptr<SensorToPointcloudNode> SensorToPointcloudNode 스마트 포인터
     */
    std::shared_ptr<SensorToPointcloudNode> getNodePtr() const;

  protected:
    std::shared_ptr<SensorToPointcloudNode> node_ptr{};
    std::string target_frame_;
};
using CloudConverterPtr = std::shared_ptr<CloudConverterStrategy>;

/**
 * @brief Camera 센서 데이터 -> PointCloud2 변환
 */
class CameraCloudConverter : public CloudConverterStrategy
{
  public:
    CameraCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsg pc_convert(const void* sensor_msg) override;
    vision_msgs::msg::BoundingBox2DArray generateObjectBBoxArray(const robot_custom_msgs::msg::CameraDataArray* msg, tPose &robot_pose,std::map<int, int> class_id_confidence_th, bool direction, double object_max_distance);
    sensor_msgs::msg::PointCloud2 generateCameraPointCloudMsg(const vision_msgs::msg::BoundingBox2DArray input_bbox_array, float resolution);

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_camera_ = true;
    bool object_direction_ = true;
    double pointcloud_resolution_ = 0.05;
    double object_max_dist_ = 1.5;
    tPoint sensor_frame_translation_ = tPoint(0.15473, 0.0, 0.5331);
    std::map<int, int> camera_class_id_confidence_th_ = {};  // yaml 에서만 수정하면 자동으로 반영, but 초기화 없음
    // std::map<int, int> camera_class_id_confidence_th_ = {       // yaml & 선언부에서 모두 수정해야 반영, but 초기화 있음
    //   {0, 0},  // cable
    //   {1, 0},  // carpet [Unused]
    //   {2, 0},  // clothes
    //   {3, 0},  // liquid [Unused]
    //   {4, 0},  // non_obstacle [Unused]
    //   {5, 0},  // obstacle [Unused]
    //   {6, 0},  // poop
    //   {7, 0},  // scale
    //   // {8, 0},  // threshold [Unused]
    //   // {9, 0},  // person
    //   {10, 0}, // dog [Unused]
    //   {11, 0}, // cat [Unused]
    //   {12, 0}, // chair
    //   {13, 0}, // base
    //   // {14, 0}, // shoes
    //   {15, 0}, // electronic_device
    //   {16, 0}, // dryingrack
    //   {17, 0}, // bed
    //   {18, 0},
    //   {19, 0}
    // };
};

/**
 * @brief Empty PointCloud2 변환
 */
class EmptyCloudConverter : public CloudConverterStrategy
{
  public:
    EmptyCloudConverter(std::shared_ptr<SensorToPointcloudNode> node_ptr_, const YAML::Node& config);

  private:
    PointCloudMsg pc_convert(const void* sensor_msg) override;
    sensor_msgs::msg::PointCloud2 generateEmptyPointCloudMsg();

    // set default: yaml 파일이 정상이 아닌 경우를 대비하여
    bool use_empty_msg_ = true;
};

} // namespace sensor_to_pointcloud
