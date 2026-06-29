#include "cloud_converter/sensors/depth_camera.hpp"

#include <sstream>
#include <utility>

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "sensor_manager_node.hpp"

namespace sensor_manager {

DepthCameraCloudConverter::DepthCameraCloudConverter(SensorManagerNode* node_ptr,
                                                     const YAML::Node& config)
    : CloudConverterStrategy(node_ptr) {
  if (!config.IsMap()) {
    auto s = YAML::Dump(config);
    throw std::runtime_error("Invalid filter config format (not a map):\n" + s);
  }

  LoadCommonConfig(config);

  // target_frame 은 전역 파라미터(node target_frame, 보통 "map")를 그대로 사용한다.
  // (base 클래스 생성자에서 target_frame_ = node->GetTargetFrame() 로 이미 설정됨)

  // 높이(z) crop (지면 위 높이). x/y crop 은 map 프레임에서 무의미하여 두지 않는다.
  if (config["filters"] && config["filters"]["height_crop"]) {
    const auto& hc = config["filters"]["height_crop"];
    crop_enable_ = hc["enable"] ? hc["enable"].as<bool>() : true;
    crop_min_z_ = hc["min_z"] ? hc["min_z"].as<double>() : crop_min_z_;
    crop_max_z_ = hc["max_z"] ? hc["max_z"].as<double>() : crop_max_z_;
  }
  if (config["tf_timeout_sec"]) {
    tf_timeout_sec_ = config["tf_timeout_sec"].as<double>();
  }

  std::ostringstream oss;
  oss << GetCommonConfigInfo("DEPTH CAMERA");
  oss << "  target_frame (global)     : " << this->target_frame_ << "\n";
  oss << "  height crop enable         : " << std::boolalpha << crop_enable_ << "\n";
  oss << "  height crop z [m]          : " << crop_min_z_ << " ~ " << crop_max_z_ << "\n";
  oss << "  tf_timeout_sec            : " << tf_timeout_sec_ << "\n";
  oss << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss.str().c_str());
}

ConverterOutput DepthCameraCloudConverter::PcConvertImpl(const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_ || !this->enable_target_frame_cloud_) return output;

  auto input = static_cast<const sensor_msgs::msg::PointCloud2*>(sensor_msg);
  if (input == nullptr) return output;
  if (static_cast<uint64_t>(input->width) * input->height == 0) return output;

  auto tf_buffer = this->node_ptr_->GetTfBuffer();
  if (!tf_buffer) {
    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                         *this->node_ptr_->get_clock(), 3000,
                         "[depth_camera] tf buffer is null.");
    return output;
  }

  // 입력 클라우드를 target_frame_ 으로 변환.
  // 먼저 입력 stamp 기준으로 시도하고, 실패 시 latest(TimePointZero)로 폴백.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(this->target_frame_, input->header.frame_id,
                                    input->header.stamp,
                                    rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const std::exception&) {
    try {
      tf = tf_buffer->lookupTransform(this->target_frame_, input->header.frame_id,
                                      tf2::TimePointZero);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                           *this->node_ptr_->get_clock(), 3000,
                           "[depth_camera] TF '%s' <- '%s' lookup failed: %s",
                           this->target_frame_.c_str(),
                           input->header.frame_id.c_str(), e.what());
      return output;
    }
  }

  sensor_msgs::msg::PointCloud2 transformed;
  tf2::doTransform(*input, transformed, tf);
  transformed.header.frame_id = this->target_frame_;
  transformed.header.stamp = input->header.stamp;

  ApplyFilterPipeline(transformed);

  output.target_frame_clouds.push_back(std::move(transformed));
  return output;
}

void DepthCameraCloudConverter::ApplyFilterPipeline(
    sensor_msgs::msg::PointCloud2& cloud) const {
  // 적용할 필터가 하나도 없으면 변환 비용을 아낀다.
  if (!crop_enable_) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *pcl_cloud);

  // ---- Stage 1: 높이(z) crop (지면 위 높이 범위만 통과) ----
  if (crop_enable_) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(crop_min_z_),
                         static_cast<float>(crop_max_z_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(
        new pcl::PointCloud<pcl::PointXYZ>());
    pass.filter(*cropped);
    pcl_cloud.swap(cropped);
  }

  // ---- (추후) Stage 2: voxel grid down-sampling ----
  // ---- (추후) Stage 3: statistical / radius outlier removal ----
  // ---- (추후) Stage 4: ground segmentation 등 ----

  const std_msgs::msg::Header header = cloud.header;
  pcl::toROSMsg(*pcl_cloud, cloud);
  cloud.header = header;  // toROSMsg 가 덮어쓰므로 복원
}

}  // namespace sensor_manager
