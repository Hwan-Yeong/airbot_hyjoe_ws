#include "cloud_converter/sensors/depth_camera.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <utility>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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

  // Load Config
  LoadCommonConfig(config);
  if (config["conversion"]) {
    const auto& cv = config["conversion"];
    depth_scale_ = cv["depth_scale"] ? cv["depth_scale"].as<double>() : depth_scale_;
    range_min_ = cv["range_min"] ? cv["range_min"].as<double>() : range_min_;
    range_max_ = cv["range_max"] ? cv["range_max"].as<double>() : range_max_;
    row_step_ = cv["row_step"] ? cv["row_step"].as<int>() : row_step_;
    col_step_ = cv["col_step"] ? cv["col_step"].as<int>() : col_step_;
  }
  row_step_ = std::max(1, row_step_);
  col_step_ = std::max(1, col_step_);
  if (config["filters"] && config["filters"]["height_crop"]) {
    const auto& hc = config["filters"]["height_crop"];
    crop_enable_ = hc["enable"] ? hc["enable"].as<bool>() : true;
    crop_min_z_ = hc["min_z"] ? hc["min_z"].as<double>() : crop_min_z_;
    crop_max_z_ = hc["max_z"] ? hc["max_z"].as<double>() : crop_max_z_;
  }
  if (config["filters"] && config["filters"]["ransac_plane"]) {
    const auto& rp = config["filters"]["ransac_plane"];
    ransac_enable_ = rp["enable"] ? rp["enable"].as<bool>() : false;
    ransac_distance_threshold_ = rp["distance_threshold"]
                                     ? rp["distance_threshold"].as<double>()
                                     : ransac_distance_threshold_;
    ransac_max_iterations_ = rp["max_iterations"]
                                 ? rp["max_iterations"].as<int>()
                                 : ransac_max_iterations_;
    ransac_eps_angle_deg_ = rp["eps_angle_deg"]
                                ? rp["eps_angle_deg"].as<double>()
                                : ransac_eps_angle_deg_;
    ransac_max_ground_offset_ = rp["max_ground_offset"]
                                    ? rp["max_ground_offset"].as<double>()
                                    : ransac_max_ground_offset_;
    ransac_min_inlier_ratio_ = rp["min_inlier_ratio"]
                                   ? rp["min_inlier_ratio"].as<double>()
                                   : ransac_min_inlier_ratio_;
    ransac_fallback_min_z_ = rp["fallback_min_z"]
                                 ? rp["fallback_min_z"].as<double>()
                                 : ransac_fallback_min_z_;
  }
  if (config["tf_timeout_sec"]) {
    tf_timeout_sec_ = config["tf_timeout_sec"].as<double>();
  }

  // Print Config
  std::ostringstream oss_depth_camera;
  oss_depth_camera << GetCommonConfigInfo("DEPTH CAMERA");
  oss_depth_camera << "  target_frame (global)     : " << this->target_frame_
                   << "\n";
  oss_depth_camera << "  depth_scale               : " << depth_scale_
                   << " (16UC1 -> m)\n";
  oss_depth_camera << "  range [m]                 : " << range_min_ << " ~ "
                   << range_max_ << "\n";
  oss_depth_camera << "  downsample (row,col)      : " << row_step_ << ", "
                   << col_step_ << "\n";
  oss_depth_camera << "  height crop enable        : " << std::boolalpha
                   << crop_enable_ << "\n";
  oss_depth_camera << "  height crop z [m]         : " << crop_min_z_ << " ~ "
                   << crop_max_z_ << "\n";
  oss_depth_camera << "  ransac plane enable       : " << std::boolalpha
                   << ransac_enable_ << "\n";
  oss_depth_camera << "  ransac dist_th / max_iter : "
                   << ransac_distance_threshold_ << " / "
                   << ransac_max_iterations_ << "\n";
  oss_depth_camera << "  ransac eps_angle [deg]    : " << ransac_eps_angle_deg_
                   << "\n";
  oss_depth_camera << "  ransac ground_offset [m]  : "
                   << ransac_max_ground_offset_ << "\n";
  oss_depth_camera << "  ransac min_inlier_ratio   : "
                   << ransac_min_inlier_ratio_ << "\n";
  oss_depth_camera << "  ransac fallback_min_z [m] : " << ransac_fallback_min_z_
                   << "\n";
  oss_depth_camera << "  tf_timeout_sec            : " << tf_timeout_sec_
                   << "\n";
  oss_depth_camera << "----------------------------------------------------";
  RCLCPP_INFO(this->node_ptr_->get_logger(), "%s", oss_depth_camera.str().c_str());
}

ConverterOutput DepthCameraCloudConverter::PcConvertImpl(const void* sensor_msg) {
  ConverterOutput output;

  if (!this->use_converter_) return output;

  auto input = static_cast<const robot_custom_msgs::msg::DepthCameraData*>(sensor_msg);
  if (input == nullptr) {
    RCLCPP_WARN(this->node_ptr_->get_logger(),
                "[depth_camera] Received null sensor_msg pointer.");
    return output;
  }

  // Create raw cloud on sensor(optical) frame
  sensor_msgs::msg::PointCloud2 depth_raw_cloud;
  if (!BuildPointCloud(input, depth_raw_cloud)) {
    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                         *this->node_ptr_->get_clock(), 3000,
                         "[depth_camera] Failed to build point cloud from "
                         "depth image.");
    return output;
  }

  if (this->enable_sensor_tf_cloud_) {
    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                         *this->node_ptr_->get_clock(), 3000,
                         "[depth_camera] enable_sensor_tf_cloud is not "
                         "supported for depth camera. Skipping local frame "
                         "cloud.");
  }

  if (!this->enable_target_frame_cloud_) return output;

  auto tf_buffer = this->node_ptr_->GetTfBuffer();
  if (!tf_buffer) {
    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                         *this->node_ptr_->get_clock(), 3000,
                         "[depth_camera] tf buffer is null.");
    return output;
  }

  // raw cloud 를 target_frame_ 으로 변환.
  // 먼저 입력 stamp 기준으로 시도하고, 실패 시 latest(TimePointZero)로 폴백.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(this->target_frame_,
                                    depth_raw_cloud.header.frame_id,
                                    depth_raw_cloud.header.stamp,
                                    rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const std::exception&) {
    try {
      tf = tf_buffer->lookupTransform(this->target_frame_,
                                      depth_raw_cloud.header.frame_id,
                                      tf2::TimePointZero);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                           *this->node_ptr_->get_clock(), 3000,
                           "[depth_camera] TF '%s' <- '%s' lookup failed: %s",
                           this->target_frame_.c_str(),
                           depth_raw_cloud.header.frame_id.c_str(), e.what());
      return output;
    }
  }

  sensor_msgs::msg::PointCloud2 transformed;
  tf2::doTransform(depth_raw_cloud, transformed, tf);
  transformed.header.frame_id = this->target_frame_;
  transformed.header.stamp = depth_raw_cloud.header.stamp;

  ApplyFilterPipeline(transformed, tf);

  output.target_frame_clouds.push_back(std::move(transformed));
  return output;
}

bool DepthCameraCloudConverter::BuildPointCloud(
    const robot_custom_msgs::msg::DepthCameraData* input,
    sensor_msgs::msg::PointCloud2& cloud_out) const {
  const auto& image = input->image;
  const auto& info = input->camera_info;

  if (image.encoding != "16UC1") {
    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                         *this->node_ptr_->get_clock(), 3000,
                         "[depth_camera] Unexpected depth encoding '%s' "
                         "(expected 16UC1) - skipping",
                         image.encoding.c_str());
    return false;
  }

  const uint32_t width = image.width;
  const uint32_t height = image.height;
  if (width == 0 || height == 0 || image.data.empty()) {
    return false;
  }

  // 카메라 내부 파라미터 (camera_info K 에서 추출)
  const float fx = static_cast<float>(info.k[0]);
  const float fy = static_cast<float>(info.k[4]);
  const float cx = static_cast<float>(info.k[2]);
  const float cy = static_cast<float>(info.k[5]);
  if (fx == 0.0f || fy == 0.0f) {
    RCLCPP_WARN_THROTTLE(this->node_ptr_->get_logger(),
                         *this->node_ptr_->get_clock(), 3000,
                         "[depth_camera] Invalid camera intrinsics "
                         "(fx/fy = 0). Waiting for valid camera_info.");
    return false;
  }

  cloud_out.header = image.header;
  // 카메라 드라이버의 image stamp 는 ROS time 과 다른 클럭이라
  // stamp 기준 TF lookup 이 항상 timeout(50ms/frame 블로킹)된다.
  // 수신 시각(ROS time)으로 재스탬프하여 TF 를 즉시 조회하게 한다.
  cloud_out.header.stamp = input->timestamp;
  cloud_out.height = 1;  // unorganized (유효 포인트만)
  cloud_out.is_dense = true;
  cloud_out.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_out);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  const size_t max_points =
      static_cast<size_t>((height + row_step_ - 1) / row_step_) *
      static_cast<size_t>((width + col_step_ - 1) / col_step_);
  modifier.resize(max_points);

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud_out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud_out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud_out, "z");

  const float scale = static_cast<float>(depth_scale_);
  const float rmin = static_cast<float>(range_min_);
  const float rmax = static_cast<float>(range_max_);

  size_t count = 0;
  for (uint32_t v = 0; v < height; v += row_step_) {
    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(
        image.data.data() + static_cast<size_t>(v) * image.step);
    for (uint32_t u = 0; u < width; u += col_step_) {
      const uint16_t raw = depth_row[u];
      if (raw == 0) {
        continue;  // 무효 픽셀
      }
      const float z = static_cast<float>(raw) * scale;
      if (z < rmin || z > rmax) {
        continue;
      }
      // 광학 좌표계: x=오른쪽, y=아래, z=정면
      *it_x = (static_cast<float>(u) - cx) * z / fx;
      *it_y = (static_cast<float>(v) - cy) * z / fy;
      *it_z = z;
      ++it_x;
      ++it_y;
      ++it_z;
      ++count;
    }
  }

  modifier.resize(count);  // 실제 유효 포인트 수로 축소
  return true;
}

void DepthCameraCloudConverter::ApplyFilterPipeline(
    sensor_msgs::msg::PointCloud2& cloud,
    const geometry_msgs::msg::TransformStamped& sensor_tf) const {
  if (!crop_enable_ && !ransac_enable_) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *pcl_cloud);

  // ---- step 1: RANSAC 바닥(평면) 제거 ----
  // height crop 보다 먼저 수행한다. min_z crop 이 바닥 노이즈 분포의
  // 아래쪽(z<min_z)만 잘라낸 뒤 평면을 fit 하면 평면이 위로 편향되기 때문.
  if (ransac_enable_ && !pcl_cloud->empty()) {
    RemoveGroundPlane(pcl_cloud, sensor_tf);
  }

  // ---- step 2: height(z) crop (지면 위 높이 범위만 통과) ----
  if (crop_enable_ && !pcl_cloud->empty()) {
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

  const std_msgs::msg::Header header = cloud.header;
  pcl::toROSMsg(*pcl_cloud, cloud);
  cloud.header = header;  // toROSMsg 가 덮어쓰므로 복원
}

void DepthCameraCloudConverter::RemoveGroundPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const geometry_msgs::msg::TransformStamped& sensor_tf) const {
  // RANSAC 으로 찾은 지배적 평면이 아래 "바닥 검증" 을 모두 통과할 때만 제거한다.
  //  1) 기울기: 평면 법선이 z축 대비 eps_angle 이내 (SACMODEL_PERPENDICULAR_PLANE
  //     모델 자체 제약으로 강제됨)
  //  2) 높이: 평면을 센서 xy 위치로 연장했을 때 |z| <= max_ground_offset.
  //     바닥은 항상 로봇 밑을 지나므로, 박스 윗면 같은 저상 수평면을
  //     바닥으로 오인해 지워버리는 것을 방지한다.
  //  3) 지지도: inlier 비율 >= min_inlier_ratio. 소수의 점에 우연히 맞은
  //     평면(노이즈)을 바닥으로 채택하지 않도록 한다.
  //
  // NOTE(문턱 승월 오감지 / IMU 연동 지점):
  //  2D localization 은 roll/pitch 를 map TF 에 반영하지 않으므로, 문턱 승월 시
  //  map 기준 cloud 전체가 기울어져 들어온다. eps_angle 이내의 기울기는 RANSAC
  //  이 자체 흡수하지만, 그 이상은 IMU 보정이 필요하다.
  //  TODO(IMU): roll/pitch 데이터 파이프라인 연결 후 이 함수에서
  //    (a) seg.setAxis() 의 기준축 (0,0,1) 을 IMU 자세로 회전시켜 전달하거나,
  //    (b) |pitch| > th 인 동안 eps_angle 을 일시 완화 or 해당 프레임의
  //        장애물 출력을 skip (camera 모듈의 object_ignore_pitch_th_deg 참고)
  //  방식으로 처리 예정.

  constexpr size_t kMinPointsForFit = 30;  // 이보다 적으면 평면 fit 신뢰 불가
  constexpr int kMaxPlaneAttempts = 3;     // 바닥 아닌 평면 검출 시 재시도 횟수

  float ga = 0.0f, gb = 0.0f, gc = 1.0f, gd = 0.0f;  // 바닥 평면 계수
  bool ground_found = false;

  if (cloud->size() >= kMinPointsForFit) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(ransac_eps_angle_deg_ * M_PI / 180.0);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setInputCloud(cloud);

    const double px = sensor_tf.transform.translation.x;
    const double py = sensor_tf.transform.translation.y;

    // 바닥 검증에 실패한 평면(inlier)은 fit 후보에서 제외하고 다음으로 큰
    // 평면을 재탐색한다. (예: 시야 대부분을 박스 윗면이 차지하는 경우)
    pcl::IndicesPtr candidates(new pcl::Indices(cloud->size()));
    std::iota(candidates->begin(), candidates->end(), 0);

    for (int attempt = 0; attempt < kMaxPlaneAttempts; ++attempt) {
      if (candidates->size() < kMinPointsForFit) break;
      seg.setIndices(candidates);

      pcl::ModelCoefficients coeffs;
      pcl::PointIndices inliers;
      seg.segment(inliers, coeffs);
      if (coeffs.values.size() != 4 || inliers.indices.empty()) break;

      float a = coeffs.values[0];
      float b = coeffs.values[1];
      float c = coeffs.values[2];
      float d = coeffs.values[3];
      const float norm = std::sqrt(a * a + b * b + c * c);
      if (norm < 1e-6f) break;
      a /= norm;
      b /= norm;
      c /= norm;
      d /= norm;
      if (c < 0.0f) {  // 법선을 +z(위쪽) 방향으로 정렬
        a = -a;
        b = -b;
        c = -c;
        d = -d;
      }

      const double inlier_ratio =
          static_cast<double>(inliers.indices.size()) / cloud->size();
      const double plane_z_at_sensor = -(a * px + b * py + d) / c;

      if (inlier_ratio >= ransac_min_inlier_ratio_ &&
          std::abs(plane_z_at_sensor) <= ransac_max_ground_offset_) {
        ga = a;
        gb = b;
        gc = c;
        gd = d;
        ground_found = true;
        break;
      }

      std::sort(inliers.indices.begin(), inliers.indices.end());
      pcl::IndicesPtr remaining(new pcl::Indices());
      remaining->reserve(candidates->size() - inliers.indices.size());
      std::set_difference(candidates->begin(), candidates->end(),
                          inliers.indices.begin(), inliers.indices.end(),
                          std::back_inserter(*remaining));
      candidates.swap(remaining);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>());
  filtered->header = cloud->header;
  filtered->reserve(cloud->size());

  if (ground_found) {
    // 평면 위쪽으로 distance_threshold 초과인 점만 유지.
    // (평면 근방 inlier 뿐 아니라 평면 아래쪽 노이즈까지 함께 제거된다)
    const float th = static_cast<float>(ransac_distance_threshold_);
    for (const auto& p : cloud->points) {
      const float signed_dist = ga * p.x + gb * p.y + gc * p.z + gd;
      if (signed_dist > th) filtered->push_back(p);
    }
  } else if (ransac_fallback_min_z_ > 0.0) {
    // 바닥 평면 미검출(바닥 미노출, 과도한 기울어짐 등) 시 보수적 z-crop 폴백
    RCLCPP_DEBUG_THROTTLE(this->node_ptr_->get_logger(),
                          *this->node_ptr_->get_clock(), 3000,
                          "[depth_camera] ground plane not found - fallback "
                          "z-crop (z < %.3f) applied.",
                          ransac_fallback_min_z_);
    const float min_z = static_cast<float>(ransac_fallback_min_z_);
    for (const auto& p : cloud->points) {
      if (p.z >= min_z) filtered->push_back(p);
    }
  } else {
    return;  // 폴백 비활성: 원본 유지
  }

  cloud.swap(filtered);
}

}  // namespace sensor_manager
