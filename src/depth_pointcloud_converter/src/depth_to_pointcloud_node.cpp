#include "depth_pointcloud_converter/depth_to_pointcloud_node.hpp"

#include <algorithm>

#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

namespace depth_pointcloud_converter
{

DepthToPointCloudNode::DepthToPointCloudNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_to_pointcloud_node", options)
{
  declareParameters();
  setupCommunications();

  last_published_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  depth_recv_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  RCLCPP_INFO(get_logger(), "depth_to_pointcloud_node started");
  RCLCPP_INFO(get_logger(), "  depth_topic       : %s", depth_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  camera_info_topic : %s", camera_info_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  output_topic      : %s", output_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  output_reliability: %s", output_reliability_.c_str());
  RCLCPP_INFO(get_logger(), "  depth_scale       : %.4f (16UC1 -> m)", depth_scale_);
  RCLCPP_INFO(get_logger(), "  range [m]         : %.2f ~ %.2f", range_min_, range_max_);
  RCLCPP_INFO(get_logger(), "  downsample (r,c)  : %d, %d", row_step_, col_step_);
  RCLCPP_INFO(get_logger(), "  publish_rate_hz   : %.1f", publish_rate_hz_);
}

void DepthToPointCloudNode::declareParameters()
{
  depth_topic_        = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  camera_info_topic_  = declare_parameter<std::string>("camera_info_topic", "/camera/depth/camera_info");
  output_topic_       = declare_parameter<std::string>("output_topic", "/camera/depth/points");
  output_frame_       = declare_parameter<std::string>("output_frame", "");
  output_reliability_ = declare_parameter<std::string>("output_reliability", "reliable");
  depth_scale_        = declare_parameter<double>("depth_scale", 0.001);
  range_min_          = declare_parameter<double>("range_min", 0.1);
  range_max_          = declare_parameter<double>("range_max", 5.0);
  row_step_           = std::max(1, static_cast<int>(declare_parameter<int>("row_step", 1)));
  col_step_           = std::max(1, static_cast<int>(declare_parameter<int>("col_step", 1)));
  publish_rate_hz_    = declare_parameter<double>("publish_rate_hz", 15.0);

  if (publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 200.0) {
    RCLCPP_WARN(get_logger(), "Invalid publish_rate_hz %.1f, using 15.0", publish_rate_hz_);
    publish_rate_hz_ = 15.0;
  }
}

void DepthToPointCloudNode::setupCommunications()
{
  // 입력: 드라이버가 best_effort 로 발행하므로 동일하게 맞춘다.
  auto sensor_qos = rclcpp::SensorDataQoS();

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, sensor_qos,
    std::bind(&DepthToPointCloudNode::depthCallback, this, _1));

  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, sensor_qos,
    std::bind(&DepthToPointCloudNode::cameraInfoCallback, this, _1));

  // 출력: 기본 reliable(RViz 기본 QoS와 호환). best_effort 선택 가능.
  rclcpp::QoS out_qos(5);
  if (output_reliability_ == "best_effort") {
    out_qos.best_effort();
  } else {
    out_qos.reliable();
  }
  points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, out_qos);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&DepthToPointCloudNode::publishTimerCallback, this));
}

// ===================== 구독 콜백: 저장만 =====================
void DepthToPointCloudNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(depth_mutex_);
  latest_depth_ = msg;                  // shared_ptr 교체(데이터 복사 없음)
  depth_recv_time_ = this->now();
}

void DepthToPointCloudNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(info_mutex_);
  intrinsics_.fx = msg->k[0];
  intrinsics_.fy = msg->k[4];
  intrinsics_.cx = msg->k[2];
  intrinsics_.cy = msg->k[5];
  intrinsics_.valid = (intrinsics_.fx != 0.0 && intrinsics_.fy != 0.0);
}

// ===================== timer: 변환·발행 =====================
void DepthToPointCloudNode::publishTimerCallback()
{
  // 1) 최신 depth 프레임을 짧게 lock 잡고 가져온다(이후 lock 밖에서 변환).
  sensor_msgs::msg::Image::SharedPtr depth;
  rclcpp::Time recv_time;
  {
    std::lock_guard<std::mutex> lk(depth_mutex_);
    if (!latest_depth_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for depth image on %s ...", depth_topic_.c_str());
      return;
    }
    recv_time = depth_recv_time_;
    // 새 프레임이 없으면(동일 프레임이면) 재발행하지 않는다.
    if (recv_time <= last_published_time_) {
      return;
    }
    depth = latest_depth_;
  }

  // 2) 카메라 내부 파라미터 확보.
  CameraIntrinsics intr;
  {
    std::lock_guard<std::mutex> lk(info_mutex_);
    intr = intrinsics_;
  }
  if (!intr.valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Waiting for valid camera_info on %s ...", camera_info_topic_.c_str());
    return;
  }

  // 3) 변환 (lock 밖, 무거운 작업).
  sensor_msgs::msg::PointCloud2 cloud;
  if (!buildPointCloud(*depth, intr, cloud)) {
    return;
  }

  points_pub_->publish(cloud);
  last_published_time_ = recv_time;
}

// ===================== depth -> PointCloud2 =====================
bool DepthToPointCloudNode::buildPointCloud(const sensor_msgs::msg::Image & depth,
                                            const CameraIntrinsics & intr,
                                            sensor_msgs::msg::PointCloud2 & cloud) const
{
  if (depth.encoding != "16UC1") {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Unexpected depth encoding '%s' (expected 16UC1) - skipping", depth.encoding.c_str());
    return false;
  }

  const uint32_t width = depth.width;
  const uint32_t height = depth.height;
  if (width == 0 || height == 0 || depth.data.empty()) {
    return false;
  }

  cloud.header = depth.header;
  if (!output_frame_.empty()) {
    cloud.header.frame_id = output_frame_;
  }
  cloud.height = 1;            // unorganized (유효 포인트만)
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  // 최악(모든 픽셀 유효) 크기로 확보 후 끝에서 실제 개수로 축소.
  const size_t max_points =
    static_cast<size_t>((height + row_step_ - 1) / row_step_) *
    static_cast<size_t>((width + col_step_ - 1) / col_step_);
  modifier.resize(max_points);

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

  const float scale = static_cast<float>(depth_scale_);
  const float rmin = static_cast<float>(range_min_);
  const float rmax = static_cast<float>(range_max_);
  const float fx = static_cast<float>(intr.fx);
  const float fy = static_cast<float>(intr.fy);
  const float cx = static_cast<float>(intr.cx);
  const float cy = static_cast<float>(intr.cy);

  size_t count = 0;
  for (uint32_t v = 0; v < height; v += row_step_) {
    const uint16_t * depth_row =
      reinterpret_cast<const uint16_t *>(depth.data.data() + static_cast<size_t>(v) * depth.step);
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
      ++it_x; ++it_y; ++it_z;
      ++count;
    }
  }

  modifier.resize(count);  // 실제 유효 포인트 수로 축소(width/row_step 자동 갱신)
  return true;
}

}  // namespace depth_pointcloud_converter
