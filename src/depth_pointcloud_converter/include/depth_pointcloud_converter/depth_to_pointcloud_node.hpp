#pragma once

// InuSensor depth 이미지(sensor_msgs/Image, 16UC1, mm) + CameraInfo 를
// sensor_msgs/PointCloud2 (XYZ) 로 변환해 발행하는 독립 노드.
//
// 구조 원칙
//  - 구독 콜백은 "저장만" 한다(버퍼에 최신 프레임 보관). 무거운 변환은 하지 않는다.
//  - 별도의 timer 가 publish_rate_hz 주기로 최신 프레임을 꺼내 변환·발행한다.
//    → 큰 depth 데이터 변환이 DDS 콜백 스레드를 막지 않도록 분리.
//  - 입력 QoS: SensorDataQoS(best_effort) → 드라이버 best_effort 발행과 매칭.
//  - 출력 QoS: 파라미터(output_reliability)로 선택. 기본 reliable(RViz 기본값과 호환).
//
// 드라이버(inusensor_ros2_driver) 코드는 전혀 수정하지 않는다.

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace depth_pointcloud_converter
{

// 카메라 핀홀 내부 파라미터(CameraInfo K 에서 추출).
struct CameraIntrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  bool valid = false;
};

class DepthToPointCloudNode : public rclcpp::Node
{
public:
  explicit DepthToPointCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- 초기화 ----
  void declareParameters();
  void setupCommunications();

  // ---- 구독 콜백 (저장만) ----
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // ---- timer (변환·발행) ----
  void publishTimerCallback();

  // depth 이미지 한 장을 PointCloud2 로 변환. 성공 시 true.
  // (향후 후처리 필터를 끼울 경우 이 결과를 입력으로 받는 단계를 추가하면 됨)
  bool buildPointCloud(const sensor_msgs::msg::Image & depth,
                       const CameraIntrinsics & intr,
                       sensor_msgs::msg::PointCloud2 & cloud_out) const;

  // ---- 파라미터 ----
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string output_frame_;        // 비우면 depth 이미지 frame_id 사용
  std::string output_reliability_;  // "reliable" | "best_effort"
  double depth_scale_ = 0.001;      // 16UC1(mm) -> m
  double range_min_ = 0.1;
  double range_max_ = 5.0;
  int row_step_ = 1;
  int col_step_ = 1;
  double publish_rate_hz_ = 15.0;

  // ---- 최신 depth 프레임 버퍼 (콜백이 저장, timer 가 소비) ----
  std::mutex depth_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  rclcpp::Time depth_recv_time_;
  rclcpp::Time last_published_time_;

  // ---- 최신 카메라 내부 파라미터 ----
  std::mutex info_mutex_;
  CameraIntrinsics intrinsics_;

  // ---- ROS 엔티티 ----
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace depth_pointcloud_converter
