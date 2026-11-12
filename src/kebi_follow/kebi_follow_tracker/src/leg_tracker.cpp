// kebi_follow_tracker — leg_tracker 컴포넌트 (docs: kebi_follow_me/docs/SPEC.md 6.2)
// /scan 다리 검출 → odom 다중 추적 → cam_legs 융합 → 타깃 획득·유지 → /kebi_follow/target
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kebi_follow_msgs/msg/cam_leg.hpp>
#include <kebi_follow_msgs/msg/cam_leg_array.hpp>
#include <kebi_follow_msgs/msg/follow_mode.hpp>
#include <kebi_follow_msgs/msg/follow_target.hpp>
#include <kebi_follow_msgs/msg/leg_track.hpp>
#include <kebi_follow_msgs/msg/leg_track_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "kebi_follow_tracker/leg_tracker_core.hpp"
#include "kebi_follow_tracker/param_table.hpp"

namespace kebi_follow {

namespace {
using tracker::CameraDetection;
using tracker::Mode;
using tracker::Pose2;
using tracker::TargetStatus;
using tracker::Vec2;

using CamLeg = kebi_follow_msgs::msg::CamLeg;
using CamLegArray = kebi_follow_msgs::msg::CamLegArray;
using FollowMode = kebi_follow_msgs::msg::FollowMode;
using FollowTarget = kebi_follow_msgs::msg::FollowTarget;
using LegTrack = kebi_follow_msgs::msg::LegTrack;
using LegTrackArray = kebi_follow_msgs::msg::LegTrackArray;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr int kWarnThrottleMs = 2000;
constexpr std::size_t kTargetQueueDepth = 5;
constexpr std::size_t kTracksQueueDepth = 5;
constexpr std::size_t kCamLegsQueueDepth = 5;
constexpr std::size_t kMarkerQueueDepth = 5;
constexpr std::size_t kModeQueueDepth = 1;
constexpr std::size_t kReserveTracks = 64;
constexpr std::size_t kReserveMarkers = 256;
constexpr std::size_t kReserveCameraDetections = 16;
constexpr int kZoneArcSteps = 16;  // 획득 영역 호를 그리는 선분 수
constexpr int kCircleSteps = 32;
// 마커 크기·색
constexpr double kTrackMarkerDiameter = 0.25;
constexpr double kTargetMarkerDiameter = 0.4;
constexpr double kMarkerHeight = 0.05;
constexpr double kLegMarkerDiameter = 0.08;
constexpr double kTextHeight = 0.12;
constexpr double kTextOffsetZ = 0.35;
constexpr double kLineWidth = 0.02;
constexpr float kMarkerAlpha = 0.8f;

Pose2 toPose2(const geometry_msgs::msg::TransformStamped& tf) {
  const auto& q = tf.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return {tf.transform.translation.x, tf.transform.translation.y, yaw};
}

void setColor(Marker& m, float r, float g, float b, float a = kMarkerAlpha) {
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
}

geometry_msgs::msg::Point makePoint(double x, double y) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

}  // namespace

class LegTracker : public rclcpp::Node {
 public:
  explicit LegTracker(const rclcpp::NodeOptions& options)
      : Node("leg_tracker", options) {
    loadParameters();
    core_ = std::make_unique<tracker::LegTrackerCore>(params_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    target_pub_ = create_publisher<FollowTarget>(
        target_topic_, rclcpp::QoS(kTargetQueueDepth).reliable());
    tracks_pub_ = create_publisher<LegTrackArray>(
        tracks_topic_, rclcpp::QoS(kTracksQueueDepth).reliable());
    marker_pub_ = create_publisher<MarkerArray>(
        markers_topic_, rclcpp::QoS(kMarkerQueueDepth).best_effort());

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          onScan(*msg);
        });
    cam_sub_ = create_subscription<CamLegArray>(
        cam_legs_topic_, rclcpp::QoS(kCamLegsQueueDepth).reliable(),
        [this](CamLegArray::ConstSharedPtr msg) { onCamLegs(*msg); });
    mode_sub_ = create_subscription<FollowMode>(
        mode_topic_, rclcpp::QoS(kModeQueueDepth).reliable().transient_local(),
        [this](FollowMode::ConstSharedPtr msg) { onMode(msg->mode); });

    tracks_msg_.tracks.reserve(kReserveTracks);
    markers_msg_.markers.reserve(kReserveMarkers);
    camera_dets_.reserve(kReserveCameraDetections);
    if (initial_mode_ != FollowMode::OFF) {
      onMode(static_cast<uint8_t>(initial_mode_));
    }
    RCLCPP_INFO(get_logger(),
                "leg_tracker ready: scan=%s odom=%s require_camera=%s",
                scan_topic_.c_str(), odom_frame_.c_str(),
                params_.acquire.require_camera ? "true" : "false");
  }

 private:
  template <typename T>
  void declareParam(const std::string& name, T& value) {
    value = declare_parameter<T>(name, value);
  }

  void declareParam(const std::string& name, int& value) {
    value = static_cast<int>(declare_parameter<int64_t>(name, value));
  }

  void loadParameters() {
    declareParam("odom_frame", odom_frame_);
    declareParam("base_frame", base_frame_);
    declareParam("scan_topic", scan_topic_);
    declareParam("cam_legs_topic", cam_legs_topic_);
    declareParam("mode_topic", mode_topic_);
    declareParam("target_topic", target_topic_);
    declareParam("tracks_topic", tracks_topic_);
    declareParam("markers_topic", markers_topic_);
    declareParam("publish_markers", publish_markers_);
    declareParam("tf_tolerance_s", tf_tolerance_s_);
    declareParam("timing_log_period_s", timing_log_period_s_);
    declareParam("initial_mode", initial_mode_);

    tracker::visitParams(params_, [this](const char* name, auto& value) {
      declareParam(name, value);
    });
  }

  // odom <- base_link at stamp. 최신 TF 보다 조금 늦은 stamp 는 tf_tolerance_s 까지 최신 값을 쓴다.
  bool lookupRobotPose(const builtin_interfaces::msg::Time& stamp,
                       Pose2& pose) {
    try {
      pose = toPose2(tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                                 tf2_ros::fromMsg(stamp)));
      return true;
    } catch (const tf2::ExtrapolationException&) {
      // 아래에서 최신 값으로 다시 본다
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "TF %s<-%s: %s", odom_frame_.c_str(),
                           base_frame_.c_str(), e.what());
      return false;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_,
                                                  tf2::TimePointZero);
      const double gap =
          (rclcpp::Time(stamp) - rclcpp::Time(tf.header.stamp)).seconds();
      if (std::abs(gap) <= tf_tolerance_s_) {
        pose = toPose2(tf);
        return true;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), kWarnThrottleMs,
          "TF %s<-%s: stamp is %.3f s away from the latest transform",
          odom_frame_.c_str(), base_frame_.c_str(), gap);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "TF %s<-%s: %s", odom_frame_.c_str(),
                           base_frame_.c_str(), e.what());
    }
    return false;
  }

  // base_link <- frame (정적 TF, 처음 한 번 찾아 둔다)
  bool lookupStatic(const std::string& frame, std::string& cached_frame,
                    Pose2& cached_pose) {
    if (frame == cached_frame) {
      return true;
    }
    if (frame.empty() || frame == base_frame_) {
      cached_frame = frame;
      cached_pose = Pose2{};
      return true;
    }
    try {
      cached_pose = toPose2(
          tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero));
      cached_frame = frame;
      return true;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "TF %s<-%s: %s", base_frame_.c_str(), frame.c_str(),
                           e.what());
      return false;
    }
  }

  void onMode(uint8_t mode) {
    Mode next;
    switch (mode) {
      case FollowMode::OFF:
        next = Mode::OFF;
        break;
      case FollowMode::ACQUIRE:
        next = Mode::ACQUIRE;
        break;
      case FollowMode::TRACK:
        next = Mode::TRACK;
        break;
      default:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                             "unknown follow mode %u", mode);
        return;
    }
    if (next != core_->mode()) {
      RCLCPP_INFO(get_logger(), "mode %u -> %u",
                  static_cast<unsigned>(core_->mode()),
                  static_cast<unsigned>(next));
      core_->setMode(next);
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan& msg) {
    Pose2 sensor_in_base;
    Pose2 robot_pose;
    if (!lookupStatic(msg.header.frame_id, scan_frame_, scan_in_base_)) {
      return;
    }
    sensor_in_base = scan_in_base_;
    if (!lookupRobotPose(msg.header.stamp, robot_pose)) {
      return;
    }
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    tracker::ScanView view;
    view.ranges = msg.ranges.data();
    view.count = msg.ranges.size();
    view.angle_min = msg.angle_min;
    view.angle_increment = msg.angle_increment;
    view.range_min = msg.range_min;
    view.range_max = msg.range_max;

    const auto t0 = std::chrono::steady_clock::now();
    const tracker::TargetOutput& out =
        core_->processScan(t, view, sensor_in_base, robot_pose);
    const auto t1 = std::chrono::steady_clock::now();

    publishTarget(msg.header.stamp, robot_pose, out);
    publishTracks(msg.header.stamp);
    if (publish_markers_ && marker_pub_->get_subscription_count() > 0) {
      publishMarkers(msg.header.stamp, robot_pose, out);
    }
    const auto t2 = std::chrono::steady_clock::now();
    recordTiming(std::chrono::duration<double, std::milli>(t1 - t0).count(),
                 std::chrono::duration<double, std::milli>(t2 - t0).count());
  }

  void onCamLegs(const CamLegArray& msg) {
    Pose2 cam_in_base;
    Pose2 robot_pose;
    if (!lookupStatic(msg.header.frame_id, cam_frame_, cam_in_base_)) {
      return;
    }
    cam_in_base = cam_in_base_;
    if (!lookupRobotPose(msg.header.stamp, robot_pose)) {
      return;
    }
    camera_dets_.clear();
    for (std::size_t i = 0; i < msg.detections.size(); ++i) {
      const auto& d = msg.detections[i];
      if (!d.has_position || (d.class_id != CamLeg::CLASS_PERSON &&
                              d.class_id != CamLeg::CLASS_LEG)) {
        continue;
      }
      CameraDetection det;
      det.position = tracker::transformPoint(cam_in_base,
                                             Vec2{d.position.x, d.position.y});
      det.score = d.score;
      det.group = params_.camera.group_legs_with_person ? personGroup(msg, i) : -1;
      det.primary = d.class_id == CamLeg::CLASS_PERSON;
      camera_dets_.push_back(det);
    }
    core_->processCamera(rclcpp::Time(msg.header.stamp).seconds(), robot_pose,
                         camera_dets_);
  }

  // 묶음 번호: 사람 검출은 자기 순번. 다리 검출은 바로 앞 사람 검출의 순번 (검출기는 사람 뒤에 그 사람의
  // 다리를 붙여 낸다) — 다리 상자 가운데가 그 사람 상자 안일 때만. 아니면 -1 (혼자)
  static int personGroup(const CamLegArray& msg, std::size_t index) {
    const CamLeg& d = msg.detections[index];
    if (d.class_id == CamLeg::CLASS_PERSON) {
      return static_cast<int>(index);
    }
    for (std::size_t k = index; k-- > 0;) {
      const CamLeg& p = msg.detections[k];
      if (p.class_id != CamLeg::CLASS_PERSON) {
        continue;
      }
      const double u = 0.5 * (d.u_min + d.u_max);
      const double v = 0.5 * (d.v_min + d.v_max);
      const bool inside = u >= p.u_min && u <= p.u_max && v >= p.v_min && v <= p.v_max;
      return inside ? static_cast<int>(k) : -1;
    }
    return -1;
  }

  void publishTarget(const builtin_interfaces::msg::Time& stamp,
                     const Pose2& robot_pose,
                     const tracker::TargetOutput& out) {
    FollowTarget m;
    m.header.stamp = stamp;
    m.header.frame_id = base_frame_;
    m.status = static_cast<uint8_t>(out.status);
    m.track_id = out.track_id;
    m.ambiguous = out.ambiguous;
    if (out.status != TargetStatus::NONE) {
      const Vec2 p = tracker::inverseTransformPoint(robot_pose, out.position);
      const Vec2 v = tracker::rotate(out.velocity, -robot_pose.yaw);
      m.position.x = p.x;
      m.position.y = p.y;
      m.velocity.x = v.x;
      m.velocity.y = v.y;
      m.range = static_cast<float>(tracker::norm(p));
      m.bearing = static_cast<float>(tracker::bearing(p));
      m.position_std = static_cast<float>(out.position_std);
      m.occluded_time = static_cast<float>(out.occluded_time);
      m.camera_confirmed = out.camera_confirmed;
    }
    target_pub_->publish(m);
  }

  void publishTracks(const builtin_interfaces::msg::Time& stamp) {
    const double t = core_->lastStamp();
    tracks_msg_.header.stamp = stamp;
    tracks_msg_.header.frame_id = odom_frame_;
    tracks_msg_.target_id = core_->lockedId();
    tracks_msg_.tracks.clear();
    for (const auto& tr : core_->tracker().tracks()) {
      if (!tr.confirmed) {
        continue;
      }
      LegTrack lt;
      lt.id = tr.id;
      lt.kind = static_cast<uint8_t>(tr.kind);
      const Vec2 p = tr.kf.position();
      const Vec2 v = tr.kf.velocity();
      lt.position.x = p.x;
      lt.position.y = p.y;
      lt.velocity.x = v.x;
      lt.velocity.y = v.y;
      lt.position_std = static_cast<float>(tr.kf.positionStd());
      lt.person_score = static_cast<float>(tr.personScore(t, params_.camera));
      lt.camera_confirmed =
          tr.cameraConfirmed(t, params_.camera.confirm_window_s);
      lt.age = static_cast<uint32_t>(tr.hits);
      lt.time_since_update = static_cast<float>(t - tr.last_update_time);
      tracks_msg_.tracks.push_back(lt);
    }
    tracks_pub_->publish(tracks_msg_);
  }

  Marker makeMarker(const builtin_interfaces::msg::Time& stamp,
                    const std::string& frame, const std::string& ns, int id,
                    int32_t type) {
    Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  }

  void publishMarkers(const builtin_interfaces::msg::Time& stamp,
                      const Pose2& robot_pose,
                      const tracker::TargetOutput& out) {
    const double t = core_->lastStamp();
    markers_msg_.markers.clear();
    Marker clear;
    clear.action = Marker::DELETEALL;
    markers_msg_.markers.push_back(clear);

    // 다리(파랑)·덩어리(보라) 관측, base_link
    int id = 0;
    for (const auto& seg : core_->detector().segments()) {
      if (!seg.is_leg && !seg.is_blob) {
        continue;
      }
      Marker m =
          makeMarker(stamp, base_frame_, "segments", id++, Marker::SPHERE);
      m.pose.position.x = seg.center.x;
      m.pose.position.y = seg.center.y;
      m.scale.x = m.scale.y = m.scale.z =
          seg.is_blob ? seg.width : kLegMarkerDiameter;
      if (seg.is_blob) {
        setColor(m, 0.7f, 0.2f, 0.9f);
      } else {
        setColor(m, 0.2f, 0.4f, 1.0f);
      }
      markers_msg_.markers.push_back(m);
    }

    // 확정 트랙 (사람 같으면 초록, 아니면 회색), odom
    const int locked = core_->lockedId();
    id = 0;
    for (const auto& tr : core_->tracker().tracks()) {
      if (!tr.confirmed) {
        continue;
      }
      const Vec2 p = tr.kf.position();
      const double score = tr.personScore(t, params_.camera);
      const bool is_target =
          tr.id == out.track_id && out.status != TargetStatus::NONE;
      Marker m = makeMarker(stamp, odom_frame_, "tracks", id, Marker::CYLINDER);
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.scale.x = m.scale.y =
          is_target ? kTargetMarkerDiameter : kTrackMarkerDiameter;
      m.scale.z = kMarkerHeight;
      if (tr.id == locked) {
        setColor(m, 1.0f, 0.1f, 0.1f);
      } else if (is_target) {
        setColor(m, 1.0f, 0.7f, 0.0f);
      } else if (score >= params_.ambiguity.min_person_score) {
        setColor(m, 0.1f, 0.9f, 0.2f);
      } else {
        setColor(m, 0.6f, 0.6f, 0.6f);
      }
      markers_msg_.markers.push_back(m);

      Marker text = makeMarker(stamp, odom_frame_, "track_text", id,
                               Marker::TEXT_VIEW_FACING);
      text.pose.position.x = p.x;
      text.pose.position.y = p.y;
      text.pose.position.z = kTextOffsetZ;
      text.scale.z = kTextHeight;
      setColor(text, 1.0f, 1.0f, 1.0f, 1.0f);
      char buf[48];
      std::snprintf(
          buf, sizeof(buf), "%d %.2f%s", tr.id, score,
          tr.cameraConfirmed(t, params_.camera.confirm_window_s) ? " C" : "");
      text.text = buf;
      markers_msg_.markers.push_back(text);
      ++id;
    }

    // 획득 영역 (ACQUIRE), base_link
    if (core_->mode() == Mode::ACQUIRE) {
      const auto& ap = params_.acquire;
      Marker zone =
          makeMarker(stamp, base_frame_, "acquire_zone", 0, Marker::LINE_STRIP);
      zone.scale.x = kLineWidth;
      setColor(zone, 1.0f, 1.0f, 0.0f);
      const double b = ap.max_bearing_deg * tracker::kDegToRad;
      for (int k = 0; k <= kZoneArcSteps; ++k) {
        const double a = -b + 2.0 * b * k / kZoneArcSteps;
        zone.points.push_back(
            makePoint(ap.min_range * std::cos(a), ap.min_range * std::sin(a)));
      }
      for (int k = kZoneArcSteps; k >= 0; --k) {
        const double a = -b + 2.0 * b * k / kZoneArcSteps;
        zone.points.push_back(
            makePoint(ap.max_range * std::cos(a), ap.max_range * std::sin(a)));
      }
      zone.points.push_back(zone.points.front());
      markers_msg_.markers.push_back(zone);
    }

    // LOST: 재획득 영역 (마지막 관측 위치·예측 위치 주변 원), odom
    if (out.has_lost_position) {
      const double r = params_.target.reacquire_radius;
      const Vec2 centers[2] = {out.lost_position, out.lost_seen_position};
      for (int c = 0; c < 2; ++c) {
        Marker circle =
            makeMarker(stamp, odom_frame_, "reacquire", c, Marker::LINE_STRIP);
        circle.scale.x = kLineWidth;
        setColor(circle, 1.0f, 0.4f, 0.0f);
        for (int k = 0; k <= kCircleSteps; ++k) {
          const double a = tracker::kTwoPi * k / kCircleSteps;
          circle.points.push_back(makePoint(centers[c].x + r * std::cos(a),
                                            centers[c].y + r * std::sin(a)));
        }
        markers_msg_.markers.push_back(circle);
      }
    }
    (void)robot_pose;
    marker_pub_->publish(markers_msg_);
  }

  void recordTiming(double core_ms, double total_ms) {
    ++timing_count_;
    timing_core_sum_ += core_ms;
    timing_core_max_ = std::max(timing_core_max_, core_ms);
    timing_total_max_ = std::max(timing_total_max_, total_ms);
    if (timing_log_period_s_ <= 0.0) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (timing_count_ == 1) {
      timing_start_ = now;
      return;
    }
    if (std::chrono::duration<double>(now - timing_start_).count() >=
        timing_log_period_s_) {
      RCLCPP_INFO(get_logger(),
                  "scan processing: core mean %.3f ms max %.3f ms, with "
                  "publish max %.3f ms "
                  "(%d scans), tracks %zu",
                  timing_core_sum_ / timing_count_, timing_core_max_,
                  timing_total_max_, timing_count_,
                  core_->tracker().tracks().size());
      timing_count_ = 0;
      timing_core_sum_ = 0.0;
      timing_core_max_ = 0.0;
      timing_total_max_ = 0.0;
    }
  }

  // 파라미터
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string scan_topic_{"/scan"};
  std::string cam_legs_topic_{"/kebi_follow/cam_legs"};
  std::string mode_topic_{"/kebi_follow/mode"};
  std::string target_topic_{"/kebi_follow/target"};
  std::string tracks_topic_{"/kebi_follow/tracks"};
  std::string markers_topic_{"/kebi_follow/leg_tracker/markers"};
  bool publish_markers_{true};
  double tf_tolerance_s_{0.05};
  double timing_log_period_s_{60.0};
  int initial_mode_{FollowMode::OFF};
  tracker::TrackerParams params_;

  std::unique_ptr<tracker::LegTrackerCore> core_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // 아직 찾지 않음 (프레임 이름에 쓰이지 않는 값)
  std::string scan_frame_{"\x01"};
  Pose2 scan_in_base_;
  std::string cam_frame_{"\x01"};
  Pose2 cam_in_base_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<CamLegArray>::SharedPtr cam_sub_;
  rclcpp::Subscription<FollowMode>::SharedPtr mode_sub_;
  rclcpp::Publisher<FollowTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<LegTrackArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  LegTrackArray tracks_msg_;
  MarkerArray markers_msg_;
  std::vector<CameraDetection> camera_dets_;

  int timing_count_{0};
  double timing_core_sum_{0.0};
  double timing_core_max_{0.0};
  double timing_total_max_{0.0};
  std::chrono::steady_clock::time_point timing_start_;
};

}  // namespace kebi_follow

RCLCPP_COMPONENTS_REGISTER_NODE(kebi_follow::LegTracker)
