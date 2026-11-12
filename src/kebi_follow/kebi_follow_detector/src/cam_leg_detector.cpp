// cam_leg_detector: YOLO person/leg detection (RK3588 NPU or OpenCV DNN) + depth -> CamLegArray.
// SPEC: kebi_follow_me/docs/SPEC.md sections 3, 4, 6.1.
//
// - Camera inputs (RGB, depth, camera_infos, /odom) are subscribed only while /kebi_follow/mode != OFF.
// - Executor callbacks only store messages. The latest RGB frame goes into a one-slot mailbox and a
//   dedicated worker thread runs inference + depth fusion and publishes; older frames are dropped.
// - Output stamps are moved to the robot clock: stamp_cam + offset - camera_min_latency_s, offset =
//   min over a window of (latest /odom stamp at image arrival - image stamp) (time_offset.hpp).
// - One CamLegArray per processed frame (also when empty). Each CLASS_PERSON entry is followed by
//   CLASS_LEG entries for its legs that have a depth position (pose model).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kebi_follow_msgs/msg/cam_leg.hpp>
#include <kebi_follow_msgs/msg/cam_leg_array.hpp>
#include <kebi_follow_msgs/msg/follow_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "kebi_follow_detector/depth_fusion.hpp"
#include "kebi_follow_detector/inference_backend.hpp"
#include "kebi_follow_detector/time_offset.hpp"
#include "kebi_follow_detector/yolo_decode.hpp"

namespace kebi_follow
{

namespace
{

using Clock = std::chrono::steady_clock;

constexpr char kRgbTopic[] = "/camera/rgb/image_raw";
constexpr char kRgbInfoTopic[] = "/camera/rgb/camera_info";
constexpr char kDepthTopic[] = "/camera/depth/image_raw";
constexpr char kDepthInfoTopic[] = "/camera/depth/camera_info";
constexpr char kOdomTopic[] = "/odom";
constexpr char kModeTopic[] = "/kebi_follow/mode";
constexpr char kOutputTopic[] = "/kebi_follow/cam_legs";
constexpr char kDebugImageTopic[] = "/kebi_follow/debug_image";
constexpr char kMarkerTopic[] = "/kebi_follow/cam_leg_detector/markers";
constexpr char kTfStaticTopic[] = "/tf_static";

constexpr int kOutputQueueDepth = 5;
constexpr int kTfStaticQueueDepth = 100;
constexpr size_t kDepthBufferSize = 5;
constexpr int kWarnThrottleMs = 5000;
constexpr double kGeometryRetryS = 1.0;
constexpr int kLetterboxPad = 114;
constexpr int kTfFallbackSize = 6;  // x y z roll pitch yaw
constexpr double kMarkerLifetimeS = 0.5;

double stamp_seconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time to_stamp(double seconds)
{
  builtin_interfaces::msg::Time t;
  const double sec = std::floor(seconds);
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(std::min(999999999.0, std::round((seconds - sec) * 1e9)));
  return t;
}

double ms_between(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

double thread_cpu_ms()
{
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

CameraModel camera_model(const sensor_msgs::msg::CameraInfo & info)
{
  CameraModel m;
  m.width = static_cast<int>(info.width);
  m.height = static_cast<int>(info.height);
  m.fx = info.k[0];
  m.fy = info.k[4];
  m.cx = info.k[2];
  m.cy = info.k[5];
  if ((info.distortion_model == "plumb_bob" || info.distortion_model == "rational_polynomial") &&
    info.d.size() >= m.dist.size())
  {
    for (size_t i = 0; i < m.dist.size(); ++i) {
      m.dist[i] = info.d[i];
    }
  }
  return m;
}

bool same_info(const sensor_msgs::msg::CameraInfo & a, const sensor_msgs::msg::CameraInfo & b)
{
  return a.width == b.width && a.height == b.height && a.k == b.k && a.d == b.d &&
         a.header.frame_id == b.header.frame_id;
}

Rigid3 to_rigid(const geometry_msgs::msg::TransformStamped & t)
{
  const auto & q = t.transform.rotation;
  const auto & p = t.transform.translation;
  return Rigid3::from_quaternion(q.x, q.y, q.z, q.w, p.x, p.y, p.z);
}

}  // namespace

class CamLegDetector : public rclcpp::Node
{
public:
  explicit CamLegDetector(const rclcpp::NodeOptions & options)
  : Node("cam_leg_detector", options)
  {
    load_parameters();
    // static transforms only (TimePointZero): a steady clock keeps sim-time jumps (bag replay) from
    // clearing the buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      kTfStaticTopic, rclcpp::QoS(kTfStaticQueueDepth).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {on_tf_static(msg);});
    legs_pub_ = create_publisher<kebi_follow_msgs::msg::CamLegArray>(
      kOutputTopic, rclcpp::QoS(kOutputQueueDepth).reliable());
    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        kDebugImageTopic, rclcpp::QoS(1).best_effort());
    }
    if (publish_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        kMarkerTopic, rclcpp::QoS(1).best_effort());
    }

    std::string error;
    backend_ = create_backend(backend_config_, error);
    if (!backend_) {
      RCLCPP_ERROR(get_logger(), "inference backend not available: %s. Detector stays idle.",
        error.c_str());
    } else {
      const BackendInfo & bi = backend_->info();
      RCLCPP_INFO(get_logger(), "backend %s (%s), model %s [%s], input %dx%d",
        bi.name.c_str(), bi.detail.c_str(), backend_config_.model_path.c_str(),
        backend_config_.model_type.c_str(), bi.input_width, bi.input_height);
      worker_ = std::thread([this]() {worker_loop();});
    }
    mode_sub_ = create_subscription<kebi_follow_msgs::msg::FollowMode>(
      kModeTopic, rclcpp::QoS(1).reliable().transient_local(),
      [this](kebi_follow_msgs::msg::FollowMode::ConstSharedPtr msg) {on_mode(*msg);});
    if (ignore_mode_) {
      set_active(true);
    }
  }

  ~CamLegDetector() override
  {
    set_active(false);
    {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      stop_ = true;
    }
    mailbox_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  struct Frame
  {
    Image::ConstSharedPtr image;
    double stamp_out{0.0};
    Clock::time_point arrival;
  };

  struct Letterbox
  {
    double scale{1.0};
    double pad_x{0.0};
    double pad_y{0.0};
  };

  // ------------------------------------------------------------------------------------- parameters
  void load_parameters()
  {
    backend_config_.backend = declare_parameter<std::string>("backend", "auto");
    backend_config_.model_path = declare_parameter<std::string>("model_path", "");
    backend_config_.model_type = declare_parameter<std::string>("model_type", "pose");
    backend_config_.rknn_core_mask = declare_parameter<std::string>("rknn_core_mask", "auto");
    backend_config_.input_width = static_cast<int>(declare_parameter<int64_t>("model_input_width", 640));
    backend_config_.input_height = static_cast<int>(declare_parameter<int64_t>("model_input_height", 480));
    backend_config_.cpu_threads = static_cast<int>(declare_parameter<int64_t>("cpu_threads", 0));

    decode_config_.score_threshold = static_cast<float>(declare_parameter<double>("score_threshold", 0.35));
    decode_config_.nms_iou = static_cast<float>(declare_parameter<double>("nms_iou", 0.45));
    decode_config_.max_detections = static_cast<int>(declare_parameter<int64_t>("max_detections", 10));
    min_box_height_px_ = declare_parameter<double>("min_box_height_px", 30.0);

    ignore_mode_ = declare_parameter<bool>("ignore_mode", false);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    camera_min_latency_s_ = declare_parameter<double>("camera_min_latency_s", 0.10);
    offset_window_s_ = declare_parameter<double>("time_offset_window_s", 5.0);
    odom_max_age_s_ = declare_parameter<double>("odom_max_age_s", 0.5);
    depth_max_dt_s_ = declare_parameter<double>("depth_max_dt_s", 0.1);
    pitch_correction_rad_ = declare_parameter<double>("camera_pitch_correction_deg", 10.5) * M_PI / 180.0;
    tf_fallback_base_to_rgb_ =
      declare_parameter<std::vector<double>>("tf_fallback.base_to_rgb", std::vector<double>{});
    tf_fallback_rgb_to_depth_ =
      declare_parameter<std::vector<double>>("tf_fallback.rgb_to_depth", std::vector<double>{});

    FusionConfig & f = fusion_config_;
    f.depth_stride = static_cast<int>(declare_parameter<int64_t>("depth.stride", f.depth_stride));
    f.depth_unit_m = declare_parameter<double>("depth.unit_m", f.depth_unit_m);
    f.depth_min_m = declare_parameter<double>("depth.min_m", f.depth_min_m);
    f.depth_max_m = declare_parameter<double>("depth.max_m", f.depth_max_m);
    f.floor_height_m = declare_parameter<double>("fusion.floor_height_m", f.floor_height_m);
    f.max_height_m = declare_parameter<double>("fusion.max_height_m", f.max_height_m);
    f.robot_exclusion_radius_m =
      declare_parameter<double>("fusion.robot_exclusion_radius_m", f.robot_exclusion_radius_m);
    f.leg_region_fraction = declare_parameter<double>("fusion.leg_region_fraction", f.leg_region_fraction);
    f.leg_region_width_fraction =
      declare_parameter<double>("fusion.leg_region_width_fraction", f.leg_region_width_fraction);
    f.keypoint_vis_threshold =
      declare_parameter<double>("keypoint_vis_threshold", f.keypoint_vis_threshold);
    f.keypoint_margin_ratio = declare_parameter<double>("fusion.keypoint_margin_ratio", f.keypoint_margin_ratio);
    f.leg_half_width_ratio = declare_parameter<double>("fusion.leg_half_width_ratio", f.leg_half_width_ratio);
    f.leg_min_half_width_px =
      declare_parameter<double>("fusion.leg_min_half_width_px", f.leg_min_half_width_px);
    f.leg_foot_extension_ratio =
      declare_parameter<double>("fusion.leg_foot_extension_ratio", f.leg_foot_extension_ratio);
    f.cluster_window_m = declare_parameter<double>("fusion.cluster_window_m", f.cluster_window_m);
    f.cluster_min_points =
      static_cast<int>(declare_parameter<int64_t>("fusion.cluster_min_points", f.cluster_min_points));
    f.cluster_min_fraction = declare_parameter<double>("fusion.cluster_min_fraction", f.cluster_min_fraction);
    f.person_cluster_extent_m =
      declare_parameter<double>("fusion.person_cluster_extent_m", f.person_cluster_extent_m);
    f.leg_cluster_extent_m = declare_parameter<double>("fusion.leg_cluster_extent_m", f.leg_cluster_extent_m);
    f.depth_noise_k = declare_parameter<double>("fusion.depth_noise_k", f.depth_noise_k);
    f.spread_gain = declare_parameter<double>("fusion.spread_gain", f.spread_gain);
    f.position_std_min_m = declare_parameter<double>("fusion.position_std_min_m", f.position_std_min_m);
    f.position_std_max_m = declare_parameter<double>("fusion.position_std_max_m", f.position_std_max_m);
    f.appearance = declare_parameter<bool>("appearance.enable", f.appearance);
    f.hue_bins = static_cast<int>(declare_parameter<int64_t>("appearance.hue_bins", f.hue_bins));
    f.sat_bins = static_cast<int>(declare_parameter<int64_t>("appearance.sat_bins", f.sat_bins));
    f.gray_bins = static_cast<int>(declare_parameter<int64_t>("appearance.gray_bins", f.gray_bins));
    fusion_ = DepthFusion(fusion_config_);
    time_offset_ = CameraTimeOffset(offset_window_s_);

    publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);
    debug_max_rate_hz_ = declare_parameter<double>("debug_image_max_rate_hz", 5.0);
    jpeg_quality_ = static_cast<int>(declare_parameter<int64_t>("debug_jpeg_quality", 70));
    publish_markers_ = declare_parameter<bool>("publish_markers", false);
    stats_period_s_ = declare_parameter<double>("stats_period_s", 10.0);
  }

  // ------------------------------------------------------------------------- mode / subscriptions
  void on_mode(const kebi_follow_msgs::msg::FollowMode & msg)
  {
    set_active(ignore_mode_ || msg.mode != kebi_follow_msgs::msg::FollowMode::OFF);
  }

  void set_active(bool active)
  {
    active = active && backend_ != nullptr;
    if (active == active_) {
      return;
    }
    active_ = active;
    if (active) {
      const auto qos = rclcpp::SensorDataQoS();
      rgb_sub_ = create_subscription<Image>(
        kRgbTopic, qos, [this](Image::ConstSharedPtr m) {on_rgb(std::move(m));});
      depth_sub_ = create_subscription<Image>(
        kDepthTopic, qos, [this](Image::ConstSharedPtr m) {on_depth(std::move(m));});
      rgb_info_sub_ = create_subscription<CameraInfo>(
        kRgbInfoTopic, qos, [this](CameraInfo::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          rgb_info_ = std::move(m);
        });
      depth_info_sub_ = create_subscription<CameraInfo>(
        kDepthInfoTopic, qos, [this](CameraInfo::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          depth_info_ = std::move(m);
        });
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kOdomTopic, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          latest_odom_stamp_ = stamp_seconds(m->header.stamp);
          latest_odom_arrival_ = Clock::now();
          have_odom_ = true;
        });
      RCLCPP_INFO(get_logger(), "camera pipeline on");
    } else {
      rgb_sub_.reset();
      depth_sub_.reset();
      rgb_info_sub_.reset();
      depth_info_sub_.reset();
      odom_sub_.reset();
      have_odom_ = false;
      time_offset_.reset();
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        depth_buffer_.clear();
      }
      {
        std::lock_guard<std::mutex> lock(mailbox_mutex_);
        pending_.reset();
      }
      RCLCPP_INFO(get_logger(), "camera pipeline off");
    }
  }

  // -------------------------------------------------------------------------------- input callbacks
  void on_rgb(Image::ConstSharedPtr msg)
  {
    const Clock::time_point now = Clock::now();
    const double cam = stamp_seconds(msg->header.stamp);
    if (have_odom_ && std::chrono::duration<double>(now - latest_odom_arrival_).count() <= odom_max_age_s_) {
      time_offset_.add(latest_odom_stamp_, cam);
    }
    last_offset_.store(time_offset_.offset());
    if (!time_offset_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
        "no recent %s: cannot convert camera stamps to the robot clock, frame dropped", kOdomTopic);
      return;
    }
    Frame f;
    f.image = std::move(msg);
    f.stamp_out = time_offset_.to_robot_time(cam, camera_min_latency_s_);
    f.arrival = now;
    {
      std::lock_guard<std::mutex> lock(mailbox_mutex_);
      if (pending_) {
        ++dropped_frames_;
      }
      pending_ = std::move(f);
    }
    mailbox_cv_.notify_one();
  }

  void on_depth(Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    depth_buffer_.push_back(std::move(msg));
    while (depth_buffer_.size() > kDepthBufferSize) {
      depth_buffer_.pop_front();
    }
  }

  void on_tf_static(tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
  {
    for (const auto & t : msg->transforms) {
      try {
        tf_buffer_->setTransform(t, "tf_static", true);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "bad static transform: %s", e.what());
      }
    }
  }

  // ----------------------------------------------------------------------------------------- worker
  void worker_loop()
  {
    stats_start_ = Clock::now();
    while (true) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mailbox_mutex_);
        mailbox_cv_.wait(lock, [this]() {return stop_ || pending_.has_value();});
        if (stop_) {
          return;
        }
        frame = std::move(*pending_);
        pending_.reset();
      }
      const double cpu0 = thread_cpu_ms();
      process(frame);
      stat_cpu_ms_ += thread_cpu_ms() - cpu0;
      log_stats();
    }
  }

  void process(const Frame & frame)
  {
    const Image & img = *frame.image;
    int cv_type = 0;
    bool is_bgr = true;
    if (img.encoding == "bgr8") {
      cv_type = CV_8UC3;
    } else if (img.encoding == "rgb8") {
      cv_type = CV_8UC3;
      is_bgr = false;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
        "unsupported RGB encoding '%s' (bgr8 / rgb8)", img.encoding.c_str());
      return;
    }
    const cv::Mat color(static_cast<int>(img.height), static_cast<int>(img.width), cv_type,
      const_cast<uint8_t *>(img.data.data()), img.step);

    // preprocessing: RGB at the model input size (letterbox only if the sizes differ)
    const BackendInfo & bi = backend_->info();
    Letterbox lb;
    if (color.cols == bi.input_width && color.rows == bi.input_height) {
      if (is_bgr) {
        cv::cvtColor(color, input_rgb_, cv::COLOR_BGR2RGB);
      } else {
        color.copyTo(input_rgb_);
      }
    } else {
      lb.scale = std::min(static_cast<double>(bi.input_width) / color.cols,
        static_cast<double>(bi.input_height) / color.rows);
      const int w = static_cast<int>(std::round(color.cols * lb.scale));
      const int h = static_cast<int>(std::round(color.rows * lb.scale));
      lb.pad_x = (bi.input_width - w) / 2;
      lb.pad_y = (bi.input_height - h) / 2;
      cv::Mat resized;
      cv::resize(color, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
      input_rgb_.create(bi.input_height, bi.input_width, CV_8UC3);
      input_rgb_.setTo(cv::Scalar::all(kLetterboxPad));
      cv::Mat roi = input_rgb_(cv::Rect(static_cast<int>(lb.pad_x), static_cast<int>(lb.pad_y), w, h));
      if (is_bgr) {
        cv::cvtColor(resized, roi, cv::COLOR_BGR2RGB);
      } else {
        resized.copyTo(roi);
      }
    }

    // inference + decode
    const Clock::time_point t_inf = Clock::now();
    std::vector<TensorView> outputs;
    std::string error;
    if (!backend_->infer(input_rgb_.ptr<uint8_t>(), outputs, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "%s", error.c_str());
      return;
    }
    const double inference_ms = ms_between(t_inf, Clock::now());
    if (!layout_checked_) {
      const std::string bad = check_output_layout(outputs, bi.outputs_per_level, bi.input_width,
          bi.input_height);
      if (!bad.empty()) {
        RCLCPP_ERROR(get_logger(), "model output layout mismatch: %s", bad.c_str());
        return;
      }
      layout_checked_ = true;
    }
    std::vector<Detection> dets = decode_yolo(outputs, bi.outputs_per_level, bi.input_width,
        decode_config_);
    std::vector<Detection> kept;
    for (Detection d : dets) {
      if (lb.scale != 1.0 || lb.pad_x != 0.0 || lb.pad_y != 0.0) {
        const auto unmap = [&lb](float & x, float & y) {
            x = static_cast<float>((x - lb.pad_x) / lb.scale);
            y = static_cast<float>((y - lb.pad_y) / lb.scale);
          };
        unmap(d.x1, d.y1);
        unmap(d.x2, d.y2);
        for (Keypoint & k : d.keypoints) {
          unmap(k.x, k.y);
        }
      }
      d.x1 = std::clamp(d.x1, 0.0f, static_cast<float>(color.cols));
      d.x2 = std::clamp(d.x2, 0.0f, static_cast<float>(color.cols));
      d.y1 = std::clamp(d.y1, 0.0f, static_cast<float>(color.rows));
      d.y2 = std::clamp(d.y2, 0.0f, static_cast<float>(color.rows));
      if (d.y2 - d.y1 >= min_box_height_px_) {
        kept.push_back(d);
      }
    }

    // depth fusion
    std::vector<PersonEstimate> est(kept.size());
    bool depth_used = false;
    if (!kept.empty()) {
      const Image::ConstSharedPtr depth = match_depth(stamp_seconds(img.header.stamp));
      if (depth && update_geometry(img.header.frame_id, depth->header.frame_id)) {
        int dtype = -1;
        if (depth->encoding == "16UC1" || depth->encoding == "mono16") {
          dtype = CV_16UC1;
        } else if (depth->encoding == "32FC1") {
          dtype = CV_32FC1;
        }
        if (dtype >= 0) {
          const cv::Mat dm(static_cast<int>(depth->height), static_cast<int>(depth->width), dtype,
            const_cast<uint8_t *>(depth->data.data()), depth->step);
          est = fusion_.fuse(dm, color, is_bgr, kept);
          depth_used = true;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
            "unsupported depth encoding '%s'", depth->encoding.c_str());
        }
      }
    }

    // output
    kebi_follow_msgs::msg::CamLegArray out;
    out.header.stamp = to_stamp(frame.stamp_out);
    out.header.frame_id = base_frame_;
    out.image_width = img.width;
    out.image_height = img.height;
    for (size_t i = 0; i < kept.size(); ++i) {
      const Detection & d = kept[i];
      const PersonEstimate & e = est[i];
      kebi_follow_msgs::msg::CamLeg p;
      p.class_id = kebi_follow_msgs::msg::CamLeg::CLASS_PERSON;
      p.score = d.score;
      p.u_min = d.x1;
      p.v_min = d.y1;
      p.u_max = d.x2;
      p.v_max = d.y2;
      p.has_position = e.has_position;
      p.position.x = e.x;
      p.position.y = e.y;
      p.position.z = e.z;
      p.position_std = static_cast<float>(e.position_std);
      p.depth_valid_ratio = static_cast<float>(e.valid_ratio);
      p.appearance = e.appearance;
      out.detections.push_back(std::move(p));
      // 사람 바로 뒤에 그 사람의 다리를 붙인다. leg_tracker 가 이 순서로 한 사람 검출을 묶는다
      for (const LegEstimate & leg : e.legs) {
        if (!leg.valid) {
          continue;
        }
        kebi_follow_msgs::msg::CamLeg l;
        l.class_id = kebi_follow_msgs::msg::CamLeg::CLASS_LEG;
        l.score = leg.score;
        l.u_min = leg.u_min;
        l.v_min = leg.v_min;
        l.u_max = leg.u_max;
        l.v_max = leg.v_max;
        l.has_position = true;
        l.position.x = leg.x;
        l.position.y = leg.y;
        l.position.z = leg.z;
        l.position_std = static_cast<float>(leg.position_std);
        l.depth_valid_ratio = static_cast<float>(leg.valid_ratio);
        out.detections.push_back(std::move(l));
      }
    }
    out.inference_ms = static_cast<float>(inference_ms);
    out.total_ms = static_cast<float>(ms_between(frame.arrival, Clock::now()));
    legs_pub_->publish(out);

    ++stat_frames_;
    stat_infer_ms_ += inference_ms;
    stat_infer_max_ms_ = std::max(stat_infer_max_ms_, inference_ms);
    stat_total_ms_ += out.total_ms;
    stat_total_max_ms_ = std::max(stat_total_max_ms_, static_cast<double>(out.total_ms));
    stat_persons_ += kept.size();
    stat_depth_frames_ += depth_used ? 1 : 0;

    if (debug_pub_) {
      publish_debug_image(color, is_bgr, img.header.frame_id, kept, est, out);
    }
    if (marker_pub_) {
      publish_markers(out);
    }
  }

  Image::ConstSharedPtr match_depth(double camera_stamp)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    Image::ConstSharedPtr best;
    double best_dt = depth_max_dt_s_;
    for (const auto & d : depth_buffer_) {
      const double dt = std::abs(stamp_seconds(d->header.stamp) - camera_stamp);
      if (dt <= best_dt) {
        best_dt = dt;
        best = d;
      }
    }
    if (!best) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
        "no depth frame within %.3f s of the RGB frame", depth_max_dt_s_);
    }
    return best;
  }

  bool lookup(const std::string & target, const std::string & source,
    const std::vector<double> & fallback, Rigid3 & out)
  {
    try {
      out = to_rigid(tf_buffer_->lookupTransform(target, source, tf2::TimePointZero));
      return true;
    } catch (const tf2::TransformException & e) {
      if (fallback.size() == kTfFallbackSize) {
        out = Rigid3::from_xyz_rpy(fallback[0], fallback[1], fallback[2], fallback[3], fallback[4],
            fallback[5]);
        RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
          "TF %s <- %s unavailable (%s): using the tf_fallback parameter", target.c_str(),
          source.c_str(), e.what());
        return true;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
        "TF %s <- %s unavailable: %s", target.c_str(), source.c_str(), e.what());
      return false;
    }
  }

  // camera models + static TF -> DepthFusion geometry (recomputed only when something changes)
  bool update_geometry(const std::string & rgb_frame, const std::string & depth_frame)
  {
    CameraInfo::ConstSharedPtr rgb_info;
    CameraInfo::ConstSharedPtr depth_info;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      rgb_info = rgb_info_;
      depth_info = depth_info_;
    }
    if (!rgb_info || !depth_info) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
        "waiting for camera_info (rgb %s, depth %s)", rgb_info ? "ok" : "missing",
        depth_info ? "ok" : "missing");
      return false;
    }
    const bool same = geometry_ok_ && geom_rgb_frame_ == rgb_frame &&
      geom_depth_frame_ == depth_frame && same_info(*rgb_info, geom_rgb_info_) &&
      same_info(*depth_info, geom_depth_info_);
    if (same) {
      return true;
    }
    const Clock::time_point now = Clock::now();
    if (!geometry_ok_ && std::chrono::duration<double>(now - last_geometry_try_).count() < kGeometryRetryS) {
      return false;
    }
    last_geometry_try_ = now;
    Rigid3 rgb_from_depth;
    Rigid3 base_from_rgb;
    if (!lookup(rgb_frame, depth_frame, tf_fallback_rgb_to_depth_, rgb_from_depth) ||
      !lookup(base_frame_, rgb_frame, tf_fallback_base_to_rgb_, base_from_rgb))
    {
      geometry_ok_ = false;
      return false;
    }
    // URDF pitch is 0 deg but the camera looks ~9-10.5 deg down (SPEC 3): rotate about base_link y at the
    // camera origin, positive = looking more downward
    Rigid3 corrected = Rigid3::rotation_y(pitch_correction_rad_) * base_from_rgb;
    corrected.t = base_from_rgb.t;
    fusion_.set_geometry(camera_model(*rgb_info), camera_model(*depth_info), rgb_from_depth, corrected);
    geometry_ok_ = fusion_.ready();
    geom_rgb_frame_ = rgb_frame;
    geom_depth_frame_ = depth_frame;
    geom_rgb_info_ = *rgb_info;
    geom_depth_info_ = *depth_info;
    if (geometry_ok_) {
      RCLCPP_INFO(get_logger(), "depth fusion geometry ready (%s <- %s, %s <- %s, pitch corr %.1f deg)",
        rgb_frame.c_str(), depth_frame.c_str(), base_frame_.c_str(), rgb_frame.c_str(),
        pitch_correction_rad_ * 180.0 / M_PI);
    }
    return geometry_ok_;
  }

  // ------------------------------------------------------------------------------------------ debug
  void publish_debug_image(
    const cv::Mat & color, bool is_bgr, const std::string & frame_id,
    const std::vector<Detection> & dets,
    const std::vector<PersonEstimate> & est, const kebi_follow_msgs::msg::CamLegArray & out)
  {
    const Clock::time_point now = Clock::now();
    if (debug_max_rate_hz_ > 0.0 &&
      std::chrono::duration<double>(now - last_debug_).count() < 1.0 / debug_max_rate_hz_)
    {
      return;
    }
    last_debug_ = now;
    cv::Mat canvas;
    if (is_bgr) {
      canvas = color.clone();
    } else {
      cv::cvtColor(color, canvas, cv::COLOR_RGB2BGR);
    }
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar magenta(255, 0, 255);
    const cv::Scalar red(0, 0, 255);
    for (size_t i = 0; i < dets.size(); ++i) {
      const Detection & d = dets[i];
      cv::rectangle(canvas, cv::Point2f(d.x1, d.y1), cv::Point2f(d.x2, d.y2), green, 2);
      char text[64];
      if (est[i].has_position) {
        std::snprintf(text, sizeof(text), "%.2f (%.2f,%.2f)", d.score, est[i].x, est[i].y);
      } else {
        std::snprintf(text, sizeof(text), "%.2f no depth", d.score);
      }
      cv::putText(canvas, text, cv::Point2f(d.x1 + 2, std::max(14.0f, d.y1 + 14)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1);
      if (d.has_keypoints) {
        for (const int k : {kLeftKnee, kRightKnee, kLeftAnkle, kRightAnkle}) {
          const Keypoint & p = d.keypoints[k];
          if (p.visibility >= fusion_config_.keypoint_vis_threshold) {
            cv::circle(canvas, cv::Point2f(p.x, p.y), 4, k >= kLeftAnkle ? red : magenta, -1);
          }
        }
      }
      for (const LegEstimate & leg : est[i].legs) {
        if (leg.valid) {
          cv::rectangle(canvas, cv::Point2f(leg.u_min, leg.v_min), cv::Point2f(leg.u_max, leg.v_max),
            magenta, 1);
        }
      }
    }
    char info[96];
    std::snprintf(info, sizeof(info), "infer %.1f ms  total %.1f ms", out.inference_ms, out.total_ms);
    cv::putText(canvas, info, cv::Point(5, canvas.rows - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1);
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = out.header.stamp;
    msg.header.frame_id = frame_id;
    msg.format = "jpeg";
    cv::imencode(".jpg", canvas, msg.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    debug_pub_->publish(std::move(msg));
  }

  void publish_markers(const kebi_follow_msgs::msg::CamLegArray & out)
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto & d : out.detections) {
      if (!d.has_position) {
        continue;
      }
      visualization_msgs::msg::Marker m;
      m.header = out.header;
      m.ns = d.class_id == kebi_follow_msgs::msg::CamLeg::CLASS_PERSON ? "person" : "leg";
      m.id = id++;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeS);
      m.pose.orientation.w = 1.0;
      m.pose.position.x = d.position.x;
      m.pose.position.y = d.position.y;
      if (d.class_id == kebi_follow_msgs::msg::CamLeg::CLASS_PERSON) {
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.pose.position.z = 0.4;
        m.scale.x = m.scale.y = std::max(0.1, 2.0 * d.position_std);
        m.scale.z = 0.8;
        m.color.g = 1.0f;
      } else {
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position.z = 0.1;
        m.scale.x = m.scale.y = m.scale.z = 0.1;
        m.color.r = 1.0f;
        m.color.b = 1.0f;
      }
      m.color.a = 0.6f;
      arr.markers.push_back(m);
    }
    marker_pub_->publish(std::move(arr));
  }

  void log_stats()
  {
    const Clock::time_point now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - stats_start_).count();
    if (elapsed < stats_period_s_) {
      return;
    }
    const uint64_t dropped = dropped_frames_.exchange(0);
    if (stat_frames_ > 0) {
      const double n = static_cast<double>(stat_frames_);
      RCLCPP_INFO(get_logger(),
        "%.1f Hz, dropped %lu | infer %.1f ms (max %.1f) | total %.1f ms (max %.1f) | worker CPU %.1f "
        "ms/frame | persons/frame %.2f | depth %.0f%% | offset %.3f s",
        n / elapsed, static_cast<unsigned long>(dropped), stat_infer_ms_ / n, stat_infer_max_ms_,
        stat_total_ms_ / n, stat_total_max_ms_, stat_cpu_ms_ / n, stat_persons_ / n,
        100.0 * stat_depth_frames_ / n, last_offset_.load());
    }
    stats_start_ = now;
    stat_frames_ = 0;
    stat_infer_ms_ = stat_infer_max_ms_ = stat_total_ms_ = stat_total_max_ms_ = stat_cpu_ms_ = 0.0;
    stat_persons_ = 0;
    stat_depth_frames_ = 0;
  }

  // parameters
  BackendConfig backend_config_;
  DecodeConfig decode_config_;
  FusionConfig fusion_config_;
  double min_box_height_px_{30.0};
  bool ignore_mode_{false};
  std::string base_frame_;
  double camera_min_latency_s_{0.10};
  double offset_window_s_{5.0};
  double odom_max_age_s_{0.5};
  double depth_max_dt_s_{0.1};
  double pitch_correction_rad_{0.0};
  std::vector<double> tf_fallback_base_to_rgb_;
  std::vector<double> tf_fallback_rgb_to_depth_;
  bool publish_debug_image_{false};
  double debug_max_rate_hz_{5.0};
  int jpeg_quality_{70};
  bool publish_markers_{false};
  double stats_period_s_{10.0};

  // ROS interfaces
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<kebi_follow_msgs::msg::FollowMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<kebi_follow_msgs::msg::CamLegArray>::SharedPtr legs_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  bool active_{false};

  // executor-side state (callbacks of this node share one mutually exclusive group)
  CameraTimeOffset time_offset_;
  bool have_odom_{false};
  double latest_odom_stamp_{0.0};
  Clock::time_point latest_odom_arrival_;
  std::atomic<double> last_offset_{0.0};

  // shared with the worker
  std::mutex input_mutex_;
  std::deque<Image::ConstSharedPtr> depth_buffer_;
  CameraInfo::ConstSharedPtr rgb_info_;
  CameraInfo::ConstSharedPtr depth_info_;
  std::mutex mailbox_mutex_;
  std::condition_variable mailbox_cv_;
  std::optional<Frame> pending_;
  bool stop_{false};
  std::atomic<uint64_t> dropped_frames_{0};
  std::thread worker_;

  // worker-only state
  std::unique_ptr<InferenceBackend> backend_;
  DepthFusion fusion_;
  cv::Mat input_rgb_;
  bool layout_checked_{false};
  bool geometry_ok_{false};
  std::string geom_rgb_frame_;
  std::string geom_depth_frame_;
  CameraInfo geom_rgb_info_;
  CameraInfo geom_depth_info_;
  Clock::time_point last_geometry_try_{};
  Clock::time_point last_debug_{};
  Clock::time_point stats_start_{};
  uint64_t stat_frames_{0};
  double stat_infer_ms_{0.0};
  double stat_infer_max_ms_{0.0};
  double stat_total_ms_{0.0};
  double stat_total_max_ms_{0.0};
  double stat_cpu_ms_{0.0};
  uint64_t stat_persons_{0};
  uint64_t stat_depth_frames_{0};
};

}  // namespace kebi_follow

RCLCPP_COMPONENTS_REGISTER_NODE(kebi_follow::CamLegDetector)
