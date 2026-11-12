// bag 재생 도구: /scan, /tf, /tf_static (+ /kebi_follow/cam_legs, /kebi_follow/mode) 를 읽어
// LegTrackerCore 에 직접 넣고(ROS 통신 없이) scan 별 결과를 CSV 로 쓴다. 처리 시간도 잰다.
//
// 사용: bag_replay <bag_dir> <out_dir> [--modes t:m,t:m,...] [--set name=value]...
//   --modes : 첫 /scan 수신 기준 시각[s]:mode(0 OFF, 1 ACQUIRE, 2 TRACK). bag 에 /kebi_follow/mode 가 있으면 그것도 쓴다
//   출력 t_bag 도 첫 /scan 수신 시각 기준이다 (정답과는 scan stamp 로 맞춘다)
//   --set   : 파라미터 덮어쓰기 (config/leg_tracker.yaml 의 이름, 예: acquire.require_camera=false)
//   --auto-track : ACQUIRE 중 LOCKED 가 되면 controller 처럼 TRACK 으로 바꾼다
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>
#include <kebi_follow_msgs/msg/cam_leg.hpp>
#include <kebi_follow_msgs/msg/cam_leg_array.hpp>
#include <kebi_follow_msgs/msg/follow_mode.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "kebi_follow_tracker/leg_tracker_core.hpp"
#include "kebi_follow_tracker/param_table.hpp"

namespace kt = kebi_follow::tracker;

namespace {

constexpr char kScanTopic[] = "/scan";
constexpr char kCamTopic[] = "/kebi_follow/cam_legs";
constexpr char kModeTopic[] = "/kebi_follow/mode";
constexpr int kTfCacheS = 36000;
constexpr double kNsToS = 1e-9;

double threadCpuMs() {
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

template <typename T>
T deserialize(const rosbag2_storage::SerializedBagMessage& m) {
  rclcpp::SerializedMessage ser(*m.serialized_data);
  T out;
  rclcpp::Serialization<T> s;
  s.deserialize_message(&ser, &out);
  return out;
}

double toSec(const builtin_interfaces::msg::Time& t) {
  return t.sec + kNsToS * t.nanosec;
}

tf2::TimePoint toTimePoint(const builtin_interfaces::msg::Time& t) {
  return tf2::TimePoint(std::chrono::seconds(t.sec) +
                        std::chrono::nanoseconds(t.nanosec));
}

kt::Pose2 toPose2(const geometry_msgs::msg::TransformStamped& tf) {
  const auto& q = tf.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return {tf.transform.translation.x, tf.transform.translation.y, yaw};
}

struct ModeEvent {
  double t;
  kt::Mode mode;
};

bool setParam(kt::TrackerParams& p, const std::string& assignment) {
  const auto eq = assignment.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  const std::string name = assignment.substr(0, eq);
  const std::string value = assignment.substr(eq + 1);
  bool found = false;
  kt::visitParams(p, [&](const char* key, auto& field) {
    if (name != key) {
      return;
    }
    found = true;
    using T = std::decay_t<decltype(field)>;
    if constexpr (std::is_same_v<T, bool>) {
      field = value == "true" || value == "1";
    } else if constexpr (std::is_same_v<T, int>) {
      field = std::atoi(value.c_str());
    } else {
      field = std::atof(value.c_str());
    }
  });
  return found;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: bag_replay <bag_dir> <out_dir> [--modes t:m,...] "
                 "[--set k=v]...\n");
    return 1;
  }
  const std::string bag = argv[1];
  const std::string out_dir = argv[2];
  kt::TrackerParams params;
  std::vector<ModeEvent> modes;
  bool auto_track = false;
  for (int i = 3; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--auto-track")) {
      auto_track = true;
    } else if (!std::strcmp(argv[i], "--set") && i + 1 < argc) {
      if (!setParam(params, argv[++i])) {
        std::fprintf(stderr, "unknown parameter: %s\n", argv[i]);
        return 1;
      }
    } else if (!std::strcmp(argv[i], "--modes") && i + 1 < argc) {
      std::string spec = argv[++i];
      std::size_t pos = 0;
      while (pos < spec.size()) {
        const std::size_t comma = spec.find(',', pos);
        const std::string item = spec.substr(
            pos, comma == std::string::npos ? std::string::npos : comma - pos);
        const auto colon = item.find(':');
        if (colon != std::string::npos) {
          modes.push_back({std::atof(item.substr(0, colon).c_str()),
                           static_cast<kt::Mode>(
                               std::atoi(item.substr(colon + 1).c_str()))});
        }
        if (comma == std::string::npos) {
          break;
        }
        pos = comma + 1;
      }
    }
  }
  std::sort(modes.begin(), modes.end(),
            [](const ModeEvent& a, const ModeEvent& b) { return a.t < b.t; });

  // 1) TF 전부 읽기, 첫 /scan 수신 시각 (시간축 기준)
  tf2::BufferCore tf_buffer{tf2::Duration(std::chrono::seconds(kTfCacheS))};
  double bag_start = 0.0;
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag);
    rosbag2_storage::StorageFilter scan_only;
    scan_only.topics = {kScanTopic};
    reader.set_filter(scan_only);
    if (reader.has_next()) {
      bag_start = reader.read_next()->time_stamp * kNsToS;
    }
  }
  {
    rosbag2_cpp::Reader reader;
    reader.open(bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {"/tf", "/tf_static"};
    reader.set_filter(filter);
    while (reader.has_next()) {
      const auto m = reader.read_next();
      const auto msg = deserialize<tf2_msgs::msg::TFMessage>(*m);
      const bool is_static = m->topic_name == "/tf_static";
      for (const auto& tf : msg.transforms) {
        tf_buffer.setTransform(tf, "bag", is_static);
      }
    }
  }

  // 2) scan·카메라·mode 순서대로 처리
  kt::LegTrackerCore core(params);
  FILE* f_scan = std::fopen((out_dir + "/scans.csv").c_str(), "w");
  FILE* f_trk = std::fopen((out_dir + "/tracks.csv").c_str(), "w");
  FILE* f_seg = std::fopen((out_dir + "/segments.csv").c_str(), "w");
  if (f_scan == nullptr || f_trk == nullptr || f_seg == nullptr) {
    std::fprintf(stderr, "cannot write to %s\n", out_dir.c_str());
    return 1;
  }
  std::fprintf(f_scan,
               "t_bag,stamp,mode,status,track_id,x,y,range,bearing,ambiguous,"
               "occluded_time,"
               "position_std,camera_confirmed,zone_candidates,competitors,hold_"
               "s,n_seg,n_legs,n_blobs,"
               "n_tracks,n_confirmed,proc_ms\n");
  std::fprintf(f_trk,
               "t_bag,stamp,id,confirmed,kind,x,y,vx,vy,std,shape,person_score,"
               "hits,tsu,updated,"
               "camera_confirmed,stationary,moved\n");
  std::fprintf(
      f_seg,
      "t_bag,stamp,x,y,width,count,is_leg,is_blob,leg_score,bulge,range\n");

  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {kScanTopic, kCamTopic, kModeTopic};
  reader.set_filter(filter);
  std::size_t next_mode = 0;
  std::string scan_frame;
  kt::Pose2 sensor_in_base;
  bool have_sensor = false;
  int tf_fallbacks = 0;
  int tf_failures = 0;
  int scans = 0;
  int cam_msgs = 0;
  std::vector<double> proc;
  std::vector<kt::CameraDetection> dets;

  auto robotPose = [&](const builtin_interfaces::msg::Time& stamp,
                       kt::Pose2& pose) {
    try {
      pose = toPose2(
          tf_buffer.lookupTransform("odom", "base_link", toTimePoint(stamp)));
      return true;
    } catch (const tf2::TransformException&) {}
    try {
      pose = toPose2(
          tf_buffer.lookupTransform("odom", "base_link", tf2::TimePointZero));
      ++tf_fallbacks;
      return true;
    } catch (const tf2::TransformException&) {
      ++tf_failures;
      return false;
    }
  };

  while (reader.has_next()) {
    const auto m = reader.read_next();
    const double t_bag = m->time_stamp * kNsToS - bag_start;
    while (next_mode < modes.size() && modes[next_mode].t <= t_bag) {
      core.setMode(modes[next_mode++].mode);
    }
    if (m->topic_name == kModeTopic) {
      core.setMode(static_cast<kt::Mode>(
          deserialize<kebi_follow_msgs::msg::FollowMode>(*m).mode));
      continue;
    }
    if (m->topic_name == kCamTopic) {
      const auto msg = deserialize<kebi_follow_msgs::msg::CamLegArray>(*m);
      kt::Pose2 pose;
      if (!robotPose(msg.header.stamp, pose)) {
        continue;
      }
      dets.clear();
      for (const auto& d : msg.detections) {
        if (d.has_position) {
          dets.push_back({{d.position.x, d.position.y}, d.score});
        }
      }
      core.processCamera(toSec(msg.header.stamp), pose, dets);
      ++cam_msgs;
      continue;
    }
    const auto scan = deserialize<sensor_msgs::msg::LaserScan>(*m);
    if (!have_sensor || scan.header.frame_id != scan_frame) {
      try {
        sensor_in_base = toPose2(tf_buffer.lookupTransform(
            "base_link", scan.header.frame_id, tf2::TimePointZero));
        scan_frame = scan.header.frame_id;
        have_sensor = true;
      } catch (const tf2::TransformException& e) {
        ++tf_failures;
        continue;
      }
    }
    kt::Pose2 robot;
    if (!robotPose(scan.header.stamp, robot)) {
      continue;
    }
    kt::ScanView view;
    view.ranges = scan.ranges.data();
    view.count = scan.ranges.size();
    view.angle_min = scan.angle_min;
    view.angle_increment = scan.angle_increment;
    view.range_min = scan.range_min;
    view.range_max = scan.range_max;
    const double stamp = toSec(scan.header.stamp);
    const double cpu0 = threadCpuMs();
    const kt::TargetOutput& out =
        core.processScan(stamp, view, sensor_in_base, robot);
    const double ms = threadCpuMs() - cpu0;  // 스레드 CPU 시간
    if (auto_track && core.mode() == kt::Mode::ACQUIRE &&
        out.status == kt::TargetStatus::LOCKED) {
      core.setMode(kt::Mode::TRACK);
    }
    proc.push_back(ms);
    ++scans;

    int n_legs = 0;
    int n_blobs = 0;
    for (const auto& s : core.detector().segments()) {
      n_legs += s.is_leg ? 1 : 0;
      n_blobs += s.is_blob ? 1 : 0;
      if (s.is_leg || s.is_blob) {
        std::fprintf(
            f_seg, "%.4f,%.6f,%.4f,%.4f,%.4f,%d,%d,%d,%.2f,%.4f,%.4f\n", t_bag,
            stamp, s.center.x, s.center.y, s.width, s.count, s.is_leg ? 1 : 0,
            s.is_blob ? 1 : 0, s.leg_score, s.bulge, s.range);
      }
    }
    int n_conf = 0;
    for (const auto& tr : core.tracker().tracks()) {
      n_conf += tr.confirmed ? 1 : 0;
      const kt::Vec2 pb = kt::inverseTransformPoint(robot, tr.kf.position());
      const kt::Vec2 vb = kt::rotate(tr.kf.velocity(), -robot.yaw);
      std::fprintf(
          f_trk,
          "%.4f,%.6f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%.3f,%d,%"
          "d,%d,%d\n",
          t_bag, stamp, tr.id, tr.confirmed ? 1 : 0, static_cast<int>(tr.kind),
          pb.x, pb.y, vb.x, vb.y, tr.kf.positionStd(), tr.shape_score,
          tr.personScore(stamp, params.camera), tr.hits,
          stamp - tr.last_update_time, tr.updated ? 1 : 0,
          tr.cameraConfirmed(stamp, params.camera.confirm_window_s) ? 1 : 0,
          tr.stationary ? 1 : 0, tr.moved ? 1 : 0);
    }
    kt::Vec2 pb;
    if (out.status != kt::TargetStatus::NONE) {
      pb = kt::inverseTransformPoint(robot, out.position);
    }
    std::fprintf(f_scan,
                 "%.4f,%.6f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.4f,%d,%d,%d,"
                 "%.3f,%zu,%d,%d,%zu,%d,%.4f\n",
                 t_bag, stamp, static_cast<int>(core.mode()),
                 static_cast<int>(out.status), out.track_id, pb.x, pb.y,
                 kt::norm(pb), kt::bearing(pb), out.ambiguous ? 1 : 0,
                 out.occluded_time, out.position_std,
                 out.camera_confirmed ? 1 : 0, out.zone_candidates,
                 out.competitors, out.hold_s, core.detector().segments().size(),
                 n_legs, n_blobs, core.tracker().tracks().size(), n_conf, ms);
  }
  std::fclose(f_scan);
  std::fclose(f_trk);
  std::fclose(f_seg);

  std::sort(proc.begin(), proc.end());
  double sum = 0.0;
  for (double p : proc) {
    sum += p;
  }
  const auto pct = [&](double q) {
    return proc.empty()
               ? 0.0
               : proc[std::min(proc.size() - 1, static_cast<std::size_t>(
                                                    q * (proc.size() - 1)))];
  };
  std::printf(
      "scans %d cam_msgs %d tf_fallbacks %d tf_failures %d | proc ms mean %.4f "
      "p50 %.4f p99 %.4f max %.4f\n",
      scans, cam_msgs, tf_fallbacks, tf_failures,
      proc.empty() ? 0.0 : sum / proc.size(), pct(0.5), pct(0.99),
      proc.empty() ? 0.0 : proc.back());
  return 0;
}
