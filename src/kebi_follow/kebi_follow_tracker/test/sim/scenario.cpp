#include "sim/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <set>

namespace kebi_follow {
namespace tracker {
namespace sim {

namespace {
constexpr double kSimDt = 0.01;  // [s] 세계·로봇 적분 주기 (TF 100 Hz 와 같다)
constexpr double kTimeOrigin = 1000.0;  // [s] 시나리오 시작 시각
constexpr double kMatchRadius = 0.5;    // [m] 추정 ↔ 사람 대응 반경
constexpr double kLegCountRadius = 0.5;  // [m] 타깃 근처 분할을 세는 반경
constexpr double kCameraOcclusionMargin = 0.3;  // [m]
constexpr double kMinLatency = 0.02;            // [s]
constexpr double kWalkingSpeed = 0.3;  // [m/s] 지연 계산에 쓰는 최소 속도
constexpr double kBearingSlowStart = 15.0 * kDegToRad;  // 추종 법칙 (SPEC 6.3)
constexpr double kBearingStop = 45.0 * kDegToRad;
constexpr double kP95 = 0.95;
constexpr double kP99 = 0.99;

// 스레드 CPU 시간 [ms] (다른 프로세스에 밀린 시간은 빠진다)
double threadCpuMs() {
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

struct PoseSample {
  double t;
  Pose2 pose;
};

struct PendingFrame {
  double stamp;
  double arrival;
  std::vector<CameraDetection> dets;
};

Pose2 interpolatePose(const std::vector<PoseSample>& hist, double t) {
  if (t <= hist.front().t) {
    return hist.front().pose;
  }
  if (t >= hist.back().t) {
    return hist.back().pose;
  }
  const auto it =
      std::lower_bound(hist.begin(), hist.end(), t,
                       [](const PoseSample& s, double v) { return s.t < v; });
  const PoseSample& b = *it;
  const PoseSample& a = *(it - 1);
  const double w = (t - a.t) / (b.t - a.t);
  return {a.pose.x + w * (b.pose.x - a.pose.x),
          a.pose.y + w * (b.pose.y - a.pose.y),
          wrapAngle(a.pose.yaw + w * wrapAngle(b.pose.yaw - a.pose.yaw))};
}

double approach(double cur, double target, double up, double down, double dt) {
  if (target > cur) {
    return std::min(target, cur + up * dt);
  }
  return std::max(target, cur - down * dt);
}

double percentile(std::vector<double> v, double q) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const std::size_t idx = std::min(
      v.size() - 1, static_cast<std::size_t>(q * (v.size() - 1) + 0.5));
  return v[idx];
}

bool isLockedStatus(TargetStatus s) {
  return s == TargetStatus::LOCKED || s == TargetStatus::OCCLUDED;
}
}  // namespace

ScenarioResult runScenario(const ScenarioSpec& spec) {
  std::mt19937 rng(spec.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  World world;
  world.circles = spec.circles;
  world.walls = spec.walls;
  for (const auto& ws : spec.walkers) {
    world.walkers.emplace_back(ws);
  }
  LegTrackerCore core(spec.params);
  const RobotMotion& rm = spec.robot;
  Pose2 robot = rm.start;
  double v = 0.0;
  double w = 0.0;
  std::vector<PoseSample> pose_hist;
  pose_hist.reserve(static_cast<std::size_t>(spec.duration / kSimDt) + 2);
  pose_hist.push_back({kTimeOrigin, robot});

  const CameraModel& cam = spec.camera;
  std::vector<PendingFrame> frames;
  std::vector<float> ranges;
  std::set<int> false_person_ids;
  ScenarioResult res;
  std::size_t next_mode = 0;
  double next_scan = spec.scan_period;
  double next_cam = cam.period * uni(rng);
  const int steps = static_cast<int>(std::round(spec.duration / kSimDt));

  for (int k = 1; k <= steps; ++k) {
    const double t = k * kSimDt;
    world.step(t, kSimDt);

    // 로봇: 정답 타깃을 따라가거나(평가와 분리) 제자리 회전
    double v_cmd = 0.0;
    double w_cmd = 0.0;
    const bool target_ok =
        spec.target_walker >= 0 &&
        spec.target_walker < static_cast<int>(world.walkers.size());
    if (rm.follow_start >= 0.0 && t >= rm.follow_start && target_ok) {
      const Vec2 pb = inverseTransformPoint(
          robot, world.walkers[spec.target_walker].center());
      const double rho = norm(pb);
      const double beta = bearing(pb);
      v_cmd = std::clamp(rm.k_v * (rho - rm.follow_distance), 0.0, rm.v_max);
      const double ab = std::abs(beta);
      if (ab >= kBearingStop) {
        v_cmd = 0.0;
      } else if (ab > kBearingSlowStart) {
        v_cmd *= (kBearingStop - ab) / (kBearingStop - kBearingSlowStart);
      }
      w_cmd = std::clamp(rm.k_w * beta, -rm.w_max, rm.w_max);
    }
    if (rm.rotate_start >= 0.0 && t >= rm.rotate_start && t < rm.rotate_end) {
      v_cmd = 0.0;
      w_cmd = rm.rotate_rate;
    }
    v = approach(v, v_cmd, rm.lin_accel, rm.lin_decel, kSimDt);
    w = approach(w, w_cmd, rm.ang_accel, rm.ang_accel, kSimDt);
    robot.x += v * std::cos(robot.yaw) * kSimDt;
    robot.y += v * std::sin(robot.yaw) * kSimDt;
    robot.yaw = wrapAngle(robot.yaw + w * kSimDt);
    pose_hist.push_back({kTimeOrigin + t, robot});

    while (next_mode < spec.modes.size() && spec.modes[next_mode].t <= t) {
      core.setMode(spec.modes[next_mode].mode);
      ++next_mode;
    }

    // 카메라: 촬영 시각 stamp(변환 오차 포함), 처리 지연 뒤 도착
    if (cam.enabled && t >= next_cam) {
      PendingFrame f;
      f.stamp = kTimeOrigin + t + cam.stamp_error_std * gauss(rng);
      f.arrival =
          kTimeOrigin + t +
          std::max(kMinLatency, cam.latency + cam.latency_std * gauss(rng));
      for (std::size_t i = 0; i < world.walkers.size(); ++i) {
        const Vec2 pb = inverseTransformPoint(robot, world.walkers[i].center());
        const double r = norm(pb);
        const double b = bearing(pb);
        if (r < cam.min_range || r > cam.max_range ||
            std::abs(b) > cam.half_fov) {
          continue;
        }
        int hit = -1;
        const double hr =
            world.raycast({robot.x, robot.y}, robot.yaw + b, &hit);
        if (hit != static_cast<int>(i) && hr < r - kCameraOcclusionMargin) {
          continue;  // 가려짐
        }
        if (uni(rng) > cam.detect_prob) {
          continue;
        }
        CameraDetection d;
        d.position = pb - (cam.bias_toward_robot / r) * pb +
                     Vec2{cam.pos_std * gauss(rng), cam.pos_std * gauss(rng)};
        d.score = cam.score;
        f.dets.push_back(d);
      }
      if (cam.false_rate > 0.0) {
        std::poisson_distribution<int> pois(cam.false_rate);
        const int nf = pois(rng);
        for (int i = 0; i < nf; ++i) {
          const double r =
              cam.min_range + uni(rng) * (cam.max_range - cam.min_range);
          const double b = (2.0 * uni(rng) - 1.0) * cam.half_fov;
          CameraDetection d;
          d.position = {r * std::cos(b), r * std::sin(b)};
          d.score = cam.score;
          f.dets.push_back(d);
        }
      }
      frames.push_back(std::move(f));
      next_cam += cam.period + cam.jitter * (2.0 * uni(rng) - 1.0);
    }
    for (auto it = frames.begin(); it != frames.end();) {
      if (it->arrival <= kTimeOrigin + t) {
        core.processCamera(it->stamp, interpolatePose(pose_hist, it->stamp),
                           it->dets);
        it = frames.erase(it);
      } else {
        ++it;
      }
    }

    if (t + 1e-9 < next_scan) {
      continue;
    }
    next_scan += spec.scan_period + spec.scan_jitter * (2.0 * uni(rng) - 1.0);
    simulateScan(world, robot, spec.lidar, rng, ranges);
    ScanView view;
    view.ranges = ranges.data();
    view.count = ranges.size();
    view.angle_min = spec.lidar.angle_min;
    view.angle_increment = spec.lidar.angle_increment;
    view.range_min = spec.lidar.range_min;
    view.range_max = spec.lidar.range_max;
    const double cpu0 = threadCpuMs();
    const TargetOutput& out =
        core.processScan(kTimeOrigin + t, view, spec.lidar.mount, robot);
    const double cpu1 = threadCpuMs();
    if (spec.on_scan) {
      spec.on_scan(t, core, world, robot);
    }

    ScanRecord rec;
    rec.t = t;
    rec.mode = core.mode();
    rec.status = out.status;
    rec.track_id = out.track_id;
    rec.ambiguous = out.ambiguous;
    rec.camera_confirmed = out.camera_confirmed;
    rec.proc_ms = cpu1 - cpu0;
    if (out.status != TargetStatus::NONE) {
      rec.est = inverseTransformPoint(robot, out.position);
      rec.est_vel = out.velocity;
    }
    if (target_ok) {
      const Walker& tw = world.walkers[spec.target_walker];
      rec.truth = inverseTransformPoint(robot, tw.center());
      rec.truth_vel = tw.velocity();
      for (const auto& seg : core.detector().segments()) {
        if ((seg.is_leg || seg.is_blob) &&
            distance(transformPoint(robot, seg.center), tw.center()) <
                kLegCountRadius) {
          ++rec.n_target_legs;
        }
      }
    }
    if (out.status == TargetStatus::CANDIDATE || isLockedStatus(out.status)) {
      double best = kMatchRadius;
      for (std::size_t i = 0; i < world.walkers.size(); ++i) {
        const double d = distance(world.walkers[i].center(), out.position);
        if (d < best) {
          best = d;
          rec.matched_walker = static_cast<int>(i);
        }
      }
    }
    const double now = kTimeOrigin + t;
    for (const auto& tr : core.tracker().tracks()) {
      ++rec.n_tracks;
      if (!tr.confirmed) {
        continue;
      }
      ++rec.n_confirmed;
      if (tr.personScore(now, spec.params.camera) <
          spec.params.ambiguity.min_person_score) {
        continue;
      }
      bool near_person = false;
      for (const auto& wk : world.walkers) {
        near_person = near_person ||
                      distance(wk.center(), tr.kf.position()) < kMatchRadius;
      }
      if (!near_person) {
        ++rec.n_false_person;
        false_person_ids.insert(tr.id);
      }
    }
    // 진행 방향 오차를 위해 로봇 yaw 로 정답 속도를 base_link 로 돌려 둔다
    rec.truth_vel = rotate(rec.truth_vel, -robot.yaw);
    rec.est_vel = rotate(rec.est_vel, -robot.yaw);
    res.records.push_back(rec);
  }

  // 지표
  ScenarioMetrics& m = res.metrics;
  m.name = spec.name;
  m.scans = static_cast<int>(res.records.size());
  double acquire_t = -1.0;
  for (const auto& ev : spec.modes) {
    if (ev.mode == Mode::ACQUIRE) {
      acquire_t = ev.t;
      break;
    }
  }
  std::vector<double> errs;
  std::vector<double> procs;
  double vel_sq = 0.0;
  int vel_n = 0;
  double along_sum = 0.0;
  double speed_sum = 0.0;
  int along_n = 0;
  double false_person_sum = 0.0;
  bool locked_once = false;
  int covered = 0;
  int after_lock = 0;
  const ScanRecord* prev = nullptr;
  for (const auto& r : res.records) {
    procs.push_back(r.proc_ms);
    false_person_sum += r.n_false_person;
    const bool locked = isLockedStatus(r.status);
    const bool was_locked = prev != nullptr && isLockedStatus(prev->status);
    if (r.status == TargetStatus::LOCKED && !was_locked) {
      if (!locked_once) {
        m.lock_time = acquire_t >= 0.0 ? r.t - acquire_t : r.t;
        m.lock_walker = r.matched_walker;
      }
      if (r.matched_walker != spec.target_walker || spec.target_walker < 0) {
        ++m.false_locks;
      }
      if (prev != nullptr && prev->status == TargetStatus::LOST) {
        ++m.reacquires;
      }
      locked_once = true;
    }
    if (locked && was_locked && r.track_id != prev->track_id) {
      ++m.id_switches;
    }
    if (locked_once) {
      ++after_lock;
      if (locked && r.matched_walker == spec.target_walker) {
        ++covered;
      }
      if (r.status == TargetStatus::LOST) {
        ++m.lost_scans;
      }
    }
    if (locked) {
      if (r.matched_walker != spec.target_walker) {
        ++m.wrong_scans;
      }
      if (r.ambiguous) {
        ++m.ambiguous_scans;
      }
      if (r.status == TargetStatus::OCCLUDED) {
        ++m.occluded_scans;
      }
    }
    if (r.status == TargetStatus::LOCKED) {
      ++m.locked_scans;
      if (r.matched_walker == spec.target_walker && spec.target_walker >= 0) {
        const Vec2 e = r.est - r.truth;
        errs.push_back(norm(e));
        vel_sq += squaredNorm(r.est_vel - r.truth_vel);
        ++vel_n;
        const double speed = norm(r.truth_vel);
        if (speed >= kWalkingSpeed) {
          along_sum += dot(e, (1.0 / speed) * r.truth_vel);
          speed_sum += speed;
          ++along_n;
        }
      }
    }
    prev = &r;
  }
  if (!errs.empty()) {
    double sq = 0.0;
    for (double e : errs) {
      sq += e * e;
    }
    m.err_rms = std::sqrt(sq / errs.size());
    m.err_p95 = percentile(errs, kP95);
    m.err_max = *std::max_element(errs.begin(), errs.end());
  }
  if (vel_n > 0) {
    m.vel_err_rms = std::sqrt(vel_sq / vel_n);
  }
  if (along_n > 0 && speed_sum > 0.0) {
    m.lag_ms = -1000.0 * along_sum / speed_sum;
  }
  if (!procs.empty()) {
    double s = 0.0;
    for (double p : procs) {
      s += p;
    }
    m.proc_mean_ms = s / procs.size();
    m.proc_p99_ms = percentile(procs, kP99);
    m.proc_max_ms = *std::max_element(procs.begin(), procs.end());
  }
  m.false_person_mean = m.scans > 0 ? false_person_sum / m.scans : 0.0;
  m.false_person_ids = static_cast<int>(false_person_ids.size());
  m.track_coverage =
      after_lock > 0 ? static_cast<double>(covered) / after_lock : 0.0;
  return res;
}

}  // namespace sim
}  // namespace tracker
}  // namespace kebi_follow
