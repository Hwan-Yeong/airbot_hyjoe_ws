#include "kebi_follow_controller/controller_core.hpp"

#include <algorithm>
#include <cmath>

namespace kebi_follow {

namespace {
// 이보다 큰 각속도 명령에는 S10 creep 을 붙인다 [rad/s]
constexpr double kZeroAngular = 1e-4;
// WAITING 진입 사유 중 감속 제한 없이 바로 세우는 것
// WAITING 중 단호 정지 (센서를 못 믿거나 회전하면 닿는다). 나머지(놓침·모호·회전 상한)는 편안 정지
constexpr uint32_t kFirmLinearWaitMask = stop_reason::kSensorTimeout;
constexpr uint32_t kFirmAngularWaitMask =
    stop_reason::kSensorTimeout | stop_reason::kRotationBlocked;
// 이보다 느리면 "서 있다"로 본다 (출발 문턱 판단) [m/s]
constexpr double kStoppedSpeed = 0.02;
// 근접 정지가 풀리는 여유 [m]
constexpr double kTooCloseHysteresis = 0.05;
}  // namespace

int16_t DetectUserFilter::update(int16_t raw, double now) {
  if (raw == detect_user::kNone) {
    reset();
    return raw;
  }
  if (raw == reported_) {
    pending_ = raw;
    return reported_;
  }
  const bool immediate = raw == detect_user::kEnded ||
                         reported_ == detect_user::kNone ||
                         reported_ == detect_user::kEnded;
  if (immediate) {
    reported_ = raw;
    pending_ = raw;
    return reported_;
  }
  if (raw != pending_) {
    pending_ = raw;
    pending_since_ = now;
  }
  if (now - pending_since_ >= debounce_s_) {
    reported_ = raw;
  }
  return reported_;
}

bool ChangeOrPeriodic::shouldPublish(int value, double now) {
  if (!have_ || value != last_value_ || now - last_time_ >= period_s_) {
    have_ = true;
    last_value_ = value;
    last_time_ = now;
    return true;
  }
  return false;
}

ControllerCore::ControllerCore(const ControllerParams& p)
    : p_(p),
      fsm_(p.fsm),
      law_(p.law),
      limiter_(p.limiter),
      gate_(p.corridor),
      detect_filter_(p.detect_debounce_s) {}

void ControllerCore::updateParams(const ControllerParams& p) {
  p_ = p;
  law_.setParams(p.law);
  limiter_.setParams(p.limiter);
  gate_.setParams(p.corridor);
}

void ControllerCore::resetMotion() {
  limiter_.reset();
  rotation_since_ = -1.0;
  rotation_latched_ = false;
  law_f_init_ = false;
  too_close_latched_ = false;
}

bool ControllerCore::onStart(double now) {
  if (!fsm_.start(now)) {
    return false;
  }
  resetMotion();
  return true;
}

bool ControllerCore::onStop(double now) {
  if (!fsm_.stop(now)) {
    return false;
  }
  resetMotion();
  return fsm_.consumeZeroBurst();
}

bool ControllerCore::onHalt(double now, uint32_t reasons) {
  if (!fsm_.halt(now, reasons)) {
    return false;  // 비활성 상태이거나 이미 HALTED
  }
  resetMotion();
  return true;
}

CycleOutput ControllerCore::step(const CycleInput& in) {
  CycleOutput out;
  const double now = in.now;
  const double dt = std::clamp(in.dt, 0.0, p_.max_dt);
  const FsmState prev = fsm_.state();

  // 레벨 신호(예: lifted 가 계속 true)도 활성 상태면 HALTED
  if (in.halt_active != 0 && fsm_.halt(now, in.halt_active)) {
    resetMotion();
  }

  // --- 타깃 ---
  const TargetInput& t = in.target;
  const bool locked = t.received && t.status == TargetStatus::kLocked;
  const bool occluded = t.received && t.status == TargetStatus::kOccluded;
  double meas_age = kInf;  // 마지막 실제 측정 이후 시간
  if (locked) {
    meas_age = t.msg_age;
  } else if (occluded) {
    meas_age = t.msg_age + std::max(0.0, t.occluded_time);
  }
  const bool tracked = (locked || occluded) && meas_age <= p_.target_lost_s;
  const bool fresh_lock = locked && t.msg_age <= p_.target_fresh_s;
  const bool candidate = t.received &&
                         (t.status == TargetStatus::kCandidate || locked) &&
                         t.msg_age <= p_.target_fresh_s;
  const bool has_geom = tracked || candidate;
  const double range = norm(t.position);
  const double bearing = std::atan2(t.position.y, t.position.x);

  FollowCommand law_cmd;
  if (tracked) {
    law_cmd = law_.compute(range, bearing, t.velocity.x);
  }
  // 추종 법칙 출력 저역 통과 (다리 흔들림·속도 추정 잡음이 가가속도로 번지지 않게).
  // 사람 앞 제동 상한은 걸러진 값 뒤에 다시 건다 (안전 상한은 늦추지 않는다)
  double law_v = law_cmd.v;
  double law_w = law_cmd.w;
  if (tracked) {
    if (!law_f_init_) {
      v_law_f_ = limiter_.v();
      w_law_f_ = limiter_.w();
      law_f_init_ = true;
    }
    const double av = (p_.v_filter_s > 0.0) ? dt / (p_.v_filter_s + dt) : 1.0;
    const double aw = (p_.w_filter_s > 0.0) ? dt / (p_.w_filter_s + dt) : 1.0;
    v_law_f_ += av * (law_cmd.v - v_law_f_);
    w_law_f_ += aw * (law_cmd.w - w_law_f_);
    law_v = std::min(v_law_f_, law_cmd.v_cap);
    law_w = w_law_f_;
  } else {
    law_f_init_ = false;
  }
  // S2: 위치를 아는 타깃(모호·가림 포함)이 stop_distance 안이면 어느 이동 상태에서든 즉시 정지
  const bool target_pos_known =
      t.received &&
      (locked || occluded || t.status == TargetStatus::kCandidate) &&
      t.msg_age <= p_.target_lost_s;
  // S2 근접: 들어갈 때는 stop_distance, 풀릴 때는 + kTooCloseHysteresis (다리 하나만 잡힐 때 거리가
  // 0.1 m 남짓 튀어 문턱에서 켜졌다 꺼졌다 하지 않게)
  bool too_close =
      law_cmd.too_close || (target_pos_known && range <= p_.law.stop_distance);
  if (!too_close && too_close_latched_ && target_pos_known &&
      range <= p_.law.stop_distance + kTooCloseHysteresis) {
    too_close = true;
  }
  too_close_latched_ = too_close;

  // --- 장애물 (S3) ---
  const double v_max = p_.limiter.max_linear_speed;
  const double decel = p_.limiter.max_linear_decel;
  Exclusion scan_excl;
  if (has_geom) {
    scan_excl = Exclusion{&t.position, 1};
  }
  const Exclusion depth_excl = (has_geom && in.depth_exclusion.count > 0)
                                   ? in.depth_exclusion
                                   : scan_excl;
  const bool use_depth = in.sensors.depth_points_ok;
  double d_scan = kInf;
  double d_depth = kInf;
  auto checkPath = [&](double kappa) {
    d_scan = std::min(d_scan,
                      gate_.corridorDistance(in.scan, kappa, true, scan_excl));
    if (use_depth) {
      d_depth = std::min(
          d_depth, gate_.corridorDistance(in.depth, kappa, true, depth_excl));
    }
  };
  const double slow_len = std::max(0.0, p_.slow_distance - p_.stop_margin);
  const double t_react = p_.obstacle_reaction_time;
  const double j_lin = p_.limiter.max_linear_jerk;
  const double t_react_cap =
      t_react + ((j_lin > 0.0) ? decel / (2.0 * j_lin) : 0.0);
  auto obstacleCap = [&]() {
    return std::min(obstacleSpeedCap(d_scan, p_.stop_margin, slow_len, decel,
                                     v_max, t_react_cap),
                    obstacleSpeedCap(d_depth, p_.depth_stop_margin, slow_len,
                                     decel, v_max, t_react_cap));
  };
  // --- 선속도 상한 (S1·S3·S6·S7) ---
  const bool measured = fresh_lock;
  const double occl_factor =
      (tracked && !measured) ? p_.occluded_speed_factor : 1.0;
  double v_limit_other = v_max;  // 장애물 외 상한
  if (!in.sensors.camera_ok) {
    v_limit_other = std::min(v_limit_other, p_.camera_missing_speed_cap);
  }
  // 1차: 법칙의 (v, w) 경로
  const double kappa0 = pathCurvature(
      law_v, law_w, p_.curvature_min_speed, p_.corridor.max_curvature);
  checkPath(kappa0);
  double cap_obs = obstacleCap();
  // 2차: 상한을 적용한 v 와 같은 w 의 실제 경로 (v 가 줄면 곡률이 커진다)
  const double v_first =
      std::min({law_v * occl_factor, v_limit_other, cap_obs});
  const double kappa1 = pathCurvature(
      v_first, law_w, p_.curvature_min_speed, p_.corridor.max_curvature);
  if (kappa1 != kappa0) {
    checkPath(kappa1);
    cap_obs = obstacleCap();
  }
  double r_min = gate_.minRadius(in.scan);
  if (use_depth) {
    r_min = std::min(r_min, gate_.minRadius(in.depth));
  }
  const double v_ref = std::max(limiter_.v(), in.measured_speed);
  const bool obs_emergency =
      obstacleStopRequired(d_scan, p_.stop_margin, v_ref, decel, t_react) ||
      obstacleStopRequired(d_depth, p_.depth_stop_margin, v_ref, decel,
                           t_react);

  const double v_cap = std::min(v_limit_other, cap_obs);
  double v_des = std::min(law_v * occl_factor, v_cap);
  if (obs_emergency || too_close) {
    v_des = 0.0;
  }
  const double w_des = law_w;

  // --- 회전 (S4) ---
  const bool in_place_req = tracked && v_des <= p_.in_place_linear_eps &&
                            std::abs(w_des) > p_.in_place_angular_eps;
  const bool rot_blocked = in_place_req && r_min <= p_.rotation_clearance;
  if (rotation_latched_ && (now - rotation_latched_at_ >= p_.rotation_rearm_s ||
                            (tracked && !in_place_req))) {
    rotation_latched_ = false;
  }

  // --- WAITING 사유 ---
  const bool sensors_ok = in.sensors.scan_ok && in.sensors.odom_ok &&
                          in.sensors.tf_ok && in.sensors.bottom_ir_ok &&
                          in.sensors.lifted_ok && in.sensors.fall_down_ok;
  const FsmState s0 = fsm_.state();
  // 재개는 신선한 측정이 있어야 한다
  const bool target_ok = (s0 == FsmState::kWaiting) ? fresh_lock : tracked;
  uint32_t wait = 0;
  if (!target_ok) {
    wait |= stop_reason::kNoTarget;
  }
  if (tracked && t.ambiguous) {
    wait |= stop_reason::kAmbiguous;
  }
  if (!sensors_ok) {
    wait |= stop_reason::kSensorTimeout;
  }
  if (rot_blocked) {
    wait |= stop_reason::kRotationBlocked;
  }
  if (rotation_latched_) {
    wait |= stop_reason::kRotationLimit;
  }

  FsmInputs fi;
  fi.target_locked = fresh_lock && !t.ambiguous;
  fi.wait_reasons = wait;
  fsm_.update(now, fi);
  if (fsm_.state() == FsmState::kFollowing && wait != 0) {
    // 락은 됐지만 바로 움직일 수 없으면 같은 주기에 WAITING
    fsm_.update(now, fi);
  }
  const FsmState s = fsm_.state();
  out.state_changed = (s != prev);
  if (out.state_changed && s != FsmState::kFollowing &&
      s != FsmState::kWaiting) {
    resetMotion();
  }

  // --- 명령 ---
  double v = 0.0;
  double w = 0.0;
  bool publish = false;
  switch (s) {
    case FsmState::kIdle:
    case FsmState::kEnded:
      limiter_.reset();
      publish = fsm_.consumeZeroBurst();
      break;
    case FsmState::kAcquiring:
    case FsmState::kHalted:
      limiter_.reset();
      publish = true;
      break;
    case FsmState::kWaiting: {
      // 놓침·모호·회전 상한은 편안 정지, 센서 끊김·회전 금지·근접·장애물은 단호 정지 (S 곡선, 즉시 0 아님)
      const bool firm_v = (wait & kFirmLinearWaitMask) != 0 || obs_emergency ||
                          too_close;
      const bool firm_w = (wait & kFirmAngularWaitMask) != 0;
      limiter_.step(0.0, 0.0, dt,
                    firm_v ? StopProfile::kFirm : StopProfile::kComfort,
                    firm_w ? StopProfile::kFirm : StopProfile::kComfort);
      v = limiter_.v();
      w = limiter_.w();
      publish = true;
      break;
    }
    case FsmState::kFollowing: {
      double v_t = v_des;
      // 서 있을 때 아주 작은 출발 요구는 무시한다 (사람이 거의 그대로면 덜컥 출발·정지를 반복하지 않게)
      if (limiter_.v() < kStoppedSpeed && v_t < p_.start_speed) {
        v_t = 0.0;
      }
      // S2·S3: 단호 정지 (S 곡선)
      const bool firm_v = too_close || obs_emergency;
      limiter_.step(v_t, w_des, dt,
                    firm_v ? StopProfile::kFirm : StopProfile::kComfort,
                    StopProfile::kComfort);
      v = limiter_.v();
      w = limiter_.w();
      publish = true;
      break;
    }
  }

  // S4: 제자리 회전 연속 상한
  if (s == FsmState::kFollowing && std::abs(w) > p_.in_place_angular_eps &&
      v <= p_.in_place_linear_eps) {
    if (rotation_since_ < 0.0) {
      rotation_since_ = now;
    }
    if (now - rotation_since_ >= p_.rotation_limit_s) {
      // 각속도를 끊지 않는다: 다음 주기부터 WAITING 에서 편안 정지(S 곡선)로 줄인다
      rotation_latched_ = true;
      rotation_latched_at_ = now;
      rotation_since_ = -1.0;
    }
  } else {
    rotation_since_ = -1.0;
  }

  // S10: 제자리 회전에도 베이스 워치독(|v|>0.001)이 걸리도록 아주 작은 선속도
  if (publish && std::abs(w) > kZeroAngular && v < p_.rotation_creep_speed) {
    v = p_.rotation_creep_speed;
  }

  out.publish = publish;
  out.v = v;
  out.w = w;
  out.state = s;
  out.mode = modeOf(s);

  // --- 상태 보고 ---
  uint32_t reasons = 0;
  switch (s) {
    case FsmState::kFollowing:
      if (too_close) {
        reasons |= stop_reason::kTooClose;
      }
      if ((law_cmd.v > 0.0 || v_ref > 0.0) &&
          (obs_emergency || cap_obs <= 0.0)) {
        reasons |= stop_reason::kObstacle;
      }
      break;
    case FsmState::kWaiting:
      reasons = wait;
      if (too_close) {
        reasons |= stop_reason::kTooClose;
      }
      break;
    case FsmState::kHalted:
      reasons = fsm_.haltReasons() | in.halt_active;
      break;
    case FsmState::kAcquiring:
      if (!candidate) {
        reasons |= stop_reason::kNoTarget;
      }
      break;
    default:
      break;
  }
  out.stop_reasons = reasons;
  out.v_limit = (s == FsmState::kFollowing)
                    ? std::min(v_cap, law_cmd.v_cap) * occl_factor
                    : 0.0;
  out.scan_obstacle_distance = d_scan;
  out.depth_obstacle_distance = d_depth;
  out.obstacle_distance = std::min(d_scan, d_depth);
  out.target_range = has_geom ? range : -1.0;
  out.target_bearing = has_geom ? bearing : 0.0;

  int16_t raw = detect_user::kNone;
  const bool moving =
      std::abs(v) > p_.moving_linear || std::abs(w) > p_.moving_angular;
  switch (s) {
    case FsmState::kIdle:
      raw = detect_user::kNone;
      break;
    case FsmState::kAcquiring:
      raw = candidate ? detect_user::kRecognized : detect_user::kNoPerson;
      break;
    case FsmState::kFollowing:
      raw = moving    ? detect_user::kMoving
            : tracked ? detect_user::kRecognized
                      : detect_user::kNoPerson;
      break;
    case FsmState::kWaiting:
      raw = tracked ? detect_user::kRecognized : detect_user::kNoPerson;
      break;
    case FsmState::kHalted:
    case FsmState::kEnded:
      raw = detect_user::kEnded;
      break;
  }
  out.detect_user = detect_filter_.update(raw, now);
  return out;
}

}  // namespace kebi_follow
