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
// STOP 을 받았을 때 이보다 빠르면(선 [m/s] 또는 각 [rad/s]) S 곡선으로 세운 뒤 IDLE, 아니면 바로 IDLE
constexpr double kStopMovingEps = 1e-3;
// 사람 앞 제동 격상: 편안 S 곡선으로 서면 사람 앞 stop_distance + 이 비율 × approach_margin 안까지 들어갈 참이면 단호 S 곡선
constexpr double kBrakeMarginFrac = 0.5;
// 사람이 로봇 쪽으로 오는 속도 중 이만큼은 다리 흔들림 잡음으로 보고 뺀다 [m/s]
constexpr double kApproachNoise = 0.2;
// 사람 발자취 (2026-11-13 실차 E2 갇힘): 기록 간격 [m], 최대 점 수 (약 30 m), 앞쪽 점을 찾는 발자취 길이 [m],
// 로봇에서 이보다 가까운 점은 목표로 안 잡는다 [m], 직선으로 돌아갈 때 시야 여유 히스테리시스 [m]
constexpr double kTrailSpacing = 0.1;
constexpr std::size_t kTrailMaxPoints = 300;
constexpr double kTrailLookahead = 1.2;
constexpr double kTrailMinCarrot = 0.3;
constexpr double kLosHysteresis = 0.05;
// 막혔을 때 중간 목표점 (2026-11-16 갇힘 개선 3단계): 살펴보는 방향 수(전 방위), 그 방향 통로가 이만큼은 비어야 후보 [m],
// 중간 목표점까지 거리 상한 [m]·통로 끝에서 남길 여유 [m], 사람에게 이만큼은 다가가야 후보 [m],
// 회전량 벌점 [m/rad], 도착 판정 [m], 한 목표점을 쫓는 시간 상한 [s], 회피 중 선속도 상한 [m/s]
constexpr int kSubHeadings = 24;
constexpr double kSubMinFree = 0.6;
constexpr double kSubMaxDist = 1.0;
constexpr double kSubEndMargin = 0.3;
constexpr double kSubMinProgress = 0.05;
constexpr double kSubTurnWeight = 0.1;
constexpr double kSubReach = 0.25;
constexpr double kSubTimeoutS = 5.0;
constexpr double kAvoidSpeed = 0.3;
constexpr double kAvoidWantSpeed = 0.1;
constexpr double kAvoidBlockedSpeed = 0.05;
// depth 로만 보이는 물체(라이다 아래 낮은 물체)는 가까워지면 카메라 근거리 사각(약 0.45 m)에서 사라져 길이 빈 것처럼 보인다.
// 그런 물체를 이 거리 안에서 본 뒤 이 시간 동안은 막힘 탈출(중간 목표점·후진)을 쓰지 않고 기다린다 [m], [s]
// (2026-11-17 ROS 시뮬레이터 low_box_depth: 탈출이 상자 쪽을 골라 0.31 → 0.17 m)
constexpr double kDepthOnlyNear = 1.5;
constexpr double kDepthOnlyHoldS = 3.0;
// 벽에 붙었을 때 지나온 길 후진 (4단계): 지나온 길 기록 간격 [m]·최대 점 수, 기록하는 최소 전진 속도 [m/s],
// 막힌 채 이만큼 지나야 후진 [s], 한 번 갇혔을 때 후진 총량 [m], 후진 속도 [m/s], 되짚을 목표점까지 거리 [m],
// 뒤 통로가 이만큼 남아야 후진 [m], 이만큼 앞으로 가야 후진 총량을 다시 채움 [m]
constexpr double kOwnPathSpacing = 0.05;
constexpr std::size_t kOwnPathMaxPoints = 80;
constexpr double kOwnPathMinSpeed = 0.02;
constexpr double kReverseAfterS = 1.0;
constexpr double kReverseBudget = 0.3;
constexpr double kReverseSpeed = 0.1;
constexpr double kReverseLookBack = 0.2;
constexpr double kReverseRearClear = 0.1;
constexpr double kReverseRearmForward = 0.5;

// 막혔을 때 중간 목표점: 전 방위 kSubHeadings 방향마다 직진 통로가 비어 있는 길이를 재고, 그 끝 가까이의 점 중
// 사람(goal, 현재 base)에게 가장 많이 다가가는 점. 회전량이 클수록 조금 벌점. 없으면 false
bool pickSubgoal(const SafetyGate& gate, const ObstacleSet& scan, const ObstacleSet& depth, bool use_depth,
                 const Exclusion& scan_excl, const Exclusion& depth_excl, const Vec2& goal, Vec2* sub) {
  double best = -kInf;
  for (int i = 0; i < kSubHeadings; ++i) {
    const double psi = -M_PI + 2.0 * M_PI * i / kSubHeadings;
    // psi 방향을 앞으로 보는 좌표로 점을 돌려 직진 통로 거리를 잰다 (제외 구역도 같이 돌린다)
    const Pose2D rot{0.0, 0.0, -psi};
    ObstacleSet sc = scan;
    sc.to_now = compose(rot, scan.to_now);
    std::vector<Vec2> ex_s;
    for (std::size_t k = 0; k < scan_excl.count; ++k) {
      ex_s.push_back(transformPoint(rot, scan_excl.points[k]));
    }
    double free = gate.corridorDistance(sc, 0.0, true, Exclusion{ex_s.data(), ex_s.size()});
    if (use_depth) {
      ObstacleSet dp = depth;
      dp.to_now = compose(rot, depth.to_now);
      std::vector<Vec2> ex_d;
      for (std::size_t k = 0; k < depth_excl.count; ++k) {
        ex_d.push_back(transformPoint(rot, depth_excl.points[k]));
      }
      free = std::min(free, gate.corridorDistance(dp, 0.0, true, Exclusion{ex_d.data(), ex_d.size()}));
    }
    if (free < kSubMinFree) {
      continue;
    }
    const double d = std::min(free - kSubEndMargin, kSubMaxDist);
    const Vec2 q{d * std::cos(psi), d * std::sin(psi)};
    const double progress = norm(goal) - norm({goal.x - q.x, goal.y - q.y});
    if (progress < kSubMinProgress) {
      continue;
    }
    const double score = progress - kSubTurnWeight * std::abs(psi);
    if (score > best) {
      best = score;
      *sub = q;
    }
  }
  return best > -kInf;
}

// 정지 예측: 제어 주기와 같은 이산 S 곡선으로 앞질러 본다 (최대 10 s)
constexpr double kPredictDt = 0.05;
constexpr int kPredictMaxSteps = 200;

// 지금 선속도 v·가속도 a 에서 S 곡선(가속 상한 a_up, 감속 a_dn, jerk j)으로 0 까지 가는 거리 [m]·시간 [s]
void predictStop(double v, double a, double a_up, double a_dn, double j, double* dist,
                 double* time) {
  double d = 0.0;
  double t = 0.0;
  for (int i = 0; i < kPredictMaxSteps && v > 1e-3; ++i) {
    sCurveStep(0.0, a_up, a_dn, j, kPredictDt, &v, &a);
    d += std::max(0.0, v) * kPredictDt;
    t += kPredictDt;
  }
  *dist = d;
  *time = t;
}
// 근접 정지가 풀리는 여유 [m]
constexpr double kTooCloseHysteresis = 0.05;
// 카메라·depth 이상 상한이 나머지 목표보다 이만큼 낮아야 "그 상한이 걸렸다"로 본다 [m/s]
constexpr double kCapBindingEps = 1e-3;
// 편안 S 곡선으로 그 상한을 이보다 더 지나칠 참이면 kCap (가속을 바로 멈추고 감속을 빨리 건다) [m/s]
constexpr double kCapOverrunTol = 0.05;
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
  cap_active_ = false;
  brake_active_ = false;
  vp_f_ = 0.0;
  trail_.reset();
  trail_active_ = false;
  own_path_.clear();
  sub_active_ = false;
  blocked_prev_ = false;
  depth_only_until_ = -1.0;
  reversing_ = false;
  reverse_left_ = kReverseBudget;
  forward_since_reverse_ = 0.0;
  stuck_since_ = -1.0;
}

bool ControllerCore::onStart(double now) {
  if (stop_pending_) {
    // STOP 으로 서는 중에 START 가 다시 오면 정지를 거두고 그대로 따라간다 (FSM 은 아직 활성)
    stop_pending_ = false;
    return false;
  }
  if (!fsm_.start(now)) {
    return false;
  }
  resetMotion();
  return true;
}

bool ControllerCore::onStop(double now) {
  const FsmState s = fsm_.state();
  const bool moving = std::abs(limiter_.v()) > kStopMovingEps ||
                      std::abs(limiter_.w()) > kStopMovingEps;
  if ((s == FsmState::kFollowing || s == FsmState::kWaiting) && moving) {
    stop_pending_ = true;  // 다음 주기부터 단호 S 곡선으로 줄이고, 서면 IDLE (step)
    return false;
  }
  stop_pending_ = false;
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
  if (stop_pending_) {
    // STOP 으로 서는 중 HALT: 즉시 0 으로 서고 요청대로 IDLE (HALTED 로 남아 0 을 계속 내지 않게)
    stop_pending_ = false;
    fsm_.stop(now);
  }
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
    if (stop_pending_) {  // onHalt 와 같다
      stop_pending_ = false;
      fsm_.stop(now);
    }
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

  // --- 사람 발자취·조향 목표 (2026-11-13 실차 E2 갇힘) ---
  // 사람에게 곧장 가는 직선이 기둥·벽에 막히면 사람이 이미 지나간 길 위 앞쪽 점을 향한다.
  // 속도(거리 법칙)는 그대로 사람까지 거리로 정하고, 장애물 게이트·정지 프로파일이 최종 권한을 가진다
  if (in.odom_pose_ok && fresh_lock && !t.ambiguous) {
    trail_.add(transformPoint(in.odom_pose, t.position), kTrailSpacing, kTrailMaxPoints);
  }
  if (in.odom_pose_ok) {
    trail_.pruneBehind(Vec2{in.odom_pose.x, in.odom_pose.y});
  }
  const double excl_r = p_.corridor.target_exclusion_radius;
  auto losClearance = [&](const Vec2& goal) {
    double c = gate_.segmentClearance(in.scan, Vec2{0.0, 0.0}, goal, t.position, excl_r);
    if (in.sensors.depth_points_ok) {
      c = std::min(c, gate_.segmentClearance(in.depth, Vec2{0.0, 0.0}, goal, t.position, excl_r));
    }
    return c;
  };
  double steer_bearing = bearing;
  Vec2 steer_goal = t.position;  // 조향 목표 (현재 base): 사람 또는 발자취 위 앞쪽 점
  SteerMode steer = SteerMode::kDirect;
  if (p_.steer_policy >= 0.5 && tracked && in.odom_pose_ok && !trail_.empty() && range > p_.law.follow_distance) {
    const double need = p_.corridor.half_width + (trail_active_ ? kLosHysteresis : 0.0);
    if (losClearance(t.position) < need) {
      // 발자취 위 앞쪽 점: 가장 가까운 점부터 발자취를 따라 kTrailLookahead 안에서, 로봇에서 직선 시야가
      // 비어 있는 가장 먼 점. 그런 점이 없으면 kTrailMinCarrot 넘게 떨어진 첫 점
      const Pose2D to_base = inverse(in.odom_pose);
      const auto& tp = trail_.points();
      double s = 0.0;
      bool have = false;
      bool have_first = false;
      Vec2 carrot{};
      Vec2 first{};
      for (std::size_t i = 0; i < tp.size(); ++i) {
        if (i > 0) {
          s += norm({tp[i].x - tp[i - 1].x, tp[i].y - tp[i - 1].y});
        }
        if (s > kTrailLookahead) {
          break;
        }
        const Vec2 q = transformPoint(to_base, tp[i]);
        if (norm(q) < kTrailMinCarrot) {
          continue;
        }
        if (!have_first) {
          first = q;
          have_first = true;
        }
        if (losClearance(q) >= p_.corridor.half_width) {
          carrot = q;
          have = true;
        }
      }
      if (!have && have_first) {
        carrot = first;
        have = true;
      }
      if (have) {
        steer_bearing = std::atan2(carrot.y, carrot.x);
        steer_goal = carrot;
        steer = SteerMode::kTrail;
      }
    }
  }
  trail_active_ = (steer == SteerMode::kTrail);

  // --- 막혔을 때 중간 목표점 (2026-11-16 갇힘 개선 3단계) ---
  // 지난 주기에 사람에게 가려는 경로가 막혔으면(blocked_prev_), 전 방위에서 직진 통로가 비어 있고 사람에게 다가가는
  // 점을 골라 그쪽으로 (멀면 제자리에서 돌고) 간다. 도착·5 s·사람이 다시 보이면 끝. 속도·간격은 그대로 사람 기준
  const bool person_far_pre = tracked && !t.ambiguous && range > p_.law.follow_distance + 0.1;
  if (sub_active_) {
    const Vec2 sb = in.odom_pose_ok ? transformPoint(inverse(in.odom_pose), sub_odom_) : Vec2{0.0, 0.0};
    const bool visible = losClearance(t.position) >= p_.corridor.half_width + kLosHysteresis;
    if (!in.odom_pose_ok || !person_far_pre || norm(sb) < kSubReach || now - sub_since_ > kSubTimeoutS || visible) {
      sub_active_ = false;
    }
  }
  if (p_.steer_policy >= 1.5 && !sub_active_ && blocked_prev_ && !reversing_ && person_far_pre && in.odom_pose_ok &&
      fsm_.state() == FsmState::kFollowing) {
    Vec2 sb{};
    const Exclusion se = has_geom ? Exclusion{&t.position, 1} : Exclusion{};
    const Exclusion de = (has_geom && in.depth_exclusion.count > 0) ? in.depth_exclusion : se;
    if (pickSubgoal(gate_, in.scan, in.depth, in.sensors.depth_points_ok, se, de, t.position, &sb)) {
      sub_active_ = true;
      sub_since_ = now;
      sub_odom_ = transformPoint(in.odom_pose, sb);
    }
  }
  if (sub_active_) {
    const Vec2 sb = transformPoint(inverse(in.odom_pose), sub_odom_);
    steer_goal = sb;
    steer_bearing = std::atan2(sb.y, sb.x);
    steer = SteerMode::kAvoid;
  }

  FollowCommand law_cmd;
  if (tracked) {
    // 제동 상한에 넣는 사람 속도: 신선한 측정일 때만, 걸러진 값과 지금 값 중 작은 쪽
    // (속도 추정 잡음·급가속에 늘지 않고, 사람이 느려지면 바로 줄어든다. 가림 중에는 0 = 바로 선다고 봄)
    const double vp_now = std::max(0.0, t.velocity.x);
    const double avp = (p_.v_filter_s > 0.0) ? dt / (p_.v_filter_s + dt) : 1.0;
    vp_f_ += avp * (vp_now - vp_f_);
    const double brake_vp = fresh_lock ? std::min(vp_now, vp_f_) : 0.0;
    law_cmd = law_.compute(range, steer_bearing, t.velocity.x, brake_vp);
  } else {
    vp_f_ = 0.0;
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
  double w_des = law_w;

  // --- 막힘 판정·지나온 길 후진 (2026-11-16 갇힘 개선 3·4단계) ---
  // 사람이 멀리 있는데(따라갈 거리 + 0.1 m 넘게) 앞으로 가려는 경로가 막혔거나, 돌아서야 하는데 회전이 막혔다.
  // 다음 주기 중간 목표점을 고르는 데 쓴다. 회전도 막혔으면(벽에 붙음) 로봇이 이미 지나온 길만 거꾸로 되짚는다
  // (한 번 갇힐 때 0.3 m 까지, 0.1 m/s, 뒤 통로 확인. 직전에 지나온 바닥이라 낭떠러지가 없다)
  const FsmState s_pre = fsm_.state();
  if (in.odom_pose_ok && limiter_.v() > kOwnPathMinSpeed) {
    const Vec2 here{in.odom_pose.x, in.odom_pose.y};
    if (own_path_.empty() || norm({here.x - own_path_.back().x, here.y - own_path_.back().y}) >= kOwnPathSpacing) {
      if (!own_path_.empty()) {
        forward_since_reverse_ += norm({here.x - own_path_.back().x, here.y - own_path_.back().y});
      }
      own_path_.push_back(here);
      if (own_path_.size() > kOwnPathMaxPoints) {
        own_path_.erase(own_path_.begin());
      }
    }
    if (forward_since_reverse_ >= kReverseRearmForward) {
      reverse_left_ = kReverseBudget;
    }
  }
  const bool person_far = tracked && !t.ambiguous && !too_close && range > p_.law.follow_distance + 0.1;
  const bool fwd_blocked = law_cmd.v > kAvoidWantSpeed && (cap_obs < kAvoidBlockedSpeed || obs_emergency);
  const bool turn_blocked = (law_cmd.in_place || law_v <= p_.in_place_linear_eps) &&
                            std::abs(law_w) > p_.in_place_angular_eps && r_min <= p_.rotation_clearance;
  const bool blocked = person_far && (fwd_blocked || turn_blocked);
  // depth 로만 보이는 가까운 물체 (라이다보다 depth 가 더 가깝게 본다)
  if (use_depth && d_depth < kDepthOnlyNear && d_depth + 0.05 < d_scan) {
    depth_only_until_ = now + kDepthOnlyHoldS;
  }
  const bool depth_hold = now < depth_only_until_;
  if (depth_hold) {
    sub_active_ = false;  // 이미 쫓던 중간 목표점도 버린다 (그 물체 쪽일 수 있다)
  }
  blocked_prev_ = blocked && !sub_active_ && !depth_hold;
  if (steer == SteerMode::kAvoid) {
    v_des = std::min(v_des, kAvoidSpeed);  // 중간 목표점으로는 천천히
  }
  bool reverse = false;
  double reverse_kappa = 0.0;
  if (p_.steer_policy >= 1.5 && s_pre == FsmState::kFollowing && in.odom_pose_ok && !depth_hold &&
      ((blocked && r_min <= p_.rotation_clearance) || reversing_)) {
    if (stuck_since_ < 0.0) {
      stuck_since_ = now;
    }
    // 되짚을 목표: 지나온 길에서 지금 위치로부터 kReverseLookBack 뒤의 점 (현재 base)
    const Pose2D to_base = inverse(in.odom_pose);
    Vec2 back{};
    bool have_back = false;
    double acc = 0.0;
    for (std::size_t i = own_path_.size(); i-- > 1;) {
      acc += norm({own_path_[i].x - own_path_[i - 1].x, own_path_[i].y - own_path_[i - 1].y});
      if (acc >= kReverseLookBack) {
        back = transformPoint(to_base, own_path_[i - 1]);
        have_back = true;
        break;
      }
    }
    if (have_back && back.x < -0.05 && reverse_left_ > 0.0 && (reversing_ || now - stuck_since_ >= kReverseAfterS)) {
      // pure pursuit (뒤로): 목표점을 지나는 원호 곡률. 뒤 통로는 π 돌린 좌표에서 곡률 −kappa 로 앞처럼 잰다
      reverse_kappa = 2.0 * back.y / std::max(1e-6, back.x * back.x + back.y * back.y);
      reverse_kappa = std::clamp(reverse_kappa, -p_.corridor.max_curvature, p_.corridor.max_curvature);
      ObstacleSet rear_scan = in.scan;
      rear_scan.to_now = compose(Pose2D{0.0, 0.0, M_PI}, in.scan.to_now);
      double d_rear = gate_.corridorDistance(rear_scan, -reverse_kappa, true, Exclusion{});
      if (use_depth) {
        ObstacleSet rear_depth = in.depth;
        rear_depth.to_now = compose(Pose2D{0.0, 0.0, M_PI}, in.depth.to_now);
        d_rear = std::min(d_rear, gate_.corridorDistance(rear_depth, -reverse_kappa, true, Exclusion{}));
      }
      reverse = d_rear >= std::min(reverse_left_, kReverseLookBack) + kReverseRearClear;
    }
  } else if (!blocked) {
    stuck_since_ = -1.0;
  }
  if (reverse) {
    if (!reversing_) {
      forward_since_reverse_ = 0.0;
      sub_active_ = false;  // 되짚은 뒤 새로 고른다
    }
    reversing_ = true;
    reverse_left_ -= std::abs(limiter_.v()) * dt;
    v_des = -kReverseSpeed;
    w_des = std::clamp(v_des * reverse_kappa, -p_.limiter.max_angular_speed, p_.limiter.max_angular_speed);
    // 회전할 자리가 생겼거나 총량을 다 쓰면 끝 (다음 주기부터 중간 목표점·제자리 회전으로)
    if (reverse_left_ <= 0.0 || r_min > p_.rotation_clearance + 0.05) {
      reversing_ = false;
      stuck_since_ = -1.0;
      blocked_prev_ = true;
    }
    steer = SteerMode::kReverse;
  } else {
    reversing_ = false;
  }
  const bool avoid = (steer == SteerMode::kAvoid);

  // --- 회전 (S4) ---
  // 제자리 회전 요청: 명령 선속도가 거의 0 이거나, 법칙이 제자리 회전을 요구할 때 (걸러진 선속도가
  // 줄어드는 몇 초 동안에도 회전 여유 검사를 건다, 리뷰 2026-11-13)
  const bool in_place_req =
      tracked && (v_des <= p_.in_place_linear_eps || law_cmd.in_place) &&
      std::abs(w_des) > p_.in_place_angular_eps;
  const bool rot_blocked = in_place_req && r_min <= p_.rotation_clearance && !avoid && !reverse;
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
  FsmState s = fsm_.state();  // STOP 의 S 곡선 정지가 이번 주기에 끝나면 IDLE 로 바뀐다 (아래 보고에 반영)
  out.state_changed = (s != prev);
  if (out.state_changed && s != FsmState::kFollowing &&
      s != FsmState::kWaiting) {
    resetMotion();
  }

  // --- 명령 ---
  double v = 0.0;
  double w = 0.0;
  bool publish = false;
  // 사람 앞 제동 여유 (2026-11-13 실차 E2): 제동 상한은 로봇이 이미 가속 중인 것을 모른다. 사람을 따라잡느라
  // 0.85 m/s 로 가속하던 중 사람이 서자, 편안 S 곡선은 가속을 되돌리고 감속을 채우는 데만 1 s 남짓 걸려
  // 사람 앞 0.54 m 까지 다가갔다 (설계 0.8 m). 지금 (v, a) 에서 편안 S 곡선으로 서면 사람까지 남을 거리를
  // 앞질러 계산한다. 사람이 다가오는 속도(잡음 문턱 넘는 만큼)도 넣는다
  double brake_gap = kInf;
  if (tracked && limiter_.v() > kStoppedSpeed) {
    const LimiterParams& L = p_.limiter;
    double d_stop = 0.0;
    double t_stop = 0.0;
    predictStop(limiter_.v(), limiter_.linearAccel(), L.max_linear_accel, L.max_linear_decel,
                L.max_linear_jerk, &d_stop, &t_stop);
    const double v_close = std::max(0.0, -t.velocity.x - kApproachNoise);
    const double t_r = p_.law.reaction_time;
    brake_gap = range - limiter_.v() * t_r - d_stop - v_close * (t_r + t_stop);
  }
  const double stop_point = p_.law.stop_distance + p_.law.approach_margin;  // 편안 정지가 서야 할 곳
  // 1단: 편안 정지가 stop_point 를 넘을 참이면 더 가속하지 않는다 (편안 S 곡선으로 가속을 줄인다)
  const bool hold_accel = p_.brake_margin >= 0.5 && brake_gap < stop_point;
  // 2단: 편안 정지로는 stop_distance + approach_margin/2 안까지 들어갈 참이면 단호 S 곡선으로 줄인다
  const bool brake_need = p_.brake_margin >= 0.5 &&
      brake_gap < p_.law.stop_distance + kBrakeMarginFrac * p_.law.approach_margin;
  const bool stopping = stop_pending_ && (s == FsmState::kFollowing || s == FsmState::kWaiting);
  // 후진은 지나온 길 되짚기(4단계)를 고른 FOLLOWING 주기에만. 그 밖에는 남은 후진 속도가 S 곡선으로 0 으로 돌아온다
  limiter_.setAllowReverse(reverse && s == FsmState::kFollowing && !stopping);
  if (stopping) {
    // STOP 명령: 선·각 모두 단호 S 곡선으로 0 까지 (HALT 는 위에서 즉시 0). 서면 IDLE (0 몇 번 뒤 침묵)
    limiter_.step(0.0, 0.0, dt, StopProfile::kFirm, StopProfile::kFirm);
    v = limiter_.v();
    w = limiter_.w();
    publish = true;
    if (std::abs(v) <= kStopMovingEps && std::abs(w) <= kStopMovingEps) {
      stop_pending_ = false;
      fsm_.stop(now);
      resetMotion();
      v = 0.0;
      w = 0.0;
      publish = fsm_.consumeZeroBurst();  // IDLE 진입 때와 같은 0 세 번 중 첫 번째
      out.state_changed = true;
      s = fsm_.state();
    }
  } else switch (s) {
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
                          too_close || brake_need;
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
      if (v_t >= 0.0 && limiter_.v() < kStoppedSpeed && v_t < p_.start_speed) {
        v_t = 0.0;
      }
      // S2·S3: 단호 정지 (S 곡선)
      // 국소 회피·후진 중의 장애물 비상은 법칙 경로(막힌 쪽) 판정이라 쓰지 않는다 (원호·뒤 통로는 따로 확인했다)
      const bool firm_v = too_close || (obs_emergency && !avoid && !reverse);
      // 카메라·depth 이상 상한(S6, 0.3 m/s)은 뒤를 받치는 단호 정지가 없다(낮은 장애물이 안 보인다).
      // 그 상한이 걸렸는데 편안 S 곡선으로는 kCapOverrunTol 넘게 지나칠 참이면 kCap (리뷰 2026-11-13).
      // 사람 앞 제동·장애물 상한은 근접(S2)·장애물 비상(S3) 단호 정지가 받치므로 편안 S 곡선 그대로 둔다
      const double v_soft = std::min(
          {law_v * occl_factor, cap_obs, v_max});  // 카메라 상한만 뺀 목표 (실행 중 낮춘 상한은 편안 S 곡선)
      const double a_up = std::max(limiter_.linearAccel(), 0.0);
      const double j_c = p_.limiter.max_linear_jerk;
      const double v_peak =
          limiter_.v() + ((j_c > 0.0) ? a_up * a_up / (2.0 * j_c) : 0.0);
      const bool cap_binding = v_t + kCapBindingEps < v_soft;
      if (!cap_binding || limiter_.v() <= v_t + kCapBindingEps) {
        cap_active_ = false;  // 상한이 풀렸거나 상한에 닿았다
      } else if (v_peak > v_t + kCapOverrunTol) {
        cap_active_ = true;
      }
      if (hold_accel) {
        v_t = std::min(v_t, limiter_.v());
      }
      // 2단은 한 번 걸리면 여유가 stop_point 로 돌아오거나 설 때까지 단호 S 곡선으로 줄인다 (중간에 바꾸면 출렁인다)
      if (brake_need) {
        brake_active_ = true;
      } else if (brake_gap >= stop_point || limiter_.v() <= kStoppedSpeed) {
        brake_active_ = false;
      }
      if (brake_active_) {
        v_t = 0.0;
      }
      limiter_.step(v_t, w_des, dt,
                    (firm_v || brake_active_) ? StopProfile::kFirm
                           : (cap_active_ ? StopProfile::kCap : StopProfile::kComfort),
                    StopProfile::kComfort);
      v = limiter_.v();
      w = limiter_.w();
      publish = true;
      break;
    }
  }

  // S4: 제자리 회전 연속 상한
  if (s == FsmState::kFollowing && std::abs(w) > p_.in_place_angular_eps &&
      std::abs(v) <= p_.in_place_linear_eps) {
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
  if (publish && std::abs(w) > kZeroAngular && v >= 0.0 && v < p_.rotation_creep_speed) {
    v = p_.rotation_creep_speed;
  }

  out.publish = publish;
  out.v = v;
  out.w = w;
  out.state = s;
  out.mode = modeOf(s);
  out.steer_mode = steer;

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
