// follow_core_sim: ControllerCore 빠른 폐루프 시뮬레이터 (ROS 없음, 실시간보다 훨씬 빠름).
// 부드러움(가가속도·출발-정지 반복)과 거리 유지 튜닝용. 안전 시나리오 전체는 ROS 수준 시뮬레이터로 따로 본다.
//
// - 로봇: 외바퀴. 명령 지연(선 0.04 s, 각 0.08 s) + 1차 지연(0.05 s) + 베이스 가감속 한계, 모터 불감대 0.01 m/s
//   (T3 실측: 명령→실제 지연 선 약 0.09 s, 각 약 0.13 s)
// - 사람: 경유점을 따라 걷는다 (가감속 1.0 m/s^2). 보행 위상에 따라 다리가 앞뒤로 엇갈린다
// - 추적기 대역: 10 Hz, stamp 는 측정 0.05 s 전, 수신은 stamp + 0.03 s. 잡음 2.5 cm,
//   걸을 때 30 % 는 한쪽 다리만 잡혀 그 다리 쪽으로 치우친다. 속도는 0.35 s EMA.
//   가림 창: OCCLUDED(예측) → 2 s 뒤 LOST. 모호 창: ambiguous
// - 라이다: 벽 선분과 사람 다리 원(반경 6 cm)에 0.9° 광선. 0.2~4 m 점만 넘긴다 (노드와 같음)
//
// usage: follow_core_sim [--scenario NAME|all] [--set name=value]... [--out DIR] [--seed N] [--quiet]
//   --set 은 tunables.hpp 의 이름 (예: max_linear_jerk=1.5)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "kebi_follow_controller/controller_core.hpp"
#include "kebi_follow_controller/tunables.hpp"

using namespace kebi_follow;

namespace {

constexpr double kCtrlDt = 0.05;        // 컨트롤러 주기 [s]
constexpr double kPhysDt = 0.005;       // 물리 [s]
constexpr double kTrackDt = 0.1;        // 추적기 [s]
constexpr double kStampLag = 0.05;      // 측정 → stamp [s]
constexpr double kRxLag = 0.03;         // stamp → 수신 [s]
constexpr double kCmdDelayV = 0.04;     // [s]
constexpr double kCmdDelayW = 0.08;     // [s]
constexpr double kBaseTau = 0.05;       // [s]
constexpr double kBaseAcc = 1.0, kBaseDec = 1.5, kBaseWAcc = 3.0;
constexpr double kMotorDeadband = 0.01;  // [m/s]
constexpr double kLegR = 0.06, kLegSpread = 0.10, kSwing = 0.15;
constexpr double kNoise = 0.025;
constexpr double kSingleLegProb = 0.3;
constexpr double kAlpha = 0.45, kBeta = 0.12;  // 추적기 대역 alpha-beta 이득
constexpr double kLostAfter = 2.0;
constexpr double kMaxExtrap = 0.5;
constexpr double kRobotR = 0.22;
constexpr int kRays = 400;
constexpr double kScanMin = 0.2, kScanMax = 4.0;
constexpr double kStartAt = 0.5;

struct Seg {
  double x0, y0, x1, y1;
};
struct Waypoint {
  double t, x, y, speed;
};
struct Window {
  double t0, t1;
};
struct Scenario {
  std::string name;
  std::string desc;
  double duration;
  double px, py;  // 사람 시작 (로봇은 원점, +x 방향)
  std::vector<Waypoint> wps;
  std::vector<Seg> walls;
  std::vector<Window> occl;
  std::vector<Window> amb;
};

bool inside(const std::vector<Window>& ws, double t) {
  for (const auto& w : ws) {
    if (t >= w.t0 && t < w.t1) return true;
  }
  return false;
}

struct Person {
  double x, y, vx{0}, vy{0}, heading{0}, phase{0};
  // speed < 0 인 경유점: 로봇 쪽으로 |speed| 로 다가와 로봇 중심 0.6 m 앞에서 선다
  void step(double t, const std::vector<Waypoint>& wps, double dt, double rx, double ry) {
    double tvx = 0, tvy = 0;
    const Waypoint* act = nullptr;
    for (const auto& w : wps) {
      if (w.t <= t) act = &w;
    }
    if (act) {
      double gx = act->x, gy = act->y, sp_max = act->speed;
      if (act->speed < 0) {
        const double dxr = x - rx, dyr = y - ry, dr = std::max(std::hypot(dxr, dyr), 1e-6);
        gx = rx + dxr / dr * 0.6;
        gy = ry + dyr / dr * 0.6;
        sp_max = -act->speed;
      }
      const double dx = gx - x, dy = gy - y, d = std::hypot(dx, dy);
      if (d > 0.02) {
        const double sp = std::min(sp_max, std::sqrt(2.0 * 1.0 * d));
        tvx = dx / d * sp;
        tvy = dy / d * sp;
      }
    }
    double ax = tvx - vx, ay = tvy - vy;
    const double n = std::hypot(ax, ay), lim = 1.0 * dt;
    if (n > lim) {
      ax *= lim / n;
      ay *= lim / n;
    }
    vx += ax;
    vy += ay;
    x += vx * dt;
    y += vy * dt;
    const double spd = std::hypot(vx, vy);
    if (spd > 0.05) heading = std::atan2(vy, vx);
    phase += 2.0 * M_PI * 1.8 * dt * std::min(1.0, spd / 0.5);
  }
  void legs(double* l) const {  // l[4] = x1,y1,x2,y2
    const double fx = std::cos(heading), fy = std::sin(heading), sx = -fy, sy = fx;
    const double sw = kSwing * std::min(1.0, std::hypot(vx, vy) / 0.5) * std::sin(phase);
    l[0] = x + sx * kLegSpread + fx * sw;
    l[1] = y + sy * kLegSpread + fy * sw;
    l[2] = x - sx * kLegSpread - fx * sw;
    l[3] = y - sy * kLegSpread - fy * sw;
  }
};

struct Robot {
  double x{0}, y{0}, yaw{0}, v{0}, w{0};
  std::deque<std::pair<double, double>> qv, qw;  // (적용 시각, 명령)
  double cv{0}, cw{0};
  void command(double t, double v_cmd, double w_cmd) {
    qv.push_back({t + kCmdDelayV, std::abs(v_cmd) < kMotorDeadband ? 0.0 : v_cmd});
    qw.push_back({t + kCmdDelayW, w_cmd});
  }
  void step(double t, double dt) {
    while (!qv.empty() && qv.front().first <= t) {
      cv = qv.front().second;
      qv.pop_front();
    }
    while (!qw.empty() && qw.front().first <= t) {
      cw = qw.front().second;
      qw.pop_front();
    }
    double dv = (cv - v) * dt / kBaseTau;
    dv = std::clamp(dv, -kBaseDec * dt, kBaseAcc * dt);
    v += dv;
    double dw = (cw - w) * dt / kBaseTau;
    dw = std::clamp(dw, -kBaseWAcc * dt, kBaseWAcc * dt);
    w += dw;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw += w * dt;
  }
};

// 광선 a + t*d 가 선분/원과 만나는 거리
double raySeg(double ox, double oy, double dx, double dy, const Seg& s) {
  const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
  const double den = dx * ey - dy * ex;
  if (std::abs(den) < 1e-12) return 1e9;
  const double t = ((s.x0 - ox) * ey - (s.y0 - oy) * ex) / den;
  const double u = ((s.x0 - ox) * dy - (s.y0 - oy) * dx) / den;
  return (t > 0 && u >= 0 && u <= 1) ? t : 1e9;
}
double rayCircle(double ox, double oy, double dx, double dy, double cx, double cy, double r) {
  const double fx = ox - cx, fy = oy - cy;
  const double b = fx * dx + fy * dy, c = fx * fx + fy * fy - r * r;
  const double disc = b * b - c;
  if (disc < 0) return 1e9;
  const double t = -b - std::sqrt(disc);
  return t > 0 ? t : 1e9;
}

double distPointSeg(double px, double py, const Seg& s) {
  const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
  const double L2 = ex * ex + ey * ey;
  double t = L2 > 0 ? ((px - s.x0) * ex + (py - s.y0) * ey) / L2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(px - (s.x0 + t * ex), py - (s.y0 + t * ey));
}

std::vector<Scenario> scenarios() {
  std::vector<Scenario> S;
  // 1 걷고 서고 다시 걷기
  S.push_back({"walk_stop_walk", "0.6 m/s 6 s → 4 s 정지 → 0.9 m/s → 정지", 24.0, 1.2, 0.0,
               {{1.0, 4.8, 0.0, 0.6}, {11.0, 10.2, 0.0, 0.9}}, {}, {}, {}});
  // 2 느리고 불규칙한 보행 (T4 의 덜컥거림 재현)
  {
    Scenario s{"slow_irregular", "0.25~0.45 m/s 로 걷다 0.5~1.5 s 씩 멈추기 반복", 30.0, 1.2, 0.0, {}, {}, {}, {}};
    double t = 1.0, x = 1.2;
    const double sp[] = {0.35, 0.25, 0.45, 0.3, 0.4, 0.25, 0.35, 0.45};
    const double walk[] = {2.0, 1.5, 2.5, 1.0, 2.0, 1.5, 2.0, 2.0};
    const double pause[] = {1.0, 0.5, 1.5, 0.8, 1.2, 0.6, 1.0, 1.0};
    for (int i = 0; i < 8; ++i) {
      x += sp[i] * walk[i];
      s.wps.push_back({t, x, 0.0, sp[i]});
      t += walk[i] + pause[i] + 0.4;
    }
    S.push_back(s);
  }
  // 3 빠른 보행 (상한보다 빠름)
  S.push_back({"fast_walk", "1.2 m/s 로 9 m → 정지", 20.0, 1.2, 0.0, {{1.0, 10.2, 0.0, 1.2}}, {}, {}, {}});
  // 4 왼쪽 90° 회전
  S.push_back({"turn_left_90", "0.6 m/s, 3 m 에서 왼쪽 90°", 22.0, 1.2, 0.0,
               {{1.0, 4.2, 0.0, 0.6}, {7.0, 4.2, 4.0, 0.6}}, {}, {}, {}});
  // 5 옆으로 비켜서기 (방위 약 70°) 후 돌아오기
  S.push_back({"side_step", "옆(약 70°)으로 비켜섰다가 앞으로 돌아오기", 16.0, 1.2, 0.0,
               {{2.0, 0.45, 1.15, 0.8}, {8.0, 1.2, 0.0, 0.8}}, {}, {}, {}});
  // 6 사람이 로봇 쪽으로 다가옴
  S.push_back({"approach_back", "걸어가다 돌아서 로봇 쪽으로 0.5 m/s", 16.0, 1.2, 0.0,
               {{1.0, 3.6, 0.0, 0.6}, {7.0, 0.0, 0.0, -0.5}}, {}, {}, {}});
  // 7 짧은 가림 후 다시 측정
  S.push_back({"occlusion_resume", "0.6 m/s 중 1 s 가림", 16.0, 1.2, 0.0, {{1.0, 9.0, 0.0, 0.6}}, {}, {{6.0, 7.0}}, {}});
  // 8 이동 중 놓침
  S.push_back({"lost_while_moving", "0.7 m/s 중 6 s 부터 계속 가림 → 놓침", 14.0, 1.2, 0.0, {{1.0, 9.0, 0.0, 0.7}}, {}, {{6.0, 99.0}}, {}});
  // 9 잠깐 모호
  S.push_back({"ambiguous_blip", "0.6 m/s 중 0.5 s, 1 s 모호", 18.0, 1.2, 0.0, {{1.0, 10.0, 0.0, 0.6}}, {}, {}, {{6.0, 6.5}, {10.0, 11.0}}});
  // 10 벽 앞에서 사람이 꺾음
  S.push_back({"wall_ahead", "벽 1 m 앞에서 사람이 오른쪽으로 꺾음", 22.0, 1.2, 0.0,
               {{1.0, 4.5, 0.0, 0.6}, {8.0, 4.5, -4.0, 0.6}},
               {{5.5, -6.0, 5.5, 3.0}}, {}, {}});
  // 11 로봇 둘레를 도는 사람 (제자리 회전)
  {
    Scenario s{"circle_around", "반경 1.3 m 로 로봇 둘레를 0.6 m/s 로 반 바퀴", 16.0, 1.3, 0.0, {}, {}, {}, {}};
    for (int k = 1; k <= 12; ++k) {
      const double a = M_PI * k / 12.0;
      s.wps.push_back({1.0 + 0.6 * (k - 1), 1.3 * std::cos(a), 1.3 * std::sin(a), 0.6});
    }
    S.push_back(s);
  }
  return S;
}

struct Metrics {
  double max_acc_cmd{0}, max_jerk_cmd{0}, rms_jerk_cmd{0};
  double max_acc_act{0}, max_jerk_act{0};
  double max_wacc_cmd{0}, max_wjerk_cmd{0}, rms_wjerk_cmd{0};
  int stop_go{0};         // 사람이 걷는 중 로봇이 섰다가 다시 출발한 횟수
  int firm{0};            // 근접·장애물 단호 정지가 새로 걸린 횟수
  double max_jerk_moving{0};  // |v| > 0.05 m/s 일 때 최대 jerk (0 근처 착지 흔적 제외)
  double tgt_err_p95{0};      // 추적기 대역 위치 오차 p95 (실측 0.095 m 와 비교)
  double start_lag{0};        // 사람이 출발(0.2 m/s 넘음)한 뒤 로봇 명령이 0.1 m/s 를 넘기까지 평균 [s]
  double bearing_p95{0};      // 따라가는 중 사람 방향 |방위| p95 [deg]
  double min_dist{1e9}, mean_dist{0}, p95_dist{0};
  double min_wall{1e9};
  double follow_frac{0}, wait_frac{0};
  double max_v_cmd{0}, max_w_cmd{0};
};

Metrics run(const Scenario& sc, const ControllerParams& cp, unsigned seed, const std::string& csv_path) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> N(0.0, 1.0);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  ControllerCore core(cp);
  Robot rb;
  Person ps{sc.px, sc.py};
  ps.heading = 0;
  struct Hist {
    double t, x, y;
  };
  std::deque<Hist> ph;  // 사람 위치 이력 (stamp 시점 조회)
  // 추적기 상태
  bool have_meas = false;
  double m_stamp = 0, m_rx = 0, m_x = 0, m_y = 0, m_vx = 0, m_vy = 0, last_meas_t = -1, prev_mx = 0, prev_my = 0, prev_mt = -1;
  bool ever_lost = false;
  double next_track = kTrackDt, next_ctrl = kCtrlDt;
  bool started = false;
  std::vector<double> T, VC, WC, VA, D;
  std::vector<int> F;
  std::vector<unsigned> SR;
  std::vector<double> terr;  // 추적기 대역 위치 오차 (검증용)
  std::vector<double> PSPD, BRG;
  std::ofstream csv;
  if (!csv_path.empty()) {
    csv.open(csv_path);
    csv << "t,person_x,person_y,person_v,robot_x,robot_y,robot_yaw,v_cmd,w_cmd,v_act,w_act,fsm,stop,range,target_status,amb\n";
  }
  Metrics M;
  std::vector<Vec2> scan_pts;
  scan_pts.reserve(kRays);
  int stopped_cycles = 0;
  bool was_moving = false;
  for (double t = 0.0; t < sc.duration; t += kPhysDt) {
    ps.step(t, sc.wps, kPhysDt, rb.x, rb.y);
    ph.push_back({t, ps.x, ps.y});
    while (!ph.empty() && ph.front().t < t - 1.0) ph.pop_front();
    rb.step(t, kPhysDt);
    for (const auto& w : sc.walls) {
      M.min_wall = std::min(M.min_wall, distPointSeg(rb.x, rb.y, w) - kRobotR);
    }
    // 추적기 (10 Hz)
    if (t >= next_track) {
      next_track += kTrackDt;
      const double ts = t - kStampLag;
      const bool occluded = inside(sc.occl, ts);
      if (!occluded) {
        // stamp 시점 사람 위치
        double hx = ps.x, hy = ps.y;
        for (auto it = ph.rbegin(); it != ph.rend(); ++it) {
          if (it->t <= ts) {
            hx = it->x;
            hy = it->y;
            break;
          }
        }
        double mx = hx, my = hy;
        const double spd = std::hypot(ps.vx, ps.vy);
        if (spd > 0.15 && U(rng) < kSingleLegProb) {  // 한쪽 다리만
          double l[4];
          ps.legs(l);
          const int k = (U(rng) < 0.5) ? 0 : 2;
          mx = l[k] + (hx - ps.x);
          my = l[k + 1] + (hy - ps.y);
        }
        mx += kNoise * N(rng);
        my += kNoise * N(rng);
        if (prev_mt >= 0) {
          // alpha-beta (등속 칼만 필터 흉내): 예측 후 잔차로 위치·속도 보정
          const double dtm = ts - prev_mt;
          const double xp = prev_mx + m_vx * dtm, yp = prev_my + m_vy * dtm;
          const double rx = mx - xp, ry = my - yp;
          mx = xp + kAlpha * rx;
          my = yp + kAlpha * ry;
          m_vx += kBeta * rx / dtm;
          m_vy += kBeta * ry / dtm;
        }
        prev_mx = mx;
        prev_my = my;
        prev_mt = ts;
        m_x = mx;
        m_y = my;
        terr.push_back(std::hypot(mx - hx, my - hy));
        m_stamp = ts;
        m_rx = ts + kRxLag;
        last_meas_t = ts;
        have_meas = true;
      } else if (have_meas) {
        m_rx = t;  // OCCLUDED 메시지도 계속 온다 (예측 위치)
      }
    }
    // 컨트롤러 (20 Hz)
    if (t >= next_ctrl) {
      next_ctrl += kCtrlDt;
      if (!started && t >= kStartAt) {
        core.onStart(t);
        started = true;
      }
      CycleInput in;
      in.now = t;
      in.dt = kCtrlDt;
      in.measured_speed = rb.v;
      in.sensors.scan_ok = in.sensors.odom_ok = in.sensors.tf_ok = true;
      in.sensors.bottom_ir_ok = in.sensors.lifted_ok = in.sensors.fall_down_ok = true;
      in.sensors.camera_ok = true;
      in.sensors.depth_points_ok = false;
      // 타깃
      if (have_meas) {
        const bool occl_now = inside(sc.occl, t - kStampLag);
        const double occl_time = occl_now ? std::max(0.0, (t - kStampLag) - last_meas_t) : 0.0;
        TargetInput& ti = in.target;
        ti.received = true;
        if (occl_now && occl_time > kLostAfter) {
          ti.status = TargetStatus::kLost;
          ever_lost = true;
        } else if (occl_now) {
          ti.status = TargetStatus::kOccluded;
        } else {
          ti.status = TargetStatus::kLocked;
        }
        ti.ambiguous = inside(sc.amb, t);
        ti.msg_age = std::max(0.0, t - m_rx);
        ti.occluded_time = occl_time;
        // 외삽 (노드와 같이 stamp 이후 경과만큼, 상한 0.5 s) 후 현재 로봇 좌표로
        const double age = std::min(kMaxExtrap, t - m_stamp);
        const double wx = m_x + m_vx * age, wy = m_y + m_vy * age;
        const double c = std::cos(rb.yaw), s = std::sin(rb.yaw);
        const double dx = wx - rb.x, dy = wy - rb.y;
        ti.position = {c * dx + s * dy, -s * dx + c * dy};
        ti.velocity = {c * m_vx + s * m_vy, -s * m_vx + c * m_vy};
      }
      // 라이다
      scan_pts.clear();
      double l[4];
      ps.legs(l);
      for (int i = 0; i < kRays; ++i) {
        const double a = rb.yaw + 2.0 * M_PI * i / kRays;
        const double dx = std::cos(a), dy = std::sin(a);
        double r = 1e9;
        for (const auto& w : sc.walls) r = std::min(r, raySeg(rb.x, rb.y, dx, dy, w));
        r = std::min(r, rayCircle(rb.x, rb.y, dx, dy, l[0], l[1], kLegR));
        r = std::min(r, rayCircle(rb.x, rb.y, dx, dy, l[2], l[3], kLegR));
        if (r > kScanMin && r < kScanMax) {
          const double rel = 2.0 * M_PI * i / kRays;
          scan_pts.push_back({r * std::cos(rel), r * std::sin(rel)});
        }
      }
      in.scan = ObstacleSet{scan_pts.data(), scan_pts.size(), Pose2D{}};
      const CycleOutput o = core.step(in);
      const double v_cmd = o.publish ? o.v : 0.0, w_cmd = o.publish ? o.w : 0.0;
      if (o.publish) rb.command(t, v_cmd, w_cmd);
      const double pd = std::hypot(ps.x - rb.x, ps.y - rb.y);
      T.push_back(t);
      VC.push_back(v_cmd);
      WC.push_back(w_cmd);
      VA.push_back(rb.v);
      D.push_back(pd);
      F.push_back(static_cast<int>(o.state));
      SR.push_back(o.stop_reasons);
      PSPD.push_back(std::hypot(ps.vx, ps.vy));
      {
        const double c = std::cos(rb.yaw), sn = std::sin(rb.yaw), dx = ps.x - rb.x, dy = ps.y - rb.y;
        BRG.push_back(std::abs(std::atan2(-sn * dx + c * dy, c * dx + sn * dy)) * 180.0 / M_PI);
      }
      // 사람이 걷는 중에 로봇이 0.6 s 이상 섰다가 다시 출발
      const double pspd = std::hypot(ps.vx, ps.vy);
      if (v_cmd > 0.05) {
        if (!was_moving && stopped_cycles >= 12 && pspd > 0.15 && T.size() > 40) ++M.stop_go;
        was_moving = true;
        stopped_cycles = 0;
      } else if (v_cmd < 0.01) {
        if (was_moving && pspd < 0.15) {
          stopped_cycles = 0;  // 사람도 선 경우는 세지 않는다
        }
        was_moving = false;
        if (pspd > 0.15) ++stopped_cycles;
      }
      if (csv.is_open()) {
        char buf[320];
        std::snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%u,%.3f,%d,%d\n", t,
                      ps.x, ps.y, pspd, rb.x, rb.y, rb.yaw, v_cmd, w_cmd, rb.v, rb.w, static_cast<int>(o.state),
                      o.stop_reasons, o.target_range, static_cast<int>(in.target.status), in.target.ambiguous ? 1 : 0);
        csv << buf;
      }
    }
  }
  (void)ever_lost;
  // 지표
  const size_t n = T.size();
  double sj = 0, swj = 0;
  int nj = 0, nf = 0, nw = 0;
  std::vector<double> dfol;
  for (size_t i = 2; i < n; ++i) {
    const double a1 = (VC[i] - VC[i - 1]) / kCtrlDt, a0 = (VC[i - 1] - VC[i - 2]) / kCtrlDt;
    const double j = (a1 - a0) / kCtrlDt;
    const double wa1 = (WC[i] - WC[i - 1]) / kCtrlDt, wa0 = (WC[i - 1] - WC[i - 2]) / kCtrlDt;
    const double wj = (wa1 - wa0) / kCtrlDt;
    const double aa1 = (VA[i] - VA[i - 1]) / kCtrlDt, aa0 = (VA[i - 1] - VA[i - 2]) / kCtrlDt;
    if (VC[i] > 0.05 && VC[i - 1] > 0.05 && VC[i - 2] > 0.05) {
      M.max_jerk_moving = std::max(M.max_jerk_moving, std::abs(j));
    }
    const unsigned firm_bits = 2u | 4u;  // TOO_CLOSE | OBSTACLE
    if ((SR[i] & firm_bits) && !(SR[i - 1] & firm_bits) && VC[i - 1] > 0.05) ++M.firm;
    M.max_acc_cmd = std::max(M.max_acc_cmd, std::abs(a1));
    M.max_jerk_cmd = std::max(M.max_jerk_cmd, std::abs(j));
    M.max_wacc_cmd = std::max(M.max_wacc_cmd, std::abs(wa1));
    M.max_wjerk_cmd = std::max(M.max_wjerk_cmd, std::abs(wj));
    M.max_acc_act = std::max(M.max_acc_act, std::abs(aa1));
    M.max_jerk_act = std::max(M.max_jerk_act, std::abs((aa1 - aa0) / kCtrlDt));
    sj += j * j;
    swj += wj * wj;
    ++nj;
    M.max_v_cmd = std::max(M.max_v_cmd, VC[i]);
    M.max_w_cmd = std::max(M.max_w_cmd, std::abs(WC[i]));
    if (T[i] > kStartAt + 1.0) {
      M.min_dist = std::min(M.min_dist, D[i]);
      if (F[i] == static_cast<int>(FsmState::kFollowing)) {
        ++nf;
        dfol.push_back(D[i]);
      }
      if (F[i] == static_cast<int>(FsmState::kWaiting)) ++nw;
    }
  }
  {
    // 출발 지연: 사람 속도가 0.2 를 넘는 순간부터 로봇 명령 0.1 초과까지
    std::vector<double> lags;
    for (size_t i = 1; i < n; ++i) {
      if (PSPD[i - 1] < 0.2 && PSPD[i] >= 0.2 && T[i] > kStartAt + 1.0 && VC[i] < 0.1) {
        for (size_t k = i; k < n; ++k) {
          if (VC[k] > 0.1) {
            lags.push_back(T[k] - T[i]);
            break;
          }
        }
      }
    }
    double sl = 0;
    for (double x : lags) sl += x;
    M.start_lag = lags.empty() ? 0.0 : sl / lags.size();
    std::vector<double> bf;
    for (size_t i = 0; i < n; ++i) {
      if (F[i] == static_cast<int>(FsmState::kFollowing) && T[i] > kStartAt + 1.0) bf.push_back(BRG[i]);
    }
    if (!bf.empty()) {
      std::sort(bf.begin(), bf.end());
      M.bearing_p95 = bf[static_cast<size_t>(0.95 * (bf.size() - 1))];
    }
  }
  if (!terr.empty()) {
    std::sort(terr.begin(), terr.end());
    M.tgt_err_p95 = terr[static_cast<size_t>(0.95 * (terr.size() - 1))];
  }
  M.rms_jerk_cmd = nj ? std::sqrt(sj / nj) : 0;
  M.rms_wjerk_cmd = nj ? std::sqrt(swj / nj) : 0;
  const double nact = std::max<double>(1, n - (kStartAt + 1.0) / kCtrlDt);
  M.follow_frac = nf / nact;
  M.wait_frac = nw / nact;
  if (!dfol.empty()) {
    double s = 0;
    for (double d : dfol) s += d;
    M.mean_dist = s / dfol.size();
    std::sort(dfol.begin(), dfol.end());
    M.p95_dist = dfol[static_cast<size_t>(0.95 * (dfol.size() - 1))];
  }
  return M;
}

}  // namespace

int main(int argc, char** argv) {
  std::string which = "all", out;
  unsigned seed = 1;
  bool quiet = false;
  ControllerParams cp;
  std::vector<std::pair<std::string, double>> sets;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--scenario" && i + 1 < argc) {
      which = argv[++i];
    } else if (a == "--out" && i + 1 < argc) {
      out = argv[++i];
    } else if (a == "--seed" && i + 1 < argc) {
      seed = static_cast<unsigned>(std::atoi(argv[++i]));
    } else if (a == "--quiet") {
      quiet = true;
    } else if (a == "--set" && i + 1 < argc) {
      const std::string kv = argv[++i];
      const auto eq = kv.find('=');
      if (eq == std::string::npos) {
        std::fprintf(stderr, "bad --set %s\n", kv.c_str());
        return 2;
      }
      sets.push_back({kv.substr(0, eq), std::atof(kv.c_str() + eq + 1)});
    } else {
      std::fprintf(stderr, "usage: %s [--scenario NAME|all] [--set name=value]... [--out DIR] [--seed N] [--quiet]\n", argv[0]);
      return 2;
    }
  }
  for (const auto& kv : sets) {
    const TunableSpec* s = findTunable(kv.first);
    if (!s) {
      std::fprintf(stderr, "unknown parameter %s\n", kv.first.c_str());
      return 2;
    }
    s->set(cp, kv.second);
  }
  syncDerived(&cp);
  const std::string why = inconsistency(cp);
  if (!why.empty()) {
    std::fprintf(stderr, "inconsistent parameters: %s\n", why.c_str());
    return 2;
  }
  std::printf("{\"results\": [\n");
  bool first = true;
  for (const auto& sc : scenarios()) {
    if (which != "all" && which != sc.name) continue;
    const std::string csv = out.empty() ? "" : out + "/" + sc.name + ".csv";
    const Metrics m = run(sc, cp, seed, csv);
    std::printf("%s{\"scenario\": \"%s\", \"max_acc_cmd\": %.3f, \"max_jerk_cmd\": %.2f, \"rms_jerk_cmd\": %.3f, "
                "\"max_acc_act\": %.3f, \"max_jerk_act\": %.2f, \"max_wacc_cmd\": %.3f, \"max_wjerk_cmd\": %.2f, "
                "\"rms_wjerk_cmd\": %.3f, \"stop_go\": %d, \"min_dist\": %.3f, \"mean_dist\": %.3f, "
                "\"p95_dist\": %.3f, \"min_wall\": %.3f, \"follow_frac\": %.3f, \"wait_frac\": %.3f, "
                "\"max_v_cmd\": %.3f, \"max_w_cmd\": %.3f, \"firm\": %d, \"max_jerk_moving\": %.2f, \"tgt_err_p95\": %.3f, \"start_lag\": %.2f, \"bearing_p95\": %.1f}",
                first ? "" : ",\n", sc.name.c_str(), m.max_acc_cmd, m.max_jerk_cmd, m.rms_jerk_cmd, m.max_acc_act,
                m.max_jerk_act, m.max_wacc_cmd, m.max_wjerk_cmd, m.rms_wjerk_cmd, m.stop_go, m.min_dist, m.mean_dist,
                m.p95_dist, m.min_wall, m.follow_frac, m.wait_frac, m.max_v_cmd, m.max_w_cmd, m.firm, m.max_jerk_moving, m.tgt_err_p95, m.start_lag, m.bearing_p95);
    first = false;
    if (!quiet) {
      std::fprintf(stderr, "%-18s stop_go %d firm %d  jerk(mv) %4.1f max %5.1f rms %.2f  wjerk max %5.1f rms %.2f  acc %.2f  dist min %.2f mean %.2f  wall %s  wait %.0f%%  vmax %.2f wmax %.2f\n",
                   sc.name.c_str(), m.stop_go, m.firm, m.max_jerk_moving, m.max_jerk_cmd, m.rms_jerk_cmd, m.max_wjerk_cmd, m.rms_wjerk_cmd,
                   m.max_acc_cmd, m.min_dist, m.mean_dist,
                   m.min_wall < 1e8 ? std::to_string(m.min_wall).substr(0, 4).c_str() : "-", 100 * m.wait_frac,
                   m.max_v_cmd, m.max_w_cmd);
    }
  }
  std::printf("\n]}\n");
  return 0;
}
