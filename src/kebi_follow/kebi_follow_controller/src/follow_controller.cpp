// follow_controller: 상태기계·추종 제어·안전 게이트 (SPEC 5, 6.3, 7절)
//
// 판단은 ControllerCore(ROS 무관)가 하고, 이 노드는 입력을 모아 20 Hz 로 step() 을 부른 뒤 발행만 한다.
// - IDLE·ENDED 에서는 /cmd_vel 을 내지 않는다 (활성 상태에서 나갈 때만 0 을 몇 번)
// - shadow_mode 면 /cmd_vel 발행자를 만들지 않고 그림자 토픽으로만 낸다
// - 낙하·충돌·로봇 이상(S5·S9)은 콜백에서 바로 0 을 내고 HALTED 로 래치
// - depth 는 별도 콜백 그룹에서 처리하고 결과(장애물 점)만 뮤텍스로 넘긴다
#include "kebi_follow_controller/follow_controller.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <follow_msgs/msg/detect.hpp>
#include <follow_msgs/msg/ui_client.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kebi_follow_msgs/msg/follow_mode.hpp>
#include <kebi_follow_msgs/msg/follow_state.hpp>
#include <kebi_follow_msgs/msg/follow_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robot_custom_msgs/msg/abnormal_event_data.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

#include "kebi_follow_controller/controller_core.hpp"
#include "kebi_follow_controller/depth_obstacle_checker.hpp"
#include "kebi_follow_controller/pose_history.hpp"
#include "kebi_follow_controller/types.hpp"

namespace kebi_follow {

namespace {

using FollowStateMsg = kebi_follow_msgs::msg::FollowState;
using FollowModeMsg = kebi_follow_msgs::msg::FollowMode;
using FollowTargetMsg = kebi_follow_msgs::msg::FollowTarget;

// 메시지 상수와 내부 상수가 같아야 한다
static_assert(FollowStateMsg::FSM_IDLE ==
              static_cast<uint8_t>(FsmState::kIdle));
static_assert(FollowStateMsg::FSM_ENDED ==
              static_cast<uint8_t>(FsmState::kEnded));
static_assert(FollowStateMsg::STOP_NO_TARGET == stop_reason::kNoTarget);
static_assert(FollowStateMsg::STOP_TOO_CLOSE == stop_reason::kTooClose);
static_assert(FollowStateMsg::STOP_OBSTACLE == stop_reason::kObstacle);
static_assert(FollowStateMsg::STOP_ROTATION_BLOCKED ==
              stop_reason::kRotationBlocked);
static_assert(FollowStateMsg::STOP_CLIFF == stop_reason::kCliff);
static_assert(FollowStateMsg::STOP_COLLISION == stop_reason::kCollision);
static_assert(FollowStateMsg::STOP_SENSOR_TIMEOUT ==
              stop_reason::kSensorTimeout);
static_assert(FollowStateMsg::STOP_AMBIGUOUS == stop_reason::kAmbiguous);
static_assert(FollowStateMsg::STOP_ROBOT_STATE == stop_reason::kRobotState);
static_assert(FollowStateMsg::STOP_ROTATION_LIMIT ==
              stop_reason::kRotationLimit);
static_assert(FollowModeMsg::TRACK == static_cast<uint8_t>(ModeValue::kTrack));
static_assert(FollowTargetMsg::STATUS_OCCLUDED ==
              static_cast<uint8_t>(TargetStatus::kOccluded));

constexpr std::size_t kOdomHistoryCapacity = 512;  // 100 Hz 로 약 5 s
constexpr std::size_t kScanReserve = 2048;
constexpr std::size_t kDepthReserve = 512;
constexpr std::size_t kTargetHistoryCapacity = 64;  // 10 Hz 로 약 6 s
// depth 수신-stamp 차 최솟값 창 (약 6 s)
constexpr std::size_t kDepthLagWindow = 64;
// stamp 가 최신 odom 보다 이만큼 늦어도 최신 자세 사용 [s]
constexpr double kOdomLookupAhead = 0.1;
// 자세 변화로 실측 선속도를 구하는 창 [s] (odom 100 Hz 로 약 10 개)
constexpr double kOdomSpeedWindowS = 0.1;
constexpr int kWarnThrottleMs = 2000;
constexpr double kAckTimeoutMs = 100.0;  // 종료 시 0 명령 전달 대기
// 계속 발행되는 HALT 토픽(IR 100 Hz, lifted 83 Hz, fall_down 1 Hz)의 true 를 "켜짐"으로 보는 시간.
// 사건형 토픽(충돌·perception·cliff s_code·e-stop)은 콜백에서 래치만 하고 레벨로 두지 않는다
constexpr double kHaltLevelHoldS = 1.5;
constexpr double kEventOnly = 0.0;
constexpr double kNanoToSec = 1e-9;
constexpr float kFarDistance = 1e3f;  // "없음" 거리 (메시지용)
constexpr std::size_t kBytesPerDepthPixel = 2;  // 16UC1
// TF 가 이미 숙여져 있는데 보정까지 주면 두 번 숙인다 → depth 를 믿지 않는다 [deg]
constexpr double kTfPitchSuspiciousDeg = 2.0;
// 타깃 경로 이력: 이전 점에서 속도·시간으로 설명 안 되는 이동이면 끊는다 [m]
constexpr double kTrackJumpMargin = 0.3;
constexpr double kStampWarnS = 5.0;  // 타깃 stamp 가 이보다 어긋나면 경고 [s]

double toSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * kNanoToSec;
}

double yawOf(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

float finiteOr(double v, float fallback) {
  return std::isfinite(v) ? static_cast<float>(v) : fallback;
}

std::string reasonsToString(uint32_t r) {
  static const struct {
    uint32_t bit;
    const char* name;
  } kNames[] = {
      {stop_reason::kNoTarget, "NO_TARGET"},
      {stop_reason::kTooClose, "TOO_CLOSE"},
      {stop_reason::kObstacle, "OBSTACLE"},
      {stop_reason::kRotationBlocked, "ROTATION_BLOCKED"},
      {stop_reason::kCliff, "CLIFF"},
      {stop_reason::kCollision, "COLLISION"},
      {stop_reason::kSensorTimeout, "SENSOR_TIMEOUT"},
      {stop_reason::kAmbiguous, "AMBIGUOUS"},
      {stop_reason::kRobotState, "ROBOT_STATE"},
      {stop_reason::kRotationLimit, "ROTATION_LIMIT"},
  };
  std::string s;
  for (const auto& n : kNames) {
    if ((r & n.bit) != 0) {
      if (!s.empty()) {
        s += "|";
      }
      s += n.name;
    }
  }
  return s.empty() ? "-" : s;
}

// ---------------------------------------------------------------- 실행 중 조정
// 조정 UI 등에서 ros2 param set 으로 바꿀 수 있는 파라미터. 범위 밖이거나 서로 모순되면 거부하고,
// 받은 값은 다음 제어 주기 시작에 한꺼번에 적용한다 (SPEC 6.3 "실행 중 조정"). 나머지는 기동 때만 읽는다.
struct TunableSpec {
  const char* name;
  double lo;
  double hi;
  const char* desc;
  double (*get)(const ControllerParams&);
  void (*set)(ControllerParams&, double);
};

constexpr double kMinStopToFollowGap = 0.1;  // stop_distance + 이 값 <= follow_distance [m]

const TunableSpec kTunables[] = {
    // S1 속도·가감속
    {"max_linear_speed", 0.0, 1.0, "선속도 상한 [m/s]",
     [](const ControllerParams& p) { return p.limiter.max_linear_speed; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_speed = v; }},
    {"max_angular_speed", 0.0, 1.5, "각속도 상한 [rad/s]",
     [](const ControllerParams& p) { return p.limiter.max_angular_speed; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_speed = v; }},
    {"max_linear_accel", 0.1, 1.0, "선가속 [m/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_linear_accel; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_accel = v; }},
    {"max_linear_decel", 0.3, 1.5, "선감속 (일반 정지) [m/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_linear_decel; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_decel = v; }},
    {"max_linear_jerk", 0.0, 10.0, "가속 쪽 jerk [m/s^3] (0: 끔)",
     [](const ControllerParams& p) { return p.limiter.max_linear_jerk; },
     [](ControllerParams& p, double v) { p.limiter.max_linear_jerk = v; }},
    {"max_angular_accel", 0.3, 3.0, "각가속 [rad/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_angular_accel; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_accel = v; }},
    {"max_angular_decel", 0.3, 3.0, "각감속 [rad/s^2]",
     [](const ControllerParams& p) { return p.limiter.max_angular_decel; },
     [](ControllerParams& p, double v) { p.limiter.max_angular_decel = v; }},
    // S2 추종 법칙
    {"follow_distance", 0.8, 2.5, "추종 거리 [m]",
     [](const ControllerParams& p) { return p.law.follow_distance; },
     [](ControllerParams& p, double v) { p.law.follow_distance = v; }},
    {"stop_distance", 0.5, 1.5, "이 거리 이내면 즉시 정지 [m]",
     [](const ControllerParams& p) { return p.law.stop_distance; },
     [](ControllerParams& p, double v) { p.law.stop_distance = v; }},
    {"approach_margin", 0.0, 0.5, "사람이 갑자기 서도 정지 거리 + 이 값 앞에서 선다 [m]",
     [](const ControllerParams& p) { return p.law.approach_margin; },
     [](ControllerParams& p, double v) { p.law.approach_margin = v; }},
    {"reaction_time", 0.0, 0.5, "사람 앞 제동 상한 계산 지연 [s]",
     [](const ControllerParams& p) { return p.law.reaction_time; },
     [](ControllerParams& p, double v) { p.law.reaction_time = v; }},
    {"k_v", 0.1, 3.0, "거리 오차 → 선속도 게인 [1/s]",
     [](const ControllerParams& p) { return p.law.k_v; },
     [](ControllerParams& p, double v) { p.law.k_v = v; }},
    {"k_w", 0.2, 3.0, "방향각 → 각속도 게인 [1/s]",
     [](const ControllerParams& p) { return p.law.k_w; },
     [](ControllerParams& p, double v) { p.law.k_w = v; }},
    {"ff_gain", 0.0, 1.5, "사람 속도 피드포워드 배율",
     [](const ControllerParams& p) { return p.law.ff_gain; },
     [](ControllerParams& p, double v) { p.law.ff_gain = v; }},
    {"bearing_deadband", 0.0, 0.3, "방향각 불감대 [rad]",
     [](const ControllerParams& p) { return p.law.bearing_deadband; },
     [](ControllerParams& p, double v) { p.law.bearing_deadband = v; }},
    {"slow_bearing_deg", 0.0, 60.0, "이 방향각부터 선속도를 줄임 [deg]",
     [](const ControllerParams& p) { return radToDeg(p.law.slow_bearing); },
     [](ControllerParams& p, double v) { p.law.slow_bearing = degToRad(v); }},
    {"turn_in_place_deg", 10.0, 90.0, "이 방향각부터 제자리 회전 [deg]",
     [](const ControllerParams& p) {
       return radToDeg(p.law.turn_in_place_bearing);
     },
     [](ControllerParams& p, double v) {
       p.law.turn_in_place_bearing = degToRad(v);
     }},
    // S3 장애물
    {"slow_distance", 0.5, 3.0, "장애물 감속 시작 거리 [m]",
     [](const ControllerParams& p) { return p.slow_distance; },
     [](ControllerParams& p, double v) { p.slow_distance = v; }},
    {"stop_margin", 0.15, 0.6, "라이다 장애물 정지 여유 [m]",
     [](const ControllerParams& p) { return p.stop_margin; },
     [](ControllerParams& p, double v) { p.stop_margin = v; }},
    {"depth_stop_margin", 0.3, 1.0, "depth 장애물 정지 여유 [m]",
     [](const ControllerParams& p) { return p.depth_stop_margin; },
     [](ControllerParams& p, double v) { p.depth_stop_margin = v; }},
    {"obstacle_reaction_time", 0.0, 0.5, "장애물 제동 계산 지연 [s]",
     [](const ControllerParams& p) { return p.obstacle_reaction_time; },
     [](ControllerParams& p, double v) { p.obstacle_reaction_time = v; }},
    // S4 회전, S6·S7
    {"rotation_limit_s", 1.0, 10.0, "제자리 회전 연속 상한 [s]",
     [](const ControllerParams& p) { return p.rotation_limit_s; },
     [](ControllerParams& p, double v) { p.rotation_limit_s = v; }},
    {"rotation_rearm_s", 0.5, 5.0, "회전 상한 뒤 다시 허용까지 [s]",
     [](const ControllerParams& p) { return p.rotation_rearm_s; },
     [](ControllerParams& p, double v) { p.rotation_rearm_s = v; }},
    {"target_lost_s", 0.3, 2.0, "측정 없는 시간이 이를 넘으면 정지 [s]",
     [](const ControllerParams& p) { return p.target_lost_s; },
     [](ControllerParams& p, double v) { p.target_lost_s = v; }},
    {"occluded_speed_factor", 0.0, 1.0, "가림 중 선속도 배율",
     [](const ControllerParams& p) { return p.occluded_speed_factor; },
     [](ControllerParams& p, double v) { p.occluded_speed_factor = v; }},
    {"camera_missing_speed_cap", 0.0, 0.5, "카메라·depth 비정상 때 선속도 상한 [m/s]",
     [](const ControllerParams& p) { return p.camera_missing_speed_cap; },
     [](ControllerParams& p, double v) { p.camera_missing_speed_cap = v; }},
};

const TunableSpec* findTunable(const std::string& name) {
  for (const auto& s : kTunables) {
    if (name == s.name) {
      return &s;
    }
  }
  return nullptr;
}

// 추종 법칙이 함께 쓰는 값 (limiter 와 같아야 한다)
void syncDerived(ControllerParams* p) {
  p->law.max_linear_speed = p->limiter.max_linear_speed;
  p->law.max_angular_speed = p->limiter.max_angular_speed;
  p->law.brake_decel = p->limiter.max_linear_decel;
}

// 서로 모순되는 조합이면 이유, 아니면 빈 문자열
std::string inconsistency(const ControllerParams& p) {
  if (p.law.stop_distance + kMinStopToFollowGap > p.law.follow_distance) {
    return "stop_distance (" + std::to_string(p.law.stop_distance) +
           ") must be at least 0.1 m below follow_distance (" +
           std::to_string(p.law.follow_distance) + ")";
  }
  if (p.law.slow_bearing >= p.law.turn_in_place_bearing) {
    return "slow_bearing_deg must be below turn_in_place_deg";
  }
  return "";
}

bool numberOf(const rclcpp::ParameterValue& v, double* out) {
  if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    *out = v.get<double>();
    return true;
  }
  if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    *out = static_cast<double>(v.get<int64_t>());
    return true;
  }
  return false;
}

}  // namespace

class FollowController : public rclcpp::Node {
 public:
  explicit FollowController(const rclcpp::NodeOptions& options)
      : Node("follow_controller", options), odom_hist_(kOdomHistoryCapacity) {
    use_sim_time_ = get_parameter("use_sim_time").as_bool();
    loadParams();
    syncDerived(&cp_);
    if (const std::string why = inconsistency(cp_); !why.empty()) {
      RCLCPP_FATAL(get_logger(), "follow_controller: %s", why.c_str());
      throw std::invalid_argument("follow_controller: " + why);
    }
    core_ = std::make_unique<ControllerCore>(cp_);
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& ps) {
          return onSetParams(ps);
        });
    checker_ = std::make_unique<DepthObstacleChecker>(dp_);
    detect_report_ = ChangeOrPeriodic(report_period_s_);
    mode_report_ = ChangeOrPeriodic(report_period_s_);
    scan_pts_.reserve(kScanReserve);
    depth_pts_shared_.reserve(kDepthReserve);
    depth_pts_local_.reserve(kDepthReserve);
    depth_result_.points.reserve(kDepthReserve);
    target_hist_.resize(kTargetHistoryCapacity);
    depth_excl_pts_.reserve(kTargetHistoryCapacity + 1);
    depth_lag_.assign(kDepthLagWindow, 0.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
    depth_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 출력
    const std::string cmd_topic = shadow_ ? topic_shadow_ : topic_cmd_vel_;
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        cmd_topic, rclcpp::QoS(1).reliable());
    detect_pub_ = create_publisher<follow_msgs::msg::Detect>(
        topic_detect_, rclcpp::QoS(1).reliable());
    mode_pub_ = create_publisher<FollowModeMsg>(
        topic_mode_, rclcpp::QoS(1).reliable().transient_local());
    state_pub_ = create_publisher<FollowStateMsg>(topic_state_,
                                                  rclcpp::QoS(5).reliable());

    // 입력
    const auto sensor_qos = rclcpp::SensorDataQoS();
    ui_sub_ = create_subscription<follow_msgs::msg::UiClient>(
        topic_ui_, rclcpp::QoS(10).reliable(),
        [this](follow_msgs::msg::UiClient::ConstSharedPtr m) { onUi(*m); });
    target_sub_ = create_subscription<FollowTargetMsg>(
        topic_target_, rclcpp::QoS(5).reliable(),
        [this](FollowTargetMsg::ConstSharedPtr m) { onTarget(m); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        topic_scan_, sensor_qos,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { onScan(*m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdom(*m); });
    createSafetySubscriptions(sensor_qos);

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { controlTick(); });
    const auto state_period =
        std::chrono::duration<double>(1.0 / state_rate_hz_);
    state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(state_period),
        [this]() { publishState(); });

    // 프로세스 종료(SIGINT 등) 때 컨텍스트가 닫히기 전에 0 을 보낸다 (S10)
    shutdown_cb_ =
        get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() { onShutdown(); });

    publishMode(monoNow(), true);
    RCLCPP_INFO(get_logger(),
                "follow_controller ready: shadow=%s cmd=%s v_max=%.2f "
                "w_max=%.2f depth=%s",
                shadow_ ? "true" : "false", cmd_topic.c_str(),
                cp_.limiter.max_linear_speed, cp_.limiter.max_angular_speed,
                depth_enable_ ? "on" : "off");
  }

  ~FollowController() override {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(
        shutdown_cb_);
    // 컴포넌트 unload 등
    if (rclcpp::ok()) {
      onShutdown();
    }
  }

 private:
  // ---------------------------------------------------------------- 파라미터
  // 모든 파라미터는 기동 때만 읽는다: 실행 중 변경은 거부(read_only)해서 "바꿨는데 안 바뀜"을 막는다
  static rcl_interfaces::msg::ParameterDescriptor readOnly() {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    return d;
  }

  [[noreturn]] void typeError(const std::string& name, const std::string& want,
                              const std::string& got) {
    const std::string msg = "follow_controller: parameter '" + name +
                            "' must be " + want + " (got " + got +
                            "). Check the YAML override.";
    RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
    throw std::invalid_argument(msg);
  }

  // 실수 파라미터: YAML 에 정수(예 0)를 써도 받는다.
  // kTunables 에 있는 이름은 실행 중 조정을 허용하고 범위를 알린다 (범위 밖이면 기동 실패)
  double param(const std::string& name, double def) {
    const TunableSpec* spec = findTunable(name);
    auto d = readOnly();
    d.dynamic_typing = true;
    if (spec != nullptr) {
      d.read_only = false;
      d.description = spec->desc;
      rcl_interfaces::msg::FloatingPointRange r;
      r.from_value = spec->lo;
      r.to_value = spec->hi;
      r.step = 0.0;
      d.floating_point_range.push_back(r);
    }
    rclcpp::ParameterValue v;
    try {
      v = declare_parameter(name, rclcpp::ParameterValue(def), d);
    } catch (const rclcpp::exceptions::InvalidParameterValueException& e) {
      typeError(name, "within its range", e.what());
    }
    double x = 0.0;
    if (!numberOf(v, &x)) {
      typeError(name, "a number", rclcpp::to_string(v.get_type()));
    }
    if (spec != nullptr && (!std::isfinite(x) || x < spec->lo || x > spec->hi)) {
      typeError(name,
                "within [" + std::to_string(spec->lo) + ", " +
                    std::to_string(spec->hi) + "]",
                std::to_string(x));
    }
    return x;
  }

  // 실행 중 조정 (kTunables 만): 검사만 하고, 적용은 다음 제어 주기 시작에 한다
  rcl_interfaces::msg::SetParametersResult onSetParams(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    std::lock_guard<std::mutex> lk(tune_mutex_);
    ControllerParams cand = tune_pending_ ? tune_cp_ : cp_;
    std::string changes;
    for (const auto& p : params) {
      const TunableSpec* spec = findTunable(p.get_name());
      if (spec == nullptr) {
        continue;  // 기동 전용(read_only)은 rclcpp 가 먼저 거부한다. use_sim_time 등은 그대로 둔다
      }
      double x = 0.0;
      if (!numberOf(p.get_parameter_value(), &x) || !std::isfinite(x)) {
        res.successful = false;
        res.reason = p.get_name() + " must be a number";
        return res;
      }
      if (x < spec->lo || x > spec->hi) {
        res.successful = false;
        res.reason = p.get_name() + " " + std::to_string(x) + " outside [" +
                     std::to_string(spec->lo) + ", " +
                     std::to_string(spec->hi) + "]";
        return res;
      }
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%s%s %.3f -> %.3f",
                    changes.empty() ? "" : ", ", spec->name, spec->get(cand),
                    x);
      changes += buf;
      spec->set(cand, x);
    }
    syncDerived(&cand);
    const std::string why = inconsistency(cand);
    if (!why.empty()) {
      res.successful = false;
      res.reason = why;
      return res;
    }
    if (!changes.empty()) {
      tune_cp_ = cand;
      tune_pending_ = true;
      RCLCPP_INFO(get_logger(), "tuning: %s", changes.c_str());
    }
    return res;
  }

  int paramInt(const std::string& name, int def) {
    auto d = readOnly();
    d.dynamic_typing = true;
    const rclcpp::ParameterValue v = declare_parameter(
        name, rclcpp::ParameterValue(static_cast<int64_t>(def)), d);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<int>(v.get<int64_t>());
    }
    typeError(name, "an integer", rclcpp::to_string(v.get_type()));
  }

  template <typename T>
  T paramT(const std::string& name, const T& def) {
    try {
      return declare_parameter<T>(name, def, readOnly());
    } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
      typeError(name, "of the default's type", e.what());
    }
  }

  void loadParams() {
    shadow_ = paramT<bool>("shadow_mode", false);
    control_rate_hz_ = param("control_rate_hz", 20.0);
    state_rate_hz_ = param("state_rate_hz", 10.0);
    report_period_s_ = param("report_period_s", 1.0);
    base_frame_ = paramT<std::string>("base_frame", "base_link");

    topic_cmd_vel_ = paramT<std::string>("topics.cmd_vel", "/cmd_vel");
    topic_shadow_ = paramT<std::string>("topics.cmd_vel_shadow",
                                        "/kebi_follow/cmd_vel_shadow");
    topic_target_ = paramT<std::string>("topics.target", "/kebi_follow/target");
    topic_scan_ = paramT<std::string>("topics.scan", "/scan");
    topic_odom_ = paramT<std::string>("topics.odom", "/odom");
    topic_depth_ =
        paramT<std::string>("topics.depth", "/camera/depth/image_raw");
    topic_depth_info_ =
        paramT<std::string>("topics.depth_info", "/camera/depth/camera_info");
    topic_ui_ = paramT<std::string>("topics.ui", "/follow/ui/client");
    topic_detect_ = paramT<std::string>("topics.detect", "/follow/detectuser");
    topic_mode_ = paramT<std::string>("topics.mode", "/kebi_follow/mode");
    topic_state_ = paramT<std::string>("topics.state", "/kebi_follow/state");

    // S1 속도·가감속
    auto& lim = cp_.limiter;
    lim.max_linear_speed = param("max_linear_speed", lim.max_linear_speed);
    lim.max_angular_speed = param("max_angular_speed", lim.max_angular_speed);
    lim.max_linear_accel = param("max_linear_accel", lim.max_linear_accel);
    lim.max_linear_decel = param("max_linear_decel", lim.max_linear_decel);
    lim.max_linear_jerk = param("max_linear_jerk", lim.max_linear_jerk);
    lim.max_angular_accel = param("max_angular_accel", lim.max_angular_accel);
    lim.max_angular_decel = param("max_angular_decel", lim.max_angular_decel);

    // 추종 법칙 (S2)
    auto& law = cp_.law;
    law.follow_distance = param("follow_distance", law.follow_distance);
    law.stop_distance = param("stop_distance", law.stop_distance);
    law.approach_margin = param("approach_margin", law.approach_margin);
    law.reaction_time = param("reaction_time", law.reaction_time);
    law.k_v = param("k_v", law.k_v);
    law.k_w = param("k_w", law.k_w);
    law.ff_gain = param("ff_gain", law.ff_gain);
    law.ff_min_target_speed =
        param("ff_min_target_speed", law.ff_min_target_speed);
    law.bearing_deadband = param("bearing_deadband", law.bearing_deadband);
    law.slow_bearing = degToRad(param("slow_bearing_deg", 15.0));
    law.turn_in_place_bearing = degToRad(param("turn_in_place_deg", 45.0));
    law.max_linear_speed = lim.max_linear_speed;
    law.max_angular_speed = lim.max_angular_speed;
    law.brake_decel = lim.max_linear_decel;
    max_extrapolation_s_ = param("max_extrapolation_s", 0.5);
    target_max_range_ = param("target_max_range", 8.0);
    target_max_speed_ = param("target_max_speed", 3.0);

    // S3 진행 경로
    auto& cor = cp_.corridor;
    cor.robot_radius = param("robot_radius", cor.robot_radius);
    cor.half_width = param("corridor_half_width", cor.half_width);
    cor.lookahead = param("corridor_lookahead", cor.lookahead);
    cor.max_curvature = param("corridor_max_curvature", cor.max_curvature);
    cor.target_exclusion_radius =
        param("target_exclusion_radius", cor.target_exclusion_radius);
    cp_.slow_distance = param("slow_distance", cp_.slow_distance);
    cp_.stop_margin = param("stop_margin", cp_.stop_margin);
    cp_.obstacle_reaction_time =
        param("obstacle_reaction_time", cp_.obstacle_reaction_time);
    cp_.depth_stop_margin = param("depth_stop_margin", cp_.depth_stop_margin);
    cp_.curvature_min_speed =
        param("curvature_min_speed", cp_.curvature_min_speed);
    body_filter_radius_ = param("body_filter_radius", 0.20);
    scan_max_range_ = param("scan_max_range", 4.0);

    // S4 회전, S10
    cp_.rotation_clearance =
        param("rotation_clearance", cp_.rotation_clearance);
    cp_.rotation_limit_s = param("rotation_limit_s", cp_.rotation_limit_s);
    cp_.rotation_rearm_s = param("rotation_rearm_s", cp_.rotation_rearm_s);
    cp_.in_place_linear_eps =
        param("in_place_linear_eps", cp_.in_place_linear_eps);
    cp_.in_place_angular_eps =
        param("in_place_angular_eps", cp_.in_place_angular_eps);
    cp_.rotation_creep_speed =
        param("rotation_creep_speed", cp_.rotation_creep_speed);

    // S6 센서 (0 이하면 그 센서는 검사 안 함: bag 재생 등 시험용)
    scan_timeout_s_ = param("scan_timeout_s", 0.3);
    odom_timeout_s_ = param("odom_timeout_s", 0.2);
    bottom_ir_timeout_s_ = param("bottom_ir_timeout_s", 0.2);
    lifted_timeout_s_ = param("lifted_timeout_s", 1.0);
    fall_down_timeout_s_ = param("fall_down_timeout_s", 3.0);
    camera_timeout_s_ = param("camera_timeout_s", 1.0);
    camera_latency_s_ = param("camera_latency_s", 0.1);
    depth_exclusion_margin_s_ = param("depth_exclusion_margin_s", 0.2);
    cp_.camera_missing_speed_cap =
        param("camera_missing_speed_cap", cp_.camera_missing_speed_cap);

    // S7 타깃
    cp_.target_lost_s = param("target_lost_s", cp_.target_lost_s);
    cp_.occluded_speed_factor =
        param("occluded_speed_factor", cp_.occluded_speed_factor);
    cp_.target_fresh_s = param("target_fresh_s", cp_.target_fresh_s);
    if (cp_.target_fresh_s > max_extrapolation_s_) {
      // stamp 가 외삽 상한보다 오래되면 "측정 있음"으로 보지 않는다
      RCLCPP_WARN(get_logger(),
                  "target_fresh_s %.2f > max_extrapolation_s %.2f: clamped",
                  cp_.target_fresh_s, max_extrapolation_s_);
      cp_.target_fresh_s = max_extrapolation_s_;
    }

    // 상태기계
    auto& fsm = cp_.fsm;
    fsm.acquire_timeout_s = param("acquire_timeout_s", fsm.acquire_timeout_s);
    fsm.resume_confirm_s = param("resume_confirm_s", fsm.resume_confirm_s);
    fsm.lost_end_timeout_s =
        param("lost_end_timeout_s", fsm.lost_end_timeout_s);
    fsm.halt_restart_holdoff_s =
        param("halt_restart_holdoff_s", fsm.halt_restart_holdoff_s);
    fsm.zero_burst_count = paramInt("zero_burst_count", fsm.zero_burst_count);

    // /follow/detectuser
    cp_.moving_linear = param("moving_linear", cp_.moving_linear);
    cp_.moving_angular = param("moving_angular", cp_.moving_angular);
    cp_.detect_debounce_s = param("detect_debounce_s", cp_.detect_debounce_s);

    // S5·S9 안전 토픽
    topic_bottom_ir_ =
        paramT<std::string>("safety.bottom_ir_topic", "/bottom_ir_data");
    topic_collision_ =
        paramT<std::string>("safety.collision_topic", "/collision_detected");
    topic_collision_clouds_ = paramT<std::vector<std::string>>(
        "safety.collision_cloud_topics",
        {"/sensor_to_pointcloud/collision/front",
         "/sensor_to_pointcloud/collision/rear"});
    topic_perception_stop_ = paramT<std::string>("safety.perception_stop_topic",
                                                 "/perception/action/stop");
    topic_lifted_ =
        paramT<std::string>("safety.lifted_topic", "/error/s_code/lifted");
    topic_fall_down_ = paramT<std::string>("safety.fall_down_topic",
                                           "/error/s_code/fall_down");
    topic_cliff_ = paramT<std::string>("safety.cliff_detected_topic",
                                       "/error/s_code/cliff_detected");
    topic_estop_ =
        paramT<std::string>("safety.emergency_stop_topic", "/emergency_stop");
    event_reliable_ = paramT<bool>("safety.event_topics_reliable", true);

    // depth (S3)
    depth_enable_ = paramT<bool>("depth.enable", true);
    depth_convention_ = paramT<std::string>("depth.frame_convention", "auto");
    pitch_correction_deg_ = param("camera_pitch_correction_deg", 9.0);
    dp_.stride = paramInt("depth.stride", dp_.stride);
    dp_.max_row = paramInt("depth.max_row", dp_.max_row);
    dp_.depth_scale = param("depth.depth_scale", dp_.depth_scale);
    dp_.min_depth = param("depth.min_depth", dp_.min_depth);
    dp_.max_depth = param("depth.max_depth", dp_.max_depth);
    dp_.max_obstacle_distance =
        param("depth.max_obstacle_distance", dp_.max_obstacle_distance);
    dp_.min_obstacle_height =
        param("depth.min_obstacle_height", dp_.min_obstacle_height);
    dp_.max_obstacle_height =
        param("depth.max_obstacle_height", dp_.max_obstacle_height);
    dp_.height_tolerance =
        param("depth.height_tolerance", dp_.height_tolerance);
    dp_.body_exclusion_radius =
        param("depth.body_exclusion_radius", dp_.body_exclusion_radius);
    dp_.bin_deg = param("depth.bin_deg", dp_.bin_deg);
    dp_.min_points_per_bin =
        paramInt("depth.min_points_per_bin", dp_.min_points_per_bin);
    dp_.floor_fit = paramT<bool>("depth.floor_fit", dp_.floor_fit);
    dp_.floor_band = param("depth.floor_band", dp_.floor_band);
    dp_.floor_min_distance =
        param("depth.floor_min_distance", dp_.floor_min_distance);
    dp_.floor_max_distance =
        param("depth.floor_max_distance", dp_.floor_max_distance);
    dp_.floor_inlier_threshold =
        param("depth.floor_inlier_threshold", dp_.floor_inlier_threshold);
    dp_.floor_iterations =
        paramInt("depth.floor_iterations", dp_.floor_iterations);
    dp_.floor_max_samples =
        paramInt("depth.floor_max_samples", dp_.floor_max_samples);
    dp_.floor_min_inliers =
        paramInt("depth.floor_min_inliers", dp_.floor_min_inliers);
    dp_.floor_max_tilt_deg =
        param("depth.floor_max_tilt_deg", dp_.floor_max_tilt_deg);
    dp_.floor_max_offset =
        param("depth.floor_max_offset", dp_.floor_max_offset);
    dp_.floor_max_tilt_change_deg =
        param("depth.floor_max_tilt_change_deg", dp_.floor_max_tilt_change_deg);
    dp_.floor_max_offset_change =
        param("depth.floor_max_offset_change", dp_.floor_max_offset_change);
    dp_.floor_rate_tilt_deg =
        param("depth.floor_rate_tilt_deg", dp_.floor_rate_tilt_deg);
    dp_.floor_rate_offset =
        param("depth.floor_rate_offset", dp_.floor_rate_offset);
    dp_.floor_near_min = param("depth.floor_near_min", dp_.floor_near_min);
    dp_.floor_near_max = param("depth.floor_near_max", dp_.floor_near_max);
    dp_.floor_near_min_inliers =
        paramInt("depth.floor_near_min_inliers", dp_.floor_near_min_inliers);
    dp_.floor_bootstrap_frames =
        paramInt("depth.floor_bootstrap_frames", dp_.floor_bootstrap_frames);
    dp_.floor_hold_s = param("depth.floor_hold_s", dp_.floor_hold_s);
    dp_.near_region_min = param("depth.near_region_min", dp_.near_region_min);
    dp_.near_region_max = param("depth.near_region_max", dp_.near_region_max);
    dp_.near_region_half_width =
        param("depth.near_region_half_width", dp_.near_region_half_width);
    dp_.min_near_valid_ratio =
        param("depth.min_near_valid_ratio", dp_.min_near_valid_ratio);
  }

  // 사건형 안전 토픽: 한 번만 오는 메시지를 놓치지 않게 reliable 로 받는다.
  // 발행자가 best effort 면 QoS 가 안 맞아 아무것도 못 받으므로 ERROR 로 알린다
  rclcpp::SubscriptionOptions eventOptions(const std::string& topic) {
    rclcpp::SubscriptionOptions opts;
    opts.event_callbacks.incompatible_qos_callback =
        [this, topic](rclcpp::QOSRequestedIncompatibleQoSInfo& info) {
          RCLCPP_ERROR(get_logger(),
                       "safety topic %s: QoS incompatible with publisher "
                       "(policy %d) - events will be MISSED. Set "
                       "safety.event_topics_reliable:=false",
                       topic.c_str(), info.last_policy_kind);
        };
    return opts;
  }

  void createSafetySubscriptions(const rclcpp::QoS& sensor_qos) {
    using std_msgs::msg::Bool;
    const rclcpp::QoS event_qos =
        event_reliable_ ? rclcpp::QoS(10).reliable() : sensor_qos;
    bottom_ir_sub_ = create_subscription<robot_custom_msgs::msg::BottomIrData>(
        topic_bottom_ir_, sensor_qos,
        [this](robot_custom_msgs::msg::BottomIrData::ConstSharedPtr m) {
          const double t = monoNow();
          ir_rx_ = t;
          have_ir_ = true;
          const bool any = m->ff || m->fl || m->fr || m->bb || m->bl || m->br;
          setLevel(&lvl_ir_, any, t, kHaltLevelHoldS, stop_reason::kCliff,
                   "bottom_ir");
        });
    collision_sub_ =
        create_subscription<robot_custom_msgs::msg::AbnormalEventData>(
            topic_collision_, event_qos,
            [this](
                robot_custom_msgs::msg::AbnormalEventData::ConstSharedPtr m) {
              setLevel(&lvl_collision_, m->event_trigger != 0, monoNow(),
                       kEventOnly, stop_reason::kCollision,
                       "collision_detected");
            },
            eventOptions(topic_collision_));
    for (const auto& topic : topic_collision_clouds_) {
      collision_cloud_subs_.push_back(
          create_subscription<sensor_msgs::msg::PointCloud2>(
              topic, event_qos,
              [this, topic](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
                const bool hit =
                    static_cast<uint64_t>(m->width) * m->height > 0;
                setLevel(&lvl_collision_cloud_, hit, monoNow(), kEventOnly,
                         stop_reason::kCollision, topic.c_str());
              },
              eventOptions(topic)));
    }
    perception_stop_sub_ = create_subscription<std_msgs::msg::Int8>(
        topic_perception_stop_, event_qos,
        [this](std_msgs::msg::Int8::ConstSharedPtr m) {
          setLevel(&lvl_perception_, m->data != 0, monoNow(), kEventOnly,
                   stop_reason::kCollision, "perception_stop");
        },
        eventOptions(topic_perception_stop_));
    lifted_sub_ = create_subscription<Bool>(
        topic_lifted_, sensor_qos, [this](Bool::ConstSharedPtr m) {
          const double t = monoNow();
          lifted_rx_ = t;
          have_lifted_ = true;
          setLevel(&lvl_lifted_, m->data, t, kHaltLevelHoldS,
                   stop_reason::kRobotState, "lifted");
        });
    fall_down_sub_ = create_subscription<Bool>(
        topic_fall_down_, sensor_qos, [this](Bool::ConstSharedPtr m) {
          const double t = monoNow();
          fall_rx_ = t;
          have_fall_ = true;
          setLevel(&lvl_fall_down_, m->data, t, kHaltLevelHoldS,
                   stop_reason::kRobotState, "fall_down");
        });
    cliff_sub_ = create_subscription<Bool>(
        topic_cliff_, event_qos,
        [this](Bool::ConstSharedPtr m) {
          setLevel(&lvl_cliff_, m->data, monoNow(), kEventOnly,
                   stop_reason::kCliff, "cliff_detected");
        },
        eventOptions(topic_cliff_));
    estop_sub_ = create_subscription<Bool>(
        topic_estop_, event_qos,
        [this](Bool::ConstSharedPtr m) {
          setLevel(&lvl_estop_, m->data, monoNow(), kEventOnly,
                   stop_reason::kRobotState, "emergency_stop");
        },
        eventOptions(topic_estop_));
  }

  // ---------------------------------------------------------------- 안전 (S5·S9)
  struct Level {
    bool on{false};  // 마지막으로 받은 값
    double stamp{0.0};
    double hold{0.0};  // true 를 이 시간 동안 레벨로 유지 (0: 사건형)
    uint32_t reason{0};
  };

  void setLevel(Level* lvl, bool on, double t, double hold, uint32_t reason,
                const char* source) {
    lvl->on = on;
    lvl->stamp = t;
    lvl->hold = hold;
    lvl->reason = reason;
    if (!on) {
      return;
    }
    // 다음 주기를 기다리지 않는다
    if (core_->onHalt(t, reason)) {
      publishCmd(0.0, 0.0);
      RCLCPP_WARN(get_logger(), "HALT by %s -> HALTED (%s)", source,
                  reasonsToString(reason).c_str());
      afterEvent(t);
    }
  }

  uint32_t activeHalts(double t) const {
    uint32_t r = 0;
    for (const Level* l :
         {&lvl_ir_, &lvl_collision_, &lvl_collision_cloud_, &lvl_perception_,
          &lvl_lifted_, &lvl_fall_down_, &lvl_cliff_, &lvl_estop_}) {
      if (l->on && t - l->stamp <= l->hold) {
        r |= l->reason;
      }
    }
    return r;
  }

  // ---------------------------------------------------------------- 입력
  void onUi(const follow_msgs::msg::UiClient& m) {
    const double t = monoNow();
    if (m.cmd == ui_cmd::kStart) {
      const FsmState before = core_->state();
      // 사건형 HALT 의 마지막 값이 아직 true 면 다시 시작하지 않는다
      if (lvl_estop_.on || lvl_cliff_.on) {
        RCLCPP_WARN(get_logger(), "START refused: last %s value is still true",
                    lvl_estop_.on ? "emergency_stop" : "cliff_detected");
      } else if (core_->onStart(t)) {
        have_target_ = false;  // START 이후 타깃만 쓴다
        target_hist_size_ = 0;
        // 이전 세션 락(LOCKED)으로 바로 넘어가지 않게
        start_ros_time_ = now().seconds();
        scan_tf_ok_ = false;  // 정적 TF 를 다시 찾는다 (바뀌었을 수 있음)
        {
          std::lock_guard<std::mutex> lk(depth_mutex_);
          depth_reconfigure_ = true;
        }
        publishCmd(0.0, 0.0);
        RCLCPP_INFO(get_logger(), "START: %s -> %s", toString(before),
                    toString(core_->state()));
      } else if (before == FsmState::kHalted) {
        RCLCPP_WARN(get_logger(),
                    "START ignored: HALTED, halt condition active or within "
                    "%.1f s of the last one",
                    cp_.fsm.halt_restart_holdoff_s);
      }
    } else {
      // START(1) 외에는 모두 정지로 본다: 배포 popo 는 STOP=3,
      // 다른 state_manager 버전은 PAUSE=2, RESUME=3, STOP_SOC=4, STOP_AMR=5
      if (m.cmd != ui_cmd::kStop) {
        RCLCPP_WARN(get_logger(), "/follow/ui/client cmd %u treated as STOP",
                    static_cast<unsigned>(m.cmd));
      }
      const FsmState before = core_->state();
      if (core_->onStop(t)) {
        publishCmd(0.0, 0.0);
      }
      if (before != core_->state()) {
        RCLCPP_INFO(get_logger(), "STOP(cmd %u): %s -> IDLE",
                    static_cast<unsigned>(m.cmd), toString(before));
      }
    }
    afterEvent(t);
  }

  void onTarget(const FollowTargetMsg::ConstSharedPtr& m) {
    target_ = m;
    target_rx_ = monoNow();
    have_target_ = true;
    const double stamp = toSec(m->header.stamp);
    // 위치가 유효했던 시각(stamp)의 odom 자세: 이후 로봇 이동을 보정한다
    target_pose_ok_ = odom_hist_.at(stamp, kOdomLookupAhead, &target_pose_);
    // depth 타깃 제외용 경로 이력 (odom 좌표)
    const auto st = static_cast<TargetStatus>(m->status);
    const Vec2 p{m->position.x, m->position.y};
    if (!target_pose_ok_ || stamp < start_ros_time_ ||
        !(st == TargetStatus::kLocked || st == TargetStatus::kOccluded ||
          st == TargetStatus::kCandidate) ||
        !std::isfinite(p.x) || !std::isfinite(p.y)) {
      return;
    }
    const Vec2 p_odom = transformPoint(target_pose_, p);
    const std::size_t cap = target_hist_.size();
    if (target_hist_size_ > 0) {
      // 트랙이 바뀌거나 설명 안 되는 점프면 경로를 끊는다 (꺾은선이 엉뚱한 곳을 덮지 않게)
      const TargetSample& last =
          target_hist_[(target_hist_head_ + cap - 1) % cap];
      const double dt = std::max(0.0, stamp - last.t);
      const double speed =
          std::min(std::hypot(m->velocity.x, m->velocity.y), target_max_speed_);
      const bool jump =
          !std::isfinite(speed) ||
          distance(p_odom, last.p) > speed * dt + kTrackJumpMargin;
      if (m->track_id != last.track_id || jump) {
        target_hist_size_ = 0;
      }
    }
    TargetSample& e = target_hist_[target_hist_head_];
    e.t = stamp;
    e.p = p_odom;
    e.track_id = m->track_id;
    target_hist_head_ = (target_hist_head_ + 1) % cap;
    target_hist_size_ = std::min(target_hist_size_ + 1, cap);
  }

  void onScan(const sensor_msgs::msg::LaserScan& m) {
    const double t = monoNow();
    if (!scan_tf_ok_ || m.header.frame_id != scan_frame_) {
      scan_frame_ = m.header.frame_id;
      scan_tf_ok_ = lookupPlanar(scan_frame_, &scan_tf_);
      scan_cos_.clear();  // TF(방향)가 바뀌었을 수 있으니 각도 표를 다시 만든다
      // TF 없음 → scan 미수신으로 취급 (S6)
      if (!scan_tf_ok_) {
        return;
      }
    }
    const std::size_t n = m.ranges.size();
    if (scan_cos_.size() != n || scan_angle_min_ != m.angle_min ||
        scan_angle_inc_ != m.angle_increment) {
      scan_cos_.resize(n);
      scan_sin_.resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        const double a = m.angle_min + m.angle_increment * i + scan_tf_.yaw;
        scan_cos_[i] = std::cos(a);
        scan_sin_[i] = std::sin(a);
      }
      scan_angle_min_ = m.angle_min;
      scan_angle_inc_ = m.angle_increment;
    }
    const double rmin = std::max(static_cast<double>(m.range_min), 0.0);
    const double rmax =
        std::min(static_cast<double>(m.range_max), scan_max_range_);
    scan_pts_.clear();
    for (std::size_t i = 0; i < n; ++i) {
      const double r = m.ranges[i];
      if (!std::isfinite(r) || r < rmin || r > rmax) {
        continue;
      }
      const Vec2 p{scan_tf_.x + r * scan_cos_[i],
                   scan_tf_.y + r * scan_sin_[i]};
      // 로봇 자기 몸체
      if (norm(p) < body_filter_radius_) {
        continue;
      }
      scan_pts_.push_back(p);
    }
    scan_rx_ = t;
    have_scan_ = true;
    scan_pose_ok_ =
        odom_hist_.at(toSec(m.header.stamp), kOdomLookupAhead, &scan_pose_);
  }

  void onOdom(const nav_msgs::msg::Odometry& m) {
    odom_rx_ = monoNow();
    have_odom_ = true;
    const double t = toSec(m.header.stamp);
    const Pose2D pose{m.pose.pose.position.x, m.pose.pose.position.y,
                      yawOf(m.pose.pose.orientation)};
    odom_hist_.add(t, pose);
    // 실측 선속도: twist 와 자세 변화(창 kOdomSpeedWindowS) 중 큰 값.
    // 일부 베이스(popo 실측)는 /odom twist 를 늘 0 으로 보낸다
    double pose_speed = 0.0;
    Pose2D past;
    if (odom_hist_.at(t - kOdomSpeedWindowS, 0.0, &past)) {
      pose_speed = ((pose.x - past.x) * std::cos(pose.yaw) +
                    (pose.y - past.y) * std::sin(pose.yaw)) /
                   kOdomSpeedWindowS;
    }
    odom_speed_ = std::max(m.twist.twist.linear.x, pose_speed);
  }

  // base_frame ← frame 평면 변환 (정적 TF, 최신 값)
  bool lookupPlanar(const std::string& frame, Pose2D* out) {
    try {
      const auto tf =
          tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      out->x = tf.transform.translation.x;
      out->y = tf.transform.translation.y;
      out->yaw = yawOf(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "TF %s <- %s: %s", base_frame_.c_str(),
                           frame.c_str(), e.what());
      return false;
    }
  }

  // ---------------------------------------------------------------- depth (별도 콜백 그룹)
  void updateDepthSubscription(ModeValue mode) {
    const bool want = depth_enable_ && mode != ModeValue::kOff;
    if (want && !depth_sub_) {
      rclcpp::SubscriptionOptions opts;
      opts.callback_group = depth_group_;
      depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          topic_depth_info_, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
            onDepthInfo(*m);
          },
          opts);
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
          topic_depth_, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::Image::ConstSharedPtr m) { onDepth(*m); },
          opts);
    } else if (!want && depth_sub_) {
      // 추종하지 않을 때는 영상을 받지 않는다 (SPEC 4절)
      depth_sub_.reset();
      depth_info_sub_.reset();
      std::lock_guard<std::mutex> lk(depth_mutex_);
      have_depth_ = false;
    }
  }

  void onDepthInfo(const sensor_msgs::msg::CameraInfo& m) {
    CameraIntrinsics K;
    K.fx = m.k[0];
    K.fy = m.k[4];
    K.cx = m.k[2];
    K.cy = m.k[5];
    K.width = static_cast<int>(m.width);
    K.height = static_cast<int>(m.height);
    if (K.fx <= 0.0 || K.fy <= 0.0 || K.width <= 0 || K.height <= 0) {
      return;
    }
    if (!have_info_ || K.fx != depth_K_.fx || K.fy != depth_K_.fy ||
        K.cx != depth_K_.cx || K.cy != depth_K_.cy ||
        K.width != depth_K_.width || K.height != depth_K_.height) {
      depth_K_ = K;
      have_info_ = true;
      depth_configured_ = false;
    }
  }

  // depth 스레드에서 읽는 값: 실행 중 조정과 겹치지 않게 잠그고 읽는다
  double cameraCapLocked() {
    std::lock_guard<std::mutex> lk(tune_mutex_);
    return cp_.camera_missing_speed_cap;
  }

  bool configureDepth(const std::string& frame) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "depth TF %s <- %s: %s (depth 무시, v<=%.2f)",
                           base_frame_.c_str(), frame.c_str(), e.what(),
                           cameraCapLocked());
      return false;
    }
    const auto& q = tf.transform.rotation;
    const tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
    double R[9];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R[r * 3 + c] = m[r][c];
      }
    }
    const double t[3] = {tf.transform.translation.x, tf.transform.translation.y,
                         tf.transform.translation.z};
    const FrameConvention conv =
        (depth_convention_ == "optical") ? FrameConvention::kOptical
        : (depth_convention_ == "body")  ? FrameConvention::kBody
                                         : FrameConvention::kAuto;
    bool optical = true;
    const CameraExtrinsics ext = toOpticalExtrinsics(R, t, conv, &optical);
    checker_->configure(depth_K_, ext, degToRad(pitch_correction_deg_));
    const double tf_pitch =
        radToDeg(std::atan2(-ext.R[8], std::hypot(ext.R[2], ext.R[5])));
    // URDF 가 이미 숙임을 갖는데 보정도 주면 두 번 숙인다: 바닥 추정이 늘 실패하므로 depth 를 쓰지 않는다
    const bool double_pitch = std::abs(tf_pitch) > kTfPitchSuspiciousDeg &&
                              pitch_correction_deg_ != 0.0;
    {
      std::lock_guard<std::mutex> lk(depth_mutex_);
      depth_config_error_ = double_pitch;
    }
    if (double_pitch) {
      RCLCPP_ERROR(get_logger(),
                   "depth TF already pitched %.1f deg and "
                   "camera_pitch_correction_deg=%.1f: double correction. "
                   "depth treated as degraded (v<=%.2f). Set the correction "
                   "to 0 if the URDF has the real pitch.",
                   tf_pitch, pitch_correction_deg_,
                   cameraCapLocked());
    }
    RCLCPP_INFO(get_logger(),
                "depth configured: frame=%s (%s) t=(%.3f %.3f %.3f) TF pitch "
                "%.1f deg + correction %.1f deg, %dx%d fx=%.1f fy=%.1f",
                frame.c_str(), optical ? "optical" : "body", ext.t[0], ext.t[1],
                ext.t[2], tf_pitch, pitch_correction_deg_, depth_K_.width,
                depth_K_.height, depth_K_.fx, depth_K_.fy);
    return true;
  }

  void onDepth(const sensor_msgs::msg::Image& m) {
    const double t = monoNow();
    const double t_ros = now().seconds();
    if (m.encoding != "16UC1" && m.encoding != "mono16") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "depth encoding %s unsupported (16UC1 expected)",
                           m.encoding.c_str());
      return;
    }
    // 크기가 맞지 않는 메시지로 범위 밖을 읽지 않게 (컨테이너 전체가 죽을 수 있다)
    if (static_cast<std::size_t>(m.step) <
            static_cast<std::size_t>(m.width) * kBytesPerDepthPixel ||
        m.data.size() < static_cast<std::size_t>(m.step) *
                            static_cast<std::size_t>(m.height)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "depth frame size mismatch: %ux%u step %u data %zu",
                           m.width, m.height, m.step, m.data.size());
      return;
    }
    if (!have_info_) {
      return;
    }
    bool reconfigure = false;
    {
      std::lock_guard<std::mutex> lk(depth_mutex_);
      reconfigure = depth_reconfigure_;
      depth_reconfigure_ = false;
    }
    if (reconfigure || !depth_configured_ ||
        m.header.frame_id != depth_frame_) {
      depth_frame_ = m.header.frame_id;
      depth_configured_ = configureDepth(depth_frame_);
      if (!depth_configured_) {
        return;
      }
    }
    // 카메라 stamp 는 다른 시계(Jetson)라 직접 못 쓴다: 수신-stamp 차의 최근 최솟값을 기준으로
    // 이 프레임이 평소보다 늦게 온 만큼(extra)을 지연에 더한다
    const double lag = t_ros - toSec(m.header.stamp);
    depth_lag_[depth_lag_head_] = lag;
    depth_lag_head_ = (depth_lag_head_ + 1) % depth_lag_.size();
    depth_lag_count_ = std::min(depth_lag_count_ + 1, depth_lag_.size());
    double lag_min = lag;
    for (std::size_t i = 0; i < depth_lag_count_; ++i) {
      lag_min = std::min(lag_min, depth_lag_[i]);
    }
    const double extra = std::max(0.0, lag - lag_min);
    if (!checker_->process(m.data.data(), m.step, static_cast<int>(m.width),
                           static_cast<int>(m.height), m.is_bigendian != 0, t,
                           &depth_result_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "depth frame %ux%u rejected (camera_info %dx%d)",
                           m.width, m.height, depth_K_.width, depth_K_.height);
      return;
    }
    std::lock_guard<std::mutex> lk(depth_mutex_);
    depth_pts_shared_ = depth_result_.points;  // 용량 재사용
    depth_rx_ = t;
    depth_rx_ros_ = t_ros;
    depth_extra_delay_ = extra;
    have_depth_ = true;
    depth_healthy_ = depth_result_.healthy;
    depth_has_points_ = depth_result_.floor_source != FloorSource::kNone;
    depth_floor_source_ = static_cast<uint8_t>(depth_result_.floor_source);
    depth_valid_ratio_ = depth_result_.near_valid_ratio;
    depth_proc_ms_ = depth_result_.proc_ms;
    floor_pitch_deg_ = (depth_result_.floor_source == FloorSource::kFitted)
                           ? depth_result_.pitch_deg
                           : std::nan("");
  }

  // ---------------------------------------------------------------- 주기 제어
  static bool fresh(bool have, double now, double rx, double timeout) {
    return timeout <= 0.0 || (have && now - rx <= timeout);
  }

  void controlTick() {
    const auto t_begin = std::chrono::steady_clock::now();
    {
      // 실행 중 조정 값을 주기 시작에 한꺼번에 적용 (FSM·속도 상태는 유지)
      std::lock_guard<std::mutex> lk(tune_mutex_);
      if (tune_pending_) {
        cp_ = tune_cp_;
        tune_pending_ = false;
        core_->updateParams(cp_);
      }
    }
    const double t = monoNow();  // 내부 시간: 주기·신선도·상태기계
    const double t_ros = now().seconds();  // stamp 와 비교하는 시간
    const double dt =
        (last_tick_ > 0.0) ? t - last_tick_ : 1.0 / control_rate_hz_;
    last_tick_ = t;

    CycleInput in;
    in.now = t;
    in.dt = dt;

    Pose2D pose_now;
    double pose_t = 0.0;
    const bool pose_ok = odom_hist_.latest(&pose_t, &pose_now);

    // 센서 상태 (S6)
    in.sensors.tf_ok = scan_tf_ok_;
    in.sensors.scan_ok =
        scan_tf_ok_ && fresh(have_scan_, t, scan_rx_, scan_timeout_s_);
    in.sensors.odom_ok = fresh(have_odom_, t, odom_rx_, odom_timeout_s_);
    in.sensors.bottom_ir_ok = fresh(have_ir_, t, ir_rx_, bottom_ir_timeout_s_);
    in.sensors.lifted_ok =
        fresh(have_lifted_, t, lifted_rx_, lifted_timeout_s_);
    in.sensors.fall_down_ok =
        fresh(have_fall_, t, fall_rx_, fall_down_timeout_s_);
    double depth_rx = 0.0;
    double depth_extra = 0.0;
    {
      std::lock_guard<std::mutex> lk(depth_mutex_);
      // 받은 지 오래됐거나 늦게 온 프레임은 신선하지 않다 (m4)
      const bool depth_fresh =
          depth_enable_ && have_depth_ && !depth_config_error_ &&
          (t - depth_rx_) + depth_extra_delay_ <= camera_timeout_s_;
      // healthy: 바닥 평면을 확보했고 가까운 영역 유효 비율이 충분 (M1). 아니면 카메라 상실 상한
      in.sensors.camera_ok = depth_fresh && depth_healthy_;
      in.sensors.depth_points_ok = depth_fresh && depth_has_points_;
      if (in.sensors.depth_points_ok) {
        depth_pts_local_ = depth_pts_shared_;
        depth_rx = depth_rx_ros_;
        depth_extra = depth_extra_delay_;
      } else {
        depth_pts_local_.clear();
      }
      diag_depth_ms_ = depth_proc_ms_;
      diag_floor_pitch_ = floor_pitch_deg_;
      diag_floor_source_ = depth_floor_source_;
      diag_valid_ratio_ = depth_valid_ratio_;
    }

    // 장애물 점: 측정 시점 base → 현재 base
    in.scan.points = scan_pts_.data();
    in.scan.count = in.sensors.scan_ok ? scan_pts_.size() : 0;
    in.scan.to_now = (pose_ok && scan_pose_ok_)
                         ? relativePose(pose_now, scan_pose_)
                         : Pose2D{};
    in.depth.points = depth_pts_local_.data();
    in.depth.count = depth_pts_local_.size();
    // depth 영상 내용은 수신보다 (camera_latency_s + 이 프레임의 추가 지연) 앞선 것
    Pose2D depth_pose;
    const double depth_capture = depth_rx - camera_latency_s_ - depth_extra;
    if (pose_ok &&
        (odom_hist_.at(depth_capture, kOdomLookupAhead, &depth_pose) ||
         odom_hist_.at(depth_rx, kOdomLookupAhead, &depth_pose))) {
      in.depth.to_now = relativePose(pose_now, depth_pose);
    } else {
      in.depth.to_now = Pose2D{};
    }

    fillTarget(t, t_ros, pose_ok, pose_now, &in.target);
    buildDepthExclusion(depth_capture - depth_exclusion_margin_s_, pose_ok,
                        pose_now, in.target, &in.depth_exclusion);
    in.halt_active = activeHalts(t);
    in.measured_speed = odom_speed_;

    const FsmState before = core_->state();
    const CycleOutput out = core_->step(in);
    if (out.publish) {
      publishCmd(out.v, out.w);
    }
    if (out.state_changed) {
      RCLCPP_INFO(get_logger(), "%s -> %s (%s)", toString(before),
                  toString(out.state),
                  reasonsToString(out.stop_reasons).c_str());
    }
    if (out.state == FsmState::kWaiting || out.state == FsmState::kFollowing) {
      if ((out.stop_reasons & stop_reason::kSensorTimeout) != 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                             "sensor timeout: scan=%d odom=%d tf=%d "
                             "bottom_ir=%d lifted=%d fall_down=%d",
                             in.sensors.scan_ok, in.sensors.odom_ok,
                             in.sensors.tf_ok, in.sensors.bottom_ir_ok,
                             in.sensors.lifted_ok, in.sensors.fall_down_ok);
      }
    }
    last_out_ = out;
    last_camera_ok_ = in.sensors.camera_ok;
    afterEvent(t);
    publishDetect(t, out.detect_user, out.state);
    last_cycle_ms_ = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - t_begin)
                         .count();
  }

  // 늦게 온 depth 에는 타깃이 지나온 자리에 사람이 찍혀 있다: t_from 이후 타깃 경로 + 현재 위치
  void buildDepthExclusion(double t_from, bool pose_ok, const Pose2D& pose_now,
                           const TargetInput& target, Exclusion* out) {
    depth_excl_pts_.clear();
    if (!target.received || !pose_ok) {
      return;
    }
    const Pose2D inv_now = inverse(pose_now);
    const std::size_t cap = target_hist_.size();
    const std::size_t oldest =
        (target_hist_head_ + cap - target_hist_size_) % cap;
    for (std::size_t k = 0; k < target_hist_size_; ++k) {
      const TargetSample& e = target_hist_[(oldest + k) % cap];
      if (e.t >= t_from) {
        depth_excl_pts_.push_back(transformPoint(inv_now, e.p));
      }
    }
    if (depth_excl_pts_.empty()) {
      return;
    }
    depth_excl_pts_.push_back(target.position);
    *out = Exclusion{depth_excl_pts_.data(), depth_excl_pts_.size()};
  }

  void fillTarget(double t, double t_ros, bool pose_ok, const Pose2D& pose_now,
                  TargetInput* out) {
    if (!have_target_ || !target_) {
      return;
    }
    const FollowTargetMsg& m = *target_;
    const double stamp = toSec(m.header.stamp);
    // START 전에 잡은 측정 (이전 세션 락): 재획득부터 (m1)
    if (stamp < start_ros_time_) {
      return;
    }
    const Vec2 p{m.position.x, m.position.y};
    Vec2 v{m.velocity.x, m.velocity.y};
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        norm(p) > target_max_range_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "target ignored: invalid position (%.2f, %.2f)", p.x,
                           p.y);
      return;  // 받지 않은 것으로 본다
    }
    if (!std::isfinite(v.x) || !std::isfinite(v.y) ||
        norm(v) > target_max_speed_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "target velocity (%.2f, %.2f) invalid: using 0", v.x,
                           v.y);
      v = Vec2{};
    }
    const double stamp_age = t_ros - stamp;
    if (std::abs(stamp_age) > kStampWarnS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "target stamp off by %.1f s (clock mismatch?)",
                           stamp_age);
    }
    out->received = true;
    out->status = static_cast<TargetStatus>(m.status);
    out->ambiguous = m.ambiguous;
    // 나이 = 받은 뒤 시간과 stamp 이후 시간 중 큰 것 (m4)
    out->msg_age = std::max(t - target_rx_, stamp_age);
    out->occluded_time = std::isfinite(m.occluded_time) ? m.occluded_time : 0.0;
    // stamp 이후 경과 시간만큼 타깃 속도로 외삽 (SPEC 6.3)
    const double ex = std::clamp(stamp_age, 0.0, max_extrapolation_s_);
    if (pose_ok && target_pose_ok_) {
      const Vec2 p_odom = transformPoint(target_pose_, p);
      const Vec2 v_odom = rotate(v, target_pose_.yaw);
      const Vec2 p_pred{p_odom.x + v_odom.x * ex, p_odom.y + v_odom.y * ex};
      out->position = transformPoint(inverse(pose_now), p_pred);
      out->velocity = rotate(v_odom, -pose_now.yaw);
    } else {
      // odom 이력이 없으면 로봇 이동 보정 없이 외삽만
      out->position = {p.x + v.x * ex, p.y + v.y * ex};
      out->velocity = v;
    }
  }

  // 내부 시간 [s]. 벽시계가 뛰어도(NTP·WSL 시계 보정) 주기·신선도·타이머가 흔들리지 않게
  // steady clock 을 쓴다. bag 재생(use_sim_time)이면 ROS 시간
  double monoNow() {
    if (use_sim_time_) {
      return now().seconds();
    }
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  // ---------------------------------------------------------------- 출력
  void publishCmd(double v, double w) {
    // 종료 콜백(다른 스레드)과 순서를 보장: 종료 0 명령 뒤로는 움직이는 명령이 나가지 않는다
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (shutting_down_.load() && (v != 0.0 || w != 0.0)) {
      return;
    }
    geometry_msgs::msg::Twist tw;
    tw.linear.x = v;
    tw.angular.z = w;
    cmd_pub_->publish(tw);
  }

  void afterEvent(double t) {
    active_.store(isActive(core_->state()));
    const ModeValue mode = modeOf(core_->state());
    publishMode(t, false);
    updateDepthSubscription(mode);
  }

  void publishMode(double t, bool force) {
    const ModeValue mode = modeOf(core_->state());
    if (!mode_report_.shouldPublish(static_cast<int>(mode), t) && !force) {
      return;
    }
    FollowModeMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.mode = static_cast<uint8_t>(mode);
    mode_pub_->publish(msg);
  }

  void publishDetect(double t, int16_t value, FsmState s) {
    // IDLE 에서는 보내지 않는다
    if (value == detect_user::kNone) {
      detect_report_.reset();
      return;
    }
    const int key = value * 16 + static_cast<int>(s);
    if (!detect_report_.shouldPublish(key, t)) {
      return;
    }
    follow_msgs::msg::Detect msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.detect_user = value;
    msg.detect_status = static_cast<int16_t>(s);
    detect_pub_->publish(msg);
  }

  void publishState() {
    const CycleOutput& o = last_out_;
    FollowStateMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_;
    msg.fsm = static_cast<uint8_t>(o.state);
    msg.stop_reasons = o.stop_reasons;
    msg.shadow = shadow_;
    msg.cmd_v = static_cast<float>(o.publish ? o.v : 0.0);
    msg.cmd_w = static_cast<float>(o.publish ? o.w : 0.0);
    msg.v_limit = static_cast<float>(o.v_limit);
    msg.obstacle_distance = finiteOr(o.obstacle_distance, kFarDistance);
    msg.target_range = static_cast<float>(o.target_range);
    msg.target_bearing = static_cast<float>(o.target_bearing);
    msg.scan_obstacle_distance =
        finiteOr(o.scan_obstacle_distance, kFarDistance);
    msg.depth_obstacle_distance =
        finiteOr(o.depth_obstacle_distance, kFarDistance);
    msg.depth_ok = last_camera_ok_;
    msg.floor_pitch_deg = static_cast<float>(diag_floor_pitch_);
    msg.depth_proc_ms = static_cast<float>(diag_depth_ms_);
    msg.cycle_ms = static_cast<float>(last_cycle_ms_);
    msg.floor_source = diag_floor_source_;
    msg.depth_valid_ratio = static_cast<float>(diag_valid_ratio_);
    state_pub_->publish(msg);
  }

  void onShutdown() {
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      if (shutting_down_.exchange(true)) {
        return;
      }
      // IDLE·ENDED: 다른 주행 기능과 겹치지 않게 내지 않는다
      if (!active_.load()) {
        return;
      }
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
    cmd_pub_->wait_for_all_acked(
        std::chrono::milliseconds(static_cast<int64_t>(kAckTimeoutMs)));
    RCLCPP_INFO(get_logger(), "shutdown: zero command sent");
  }

  // ---------------------------------------------------------------- 멤버
  ControllerParams cp_;
  DepthCheckerParams dp_;
  std::unique_ptr<ControllerCore> core_;
  std::unique_ptr<DepthObstacleChecker> checker_;
  ChangeOrPeriodic detect_report_;
  ChangeOrPeriodic mode_report_;

  bool shadow_{false};
  bool use_sim_time_{false};
  double control_rate_hz_{20.0};
  double state_rate_hz_{10.0};
  double report_period_s_{1.0};
  std::string base_frame_;
  std::string topic_cmd_vel_, topic_shadow_, topic_target_, topic_scan_,
      topic_odom_, topic_depth_, topic_depth_info_, topic_ui_, topic_detect_,
      topic_mode_, topic_state_;
  std::string topic_bottom_ir_, topic_collision_, topic_perception_stop_,
      topic_lifted_, topic_fall_down_, topic_cliff_, topic_estop_;
  std::vector<std::string> topic_collision_clouds_;
  bool event_reliable_{true};
  double max_extrapolation_s_{0.5};
  double target_max_range_{8.0};
  double target_max_speed_{3.0};
  double body_filter_radius_{0.2};
  double scan_max_range_{4.0};
  double scan_timeout_s_{0.3};
  double odom_timeout_s_{0.2};
  double bottom_ir_timeout_s_{0.2};
  double lifted_timeout_s_{1.0};
  double fall_down_timeout_s_{3.0};
  double camera_timeout_s_{1.0};
  double camera_latency_s_{0.1};
  double depth_exclusion_margin_s_{0.2};
  bool depth_enable_{true};
  std::string depth_convention_;
  double pitch_correction_deg_{9.0};

  // 기본 콜백 그룹 (상호 배제) 에서만 쓰는 상태
  FollowTargetMsg::ConstSharedPtr target_;
  bool have_target_{false};
  double target_rx_{0.0};
  double start_ros_time_{-kInf};
  Pose2D target_pose_;
  bool target_pose_ok_{false};
  std::vector<Vec2> scan_pts_;
  std::vector<double> scan_cos_, scan_sin_;
  float scan_angle_min_{0.0f};
  float scan_angle_inc_{0.0f};
  std::string scan_frame_;
  Pose2D scan_tf_;
  bool scan_tf_ok_{false};
  Pose2D scan_pose_;
  bool scan_pose_ok_{false};
  bool have_scan_{false};
  double scan_rx_{0.0};
  PoseHistory odom_hist_;
  bool have_odom_{false};
  double odom_rx_{0.0};
  double odom_speed_{0.0};
  bool have_ir_{false};
  double ir_rx_{0.0};
  bool have_lifted_{false};
  double lifted_rx_{0.0};
  bool have_fall_{false};
  double fall_rx_{0.0};
  Level lvl_ir_, lvl_collision_, lvl_collision_cloud_, lvl_perception_,
      lvl_lifted_, lvl_fall_down_, lvl_cliff_, lvl_estop_;
  std::vector<Vec2> depth_pts_local_;
  struct TargetSample {
    double t{0.0};
    Vec2 p;  // odom 좌표
    int32_t track_id{0};
  };
  std::vector<TargetSample> target_hist_;
  std::size_t target_hist_head_{0};
  std::size_t target_hist_size_{0};
  std::vector<Vec2> depth_excl_pts_;
  double last_tick_{0.0};
  CycleOutput last_out_;
  bool last_camera_ok_{false};
  double diag_depth_ms_{0.0};
  double diag_floor_pitch_{0.0};
  uint8_t diag_floor_source_{0};
  double diag_valid_ratio_{0.0};
  double last_cycle_ms_{0.0};

  // depth 콜백 그룹에서만 쓰는 상태
  CameraIntrinsics depth_K_;
  bool have_info_{false};
  bool depth_configured_{false};
  std::string depth_frame_;
  DepthResult depth_result_;
  std::vector<double> depth_lag_;
  std::size_t depth_lag_head_{0};
  std::size_t depth_lag_count_{0};

  // 두 그룹이 같이 쓰는 상태 (depth_mutex_)
  std::mutex depth_mutex_;
  std::vector<Vec2> depth_pts_shared_;
  double depth_rx_{0.0};      // 내부 시간 (신선도)
  double depth_rx_ros_{0.0};  // ROS 시간 (odom 이력 조회)
  double depth_extra_delay_{0.0};
  bool have_depth_{false};
  bool depth_healthy_{false};
  bool depth_has_points_{false};
  bool depth_config_error_{false};
  uint8_t depth_floor_source_{0};
  double depth_valid_ratio_{0.0};
  bool depth_reconfigure_{false};
  double depth_proc_ms_{0.0};
  double floor_pitch_deg_{std::nan("")};

  std::mutex cmd_mutex_;
  // 실행 중 조정: 콜백이 검사한 값을 넣고 controlTick 이 적용한다
  std::mutex tune_mutex_;
  ControllerParams tune_cp_;
  bool tune_pending_{false};
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::atomic<bool> active_{false};
  std::atomic<bool> shutting_down_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::CallbackGroup::SharedPtr depth_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<follow_msgs::msg::Detect>::SharedPtr detect_pub_;
  rclcpp::Publisher<FollowModeMsg>::SharedPtr mode_pub_;
  rclcpp::Publisher<FollowStateMsg>::SharedPtr state_pub_;
  rclcpp::Subscription<follow_msgs::msg::UiClient>::SharedPtr ui_sub_;
  rclcpp::Subscription<FollowTargetMsg>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr
      bottom_ir_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr
      collision_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      collision_cloud_subs_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr perception_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lifted_sub_,
      fall_down_sub_, cliff_sub_, estop_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_;
};

std::shared_ptr<rclcpp::Node> createFollowController(
    const rclcpp::NodeOptions& options) {
  return std::make_shared<FollowController>(options);
}

}  // namespace kebi_follow

RCLCPP_COMPONENTS_REGISTER_NODE(kebi_follow::FollowController)
