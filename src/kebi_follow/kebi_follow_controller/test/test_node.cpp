// 노드 단위 시험: 실제 follow_controller 를 같은 프로세스에 띄우고 가짜 발행자로 확인한다.
// 격리: 로봇 도메인(30)이나 로봇 peer 가 든 사용자 DDS 프로파일로 나가지 않게
//       main 에서 도메인을 바꾸고 127.0.0.1 UDP 전용 프로파일을 쓴다.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/static_transform_broadcaster.h>
#include <follow_msgs/msg/detect.hpp>
#include <follow_msgs/msg/ui_client.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kebi_follow_msgs/msg/follow_mode.hpp>
#include <kebi_follow_msgs/msg/follow_state.hpp>
#include <kebi_follow_msgs/msg/follow_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_custom_msgs/msg/abnormal_event_data.hpp>
#include <robot_custom_msgs/msg/bottom_ir_data.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include "kebi_follow_controller/follow_controller.hpp"
#include "kebi_follow_controller/types.hpp"
#include "synthetic_depth.hpp"

using namespace std::chrono_literals;
using kebi_follow_msgs::msg::FollowState;
using kebi_follow_msgs::msg::FollowTarget;

namespace {

constexpr int kScanRays = 401;
constexpr double kTau = 2.0 * M_PI;

double wallNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

class NodeTest : public ::testing::Test {
 protected:
  struct Cmd {
    double t;
    double v;
    double w;
  };

  void startNode(std::vector<rclcpp::Parameter> params = {}) {
    params.emplace_back("depth.enable", depth_enable_);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);
    ctrl_ = kebi_follow::createFollowController(opts);
    io_ = std::make_shared<rclcpp::Node>("test_io");
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    const auto rel = rclcpp::QoS(10).reliable();
    pub_ui_ = io_->create_publisher<follow_msgs::msg::UiClient>(
        "/follow/ui/client", rel);
    pub_target_ = io_->create_publisher<FollowTarget>(
        "/kebi_follow/target", rclcpp::QoS(5).reliable());
    pub_scan_ = io_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 5);
    pub_odom_ = io_->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
    pub_ir_ = io_->create_publisher<robot_custom_msgs::msg::BottomIrData>(
        "/bottom_ir_data", rel);
    pub_lifted_ =
        io_->create_publisher<std_msgs::msg::Bool>("/error/s_code/lifted", rel);
    pub_fall_ = io_->create_publisher<std_msgs::msg::Bool>(
        "/error/s_code/fall_down", rel);
    pub_coll_ =
        io_->create_publisher<robot_custom_msgs::msg::AbnormalEventData>(
            "/collision_detected", rel);
    pub_estop_ =
        io_->create_publisher<std_msgs::msg::Bool>("/emergency_stop", rel);
    pub_depth_ = io_->create_publisher<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", rclcpp::SensorDataQoS());
    pub_info_ = io_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", rclcpp::SensorDataQoS());
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(io_);
    sub_cmd_ = io_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(100).reliable(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          cmds_.push_back({wallNow(), m->linear.x, m->angular.z});
        });
    sub_state_ = io_->create_subscription<FollowState>(
        "/kebi_follow/state", rclcpp::QoS(10).reliable(),
        [this](FollowState::ConstSharedPtr m) {
          state_ = *m;
          have_state_ = true;
        });
    publishScanTf(0.0, 0.0);
    t0_ = wallNow();
    feed_timer_ = io_->create_wall_timer(10ms, [this]() { feed(); });
    exec_->add_node(ctrl_);
    exec_->add_node(io_);
    spinFor(0.5);  // 발견·연결
  }

  void TearDown() override {
    if (exec_) {
      exec_->cancel();
    }
    feed_timer_.reset();
    exec_.reset();
    ctrl_.reset();
    static_tf_.reset();
    io_.reset();
  }

  void publishScanTf(double x, double yaw) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = io_->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "base_scan";
    t.transform.translation.x = x;
    t.transform.translation.z = 0.155;
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);
    std::vector<geometry_msgs::msg::TransformStamped> v{t};
    if (depth_enable_) {
      v.push_back(depthTf());
    }
    static_tf_->sendTransform(v);
  }

  geometry_msgs::msg::TransformStamped depthTf() {
    // 로봇과 같은 구조: TF 는 pitch 0 (실제 숙임은 보정 9° + 바닥 추정)
    const kebi_follow::CameraExtrinsics T =
        kebi_follow::synth::urdfExtrinsics(tf_pitch_deg_);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = io_->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "inusensor_depth";
    t.transform.translation.x = T.t[0];
    t.transform.translation.y = T.t[1];
    t.transform.translation.z = T.t[2];
    const double* R = T.R;
    const double w = std::sqrt(std::max(0.0, 1.0 + R[0] + R[4] + R[8])) / 2.0;
    t.transform.rotation.w = w;
    t.transform.rotation.x = (R[7] - R[5]) / (4.0 * w);
    t.transform.rotation.y = (R[2] - R[6]) / (4.0 * w);
    t.transform.rotation.z = (R[3] - R[1]) / (4.0 * w);
    return t;
  }

  // 10 ms 마다 센서·타깃 발행 (플래그로 끊을 수 있다)
  void feed() {
    const double t = wallNow();
    ++tick_;
    const auto stamp = io_->now();
    if (feed_odom_) {
      nav_msgs::msg::Odometry o;
      o.header.stamp = stamp;
      o.header.frame_id = "odom";
      o.pose.pose.position.x = odom_v_ * (t - t0_);
      o.pose.pose.orientation.w = 1.0;
      o.twist.twist.linear.x = odom_v_;
      pub_odom_->publish(o);
    }
    if (tick_ % 2 == 0) {
      if (feed_ir_) {
        pub_ir_->publish(robot_custom_msgs::msg::BottomIrData());
      }
      if (feed_lifted_) {
        std_msgs::msg::Bool b;
        b.data = lifted_value_;
        pub_lifted_->publish(b);
      }
    }
    if (tick_ % 50 == 0 && feed_fall_) {
      pub_fall_->publish(std_msgs::msg::Bool());
    }
    if (tick_ % 10 == 0) {
      if (feed_scan_) {
        sensor_msgs::msg::LaserScan s;
        s.header.stamp = stamp;
        s.header.frame_id = "base_scan";
        s.angle_min = -M_PI;
        s.angle_max = M_PI;
        s.angle_increment = kTau / (kScanRays - 1);
        s.range_min = 0.03;
        s.range_max = 8.0;
        s.ranges.assign(kScanRays, std::numeric_limits<float>::infinity());
        for (const auto& [idx, r] : scan_returns_) {
          s.ranges[idx] = r;
        }
        pub_scan_->publish(s);
      }
      if (feed_target_) {
        FollowTarget m;
        m.header.frame_id = "base_link";
        m.header.stamp = target_fixed_stamp_
                             ? *target_fixed_stamp_
                             : rclcpp::Time(stamp.nanoseconds() +
                                            static_cast<int64_t>(
                                                target_stamp_offset_ * 1e9));
        m.status = target_status_;
        m.track_id = 1;
        m.position.x = target_x_;
        m.position.y = target_y_;
        m.velocity.x = target_vx_;
        m.range = std::hypot(target_x_, target_y_);
        pub_target_->publish(m);
      }
      if (depth_enable_ && depth_frame_) {
        depth_frame_(t, stamp);
      }
    }
  }

  // 바닥만 있는 합성 depth (작은 해상도). stamp 늦춤·잘린 데이터를 고를 수 있다
  void useFloorDepth() {
    depth_frame_ = [this](double, const rclcpp::Time& stamp) {
      const kebi_follow::CameraIntrinsics K{106.0, 85.0, 105.975,
                                            59.95, 212,  120};
      static const auto img =
          kebi_follow::synth::render(K, 9.0, kebi_follow::synth::Scene{});
      sensor_msgs::msg::Image m;
      m.header.stamp = rclcpp::Time(
          stamp.nanoseconds() - static_cast<int64_t>(depth_stamp_lag_ * 1e9));
      m.header.frame_id = "inusensor_depth";
      m.width = K.width;
      m.height = K.height;
      m.encoding = "16UC1";
      m.step = static_cast<uint32_t>(img.step());
      m.data = img.data;
      if (depth_truncate_) {
        m.data.resize(m.data.size() / 2);  // 크기가 맞지 않는 메시지
      }
      sensor_msgs::msg::CameraInfo info;
      info.header = m.header;
      info.width = K.width;
      info.height = K.height;
      info.k = {K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0};
      pub_info_->publish(info);
      pub_depth_->publish(m);
    };
  }

  void spinFor(double seconds) {
    const double end = wallNow() + seconds;
    while (wallNow() < end) {
      exec_->spin_some(5ms);
      std::this_thread::sleep_for(1ms);
    }
  }

  bool waitFor(const std::function<bool()>& pred, double timeout) {
    const double end = wallNow() + timeout;
    while (wallNow() < end) {
      exec_->spin_some(5ms);
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  void ui(uint16_t cmd) {
    follow_msgs::msg::UiClient m;
    m.cmd = cmd;
    pub_ui_->publish(m);
  }

  uint8_t fsm() const { return have_state_ ? state_.fsm : 255; }

  bool followingAndMoving(double v_min = 0.05) {
    return waitFor(
        [&]() {
          return fsm() == FollowState::FSM_FOLLOWING && !cmds_.empty() &&
                 cmds_.back().v > v_min;
        },
        8.0);
  }

  std::size_t cmdsAfter(double t) const {
    std::size_t n = 0;
    for (const auto& c : cmds_)
      n += (c.t >= t) ? 1 : 0;
    return n;
  }

  bool depth_enable_{false};
  std::shared_ptr<rclcpp::Node> ctrl_;
  std::shared_ptr<rclcpp::Node> io_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  rclcpp::Publisher<follow_msgs::msg::UiClient>::SharedPtr pub_ui_;
  rclcpp::Publisher<FollowTarget>::SharedPtr pub_target_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<robot_custom_msgs::msg::BottomIrData>::SharedPtr pub_ir_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_lifted_, pub_fall_;
  rclcpp::Publisher<robot_custom_msgs::msg::AbnormalEventData>::SharedPtr
      pub_coll_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<FollowState>::SharedPtr sub_state_;
  rclcpp::TimerBase::SharedPtr feed_timer_;
  std::vector<Cmd> cmds_;
  FollowState state_;
  bool have_state_{false};
  double t0_{0.0};
  long tick_{0};
  // 발행 설정
  bool feed_odom_{true}, feed_ir_{true}, feed_lifted_{true}, feed_fall_{true},
      feed_scan_{true}, feed_target_{true};
  bool lifted_value_{false};
  double odom_v_{0.0};
  std::vector<std::pair<int, float>> scan_returns_;
  uint8_t target_status_{FollowTarget::STATUS_LOCKED};
  double target_x_{2.0}, target_y_{0.0}, target_vx_{0.0};
  double target_stamp_offset_{0.0};
  std::unique_ptr<rclcpp::Time> target_fixed_stamp_;
  std::function<void(double, const rclcpp::Time&)> depth_frame_;
  double tf_pitch_deg_{0.0};     // 카메라 TF 숙임 (로봇 URDF 는 0)
  double depth_stamp_lag_{0.0};  // depth stamp 를 이만큼 과거로 [s]
  bool depth_truncate_{false};
};

}  // namespace

TEST_F(NodeTest, HaltTopicGivesImmediateZeroAndLatches) {
  startNode();
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  const double t_event = wallNow();
  robot_custom_msgs::msg::AbnormalEventData e;
  e.event_trigger = 1;
  pub_coll_->publish(e);
  ASSERT_TRUE(waitFor(
      [&]() {
        for (const auto& c : cmds_) {
          if (c.t >= t_event && c.v == 0.0 && c.w == 0.0)
            return true;
        }
        return false;
      },
      1.0));
  double first_zero = 0.0;
  for (const auto& c : cmds_) {
    if (c.t >= t_event && c.v == 0.0 && c.w == 0.0) {
      first_zero = c.t;
      break;
    }
  }
  // 콜백에서 바로 낸다 (다음 20 Hz 주기를 기다리지 않음)
  EXPECT_LT(first_zero - t_event, 0.04);
  spinFor(1.0);
  EXPECT_EQ(fsm(), FollowState::FSM_HALTED);
  EXPECT_TRUE(state_.stop_reasons & FollowState::STOP_COLLISION);
  for (const auto& c : cmds_) {
    if (c.t > t_event + 0.01) {
      EXPECT_EQ(c.v, 0.0);
      EXPECT_EQ(c.w, 0.0);
    }
  }
}

TEST_F(NodeTest, LevelSignalHeld1_5sBlocksRestart) {
  startNode();
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  lifted_value_ = true;
  ASSERT_TRUE(waitFor([&]() { return fsm() == FollowState::FSM_HALTED; }, 1.0));
  spinFor(0.2);
  // 마지막 true 뒤로 아무것도 안 옴 → 1.5 s 동안 레벨 유지
  feed_lifted_ = false;
  const double t_last = wallNow();
  spinFor(1.2 - (wallNow() - t_last));
  ui(1);
  spinFor(0.3);
  EXPECT_EQ(fsm(), FollowState::FSM_HALTED) << "START within hold accepted";
  spinFor(2.8 - (wallNow() - t_last));
  ui(1);
  // START 는 받아들여진다. 다만 lifted 가 1 s 넘게 끊겼으므로 바로 WAITING(센서 끊김)
  EXPECT_TRUE(
      waitFor([&]() { return fsm() == FollowState::FSM_WAITING; }, 1.0));
  EXPECT_TRUE(state_.stop_reasons & FollowState::STOP_SENSOR_TIMEOUT);
  // lifted 가 다시 (false 로) 오면 재개
  lifted_value_ = false;
  feed_lifted_ = true;
  EXPECT_TRUE(
      waitFor([&]() { return fsm() == FollowState::FSM_FOLLOWING; }, 2.0));
}

TEST_F(NodeTest, BottomIrTimeout0_2s) {
  startNode();
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  feed_ir_ = false;
  const double t_stop = wallNow();
  ASSERT_TRUE(
      waitFor([&]() { return !cmds_.empty() && cmds_.back().v == 0.0; }, 1.0));
  const double dt = cmds_.back().t - t_stop;
  EXPECT_GT(dt, 0.15);
  EXPECT_LT(dt, 0.4);
  spinFor(0.3);
  EXPECT_EQ(fsm(), FollowState::FSM_WAITING);
  EXPECT_TRUE(state_.stop_reasons & FollowState::STOP_SENSOR_TIMEOUT);
}

TEST_F(NodeTest, IdleSilenceIncludingAfterStopAndHalt) {
  startNode();
  spinFor(1.0);
  EXPECT_TRUE(cmds_.empty()) << "IDLE published /cmd_vel";
  // STOP 뒤: 0 을 3 번 낸 뒤 침묵 (STOP 처리 전에 이미 나간 명령 1 개는 허용)
  auto checkStopBurst = [&](double t_stop) {
    spinFor(1.5);
    std::vector<Cmd> after;
    for (const auto& c : cmds_) {
      if (c.t >= t_stop)
        after.push_back(c);
    }
    EXPECT_GE(after.size(), 3u);
    EXPECT_LE(after.size(), 4u);
    for (std::size_t i = after.size() >= 3 ? after.size() - 3 : 0;
         i < after.size(); ++i) {
      EXPECT_EQ(after[i].v, 0.0);
      EXPECT_EQ(after[i].w, 0.0);
    }
    EXPECT_EQ(fsm(), FollowState::FSM_IDLE);
    const double t_quiet = wallNow();
    spinFor(1.0);
    EXPECT_EQ(cmdsAfter(t_quiet), 0u) << "IDLE published after the burst";
  };
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  ui(3);
  checkStopBurst(wallNow());
  // START → HALT → STOP: HALTED 에서 나갈 때도 같다
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  robot_custom_msgs::msg::AbnormalEventData e;
  e.event_trigger = -1;
  pub_coll_->publish(e);
  ASSERT_TRUE(waitFor([&]() { return fsm() == FollowState::FSM_HALTED; }, 1.0));
  spinFor(0.3);
  ui(3);
  checkStopBurst(wallNow());
}

TEST_F(NodeTest, AnyOtherCmdIsStop) {
  startNode();
  for (uint16_t cmd : {2, 3, 4, 5, 0, 99}) {
    ui(1);
    ASSERT_TRUE(waitFor(
        [&]() {
          return fsm() == FollowState::FSM_ACQUIRING ||
                 fsm() == FollowState::FSM_FOLLOWING;
        },
        2.0))
        << "cmd " << cmd;
    ui(cmd);
    EXPECT_TRUE(waitFor([&]() { return fsm() == FollowState::FSM_IDLE; }, 1.0))
        << "cmd " << cmd << " did not stop";
  }
}

TEST_F(NodeTest, ShutdownPublishesZero) {
  startNode();
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  // 소멸자 (컴포넌트 unload 와 같음) → 0 명령. 실행기는 시험 노드만으로 다시 만든다
  exec_->remove_node(ctrl_);
  exec_->remove_node(io_);
  exec_.reset();
  ctrl_.reset();
  exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(io_);
  ASSERT_TRUE(waitFor(
      [&]() {
        return !cmds_.empty() && cmds_.back().v == 0.0 && cmds_.back().w == 0.0;
      },
      1.0));
  const double t_dead = wallNow();
  spinFor(0.5);
  EXPECT_EQ(cmdsAfter(t_dead), 0u);
}

TEST_F(NodeTest, TargetExtrapolationSignAndEgoMotion) {
  startNode();
  target_status_ = FollowTarget::STATUS_CANDIDATE;  // 움직이지 않게 (ACQUIRING)
  target_stamp_offset_ = -0.15;
  target_vx_ = 0.5;
  ui(1);
  ASSERT_TRUE(
      waitFor([&]() { return fsm() == FollowState::FSM_ACQUIRING; }, 1.0));
  spinFor(0.5);
  // stamp 뒤 0.15~0.25 s (메시지 간격만큼 더) 동안 +0.5 m/s 로 멀어짐 → 약 2.1 m
  EXPECT_NEAR(state_.target_range, 2.0 + 0.5 * 0.20, 0.04);
  target_vx_ = -0.5;
  spinFor(0.5);
  EXPECT_NEAR(state_.target_range, 2.0 - 0.5 * 0.20, 0.04);
  // 로봇이 +0.4 m/s 로 움직이는 중이면 stamp 뒤 이동(약 0.08 m)만큼 더 가깝다
  target_vx_ = 0.0;
  odom_v_ = 0.4;
  spinFor(0.6);
  EXPECT_NEAR(state_.target_range, 2.0 - 0.4 * 0.20, 0.04);
}

TEST_F(NodeTest, ScanTfTransformAndCacheRefresh) {
  startNode();
  // 라이다를 뒤로 달았다(yaw 180°, x +0.1): 라이다 각 180°(인덱스 끝) 의 1 m 점 → base (1.1, 0)
  publishScanTf(0.1, M_PI);
  scan_returns_ = {{kScanRays - 1, 1.0f}};
  ui(1);
  ASSERT_TRUE(waitFor(
      [&]() {
        return fsm() == FollowState::FSM_ACQUIRING ||
               fsm() == FollowState::FSM_FOLLOWING;
      },
      1.0));
  ASSERT_TRUE(waitFor(
      [&]() { return std::abs(state_.scan_obstacle_distance - 0.88) < 0.02; },
      2.0))
      << "scan_obstacle_distance " << state_.scan_obstacle_distance;
  // TF 를 정방향으로 바꾸고 다시 START: 같은 점이 뒤(-0.9, 0) → 통로 밖
  ui(3);
  publishScanTf(0.1, 0.0);
  spinFor(0.3);
  ui(1);
  EXPECT_TRUE(
      waitFor([&]() { return state_.scan_obstacle_distance > 100.0; }, 2.0))
      << "stale cos/sin cache: " << state_.scan_obstacle_distance;
}

TEST_F(NodeTest, StartDropsTargetsStampedBeforeStart) {
  startNode();
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  robot_custom_msgs::msg::AbnormalEventData e;
  e.event_trigger = 1;
  pub_coll_->publish(e);
  ASSERT_TRUE(waitFor([&]() { return fsm() == FollowState::FSM_HALTED; }, 1.0));
  spinFor(1.2);  // 홀드오프 지남
  // START 직전에 찍힌 측정이 START 뒤에 도착하는 경우 (이전 세션 락)
  target_fixed_stamp_ = std::make_unique<rclcpp::Time>(io_->now());
  spinFor(0.05);
  ui(1);
  spinFor(0.25);
  EXPECT_EQ(fsm(), FollowState::FSM_ACQUIRING) << "stale lock consumed";
  target_fixed_stamp_.reset();  // 새 측정
  EXPECT_TRUE(
      waitFor([&]() { return fsm() == FollowState::FSM_FOLLOWING; }, 1.0));
}

TEST_F(NodeTest, DepthLatencyCompensation) {
  depth_enable_ = true;
  // 로봇이 1 m/s 로 상자에 다가가고, depth 영상은 0.3 s 늦게 온다 (camera_latency_s 0.3)
  constexpr double kLatency = 0.3;
  constexpr double kBoxX = 2.4;  // odom 에서 상자 앞면
  const kebi_follow::CameraIntrinsics K{106.0, 85.0, 105.975, 59.95, 212, 120};
  startNode({rclcpp::Parameter("camera_latency_s", kLatency),
             rclcpp::Parameter("depth.stride", 1)});
  odom_v_ = 0.0;
  feed_target_ = false;
  depth_frame_ = [&](double t, const rclcpp::Time& stamp) {
    const double x_cap = odom_v_ * (t - kLatency - t0_);  // 촬영 때 로봇 위치
    kebi_follow::synth::Scene sc;
    sc.boxes.push_back(
        {kBoxX - x_cap, kBoxX - x_cap + 0.3, -0.3, 0.3, 0.0, 0.3});
    const auto img = kebi_follow::synth::render(K, 9.0, sc);
    sensor_msgs::msg::Image m;
    m.header.stamp = stamp;
    m.header.frame_id = "inusensor_depth";
    m.width = K.width;
    m.height = K.height;
    m.encoding = "16UC1";
    m.step = static_cast<uint32_t>(img.step());
    m.data = img.data;
    sensor_msgs::msg::CameraInfo info;
    info.header = m.header;
    info.width = K.width;
    info.height = K.height;
    info.k = {K.fx, 0.0, K.cx, 0.0, K.fy, K.cy, 0.0, 0.0, 1.0};
    pub_info_->publish(info);
    pub_depth_->publish(m);
  };
  publishScanTf(0.0, 0.0);
  ui(1);
  ASSERT_TRUE(
      waitFor([&]() { return fsm() == FollowState::FSM_ACQUIRING; }, 1.0));
  ASSERT_TRUE(waitFor([&]() { return state_.depth_ok; }, 3.0))
      << "depth not ok";
  odom_v_ = 1.0;
  t0_ = wallNow();
  spinFor(0.9);
  // 지금 로봇 위치 기준 기대 거리 (상태 메시지는 최대 1 주기 늦다)
  const double x_now = odom_v_ * (wallNow() - t0_);
  const double expect = kBoxX - x_now - 0.22;
  EXPECT_NEAR(state_.depth_obstacle_distance, expect, 0.12)
      << "uncompensated would be ~" << expect + odom_v_ * kLatency;
}

TEST_F(NodeTest, InvalidTargetIsIgnored) {
  // m3: NaN·Inf·8 m 초과 위치는 받지 않은 것으로, NaN 속도는 0 으로
  startNode();
  ui(1);
  for (double bad :
       {std::nan(""), std::numeric_limits<double>::infinity(), 9.0}) {
    target_x_ = bad;
    spinFor(0.6);
    EXPECT_EQ(fsm(), FollowState::FSM_ACQUIRING) << "x=" << bad;
    EXPECT_FLOAT_EQ(state_.target_range, -1.0f) << "x=" << bad;
  }
  target_x_ = 2.0;
  target_vx_ = std::nan("");
  ASSERT_TRUE(followingAndMoving());
  for (const auto& c : cmds_) {
    EXPECT_TRUE(std::isfinite(c.v) && std::isfinite(c.w));
  }
}

TEST_F(NodeTest, EmergencyStopLastValueBlocksStart) {
  // m6: /emergency_stop 의 마지막 값이 true 인 동안 START 거부
  startNode();
  ui(1);
  ASSERT_TRUE(followingAndMoving());
  std_msgs::msg::Bool on;
  on.data = true;
  pub_estop_->publish(on);
  ASSERT_TRUE(waitFor([&]() { return fsm() == FollowState::FSM_HALTED; }, 1.0));
  spinFor(1.5);  // 홀드오프 지남
  ui(1);
  spinFor(0.3);
  EXPECT_EQ(fsm(), FollowState::FSM_HALTED);
  pub_estop_->publish(std_msgs::msg::Bool());
  spinFor(0.1);
  ui(1);
  EXPECT_TRUE(waitFor([&]() { return fsm() != FollowState::FSM_HALTED; }, 1.0));
}

TEST_F(NodeTest, MalformedDepthIsDroppedWithoutCrash) {
  // m8: data 가 step*height 보다 짧은 영상은 버린다 (depth_ok false), 정상 영상이 오면 회복
  depth_enable_ = true;
  startNode({rclcpp::Parameter("depth.stride", 1)});
  feed_target_ = false;
  depth_truncate_ = true;
  useFloorDepth();
  ui(1);
  spinFor(1.5);
  EXPECT_FALSE(state_.depth_ok);
  depth_truncate_ = false;
  EXPECT_TRUE(waitFor([&]() { return state_.depth_ok; }, 3.0));
}

TEST_F(NodeTest, LateDepthFramesAreNotFresh) {
  // m4: 평소보다 1.5 s 늦게 온 영상(수신-stamp 차가 최근 최솟값보다 1.5 s 큼)은 신선하지 않다
  depth_enable_ = true;
  startNode({rclcpp::Parameter("depth.stride", 1)});
  feed_target_ = false;
  useFloorDepth();
  ui(1);
  ASSERT_TRUE(waitFor([&]() { return state_.depth_ok; }, 3.0));
  depth_stamp_lag_ = 1.5;
  EXPECT_TRUE(waitFor([&]() { return !state_.depth_ok; }, 1.0));
}

TEST_F(NodeTest, DoublePitchCorrectionDegradesDepth) {
  // m10: URDF 가 이미 9° 숙였는데 보정 9° 도 주면 depth 를 믿지 않는다 (0.3 m/s 상한)
  depth_enable_ = true;
  tf_pitch_deg_ = 9.0;
  startNode({rclcpp::Parameter("depth.stride", 1)});
  feed_target_ = false;
  useFloorDepth();
  ui(1);
  bool ever_ok = false;
  const double end = wallNow() + 2.0;
  while (wallNow() < end) {
    spinFor(0.05);
    ever_ok = ever_ok || state_.depth_ok;
  }
  EXPECT_FALSE(ever_ok);
}

TEST_F(NodeTest, ParametersTypeTolerantAndStartupOnlyReadOnly) {
  // m9: 정수로 쓴 실수 파라미터는 받고, 기동 전용 파라미터의 실행 중 변경은 거부,
  // 엉뚱한 타입·범위 밖 값은 이름을 밝혀 기동 실패
  startNode({rclcpp::Parameter("max_linear_speed", 0)});
  EXPECT_FALSE(
      ctrl_->set_parameter(rclcpp::Parameter("control_rate_hz", 10.0))
          .successful);
  ui(1);
  spinFor(1.0);
  for (const auto& c : cmds_) {
    EXPECT_LE(c.v, 0.002 + 1e-9);  // 상한 0 (제자리 회전 creep 만 허용)
  }
  rclcpp::NodeOptions bad;
  bad.parameter_overrides({rclcpp::Parameter("max_linear_speed", "fast")});
  EXPECT_THROW(kebi_follow::createFollowController(bad), std::invalid_argument);
  rclcpp::NodeOptions out;
  out.parameter_overrides({rclcpp::Parameter("max_linear_speed", 2.0)});
  EXPECT_THROW(kebi_follow::createFollowController(out), std::invalid_argument);
}

TEST_F(NodeTest, TunableParametersApplyAtRuntime) {
  startNode();
  // 범위 밖, 서로 모순, 엉뚱한 타입은 거부하고 값은 그대로
  EXPECT_FALSE(
      ctrl_->set_parameter(rclcpp::Parameter("max_linear_speed", 1.5))
          .successful);
  EXPECT_FALSE(ctrl_->set_parameter(rclcpp::Parameter("stop_distance", 0.95))
                   .successful);  // 기본 follow 1.0: 0.95 + 0.1 > 1.0
  EXPECT_FALSE(
      ctrl_->set_parameter(rclcpp::Parameter("max_linear_speed", "fast"))
          .successful);
  EXPECT_DOUBLE_EQ(ctrl_->get_parameter("stop_distance").as_double(), 0.8);
  // 한꺼번에 바꾸면 서로 맞는지 같이 본다
  EXPECT_TRUE(ctrl_
                  ->set_parameters_atomically(
                      {rclcpp::Parameter("follow_distance", 1.5),
                       rclcpp::Parameter("stop_distance", 1.2)})
                  .successful);
  EXPECT_TRUE(
      ctrl_->set_parameter(rclcpp::Parameter("max_linear_speed", 0.3))
          .successful);
  ui(1);
  ASSERT_TRUE(waitFor([&]() { return followingAndMoving(); }, 3.0));
  spinFor(1.5);
  for (const auto& c : cmds_) {
    EXPECT_LE(c.v, 0.3 + 1e-9);
  }
  // 따라가는 중에 낮추면 감속 제한으로 내려가고 상태는 그대로
  EXPECT_TRUE(ctrl_->set_parameter(rclcpp::Parameter("max_linear_speed", 0.15))
                  .successful);
  cmds_.clear();
  spinFor(1.0);
  ASSERT_FALSE(cmds_.empty());
  for (const auto& c : cmds_) {
    if (c.t > cmds_.back().t - 0.5) {
      EXPECT_LE(c.v, 0.15 + 1e-9);
    }
  }
  EXPECT_EQ(fsm(), FollowState::FSM_FOLLOWING);
}

int main(int argc, char** argv) {
  // 격리: 로봇 도메인과 사용자 DDS 프로파일(로봇 peer)을 절대 쓰지 않는다
  const char* dom = std::getenv("ROS_DOMAIN_ID");
  if (dom == nullptr || std::string(dom).empty() || std::string(dom) == "30") {
    setenv("ROS_DOMAIN_ID", "73", 1);
  }
  const std::string profile =
      ::testing::TempDir() + "kebi_follow_node_test_fastdds.xml";
  std::ofstream(profile)
      << "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n"
         "<dds xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">"
         "<profiles><transport_descriptors><transport_descriptor>"
         "<transport_id>udp_lo</transport_id><type>UDPv4</type>"
         "<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>"
         "</transport_descriptor></transport_descriptors>"
         "<participant profile_name=\"p\" is_default_profile=\"true\"><rtps>"
         "<builtin><initialPeersList><locator><udpv4><address>127.0.0.1"
         "</address></udpv4></locator></initialPeersList></builtin>"
         "<userTransports><transport_id>udp_lo</transport_id></userTransports>"
         "<useBuiltinTransports>false</useBuiltinTransports>"
         "</rtps></participant></profiles></dds>\n";
  setenv("FASTRTPS_DEFAULT_PROFILES_FILE", profile.c_str(), 1);
  setenv("ROS_LOCALHOST_ONLY", "0", 1);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
