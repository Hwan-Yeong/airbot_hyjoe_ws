#ifndef STATE_BASE_HPP_
#define STATE_BASE_HPP_

#include <algorithm>
#include <chrono>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

#include "robot_custom_msgs/msg/position.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"


#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "state_manager/utils/state_utils.hpp"
#include "state_manager/utils/state_defines.hpp"
#include "state_manager/utils/navi_defines.hpp"

#include <functional>

#define CHARGE_STOP 0xF
#define CHARGE_START 0x0

struct pose {
  double x;
  double y;
  double theta;
  double timestamp;

  pose() : x(0.0), y(0.0), theta(0.0), timestamp(0.0) {}
};

struct rotationData
{
    bool immediately= false;
    bool progress = false; // 초기화 추가. ** 스리짇 로그에 나온 에러 원인.
    uint8_t type = 0;
    double target = 0.0;
    double accAngle = 0.0;
    double preTheta = 0.0;
};

namespace airbot_state {
class stateBase {
public:
  stateBase(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils);

  virtual void run(const std::shared_ptr<StateUtils> &state_utils);

  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils);
  bool isFirstRunning();

  //ROTATION
  void rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg);
  void startRotation(bool emmediately,uint8_t type, double targetAngle);
  void setRotationTarget(bool emmediately,uint8_t type, double targetAngle);
  void startRotateMonitor();
  void progressRotation();
  void progressRotationTarget(double target, double current);
  double normalize_angle(double angle);
  bool checkRotationTarget(double diff);
  void stopMonitorRotate();
  int checkRotationDirection(double diff);
  void reset_Rotationtimer();
  void progressRotation360(int direction, double current);
  void startSearchOpenSpace();
  void disableSearchOpenSpace();
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool isSearchedCompleteOpenSpace();
  void calculateScanOpenSpace();
  double getOpenSpaceHeading();


  protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<StateUtils> state_utils;
  int id;
  bool first_running;
  int odom_retry_count = 0;
  double wait_odom_time = 0.0;
  std::chrono::time_point<std::chrono::steady_clock> odom_set_start_time;
  // ROTATION
  bool get_start_rotate = false;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::MoveNRotation>::SharedPtr rotation_sub_;
  rclcpp::TimerBase::SharedPtr rotation_target_timer_;
  robot_custom_msgs::msg::MoveNRotation::SharedPtr rotate_msg;
  ROTATE_STATE rotate_state = ROTATE_STATE::ROTATE_ODOM_SET;
  rotationData rotation;
  std::chrono::time_point<std::chrono::steady_clock> rotation_start_time;
  double debug_rotation_time = 0.0;
  double openspaceRad = 0.0;
  bool bSearched = false;
  const int num_sections = (int)(360 / 30);  // 360도 / 30도
  sensor_msgs::msg::LaserScan auto_rotation_scan_msg;
  bool bReceivedScan = false;
  bool scan_start = false;
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

} // namespace airbot_state

#endif // STATE_BASE_HPP_