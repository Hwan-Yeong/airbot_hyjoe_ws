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

#include "robot_custom_msgs/msg/navi_state.hpp"
#include "robot_custom_msgs/msg/position.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "robot_custom_msgs/msg/robot_state.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "state_manager/utils/state_utils.hpp"
#include "state_manager/utils/state_defines.hpp"

#include <functional>

// enum class DockingChargeCommand : uint8_t
// {
//     // ��oA�� 4 ��n���� (CHARGE)
//     CHARGE_CONTROL_MCU = 0x0,      // MCU�������� A��Au A|��i (Auto) - 3.8A ��i��OA��Au (Default)
//     CHARGE_CONTROL_AP = 0x1,       // MCU�������� A��Au A|��i (Auto) - 1A Au��OA��Au
//     HIGH_SPEED_CHARGE_START = 0x2, // ��i��O A��Au ��AAU (3.8A ��i��OA��Au)
//     LOW_SPEED_CHARGE_START = 0x3,  // Au��O A��Au ��AAU (1A Au��OA��Au)
//     CHARGE_STOP = 0xF,             // A��Au A��Ao

//     // CIA�� 4 ��n���� (DOCKING)
//     DOCKING_START = 0x1, // ���A�� ��AAU
//     DOCKING_STOP = 0x0   // ���A�� ����A��
// };

#define DOCK_START 0x01
#define DOCK_STOP 0x00
#define CHARGE_STOP 0xF
#define CHARGE_START 0x0

struct pose {
  double x;
  double y;
  double theta;
  double timestamp;

  pose() : x(0.0), y(0.0), theta(0.0), timestamp(0.0) {}
};

namespace airbot_state {
class stateBase {
public:
  stateBase(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils);

  virtual void run(const std::shared_ptr<StateUtils> &state_utils);

  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils);
  bool isFirstRunning();

  void reserveTarget(pose pose_data);
  bool set_movetarget;
  pose target_pose;
  pose pre_target_pose;

protected:
  int id, id_pre;
  bool bStatusChange;
  bool first_running;
  std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<StateUtils> state_utils;
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

} // namespace airbot_state

#endif // STATE_BASE_HPP_