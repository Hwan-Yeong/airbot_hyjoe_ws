#ifndef RETURNCHARGER_HPP_
#define RETURNCHARGER_HPP_

#define USE_JSLLOC 1

#include "state_manager/states/state_base.hpp"

namespace airbot_state {

class ReturnCharger : public stateBase {
public:
  ReturnCharger(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // return charger function
  void startMonitorReturnCharger();
  void stopMonitorReturnCharger();
  void reset_timerNaviStatus();
  void monitor_returnCharger();

  void exitMappingNode();

  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();
  int8_t localizationChecker();

  void setReadyNavigation(READY_NAVIGATION set);

  bool detectDockShortSig();
  bool detectDockLongSig();

  rclcpp::TimerBase::SharedPtr nav_status_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dock_pub;
  
  rclcpp::Time nav_node_start_time;
  double node_start_time;
  bool dock_pose_estimate = false;
  std_msgs::msg::UInt8 dock_cmd_;
  uint8_t altgoal_retry_count = 0;
  pose return_pose;  

  MOVING_DATA movingData;
  NAVI_STATE movingState;
  READY_NAVIGATION readyNavi;
  bool bNaviNodeRecovery = false;

  std::chrono::time_point<std::chrono::steady_clock> localization_start_time;
  uint8_t local_mode_retry_cnt = 0;
  
  uint8_t keepout_enable_cnt = 0;
};

} // namespace airbot_state

#endif // RETURNCHARGER_HPP_
