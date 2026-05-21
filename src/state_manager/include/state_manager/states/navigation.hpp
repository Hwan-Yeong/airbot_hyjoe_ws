#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

#define USE_JSLLOC 1

#include "state_manager/states/state_base.hpp"
#include "robot_custom_msgs/msg/position.hpp"
#include "state_manager/utils/navi_defines.hpp"
#include <cmath>


namespace airbot_state {

class Navigation : public stateBase {
  
public:
  Navigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  void clearMoveTarget();

  rclcpp::Time nav_node_start_time;
  
  MOVING_DATA movingData;
  NAVI_STATE movingState;
  READY_NAVIGATION readyNavi;
  
  double node_start_time;
  uint8_t retry_localization;
  uint8_t local_mode_retry_cnt = 0;
  bool bSkipInitPoseLocalization = false;
  bool bNaviNodeRecovery = false;
  uint8_t keepout_enable_cnt = 0;

  std::chrono::time_point<std::chrono::steady_clock> localization_start_time;
  std::chrono::time_point<std::chrono::steady_clock> check_nav_active_start_time_;

  void setReadyNavigation(READY_NAVIGATION set);
  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();
  int8_t localizationChecker();

};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
